from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from jinja2 import Environment, FileSystemLoader, TemplateNotFound
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.topic_endpoint_info import TopicEndpointInfo
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


@dataclass(frozen=True)
class NodeDescriptor:
    identifier: str
    name: str
    namespace: str

    @property
    def label(self) -> str:
        if self.namespace and self.namespace != "/":
            return f"{self.namespace}/{self.name}"
        return self.name


@dataclass(frozen=True)
class TopicDescriptor:
    identifier: str
    name: str
    types: Tuple[str, ...]

    @property
    def label(self) -> str:
        if not self.types:
            return self.name
        type_suffix = "\\n".join(sorted(self.types))
        return f"{self.name}\\n{type_suffix}"


@dataclass(frozen=True)
class EdgeDescriptor:
    source: str
    target: str
    topic_name: str
    topic_type: str
    direction: str  # "publish" or "subscribe"

    @property
    def label(self) -> str:
        action = "pub" if self.direction == "publish" else "sub"
        return f"{action}: {self.topic_type}"


class RosGraphExport(Node):
    """Exports a live ROS graph description as D2 and SVG files."""

    def __init__(self) -> None:
        super().__init__("ros_graph_export")

        self.declare_parameter("output_directory", str(Path.home() / "ros_graph_exports"))
        self.declare_parameter("d2_template_path", "")
        self.declare_parameter("d2_output_filename", "ros_graph.d2")
        self.declare_parameter("vector_output_filename", "ros_graph.svg")
        self.declare_parameter("export_interval_seconds", 30.0)
        self.declare_parameter("export_on_startup", True)
        self.declare_parameter("include_hidden_entities", False)

        self._template_env: Environment | None = None
        self._template_name: str | None = None
        self._template_directory: Path | None = None

        self.output_directory: Path = Path(self.get_parameter("output_directory").get_parameter_value().string_value).expanduser()
        self.d2_template_path: Path | None = self._resolve_template_path(
            self.get_parameter("d2_template_path").get_parameter_value().string_value
        )
        self.d2_output_filename: str = self.get_parameter("d2_output_filename").get_parameter_value().string_value
        self.vector_output_filename: str = self.get_parameter("vector_output_filename").get_parameter_value().string_value
        self.export_interval: float = (
            self.get_parameter("export_interval_seconds").get_parameter_value().double_value
        )
        self.export_on_startup: bool = self.get_parameter("export_on_startup").get_parameter_value().bool_value
        self.include_hidden_entities: bool = self.get_parameter("include_hidden_entities").get_parameter_value().bool_value

        self.output_directory.mkdir(parents=True, exist_ok=True)

        # Prepare parameter change callback for runtime reconfiguration.
        self.add_on_set_parameters_callback(self._parameters_callback)

        # Provide a service for manual export requests.
        self.create_service(Trigger, "~/export_graph", self._handle_export_request)

        self._export_timer = None
        if self.export_interval > 0.0:
            self._export_timer = self.create_timer(self.export_interval, self._perform_export)
            self.get_logger().info(f"Scheduled periodic graph export every {self.export_interval:.1f} s")

        if self.export_on_startup:
            self._perform_export()

    def _parameters_callback(self, params: List[Parameter]) -> SetParametersResult:
        result = SetParametersResult(successful=True)

        for param in params:
            if param.name == "output_directory":
                self.output_directory = Path(param.value).expanduser()
                self.output_directory.mkdir(parents=True, exist_ok=True)
                self.get_logger().info(f"Updated output directory to {self.output_directory}")
            elif param.name == "d2_template_path":
                self.d2_template_path = self._resolve_template_path(param.value)
                self.get_logger().info(f"Using D2 template at {self._effective_template_path()}")
            elif param.name == "d2_output_filename":
                self.d2_output_filename = param.value
            elif param.name == "vector_output_filename":
                self.vector_output_filename = param.value
            elif param.name == "export_interval_seconds":
                interval = float(param.value)
                self.export_interval = interval
                if interval <= 0.0:
                    if self._export_timer is not None:
                        self._export_timer.cancel()
                        self._export_timer = None
                    self.get_logger().info("Disabled periodic graph export")
                else:
                    if self._export_timer is not None:
                        self._export_timer.cancel()
                    self._export_timer = self.create_timer(interval, self._perform_export)
                    self.get_logger().info(f"Updated export interval to {interval:.1f} s")
            elif param.name == "export_on_startup":
                self.export_on_startup = bool(param.value)
            elif param.name == "include_hidden_entities":
                self.include_hidden_entities = bool(param.value)

        return result

    def _resolve_template_path(self, requested_path: str) -> Path | None:
        if requested_path.strip():
            candidate = Path(requested_path).expanduser()
            if candidate.exists():
                return candidate
            self.get_logger().warn(f"Requested D2 template '{candidate}' not found, using default template")

        package_template = Path(__file__).parent / "templates" / "ros_graph.d2.j2"
        if package_template.exists():
            return package_template

        try:
            share_template = Path(get_package_share_directory("ros_graph_export")) / "templates" / "ros_graph.d2.j2"
            if share_template.exists():
                return share_template
        except (PackageNotFoundError, ImportError):
            pass

        self.get_logger().error("Default D2 template missing from package resources")
        return package_template

    def _effective_template_path(self) -> Path:
        return self.d2_template_path if self.d2_template_path is not None else Path()

    def _load_template(self) -> Environment:
        template_path = self._effective_template_path()
        template_dir = template_path.parent
        template_name = template_path.name

        if self._template_env is None or template_dir != self._template_directory:
            self._template_directory = template_dir
            self._template_env = Environment(loader=FileSystemLoader(str(template_dir)))

        self._template_name = template_name

        return self._template_env

    def _handle_export_request(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            self._perform_export()
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().error(f"Failed to export ROS graph: {exc}")
            response.success = False
            response.message = str(exc)
            return response

        response.success = True
        response.message = "Export completed"
        return response

    def _perform_export(self) -> None:
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S%z")
        graph = self._collect_graph()

        self._render_d2(graph, timestamp)
        self._render_svg(graph, timestamp)

        self.get_logger().info("ROS graph export completed")

    def _collect_graph(self) -> Tuple[List[NodeDescriptor], List[TopicDescriptor], List[EdgeDescriptor]]:
        try:
            nodes = self.get_node_names_and_namespaces(include_hidden_nodes=self.include_hidden_entities)
        except TypeError:
            nodes = self.get_node_names_and_namespaces()

        try:
            topics = self.get_topic_names_and_types(
                no_demangle=False, include_hidden_topics=self.include_hidden_entities
            )
        except TypeError:
            topics = self.get_topic_names_and_types(no_demangle=False)

        if not self.include_hidden_entities:
            nodes = [(name, namespace) for name, namespace in nodes if not name.startswith("_")]
            topics = [(name, types) for name, types in topics if not name.startswith("_")]

        node_descriptors: List[NodeDescriptor] = []
        node_lookup: Dict[Tuple[str, str], str] = {}
        for index, (name, namespace) in enumerate(sorted(nodes, key=lambda item: (item[1], item[0]))):
            identifier = f"nodes.node_{index}"
            node_descriptors.append(NodeDescriptor(identifier, name, namespace))
            node_lookup[(namespace, name)] = identifier

        topic_descriptors: List[TopicDescriptor] = []
        topic_lookup: Dict[str, str] = {}
        for index, (topic_name, types) in enumerate(sorted(topics, key=lambda item: item[0])):
            identifier = f"topics.topic_{index}"
            topic_descriptors.append(TopicDescriptor(identifier, topic_name, tuple(sorted(types))))
            topic_lookup[topic_name] = identifier

        edges: List[EdgeDescriptor] = []
        for topic_name, types in topics:
            for publisher in self._unique_endpoints(self._get_publishers_info(topic_name)):
                key = (publisher.node_namespace, publisher.node_name)
                if key not in node_lookup:
                    continue
                edges.append(
                    EdgeDescriptor(
                        source=node_lookup[key],
                        target=topic_lookup[topic_name],
                        topic_name=topic_name,
                        topic_type=publisher.topic_type or (types[0] if types else ""),
                        direction="publish",
                    )
                )
            for subscriber in self._unique_endpoints(self._get_subscriptions_info(topic_name)):
                key = (subscriber.node_namespace, subscriber.node_name)
                if key not in node_lookup:
                    continue
                edges.append(
                    EdgeDescriptor(
                        source=topic_lookup[topic_name],
                        target=node_lookup[key],
                        topic_name=topic_name,
                        topic_type=subscriber.topic_type or (types[0] if types else ""),
                        direction="subscribe",
                    )
                )

        return node_descriptors, topic_descriptors, edges

    def _get_publishers_info(self, topic_name: str) -> List[TopicEndpointInfo]:
        try:
            infos = self.get_publishers_info_by_topic(topic_name, include_hidden_nodes=self.include_hidden_entities)
        except TypeError:
            infos = self.get_publishers_info_by_topic(topic_name)
        return self._filter_hidden_endpoints(infos)

    def _get_subscriptions_info(self, topic_name: str) -> List[TopicEndpointInfo]:
        try:
            infos = self.get_subscriptions_info_by_topic(topic_name, include_hidden_nodes=self.include_hidden_entities)
        except TypeError:
            infos = self.get_subscriptions_info_by_topic(topic_name)
        return self._filter_hidden_endpoints(infos)

    def _filter_hidden_endpoints(self, endpoints: Iterable[TopicEndpointInfo]) -> List[TopicEndpointInfo]:
        if self.include_hidden_entities:
            return list(endpoints)

        filtered: List[TopicEndpointInfo] = []
        for info in endpoints:
            if info.node_name.startswith("_"):
                continue
            namespace_tail = info.node_namespace.split("/")[-1] if info.node_namespace else ""
            if namespace_tail.startswith("_"):
                continue
            filtered.append(info)
        return filtered

    def _unique_endpoints(self, endpoints: Iterable[TopicEndpointInfo]) -> Iterable[TopicEndpointInfo]:
        seen = set()
        for info in endpoints:
            signature = (info.node_name, info.node_namespace, info.topic_type)
            if signature in seen:
                continue
            seen.add(signature)
            yield info

    def _render_d2(self, graph_data: Tuple[List[NodeDescriptor], List[TopicDescriptor], List[EdgeDescriptor]], timestamp: str) -> None:
        nodes, topics, edges = graph_data
        environment = self._load_template()

        try:
            template = environment.get_template(self._template_name or "")
        except TemplateNotFound as exc:
            raise RuntimeError(f"Unable to load D2 template '{self._effective_template_path()}': {exc}") from exc

        context = {
            "generated_at": timestamp,
            "node_count": len(nodes),
            "topic_count": len(topics),
            "edge_count": len(edges),
            "nodes": nodes,
            "topics": topics,
            "edges": edges,
        }

        rendered = template.render(context)

        output_path = self.output_directory / self.d2_output_filename
        output_path.write_text(rendered, encoding="utf-8")
        self.get_logger().info(f"Wrote D2 graph to {output_path}")

    def _render_svg(self, graph_data: Tuple[List[NodeDescriptor], List[TopicDescriptor], List[EdgeDescriptor]], timestamp: str) -> None:
        nodes, topics, edges = graph_data
        positions: Dict[str, Tuple[float, float]] = {}

        node_count = max(len(nodes), 1)
        topic_count = max(len(topics), 1)

        for index, node in enumerate(nodes):
            angle = 2.0 * math.pi * index / node_count
            positions[node.identifier] = (math.cos(angle), math.sin(angle))

        for index, topic in enumerate(topics):
            angle = 2.0 * math.pi * index / topic_count
            positions[topic.identifier] = (0.55 * math.cos(angle), 0.55 * math.sin(angle))

        figure, axis = plt.subplots(figsize=(10, 10))

        axis.set_title(f"ROS Graph ({timestamp})", fontsize=12)
        axis.axis("off")

        if not nodes and not topics:
            axis.text(
                0.5,
                0.5,
                "No active ROS nodes or topics detected",
                transform=axis.transAxes,
                ha="center",
                va="center",
                fontsize=12,
                color="#333333",
            )

        for node in nodes:
            x, y = positions[node.identifier]
            axis.scatter(x, y, s=400, color="#1f77b4", edgecolors="white", linewidths=1.5, zorder=3)
            axis.text(x, y, node.label, ha="center", va="center", fontsize=8, color="white", zorder=4, wrap=True)

        for topic in topics:
            x, y = positions[topic.identifier]
            axis.scatter(x, y, s=300, color="#ff7f0e", edgecolors="white", linewidths=1.5, zorder=3, marker="s")
            axis.text(x, y, topic.label.replace("\\n", "\n"), ha="center", va="center", fontsize=7, color="white", zorder=4, wrap=True)

        for edge in edges:
            start = positions.get(edge.source)
            end = positions.get(edge.target)
            if start is None or end is None:
                continue
            color = "#17becf" if edge.direction == "publish" else "#bcbd22"
            axis.annotate(
                "",
                xy=end,
                xytext=start,
                arrowprops=dict(arrowstyle="->", color=color, lw=1.5),
                zorder=2,
            )
            label_pos = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0)
            axis.text(label_pos[0], label_pos[1], edge.topic_name, fontsize=6, color="#333333", zorder=5, ha="center")

        axis.set_xlim(-1.4, 1.4)
        axis.set_ylim(-1.4, 1.4)

        output_path = self.output_directory / self.vector_output_filename
        figure.savefig(output_path, format="svg")
        plt.close(figure)

        self.get_logger().info(f"Wrote SVG graph to {output_path}")


def main() -> None:
    rclpy.init()
    node = RosGraphExport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
