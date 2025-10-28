from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple
import re

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

IGNORED_TOPICS = {"/parameter_events", "/rosout"}
TRANSFORM_PREFIX = "transform_listener_impl_"


@dataclass(frozen=True)
class NodeDescriptor:
    identifier: str
    local_identifier: str
    name: str
    namespace: str
    is_dummy: bool = False
    group_identifier: str | None = None
    group_label: str | None = None

    @property
    def label(self) -> str:
        if self.is_dummy:
            return self.name
        if self.group_identifier:
            return self.name
        if self.namespace and self.namespace != "/":
            return f"{self.namespace}/{self.name}"
        return self.name


@dataclass(frozen=True)
class NamespaceGroup:
    identifier: str
    label: str
    nodes: List[NodeDescriptor]


@dataclass(frozen=True)
class EdgeDescriptor:
    source: str
    target: str
    topic_name: str
    topic_type: str
    is_virtual: bool = False

    @property
    def label(self) -> str:
        if self.topic_type:
            return f"{self.topic_name}\\n{self.topic_type}"
        return self.topic_name


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

    def _collect_graph(self) -> Tuple[List[NodeDescriptor], List[EdgeDescriptor], List[str], List[NamespaceGroup]]:
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

        nodes = [
            (name, namespace)
            for name, namespace in nodes
            if self._should_include_node(name, namespace)
        ]

        topics = [
            (name, types)
            for name, types in topics
            if self._should_include_topic(name)
        ]

        node_descriptors: List[NodeDescriptor] = []
        node_lookup: Dict[Tuple[str, str], str] = {}

        def node_priority(entry: Tuple[str, str]) -> Tuple[str, str, str]:
            name, namespace = entry
            canonical_ns = namespace if namespace else "/"
            return (canonical_ns, name, namespace)

        used_names: Dict[str | None, set[str]] = {}

        for index, (name, namespace) in enumerate(sorted(nodes, key=node_priority)):
            namespace = namespace or ""
            stripped_ns = namespace.strip("/")
            group_identifier: str | None = None
            group_label: str | None = None
            if stripped_ns:
                first_segment = stripped_ns.split("/", 1)[0]
                group_identifier = self._sanitize_identifier(first_segment)
                group_label = f"/{first_segment}"

            base_label = name or f"node_{index}"
            base_identifier = self._sanitize_identifier(base_label)

            group_key = group_identifier
            group_used = used_names.setdefault(group_key, set())
            unique_identifier = base_identifier
            counter = 1
            while unique_identifier in group_used:
                counter += 1
                unique_identifier = f"{base_identifier}_{counter}"
            group_used.add(unique_identifier)

            if group_identifier:
                identifier = f"{group_identifier}.{unique_identifier}"
                local_identifier = unique_identifier
            else:
                identifier = unique_identifier
                local_identifier = unique_identifier

            descriptor = NodeDescriptor(
                identifier=identifier,
                local_identifier=local_identifier,
                name=name,
                namespace=namespace,
                is_dummy=False,
                group_identifier=group_identifier,
                group_label=group_label,
            )
            node_descriptors.append(descriptor)
            node_lookup[(namespace, name)] = identifier

        topic_details: Dict[str, Dict[str, set]] = {}
        edges: List[EdgeDescriptor] = []
        dummy_nodes: Dict[Tuple[str, str], NodeDescriptor] = {}

        for topic_name, types in topics:
            topic_types = set(types)
            details = topic_details.setdefault(topic_name, {"types": topic_types, "publishers": set(), "subscribers": set()})
            details["types"].update(topic_types)

            for publisher in self._unique_endpoints(self._get_publishers_info(topic_name)):
                key = (publisher.node_namespace, publisher.node_name)
                if key not in node_lookup:
                    continue
                details["publishers"].add(node_lookup[key])
                if publisher.topic_type:
                    details["types"].add(publisher.topic_type)

            for subscriber in self._unique_endpoints(self._get_subscriptions_info(topic_name)):
                key = (subscriber.node_namespace, subscriber.node_name)
                if key not in node_lookup:
                    continue
                details["subscribers"].add(node_lookup[key])
                if subscriber.topic_type:
                    details["types"].add(subscriber.topic_type)

        for topic_name, details in sorted(topic_details.items()):
            publishers: set[str] = details["publishers"]
            subscribers: set[str] = details["subscribers"]
            topic_types: set[str] = details["types"]
            topic_type = next((t for t in sorted(topic_types) if t), "")

            if publishers and subscribers:
                for publisher_id in publishers:
                    for subscriber_id in subscribers:
                        edges.append(
                            EdgeDescriptor(
                                source=publisher_id,
                                target=subscriber_id,
                                topic_name=topic_name,
                                topic_type=topic_type,
                                is_virtual=False,
                            )
                        )
            elif publishers and not subscribers:
                dummy = self._ensure_dummy_node(
                    node_descriptors,
                    dummy_nodes,
                    topic_name,
                    role="subscriber",
                )
                for publisher_id in publishers:
                    edges.append(
                        EdgeDescriptor(
                            source=publisher_id,
                            target=dummy.identifier,
                            topic_name=topic_name,
                            topic_type=topic_type,
                            is_virtual=True,
                        )
                    )
            elif subscribers and not publishers:
                dummy = self._ensure_dummy_node(
                    node_descriptors,
                    dummy_nodes,
                    topic_name,
                    role="publisher",
                )
                for subscriber_id in subscribers:
                    edges.append(
                        EdgeDescriptor(
                            source=dummy.identifier,
                            target=subscriber_id,
                            topic_name=topic_name,
                            topic_type=topic_type,
                            is_virtual=True,
                        )
                    )

        topic_names = sorted(topic_details.keys())
        groups = self._build_namespace_groups(node_descriptors)
        return node_descriptors, edges, topic_names, groups

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
        filtered: List[TopicEndpointInfo] = []
        for info in endpoints:
            if not self._should_include_node(info.node_name, info.node_namespace):
                continue
            filtered.append(info)
        return filtered

    def _should_include_node(self, name: str, namespace: str) -> bool:
        namespace = namespace or ""
        namespace_segments = [segment for segment in namespace.split("/") if segment]

        if name.startswith(TRANSFORM_PREFIX) or TRANSFORM_PREFIX in name:
            return False
        if any(segment.startswith(TRANSFORM_PREFIX) for segment in namespace_segments):
            return False
        if name == "ros_graph_export":
            return False

        if not self.include_hidden_entities and name.startswith("_"):
            return False

        return True

    def _should_include_topic(self, topic_name: str) -> bool:
        if topic_name in IGNORED_TOPICS:
            return False
        if not self.include_hidden_entities and topic_name.startswith("_"):
            return False
        return True

    def _sanitize_identifier(self, value: str) -> str:
        safe = re.sub(r"[^0-9a-zA-Z_]+", "_", value)
        if not safe:
            safe = "group"
        if safe[0].isdigit():
            safe = f"_{safe}"
        return safe

    def _build_namespace_groups(self, nodes: Iterable[NodeDescriptor]) -> List[NamespaceGroup]:
        groups: Dict[str, NamespaceGroup] = {}

        for node in nodes:
            if node.is_dummy or node.group_identifier is None or node.group_label is None:
                continue

            group = groups.get(node.group_identifier)
            if group is None:
                group = NamespaceGroup(identifier=node.group_identifier, label=node.group_label, nodes=[])
                groups[node.group_identifier] = group
            group.nodes.append(node)

        ordered_groups = []
        for identifier in sorted(groups.keys(), key=lambda ident: groups[ident].label.lower()):
            group = groups[identifier]
            group.nodes.sort(key=lambda n: n.label.lower())
            ordered_groups.append(group)

        return ordered_groups

    def _ensure_dummy_node(
        self,
        node_descriptors: List[NodeDescriptor],
        dummy_nodes: Dict[Tuple[str, str], NodeDescriptor],
        topic_name: str,
        role: str,
    ) -> NodeDescriptor:
        key = (topic_name, role)
        if key in dummy_nodes:
            return dummy_nodes[key]

        identifier = f"ghost_{role}_{len(dummy_nodes)}"
        suffix = "no publishers" if role == "publisher" else "no subscribers"
        dummy = NodeDescriptor(
            identifier=identifier,
            local_identifier=identifier,
            name=f"{topic_name} ({suffix})",
            namespace="",
            is_dummy=True,
        )
        node_descriptors.append(dummy)
        dummy_nodes[key] = dummy
        return dummy

    def _unique_endpoints(self, endpoints: Iterable[TopicEndpointInfo]) -> Iterable[TopicEndpointInfo]:
        seen = set()
        for info in endpoints:
            signature = (info.node_name, info.node_namespace, info.topic_type)
            if signature in seen:
                continue
            seen.add(signature)
            yield info

    def _render_d2(self, graph_data: Tuple[List[NodeDescriptor], List[EdgeDescriptor], List[str], List[NamespaceGroup]], timestamp: str) -> None:
        nodes, edges, topic_names, groups = graph_data
        environment = self._load_template()

        try:
            template = environment.get_template(self._template_name or "")
        except TemplateNotFound as exc:
            raise RuntimeError(f"Unable to load D2 template '{self._effective_template_path()}': {exc}") from exc

        context = {
            "generated_at": timestamp,
            "node_count": len(nodes),
            "topic_count": len(topic_names),
            "edge_count": len(edges),
            "nodes": nodes,
            "edges": edges,
            "topic_names": topic_names,
            "groups": groups,
        }

        rendered = template.render(context)

        output_path = self.output_directory / self.d2_output_filename
        output_path.write_text(rendered, encoding="utf-8")
        self.get_logger().info(f"Wrote D2 graph to {output_path}")

    def _render_svg(self, graph_data: Tuple[List[NodeDescriptor], List[EdgeDescriptor], List[str], List[NamespaceGroup]], timestamp: str) -> None:
        nodes, edges, _, _ = graph_data
        positions: Dict[str, Tuple[float, float]] = {}

        real_nodes = [node for node in nodes if not node.is_dummy]
        dummy_nodes = [node for node in nodes if node.is_dummy]

        real_count = max(len(real_nodes), 1)
        dummy_count = max(len(dummy_nodes), 1)

        for index, node in enumerate(real_nodes):
            angle = 2.0 * math.pi * index / real_count
            positions[node.identifier] = (math.cos(angle), math.sin(angle))

        for index, node in enumerate(dummy_nodes):
            angle = 2.0 * math.pi * index / dummy_count
            positions[node.identifier] = (0.5 * math.cos(angle), 0.5 * math.sin(angle))

        figure, axis = plt.subplots(figsize=(10, 10))

        axis.set_title(f"ROS Graph ({timestamp})", fontsize=12)
        axis.axis("off")

        if not nodes:
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

        for node in real_nodes:
            x, y = positions[node.identifier]
            axis.scatter(x, y, s=400, color="#1f77b4", edgecolors="white", linewidths=1.5, zorder=3)
            axis.text(x, y, node.label, ha="center", va="center", fontsize=8, color="white", zorder=4, wrap=True)

        for edge in edges:
            start = positions.get(edge.source)
            end = positions.get(edge.target)
            if start is None or end is None:
                continue
            color = "#ff9896" if edge.is_virtual else "#17becf"
            axis.annotate(
                "",
                xy=end,
                xytext=start,
                arrowprops=dict(arrowstyle="->", color=color, lw=1.5),
                zorder=2,
            )
            label_pos = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0)
            axis.text(
                label_pos[0],
                label_pos[1],
                edge.label.replace("\\n", "\n"),
                fontsize=6,
                color="#333333",
                zorder=5,
                ha="center",
            )

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
