# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import re
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from jinja2 import Environment, FileSystemLoader, TemplateNotFound
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.topic_endpoint_info import TopicEndpointInfo
from std_srvs.srv import Trigger

IGNORED_TOPICS = {"/parameter_events", "/rosout"}
TRANSFORM_PREFIX = "transform_listener_impl_"


@dataclass(frozen=True)
class NodeDescriptor:
    """A ROS node in the exported graph."""

    identifier: str
    label: str
    namespace: str
    is_dummy: bool = False


@dataclass(frozen=True)
class ContainerDescriptor:
    """A namespace container grouping nodes in the exported graph."""

    identifier: str
    label: str
    parent: str | None = None


@dataclass(frozen=True)
class EdgeDescriptor:
    """A topic connection between two nodes in the exported graph."""

    source: str
    target: str
    topic_name: str
    topic_type: str
    edge_type: str = "normal"  # normal, missing_subscribers, missing_publishers

    @property
    def label(self) -> str:
        """Return the edge label, combining topic name and type if available."""
        if self.topic_type:
            return f"{self.topic_name}\\n{self.topic_type}"
        return self.topic_name


class Ros2GraphExport(Node):
    """Exports a live ROS graph description as a D2 diagram."""

    def __init__(self) -> None:
        """Declare parameters, resolve the template and start the export timer."""
        super().__init__("ros2_graph_export")

        self.declare_parameter("output_path", str(Path.home() / ".ros" / "ros_graph.d2"))
        self.declare_parameter("export_interval_seconds", 5.0)
        self.declare_parameter("ignore_topics_without_publishers", True)
        self.declare_parameter("ignore_topics_without_subscribers", True)

        self._template_env: Environment | None = None
        self._template_name: str | None = None
        self._template_directory: Path | None = None

        self.output_path: Path = Path(self.get_parameter("output_path").get_parameter_value().string_value).expanduser()
        self.d2_template_path: Path = self._resolve_template_path()
        self.export_interval: float = self.get_parameter("export_interval_seconds").get_parameter_value().double_value
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.ignore_topics_without_publishers: bool = (
            self.get_parameter("ignore_topics_without_publishers").get_parameter_value().bool_value
        )
        self.ignore_topics_without_subscribers: bool = (
            self.get_parameter("ignore_topics_without_subscribers").get_parameter_value().bool_value
        )

        # Prepare parameter change callback for runtime reconfiguration.
        self.add_on_set_parameters_callback(self._parameters_callback)

        # Provide a service for manual export requests.
        self.create_service(Trigger, "~/export_graph", self._handle_export_request)

        self._export_timer = None
        if self.export_interval > 0.0:
            self._export_timer = self.create_timer(self.export_interval, self._perform_export)
            self.get_logger().info(f"Scheduled periodic graph export every {self.export_interval:.1f} s")

        self._perform_export()

    def _parameters_callback(self, params: List[Parameter]) -> SetParametersResult:
        result = SetParametersResult(successful=True)

        for param in params:
            if param.name == "output_path":
                self.output_path = Path(param.value).expanduser()
                self.output_path.parent.mkdir(parents=True, exist_ok=True)
                self.get_logger().info(f"Updated output path to {self.output_path}")
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
            elif param.name == "ignore_topics_without_publishers":
                self.ignore_topics_without_publishers = bool(param.value)
            elif param.name == "ignore_topics_without_subscribers":
                self.ignore_topics_without_subscribers = bool(param.value)

        return result

    def _resolve_template_path(self) -> Path:
        package_template = Path(__file__).parent / "templates" / "ros_graph.d2.j2"
        if package_template.exists():
            return package_template

        try:
            share_template = Path(get_package_share_directory("ros2_graph_export")) / "templates" / "ros_graph.d2.j2"
            if share_template.exists():
                return share_template
        except (PackageNotFoundError, ImportError):
            pass

        self.get_logger().error("Default D2 template missing from package resources")
        return package_template

    def _load_template(self) -> Environment:
        template_path = self.d2_template_path
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

        self.get_logger().info("ROS graph export completed")

    def _collect_graph(self) -> Tuple[List[NodeDescriptor], List[EdgeDescriptor], List[str], List[ContainerDescriptor]]:
        nodes = self.get_node_names_and_namespaces()

        topics = self.get_topic_names_and_types(no_demangle=False)

        nodes = [(name, namespace) for name, namespace in nodes if self._should_include_node(name, namespace)]

        topics = [(name, types) for name, types in topics if self._should_include_topic(name)]

        node_descriptors: List[NodeDescriptor] = []
        node_lookup: Dict[Tuple[str, str], str] = {}
        containers: Dict[str, ContainerDescriptor] = {}

        def node_priority(entry: Tuple[str, str]) -> Tuple[str, str, str]:
            name, namespace = entry
            canonical_ns = namespace if namespace else "/"
            return (canonical_ns, name, namespace)

        used_names: Dict[str | None, set[str]] = {}

        for index, (name, namespace) in enumerate(sorted(nodes, key=node_priority)):
            namespace = namespace or ""
            segments = [segment for segment in namespace.strip("/").split("/") if segment]

            parent_identifier: str | None = None
            for depth, segment in enumerate(segments):
                sanitized_segment = self._sanitize_identifier(segment)
                segment_identifier = f"ns_{sanitized_segment}"
                container_identifier = f"{parent_identifier}.{segment_identifier}" if parent_identifier else segment_identifier
                label = f"/{segment}" if depth == 0 else segment
                if container_identifier not in containers:
                    containers[container_identifier] = ContainerDescriptor(
                        identifier=container_identifier,
                        label=label,
                        parent=parent_identifier,
                    )
                parent_identifier = container_identifier

            base_label = name or f"node_{index}"
            base_identifier = self._sanitize_identifier(base_label)
            group_key = parent_identifier
            group_used = used_names.setdefault(group_key, set())
            unique_identifier = base_identifier
            counter = 1
            while unique_identifier in group_used:
                counter += 1
                unique_identifier = f"{base_identifier}_{counter}"
            group_used.add(unique_identifier)

            identifier = f"{parent_identifier}.{unique_identifier}" if parent_identifier else unique_identifier
            descriptor = NodeDescriptor(
                identifier=identifier,
                label=name or unique_identifier,
                namespace=namespace,
                is_dummy=False,
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
                                edge_type="normal",
                            )
                        )
            elif publishers and not subscribers:
                if not self.ignore_topics_without_subscribers:
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
                                edge_type="missing_subscribers",
                            )
                        )
            elif subscribers and not publishers:
                if not self.ignore_topics_without_publishers:
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
                                edge_type="missing_publishers",
                            )
                        )

        topic_names = sorted(topic_details.keys())
        container_list = self._order_containers(containers)
        node_descriptors.sort(key=lambda node: node.identifier)
        return node_descriptors, edges, topic_names, container_list

    def _get_publishers_info(self, topic_name: str) -> List[TopicEndpointInfo]:
        infos = self.get_publishers_info_by_topic(topic_name)
        return self._filter_hidden_endpoints(infos)

    def _get_subscriptions_info(self, topic_name: str) -> List[TopicEndpointInfo]:
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
        if name.startswith("launch_ros_"):
            return False
        if any(segment.startswith("launch_ros_") for segment in namespace_segments):
            return False
        if name == "ros2_graph_export":
            return False

        if name.startswith("_"):
            return False

        return True

    def _should_include_topic(self, topic_name: str) -> bool:
        if topic_name in IGNORED_TOPICS:
            return False
        if topic_name.endswith("/transition_event"):
            return False
        if topic_name.startswith("_"):
            return False
        return True

    def _sanitize_identifier(self, value: str) -> str:
        safe = re.sub(r"[^0-9a-zA-Z_]+", "_", value)
        if not safe:
            safe = "group"
        if safe[0].isdigit():
            safe = f"_{safe}"
        return safe

    def _order_containers(self, containers: Dict[str, ContainerDescriptor]) -> List[ContainerDescriptor]:
        return sorted(
            containers.values(),
            key=lambda container: (self._container_depth(container.identifier), container.identifier),
        )

    def _container_depth(self, identifier: str) -> int:
        return identifier.count(".")

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
            label=f"{topic_name} ({suffix})",
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

    def _render_d2(
        self, graph_data: Tuple[List[NodeDescriptor], List[EdgeDescriptor], List[str], List[ContainerDescriptor]], timestamp: str
    ) -> None:
        nodes, edges, topic_names, containers = graph_data
        environment = self._load_template()

        try:
            template = environment.get_template(self._template_name or "")
        except TemplateNotFound as exc:
            raise RuntimeError(f"Unable to load D2 template '{self.d2_template_path}': {exc}") from exc

        context = {
            "generated_at": timestamp,
            "node_count": len(nodes),
            "topic_count": len(topic_names),
            "edge_count": len(edges),
            "nodes": nodes,
            "edges": edges,
            "topic_names": topic_names,
            "containers": containers,
        }

        rendered = template.render(context)

        self.output_path.write_text(rendered, encoding="utf-8")
        self.get_logger().info(f"Wrote D2 graph to {self.output_path}")

        # Automatic SVG export using d2 CLI with --layout elk
        d2_path = "d2"  # Assumes d2 is in the PATH
        svg_path = self.output_path.with_suffix(".svg")
        d2_args = ["--layout", "elk"]
        try:
            subprocess.run([d2_path, *d2_args, str(self.output_path), str(svg_path)], check=True)
            self.get_logger().info(f"Wrote SVG graph to {svg_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to render SVG with d2: {e}")


def main() -> None:
    """Spin the graph export node until interrupted."""
    rclpy.init()
    node = Ros2GraphExport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
