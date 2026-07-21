# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import fnmatch
import re
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple, Union

import rclpy
import rclpy.exceptions
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from jinja2 import Environment, FileSystemLoader, TemplateNotFound
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.topic_endpoint_info import TopicEndpointInfo
from std_srvs.srv import Trigger

IGNORED_TOPICS = {"/parameter_events", "/rosout"}
TRANSFORM_PREFIX = "transform_listener_impl_"
GRAPH_DIRECTIONS = ("right", "down", "left", "up")
DEFAULT_GRAPH_DIRECTION = "right"


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

        self.auto_reconfigurable_params: list[str] = []
        output_path = self.declare_and_load_parameter(
            name="output_path",
            param_type=rclpy.Parameter.Type.STRING,
            description="graph export path",
            default=str(Path.home() / ".ros" / "ros_graph.d2"),
        )
        self.output_path: Path = Path(output_path).expanduser()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        self.export_interval = self.declare_and_load_parameter(
            name="export_interval_seconds",
            param_type=rclpy.Parameter.Type.DOUBLE,
            description="graph export interval in seconds",
            default=5.0,
        )
        self.ignore_topics_without_publishers = self.declare_and_load_parameter(
            name="ignore_topics_without_publishers",
            param_type=rclpy.Parameter.Type.BOOL,
            description="ignore topics without publishers",
            default=True,
        )
        self.ignore_topics_without_subscribers = self.declare_and_load_parameter(
            name="ignore_topics_without_subscribers",
            param_type=rclpy.Parameter.Type.BOOL,
            description="ignore topics without subscribers",
            default=True,
        )
        self.graph_direction = self.declare_and_load_parameter(
            name="graph_direction",
            param_type=rclpy.Parameter.Type.STRING,
            description="layout direction of the exported graph: 'right' arranges nodes left-to-right, "
            "'down' arranges them top-down for a more compact fit on A4 pages",
            default="right",
            additional_constraints="one of: right, down, left, up",
        )
        excluded_nodes = self.declare_and_load_parameter(
            name="excluded_nodes",
            param_type=rclpy.Parameter.Type.STRING_ARRAY,
            description="Nodes to exclude from the graph, as fully qualified names (/ns/node) or bare node names. "
            "Shell-style wildcards are supported, e.g. /debug/* or *_monitor.",
            default=[],
        )
        self.excluded_nodes: List[str] = self._coerce_node_patterns(excluded_nodes)
        if self.excluded_nodes:
            self.get_logger().info(f"Excluding nodes matching: {', '.join(self.excluded_nodes)}")

        self._template_env: Environment | None = None
        self._template_name: str | None = None
        self._template_directory: Path | None = None
        self.d2_template_path: Path = self._resolve_template_path()
        self.setup()

    def declare_and_load_parameter(
        self,
        name: str,
        param_type: rclpy.Parameter.Type,
        description: str,
        default: Optional[Any] = None,
        add_to_auto_reconfigurable_params: bool = True,
        is_required: bool = False,
        read_only: bool = False,
        from_value: Optional[Union[int, float]] = None,
        to_value: Optional[Union[int, float]] = None,
        step_value: Optional[Union[int, float]] = None,
        additional_constraints: str = "",
    ) -> Any:
        """Declares and loads a ROS parameter

        Args:
            name (str): name
            param_type (rclpy.Parameter.Type): parameter type
            description (str): description
            default (Optional[Any], optional): default value
            add_to_auto_reconfigurable_params (bool, optional): enable reconfiguration of parameter
            is_required (bool, optional): whether failure to load parameter will stop node
            read_only (bool, optional): set parameter to read-only
            from_value (Optional[Union[int, float]], optional): parameter range minimum
            to_value (Optional[Union[int, float]], optional): parameter range maximum
            step_value (Optional[Union[int, float]], optional): parameter range step
            additional_constraints (str, optional): additional constraints description

        Returns:
            Any: parameter value
        """

        # declare parameter
        param_desc = ParameterDescriptor()
        param_desc.description = description
        param_desc.additional_constraints = additional_constraints
        param_desc.read_only = read_only
        if from_value is not None and to_value is not None:
            if param_type == rclpy.Parameter.Type.INTEGER:
                range = IntegerRange(from_value=from_value, to_value=to_value)
                if step_value is not None:
                    range.step = step_value
                param_desc.integer_range = [range]
            elif param_type == rclpy.Parameter.Type.DOUBLE:
                range = FloatingPointRange(from_value=from_value, to_value=to_value)
                if step_value is not None:
                    range.step = step_value
                param_desc.floating_point_range = [range]
            else:
                self.get_logger().warn(f"Parameter type of parameter '{name}' does not support specifying a range")
        self.declare_parameter(name, param_type, param_desc)

        # load parameter
        try:
            param = self.get_parameter(name).value
            self.get_logger().info(f"Loaded parameter '{name}': {param}")
        except rclpy.exceptions.ParameterUninitializedException:
            if is_required:
                self.get_logger().fatal(f"Missing required parameter '{name}', exiting")
                raise SystemExit(1)
            else:
                self.get_logger().warn(f"Missing parameter '{name}', using default value: {default}")
                param = default
                self.set_parameters([rclpy.Parameter(name=name, value=param)])

        # add parameter to auto-reconfigurable parameters
        if add_to_auto_reconfigurable_params:
            self.auto_reconfigurable_params.append(name)

        return param

    def parameters_callback(self, parameters: list[rclpy.Parameter]) -> SetParametersResult:
        """Handles reconfiguration when a parameter value is changed

        Args:
            parameters (list[rclpy.Parameter]): parameters

        Returns:
            SetParametersResult: parameter change result
        """

        for param in parameters:
            if param.name in self.auto_reconfigurable_params:
                setattr(self, param.name, param.value)
                self.get_logger().info(f"Reconfigured parameter '{param.name}' to: {param.value}")

        result = SetParametersResult()
        result.successful = True

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

    def setup(self):
        """Sets up subscribers, publishers, etc. to configure the node"""

        # callback for dynamic parameter configuration
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Provide a service for manual export requests.
        self.create_service(Trigger, "~/export_graph", self._handle_export_request)

        self._export_timer = None
        if self.export_interval > 0.0:
            self._export_timer = self.create_timer(self.export_interval, self._perform_export)
            self.get_logger().info(f"Scheduled periodic graph export every {self.export_interval:.1f} s")

        self._perform_export()

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

        if self._is_node_excluded(name, namespace):
            return False

        return True

    def _coerce_node_patterns(self, value: Sequence[object] | None) -> List[str]:
        if not value:
            return []
        return [str(pattern).strip() for pattern in value if str(pattern).strip()]

    def _is_node_excluded(self, name: str, namespace: str) -> bool:
        if not self.excluded_nodes:
            return False

        namespace = namespace or "/"
        fully_qualified_name = f"{namespace.rstrip('/')}/{name}"
        candidates = (fully_qualified_name, name)

        return any(fnmatch.fnmatchcase(candidate, pattern) for pattern in self.excluded_nodes for candidate in candidates)

    def _should_include_topic(self, topic_name: str) -> bool:
        if topic_name in IGNORED_TOPICS:
            return False
        if topic_name.endswith("/transition_event"):
            return False
        if topic_name.startswith("_"):
            return False
        return True

    def _resolve_graph_direction(self) -> str:
        direction = str(self.graph_direction or "").strip().lower()
        if direction not in GRAPH_DIRECTIONS:
            self.get_logger().warn(
                f"Unsupported graph_direction '{self.graph_direction}', "
                f"expected one of {', '.join(GRAPH_DIRECTIONS)}; using '{DEFAULT_GRAPH_DIRECTION}'"
            )
            return DEFAULT_GRAPH_DIRECTION
        return direction

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
            "direction": self._resolve_graph_direction(),
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
