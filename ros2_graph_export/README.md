# `ros2_graph_export`

Exports ROS node and topic graphs

## Launch Files

### [`ros2_graph_export_launch.py`](launch/ros2_graph_export_launch.py)

| Argument | Default | Description |
| --- | --- | --- |
| `export_graph` | `"~/export_graph"` | service topic for triggering graph export |
| `name` | `"ros2_graph_export"` | node name |
| `namespace` | `""` | node namespace |
| `params` | `os.path.join(get_package_share_directory("ros2_graph_export"), "config", "params.yml")` | path to parameter file |
| `log_level` | `"info"` | ROS logging level (debug, info, warn, error, fatal) |
| `use_sim_time` | `"false"` | use simulation clock |

## Parameter File

The default parameter file is [`config/params.yml`](config/params.yml).

| Parameter | Default | Description |
| --- | --- | --- |
| `output_path` | `"~/.ros/ros_graph.d2"` | Destination path for the generated D2 graph file. |
| `export_interval_seconds` | `5.0` | Periodic export interval in seconds. Set this to `0.0` or a negative value to disable automatic exports and only export on manual service calls. |
| `ignore_topics_without_publishers` | `true` | Omits topics that currently have subscribers but no publishers from the exported graph. |
| `ignore_topics_without_subscribers` | `true` | Omits topics that currently have publishers but no subscribers from the exported graph. |
| `excluded_nodes` | `[]` | List of nodes to exclude from the graph. Supports fully qualified names such as `/ns/node`, bare node names, and shell-style wildcards such as `/debug/*` or `*_monitor`. |
