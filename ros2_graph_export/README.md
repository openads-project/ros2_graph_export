# `ros2_graph_export`

Exports ROS node and topic graphs

## Launch Files

### [`ros2_graph_export_launch.py`](launch/ros2_graph_export_launch.py)

| Argument | Default | Description |
| --- | --- | --- |
| `export_graph` | `"~/export_graph"` | trigger graph export |
| `name` | `"ros2_graph_export"` | node name |
| `namespace` | `""` | node namespace |
| `params` | `os.path.join(get_package_share_directory("ros2_graph_export"), "config", "params.yml")` | path to parameter file |
| `log_level` | `"info"` | ROS logging level (debug, info, warn, error, fatal) |
| `use_sim_time` | `"false"` | use simulation clock |
