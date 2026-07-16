# `ros2_graph_export`

Exports ROS node and topic graphs

## Launch Files

### [`ros2_graph_export_launch.py`](launch/ros2_graph_export_launch.py)

| Argument | Default | Description |
| --- | --- | --- |
| `export_graph` | `"~/export_graph"` | trigger graph export |
| `name` | `"ros2_graph_export"` | node name |
| `namespace` | `""` | node namespace |
| `log_level` | `"info"` | ROS logging level (debug, info, warn, error, fatal) |
| `use_sim_time` | `"false"` | use simulation clock |
