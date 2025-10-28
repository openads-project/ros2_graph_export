# ros2_graph_export

Exports ROS node and topic graphs

```bash
# render D2 diagram
docker run --rm -it -u "$(id -u):$(id -g)" -v "$PWD:/home/debian/src" -p 8080:8080 terrastruct/d2:v0.7.0 --layout elk --watch ros_graph.d2
```
