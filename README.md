# ros2_graph_export

<p align="center">
  <a href="https://github.com/openads-project"><img src="https://img.shields.io/badge/OpenADS-f5ff01"/></a>
  <a href="https://www.ros.org"><img src="https://img.shields.io/badge/ROS 2-jazzy-22314e"/></a>
  <a href="https://github.com/openads-project/ros2_graph_export/releases/latest"><img src="https://img.shields.io/github/v/release/openads-project/ros2_graph_export"/></a>
  <a href="https://github.com/openads-project/ros2_graph_export/blob/main/LICENSE"><img src="https://img.shields.io/github/license/openads-project/ros2_graph_export"/></a>
  <br>
  <a href="https://github.com/openads-project/ros2_graph_export/actions/workflows/docker-ros.yml"><img src="https://github.com/openads-project/ros2_graph_export/actions/workflows/docker-ros.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/ros2_graph_export/actions/workflows/compose-oci.yml"><img src="https://github.com/openads-project/ros2_graph_export/actions/workflows/compose-oci.yml/badge.svg"/></a>
  <a href="https://openads-project.github.io/ros2_graph_export"><img src="https://github.com/openads-project/ros2_graph_export/actions/workflows/docs.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/ros2_graph_export/actions/workflows/consistency.yml"><img src="https://github.com/openads-project/ros2_graph_export/actions/workflows/consistency.yml/badge.svg"/></a>
</p>

**Exports ROS 2 node and topic graphs**

<p align="center">
  <strong>🚀 <a href="#-quick-start">Quick Start</a></strong> • <strong>💻 <a href="#-development">Development</a></strong> • <strong>📝 <a href="#-documentation">Documentation</a></strong>
</p>

> [!IMPORTANT]
> This repository is part of [***OpenADS***](https://github.com/openads-project), the *Open Automated Driving Systems* project. *OpenADS* and its modules have been initiated and are currently being maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/de/).

## 🚀 Quick Start

1. Start a container of the pre-built runtime image.
    ```bash
    docker run --rm -it ghcr.io/openads-project/ros2_graph_export:latest bash
    ```
1. Inside the container, launch the pre-built nodes.
    ```bash
    ros2 launch ros2_graph_export ros2_graph_export_launch.py
    ```

The generated D2 diagram can be rendered with:

```bash
docker run --rm -it -u "$(id -u):$(id -g)" -v "$PWD:/home/debian/src" -p 8080:8080 terrastruct/d2:v0.7.0 --layout elk --watch ros_graph.d2
```

## 💻 Development

### Set up Development Environment

1. Clone the repository.
    ```bash
    git clone https://github.com/openads-project/ros2_graph_export.git
    ```
1. Initialize the [`.openads-dev-environment`](https://github.com/openads-project/openads-dev-environment) submodule containing development environment configuration.
    ```bash
    cd ros2_graph_export
    git submodule update --init --recursive
    ```
1. Open the repository in [Visual Studio Code](https://code.visualstudio.com).
    ```bash
    code .
    ```
1. Install the recommended VS Code extensions.
    > *Ctrl+Shift+P / Extensions: Show Recommended Extensions / Install Workspace Recommended Extensions (Cloud Download Icon)*
1. Reopen the repository in a [Dev Container](https://code.visualstudio.com/docs/devcontainers/containers).
    > *Ctrl+Shift+P / Dev Containers: Rebuild and Reopen in Container*

### Build

> *Ctrl+Shift+B*

```bash
colcon build
```

### Run Tests

> *Ctrl+Shift+P / Tasks: Run Test Task*

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
colcon test
colcon test-result --verbose
```


## 📝 Documentation

Package and node interfaces are documented in the respective package READMEs listed below. Implementation details are found in the [Source Code Documentation](https://openads-project.github.io/ros2_graph_export).

| Package | Description |
| --- | --- |
| [ros2_graph_export](ros2_graph_export/README.md) | Exports ROS node and topic graphs |

## ⚖️ Licensing

The source code in this repository is licensed under Apache-2.0, see [LICENSE](LICENSE). Container images provided by this repository may contain third-party software shipped with their own license terms.

## 🙏 Acknowledgements

Development and maintenance of this repository are supported by the following projects. We acknowledge the funding of the respective institutions.

| Project | Funding Institution | Grant Number |
| --- | --- | --- |
| [6GEM+](https://6gem.de/) | 🇩🇪 Federal Ministry for Research, Technology and Space (BMFTR) | 16KIS2409K |

<p>
  <img src="https://www.drought.uni-freiburg.de/stressres/images/bmftr-logo/image" height=70>
</p>
