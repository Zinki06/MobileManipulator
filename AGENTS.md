## 아래 3개의 폴더는 건들지 않기
- src/realsense_bringup 
- src/segmentation
- src/turtlebot3_manipulation

## 패키지 만드는 법
- ros2 pkg create --build-type ament_cmake <package_name> //cpp
- ros2 pkg create --build-type ament_python <package_name> //python

# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS 2 colcon workspace. All packages live under `src/`: `aruco_localizer`, `pick_and_place`, `realsense_bringup`, `segmentation`, two LiDAR drivers, and the multi-package `turtlebot3_manipulation` stack. C++ nodes are in each package's `src/` directory, with public headers under `include/`. Python code is under `src/segmentation/segmentation/`. Runtime resources belong in package-local `launch/`, `config/`, `map/`, `param/`, `rviz/`, `urdf/`, or `meshes/` directories. Generated `build/`, `install/`, and `log/` trees must remain untracked.

The root README marks `src/realsense_bringup`, `src/segmentation`, and `src/turtlebot3_manipulation` as do-not-modify areas. Confirm scope with a maintainer before changing them.

## Build, Test, and Development Commands

Run commands from the workspace root after sourcing your ROS 2 installation:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

For faster iteration, use `colcon build --packages-select aruco_localizer`. Launch installed nodes with commands such as `ros2 launch aruco_localizer aruco_launcher.launch.py`; source `install/setup.bash` again after rebuilding.

## Coding Style & Naming Conventions

Use four spaces in Python and two spaces in CMake/C++. Follow existing ROS 2 conventions: `snake_case` for packages, files, functions, topics, and parameters; `PascalCase` for C++ types; and descriptive `_node.cpp` filenames. Keep compiler warnings enabled (`-Wall -Wextra -Wpedantic`). Python changes must pass `ament_flake8` and `ament_pep257`; document public modules and functions. Keep dependencies synchronized between `package.xml` and `CMakeLists.txt` or `setup.py`.

## Testing Guidelines

Place Python tests in `<package>/test/` and name them `test_*.py`. CMake packages should register lint or unit tests inside `if(BUILD_TESTING)`. Run all tests with the colcon command above, or target one package using `colcon test --packages-select segmentation`. No coverage threshold is currently enforced; add focused regression tests for behavior changes.

## Commit & Pull Request Guidelines

Recent commits use short date/version summaries such as `0826 init navigation&grasping` and `v0.1.0 ...`. Keep the first line concise, but add a package scope and imperative action, for example `aruco_localizer: stabilize marker pose filtering`. Follow package-level `CONTRIBUTING.md` files; imported driver/manipulation packages require DCO sign-off (`git commit -s`). Pull requests should explain intent, list affected packages, report build/test commands, link issues, and attach RViz or hardware evidence when runtime behavior changes.
