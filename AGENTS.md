# Repository Guidelines

## Project Structure & Modules
- `Btraj/`: ROS (catkin) package for Bezier-based trajectory planning; launch files in `Btraj/launch/`.
- `Fast-Planner/fast_planner/*`: Multiple ROS packages for env mapping, kinodynamic/topo planning, B‑spline traj, plus `uav_simulator/`.
- `DecompROS/`: ROS wrappers, msgs, utils, and demos for convex decomposition.
- `notes/`: Learning notes and workflows. `Dockerfile`: ROS Melodic base. `.gitmodules`: three submodules.

## Setup, Build, and Run
- Initialize submodules: `git submodule update --init --recursive`.
- Use a catkin workspace: place this repo under `~/catkin_ws/src/`, then:
  - `rosdep install --from-paths src --ignore-src -r -y`
  - `catkin_make` (or `catkin build`)
  - `source devel/setup.bash`
- Example run: `roslaunch plan_manage rviz.launch` (Fast‑Planner) or `roslaunch bezier_planer simulation.launch` (Btraj).

## Coding Style & Naming
- C++11 (per CMake): use 4‑space indentation; 100‑char soft limit.
- Files/ROS packages: lower_snake_case. Classes: `CamelCase`. Functions/vars: lower_snake_case.
- Topics/params/frames: lower_snake_case; frames like `world`, `uav`.
- Prefer `clang-format` (Google style). Keep includes ordered: std, third‑party, ROS, local.

## Testing Guidelines
- Frameworks: `gtest` via catkin and `rostest` for launch‑level checks.
- Conventions: place unit tests under `test/` with names like `test_*.cpp`; launch tests `*.test`.
- Run: `catkin run_tests` or `rostest <pkg> <file.test>`. No coverage gate enforced; add assertions for planner outputs and RViz topics.

## Commit & PR Guidelines
- Messages: imperative mood; short scope prefix optional (e.g., `btraj:`). English preferred; concise Chinese notes acceptable.
  - Example: `fix(plan_manage): handle empty ESDF on startup`
- PRs: clear description, reproduction/run commands, RViz screenshots for behavior changes, linked issues, and updated docs/launch params when applicable.

## Security & Configuration
- Submodules use SSH URLs; ensure an SSH key or switch to HTTPS if needed.
- Btraj may require a Mosek license at `~/mosek/`. Fast‑Planner depends on NLopt (v2.7.1 recommended).
- Use Docker (`docker build -t learn-pnc:melodic .`) to pin ROS Melodic.
