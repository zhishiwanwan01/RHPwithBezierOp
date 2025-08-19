# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This repository is a learning workspace containing three major UAV trajectory planning and control frameworks:

1. **Btraj** - Online UAV trajectory planner using Bezier curves with Fast Marching*/A* path finding
2. **Fast-Planner** - Comprehensive quadrotor planning system with kinodynamic and topological path planning  
3. **DecompROS** - ROS wrapper for convex decomposition of free space in cluttered environments
4. **myProj** - Empty directory for custom project work
5. **notes/** - Learning notes and Git operation guides for code analysis

## Build System & Commands

This is a ROS-based project using `catkin_make` for building:

```bash
# Build all packages
catkin_make

# Build with specific configuration  
catkin_make -DCMAKE_BUILD_TYPE=Release

# Using catkin tools (recommended)
catkin build
```

### Prerequisites Installation

Before building, install required dependencies:

```bash
# For Btraj (requires plan_utils dependency)
sudo apt-get install libarmadillo-dev
cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/plan_utils.git

# For Fast-Planner - install NLopt v2.7.1
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt && mkdir build && cd build
cmake .. && make && sudo make install

# For DecompROS - install catkin_simple and QT4+
sudo apt-get install qt4-dev-tools libqt4-dev libqt4-core libqt4-gui
# Initialize DecompUtil submodule
git submodule update --init
```

### Launch Commands

**Btraj simulation:**
```bash
roslaunch bezier_planer simulation.launch
```

**Fast-Planner simulations:**
```bash
# Visualization
roslaunch plan_manage rviz.launch

# Kinodynamic planning
roslaunch plan_manage kino_replan.launch

# Topological planning  
roslaunch plan_manage topo_replan.launch
```

**DecompROS testing:**
```bash
roslaunch decomp_test_node rviz.launch
roslaunch decomp_test_node test_path_decomp_2d.launch
roslaunch decomp_test_node test_path_decomp_3d.launch
roslaunch decomp_test_node test_seed_decomp.launch
```

## Architecture Overview

### Btraj Structure
- **Front-end:** Fast Marching* (FM*) or A* pathfinding on velocity fields/grid maps
- **Back-end:** Bezier curve trajectory optimization with corridor constraints
- **Key files:** `trajectory_generator.h/cpp`, `bezier_base.h/cpp`, `a_star.h/cpp`
- **Dependencies:** Mosek 8.1 (for QP solving), sdf_tools, fast_methods

### Fast-Planner Structure
- **plan_env:** Online mapping from depth images, ESDF generation
- **path_searching:** Kinodynamic A*, topological path searching
- **bspline:** B-spline trajectory representation
- **bspline_opt:** Gradient-based B-spline optimization
- **plan_manage:** High-level planning FSM and coordination
- **uav_simulator:** Lightweight quadrotor simulation environment

### DecompROS Structure
- **DecompUtil:** Core convex decomposition library (git submodule)
- **decomp_ros_msgs:** ROS message definitions
- **decomp_ros_utils:** ROS interface utilities
- **decomp_test_node:** Example and testing nodes

## Key Configuration Points

### Btraj Parameters
- Path planning method: Set `is_use_fm` parameter (true for FM*, false for A*)
- Trajectory constraints: `_MAX_Vel`, `_MAX_Acc` in launch files
- Bezier curve order: `_traj_order` parameter

### Fast-Planner Parameters  
- Planner type: Set `planner` parameter (1 for kinodynamic, 2 for topological)
- Depth image resolution: Default 640x480 with downsampling (`skip_pixel = 2`)
- GPU/CPU rendering: `ENABLE_CUDA` flag in local_sensing CMakeLists.txt

### Integration Notes
- All projects use ROS Kinetic/Melodic/Noetic compatibility
- Coordinate system: Standard ROS conventions (ENU)
- Visualization: All projects provide RViz configurations
- Goal setting: Use "2D Nav Goal" tool in RViz, or "3D Nav Goal" for Btraj

## Special Requirements

### Mosek License (Btraj only)
Academic license required from mosek.com for Mosek 8.1. Place license file in `~/mosek/` directory. Btraj includes pre-compiled Mosek 8.1 libraries in `Btraj/third_party/mosek/lib/mosek8_1/`.

### GPU Depth Rendering (Fast-Planner)
For realistic depth simulation, install CUDA Toolkit and set appropriate GPU arch flags in CMakeLists.txt.

### Git Submodules (DecompROS)
Initialize DecompUtil submodule: `git submodule update --init`

### Docker Support
Basic Docker configuration available:
- `Dockerfile` - Based on `osrf/ros:melodic-desktop-full` (minimal configuration)
- `docker-compose.yml` - For containerized development (minimal configuration)
- `manage.sh` - Management script for development workflow

## Development Workflow

1. Make changes to specific package source files
2. Run `catkin_make` from workspace root (or `catkin build` with catkin tools)
3. Source setup: `source devel/setup.bash`  
4. Test with appropriate launch files
5. Use RViz for visualization and goal setting

### Debugging and Development Tips
- **Btraj**: Use `is_use_fm` parameter to switch between FM* and A* algorithms
- **Fast-Planner**: Toggle GPU depth rendering with `ENABLE_CUDA` flag in local_sensing CMakeLists.txt
- **DecompROS**: Test different decomposition methods with provided launch files
- **Goal Setting**: Use "2D Nav Goal" for Fast-Planner, "3D Nav Goal" for Btraj
- **Visualization**: All projects include RViz configurations for debugging

### Common Issues
- **Mosek License**: Ensure Mosek 8.1 academic license is in `~/mosek/` directory
- **NLopt Version**: Fast-Planner requires specifically v2.7.1
- **Submodules**: Initialize DecompUtil with `git submodule update --init`
- **Dependencies**: Install all prerequisite packages before building

## Git Workflow & Learning Notes

This repository includes structured learning notes in the `notes/` directory:
- `btraj_code_reading_guide.md` - Git workflow guide for code analysis with commit strategies
- Branch structure: Uses `readcode` branch for learning commits, `master` for main development

No specific linting or testing commands are configured - validation primarily through simulation launches.