# Ghost-Robot

A ROS 2 simulation package that creates a "ghost" follower where a visual robot in NVIDIA Isaac Sim mimics the movement of a ghost target published by ROS 2.

## Overview

This project demonstrates:
- A ROS 2 C++ publisher node that broadcasts `geometry_msgs/Twist` messages in a figure-8 pattern
- Integration with NVIDIA Isaac Sim using the ROS 2 Bridge extension
- Parametric trajectory generation for robotic movement simulation

## Features

- **Figure-8 Movement Pattern**: Uses parametric equations (lemniscate of Gerono) to generate smooth figure-8 trajectories
- **Configurable Parameters**: Adjustable linear/angular scales, frequency, and publish rate
- **ROS 2 Native**: Built with `rclcpp` for efficient C++ ROS 2 integration
- **Isaac Sim Compatible**: Designed to work seamlessly with NVIDIA Isaac Sim's ROS 2 Bridge

## Prerequisites

### ROS 2 Environment
- ROS 2 (Humble, Iron, or newer recommended)
- C++ compiler with C++14 support or newer
- `ament_cmake` build system
- Required ROS 2 packages:
  - `rclcpp`
  - `geometry_msgs`
  - `std_msgs`

### NVIDIA Isaac Sim
- NVIDIA Isaac Sim (2022.2 or newer)
- ROS 2 Bridge extension enabled in Isaac Sim
- A robot model (Turtlebot, Carter, or simple cube/sphere)

## Installation

### 1. Setup ROS 2 Workspace

```bash
# Create a workspace if you don't have one
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/silabzkrt/Ghost-Robot.git
cd ~/ros2_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select ghost_follower

# Source the workspace
source install/setup.bash
```

### 2. Setup Isaac Sim

1. **Launch Isaac Sim**
   ```bash
   # Navigate to your Isaac Sim installation
   cd ~/.local/share/ov/pkg/isaac_sim-*
   ./isaac-sim.sh
   ```

2. **Enable ROS 2 Bridge**
   - Go to `Window` → `Extensions`
   - Search for "ROS2 Bridge"
   - Enable the extension if not already enabled

3. **Import a Robot**
   - Option A: Use built-in Turtlebot
     - Go to `Isaac Examples` → `ROS2` → `Navigation`
     - Load the Turtlebot example
   
   - Option B: Create a simple cube/sphere
     - `Create` → `Mesh` → `Cube` or `Sphere`
     - Add physics and articulation components
     - Configure as a differential drive robot

4. **Configure ROS 2 Bridge**
   - In the Stage tree, select your robot
   - Add a `ROS2 Subscribe Twist` component
   - Set the topic name to `/cmd_vel`
   - Configure the articulation controller to respond to twist messages

## Usage

### Running the Ghost Publisher

#### Option 1: Using Launch File (Recommended)

```bash
# Source your workspace
source ~/ros2_ws/install/setup.bash

# Launch with default parameters
ros2 launch ghost_follower ghost_publisher.launch.py

# Launch with custom parameters
ros2 launch ghost_follower ghost_publisher.launch.py linear_scale:=0.8 frequency:=0.3
```

#### Option 2: Running the Node Directly

```bash
# Run with default parameters
ros2 run ghost_follower ghost_publisher

# Run with custom parameters
ros2 run ghost_follower ghost_publisher --ros-args \
  -p linear_scale:=0.5 \
  -p angular_scale:=1.0 \
  -p frequency:=0.5 \
  -p publish_rate_hz:=50
```

#### Option 3: Using Parameter File

```bash
ros2 run ghost_follower ghost_publisher \
  --ros-args --params-file src/Ghost-Robot/ghost_follower/config/ghost_publisher_params.yaml
```

### Verifying the Setup

1. **Check if the node is running:**
   ```bash
   ros2 node list
   # Should show: /ghost_publisher
   ```

2. **Check published topics:**
   ```bash
   ros2 topic list
   # Should show: /cmd_vel
   ```

3. **View published messages:**
   ```bash
   ros2 topic echo /cmd_vel
   ```

4. **Visualize with PlotJuggler (Optional):**
   ```bash
   ros2 run plotjuggler plotjuggler
   ```

### Expected Behavior

Once both the ROS 2 publisher and Isaac Sim are running:
1. The `ghost_publisher` node will start publishing velocity commands in a figure-8 pattern
2. Isaac Sim's robot should begin moving, tracing out a figure-8 trajectory
3. The movement should be smooth and continuous

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `linear_scale` | double | 0.5 | Scale factor for linear velocity (affects speed) |
| `angular_scale` | double | 1.0 | Scale factor for angular velocity (affects turn sharpness) |
| `frequency` | double | 0.5 | Frequency of figure-8 pattern in Hz (affects size/speed) |
| `publish_rate_hz` | int | 50 | Rate at which commands are published (Hz) |

### Tuning Tips

- **Slower, larger figure-8**: Decrease `frequency` (e.g., 0.2)
- **Faster movement**: Increase `linear_scale` (e.g., 1.0)
- **Sharper turns**: Increase `angular_scale` (e.g., 1.5)
- **Smoother control**: Increase `publish_rate_hz` (e.g., 100)

## Package Structure

```
ghost_follower/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata and dependencies
├── config/
│   └── ghost_publisher_params.yaml  # Default parameters
├── launch/
│   └── ghost_publisher.launch.py    # Launch file
└── src/
    └── ghost_publisher.cpp     # Main publisher node
```

## How It Works

### The Figure-8 Pattern

The node generates a figure-8 trajectory using the parametric equations of a lemniscate of Gerono:

```
x(t) = sin(2πft)
y(t) = sin(4πft)
```

Where:
- `f` is the frequency parameter
- `t` is time

The velocities are computed by taking derivatives:
```
dx/dt = 2πf · cos(2πft)
dy/dt = 4πf · cos(4πft)
```

### Velocity Computation

1. **Linear Velocity**: Computed as the magnitude of the velocity vector
2. **Angular Velocity**: Computed using the curvature formula to ensure smooth turning

### ROS 2 Integration

The node:
1. Publishes to the `/cmd_vel` topic (standard for robot velocity commands)
2. Uses `geometry_msgs/Twist` message type (standard for velocity commands)
3. Operates at a configurable rate (default: 50 Hz)
4. Supports runtime parameter configuration

## Troubleshooting

### Robot doesn't move in Isaac Sim

1. **Check topic connection:**
   ```bash
   ros2 topic info /cmd_vel
   ```
   Should show publishers and subscribers

2. **Verify ROS 2 Bridge is running in Isaac Sim:**
   - Check the console for ROS 2 Bridge messages
   - Ensure the bridge is connected to the correct ROS domain

3. **Check ROS_DOMAIN_ID:**
   ```bash
   echo $ROS_DOMAIN_ID
   ```
   Ensure it matches between your ROS 2 environment and Isaac Sim

4. **Verify robot configuration:**
   - Ensure the robot has the correct articulation controller
   - Check that the twist subscriber is properly configured

### Node starts but no messages

1. **Check node status:**
   ```bash
   ros2 node info /ghost_publisher
   ```

2. **Verify parameters:**
   ```bash
   ros2 param list /ghost_publisher
   ros2 param get /ghost_publisher linear_scale
   ```

### Performance issues

- Reduce `publish_rate_hz` if the system is overloaded
- Adjust `frequency` for slower movements
- Check CPU usage with `top` or `htop`

## Development

### Building from Source

```bash
cd ~/ros2_ws
colcon build --packages-select ghost_follower --symlink-install
```

### Running Tests

```bash
colcon test --packages-select ghost_follower
colcon test-result --all
```

### Code Style

This package follows the ROS 2 C++ style guide:
- Google C++ Style Guide
- ROS 2 naming conventions
- Modern C++ practices (C++14/17)

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## References

- [ROS 2 Documentation](https://docs.ros.org/)
- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)
- [Isaac Sim ROS 2 Bridge](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/index.html)
- [geometry_msgs/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)

## Authors

- Ghost Robot Team

## Acknowledgments

- NVIDIA for Isaac Sim
- ROS 2 community
- Open Robotics