# Quick Start Guide

Get up and running with Ghost Robot in 5 minutes!

## Prerequisites

- ROS 2 installed (Humble, Iron, or newer)
- NVIDIA Isaac Sim installed with ROS 2 Bridge enabled
- C++ compiler (gcc/g++)

## Step 1: Build the Package (2 minutes)

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/silabzkrt/Ghost-Robot.git

# Build
cd ~/ros2_ws
colcon build --packages-select ghost_follower

# Source
source install/setup.bash
```

## Step 2: Setup Isaac Sim (2 minutes)

1. Launch Isaac Sim
2. Go to `Isaac Examples` → `ROS2` → `Turtlebot`
3. Click "Load"
4. Press the "Play" button (or Space bar)

That's it! The Turtlebot is ready to receive commands.

## Step 3: Run the Publisher (1 minute)

In a new terminal:

```bash
# Source your workspace
source ~/ros2_ws/install/setup.bash

# Launch the ghost publisher
ros2 launch ghost_follower ghost_publisher.launch.py
```

## What You Should See

- **In Terminal**: Log messages showing the node is running
- **In Isaac Sim**: The Turtlebot should start moving in a figure-8 pattern!

## Verify It's Working

Open another terminal:

```bash
# Check the node is running
ros2 node list
# Output: /ghost_publisher

# Check messages are being published
ros2 topic echo /cmd_vel
# You should see Twist messages with linear and angular velocities
```

## Troubleshooting

**Robot doesn't move?**
1. Check that Isaac Sim's Play button is pressed
2. Verify ROS_DOMAIN_ID matches:
   ```bash
   echo $ROS_DOMAIN_ID
   # Should be 0 by default
   ```

**No messages in terminal?**
- Ensure you sourced the workspace: `source ~/ros2_ws/install/setup.bash`

**Still having issues?**
- See [Isaac Sim Setup Guide](docs/ISAAC_SIM_SETUP.md) for detailed troubleshooting
- Check [README.md](README.md) for full documentation

## Next Steps

- Adjust parameters: Try different speeds and patterns
  ```bash
  ros2 launch ghost_follower ghost_publisher.launch.py linear_scale:=1.0 frequency:=0.3
  ```
- Explore the code in `ghost_follower/src/ghost_publisher.cpp`
- Read the [full documentation](README.md)

## Need Help?

- [Full README](README.md)
- [Isaac Sim Setup Guide](docs/ISAAC_SIM_SETUP.md)
- [Open an issue](https://github.com/silabzkrt/Ghost-Robot/issues)

Enjoy your Ghost Robot! 🤖👻
