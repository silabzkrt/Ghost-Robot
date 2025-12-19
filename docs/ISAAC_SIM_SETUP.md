# Isaac Sim Setup Guide for Ghost Robot

This guide provides detailed instructions for setting up NVIDIA Isaac Sim to work with the Ghost Robot ROS 2 publisher.

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Installing Isaac Sim](#installing-isaac-sim)
3. [Configuring the ROS 2 Bridge](#configuring-the-ros-2-bridge)
4. [Setting Up the Robot](#setting-up-the-robot)
5. [Testing the Integration](#testing-the-integration)
6. [Troubleshooting](#troubleshooting)

## Prerequisites

- **Operating System**: Ubuntu 20.04 or 22.04 (recommended)
- **GPU**: NVIDIA GPU with RTX capabilities (RTX 2060 or better recommended)
- **NVIDIA Driver**: Latest NVIDIA drivers (version 525 or newer)
- **ROS 2**: Humble, Iron, or newer
- **Disk Space**: At least 20 GB free space

## Installing Isaac Sim

### Option 1: Using Omniverse Launcher (Recommended)

1. **Download Omniverse Launcher**
   - Visit: https://www.nvidia.com/en-us/omniverse/download/
   - Download the launcher for Linux
   - Install: `chmod +x omniverse-launcher-linux.AppImage && ./omniverse-launcher-linux.AppImage`

2. **Install Isaac Sim through Launcher**
   - Open Omniverse Launcher
   - Go to the "Exchange" tab
   - Search for "Isaac Sim"
   - Click "Install" (choose version 2023.1 or newer)
   - Wait for installation to complete

3. **Launch Isaac Sim**
   - In the Launcher, go to "Library"
   - Find Isaac Sim and click "Launch"
   - Or launch from terminal:
     ```bash
     ~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh
     ```

### Option 2: Using pip (Experimental)

```bash
pip install isaacsim
```

## Configuring the ROS 2 Bridge

### 1. Enable the ROS 2 Bridge Extension

1. **Open Isaac Sim**
2. Navigate to `Window` → `Extensions`
3. In the search bar, type "ROS2 Bridge"
4. Find "omni.isaac.ros2_bridge" and click the toggle to enable it
5. Wait for the extension to load (check the console for confirmation)

### 2. Configure ROS Domain ID

Isaac Sim and your ROS 2 environment must use the same ROS_DOMAIN_ID:

**In Isaac Sim:**
- Go to `Edit` → `Preferences` → `ROS 2`
- Set the Domain ID (default is 0)

**In your terminal:**
```bash
export ROS_DOMAIN_ID=0  # Match Isaac Sim's domain ID
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc  # Make permanent
```

### 3. Configure Network Settings (if needed)

For multi-machine setups:
```bash
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # or rmw_fastrtps_cpp
```

## Setting Up the Robot

### Method 1: Using Built-in Turtlebot3

1. **Load Turtlebot Example**
   - In Isaac Sim, go to `Isaac Examples` → `ROS2` → `Turtlebot`
   - Click "Load" to load the scene
   - The Turtlebot will appear in the viewport

2. **Configure Velocity Subscription**
   - The Turtlebot is pre-configured to listen to `/cmd_vel`
   - No additional configuration needed!

3. **Start Simulation**
   - Click the "Play" button (or press Space)
   - The robot is now ready to receive commands

### Method 2: Using Carter Robot

1. **Load Carter Robot**
   - `Isaac Examples` → `ROS2` → `Carter`
   - Click "Load"

2. **Verify cmd_vel topic**
   - Carter is pre-configured for `/cmd_vel`
   - Start simulation with Play button

### Method 3: Creating a Custom Simple Robot

#### Step-by-Step: Create a Cube Robot

1. **Create the Robot Body**
   ```
   Create → Mesh → Cube
   ```
   - Scale: Set to (0.5, 0.5, 0.3) for a reasonable robot size
   - Position: (0, 0, 0.15) to place it above the ground

2. **Add Physics**
   - Select the cube in the Stage tree
   - Right-click → Add → Physics → Rigid Body
   - Set mass to 10.0 kg

3. **Create a Ground Plane**
   ```
   Create → Mesh → Plane
   ```
   - Scale: Set to (10, 10, 1)
   - Add Physics → Collision

4. **Add Articulation**
   - Select the cube
   - Right-click → Add → Physics → Articulation Root

5. **Add Differential Drive Controller**
   - Select the cube
   - Right-click → Add → Isaac → Wheeled Robot → Differential Controller
   - Configure parameters:
     - Wheel Radius: 0.1
     - Wheel Base: 0.4

6. **Add ROS 2 Twist Subscriber**
   - With cube selected, go to Property panel
   - Click "Add" → search for "ROS2 Subscribe Twist"
   - Configure:
     - Topic Name: `/cmd_vel`
     - Queue Size: 10
   
7. **Connect Twist to Controller**
   - In the Property panel, find the "ROS2 Subscribe Twist" component
   - Link its output to the Differential Controller's input
   - This is usually done through the Action Graph (explained below)

#### Using Action Graph (Advanced)

For more control, use Omniverse Action Graph:

1. **Open Action Graph Editor**
   ```
   Window → Visual Scripting → Action Graph
   ```

2. **Create a New Graph**
   - Click "New Action Graph"
   - Name it "RobotControl"

3. **Add Nodes**
   - Add "On Playback Tick" (triggers every frame)
   - Add "ROS2 Subscribe Twist"
     - Set topic to `/cmd_vel`
   - Add "Differential Controller"
     - Link to your robot's articulation
   - Add "Apply Velocity" or "Apply Forces"

4. **Connect Nodes**
   ```
   On Playback Tick → ROS2 Subscribe Twist → Process Data → Differential Controller → Apply to Robot
   ```

5. **Configure and Test**
   - Press Play
   - Use the graph to visualize data flow

## Testing the Integration

### 1. Verify ROS 2 Topics

**In Isaac Sim console (bottom panel):**
Look for messages like:
```
[Info] ROS2 Bridge initialized
[Info] Subscribing to /cmd_vel
```

**In a terminal:**
```bash
# Check if Isaac Sim is advertising topics
ros2 topic list

# You should see Isaac Sim topics, potentially including:
# /tf
# /tf_static
# /joint_states (if using articulated robot)
```

### 2. Test Communication

**Terminal 1 - Start Ghost Publisher:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ghost_follower ghost_publisher.launch.py
```

**Terminal 2 - Monitor Messages:**
```bash
ros2 topic echo /cmd_vel
```

You should see Twist messages being published.

**Isaac Sim:**
- The robot should start moving in a figure-8 pattern
- If it doesn't move, check the troubleshooting section

### 3. Visualize with RViz2 (Optional)

```bash
ros2 run rviz2 rviz2
```

- Add a TF display to see coordinate frames
- Add a Path display to visualize the trajectory

## Troubleshooting

### Robot doesn't move

**Check 1: ROS 2 Bridge Active**
```
Isaac Sim console should show:
[Info] ROS2 bridge node running
```

**Check 2: Topic Connection**
```bash
ros2 topic info /cmd_vel
```
Should show:
- Publisher: ghost_publisher
- Subscriber: Isaac Sim node

**Check 3: Domain ID Match**
```bash
echo $ROS_DOMAIN_ID  # In terminal
# Compare with Isaac Sim's domain ID setting
```

**Check 4: Physics Running**
- Ensure the Play button is pressed in Isaac Sim
- Check that physics is enabled (should see objects falling/settling)

**Check 5: Robot Configuration**
- Verify the robot has an articulation root
- Check that velocity commands are connected to the controller
- Ensure the robot has physics enabled

### Messages not showing in Isaac Sim

**Check network configuration:**
```bash
# Terminal
ros2 doctor --report

# Should show healthy ROS 2 environment
```

**Check firewall settings:**
```bash
# Ubuntu
sudo ufw status
# Allow ROS 2 DDS ports if firewall is enabled
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp
```

### Performance Issues

**Reduce simulation load:**
- Lower rendering quality: `Edit` → `Preferences` → `Rendering`
- Reduce physics rate: `Edit` → `Preferences` → `Physics`
- Close unnecessary applications

**Optimize ROS 2 settings:**
```bash
# Use Cyclone DDS for better performance
sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### GPU Issues

**Check NVIDIA driver:**
```bash
nvidia-smi
# Should show driver version and GPU info
```

**Update driver if needed:**
```bash
sudo ubuntu-drivers autoinstall
sudo reboot
```

## Advanced Configuration

### Recording Simulation Data

Use rosbag to record your simulation:
```bash
ros2 bag record /cmd_vel /tf /joint_states
```

### Multiple Robots

To control multiple robots:
1. Create multiple robot instances in Isaac Sim
2. Configure each with a unique namespace
3. Modify the publisher to use specific namespaces:
   ```bash
   ros2 run ghost_follower ghost_publisher --ros-args -r cmd_vel:=/robot1/cmd_vel
   ```

### Custom Environments

Import USD files or create custom scenes:
1. `File` → `Open` or `File` → `Import`
2. Isaac Sim supports USD, URDF, and other formats
3. Add physics to imported objects as needed

## Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [ROS 2 Bridge Tutorial](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/index.html)
- [Isaac Sim Forum](https://forums.developer.nvidia.com/c/omniverse/simulation/69)
- [ROS 2 Documentation](https://docs.ros.org/)

## Next Steps

1. Experiment with different parameter values in the publisher
2. Try creating more complex trajectories
3. Add sensors (cameras, lidars) to your robot
4. Implement obstacle avoidance
5. Create multi-robot simulations

---

**Need Help?**
- Check the [main README](../README.md) for package documentation
- Open an issue on GitHub
- Consult the Isaac Sim community forums
