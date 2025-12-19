# Ghost Robot Usage Examples

This directory contains example configurations and usage scenarios for the Ghost Robot package.

## Basic Usage Examples

### 1. Default Configuration

Run with default parameters (linear_scale=0.5, frequency=0.5):

```bash
ros2 launch ghost_follower ghost_publisher.launch.py
```

### 2. Slower, Larger Figure-8

Create a larger, slower figure-8 pattern:

```bash
ros2 launch ghost_follower ghost_publisher.launch.py \
  linear_scale:=0.3 \
  frequency:=0.3
```

### 3. Faster, Tighter Figure-8

Create a faster, more compact figure-8:

```bash
ros2 launch ghost_follower ghost_publisher.launch.py \
  linear_scale:=1.0 \
  frequency:=0.8 \
  angular_scale:=1.5
```

### 4. High-Speed Demo

For showcasing with a fast robot:

```bash
ros2 launch ghost_follower ghost_publisher.launch.py \
  linear_scale:=2.0 \
  angular_scale:=2.0 \
  frequency:=0.6 \
  publish_rate_hz:=100
```

### 5. Precision Mode

For accurate, smooth movement with a precision robot:

```bash
ros2 launch ghost_follower ghost_publisher.launch.py \
  linear_scale:=0.2 \
  angular_scale:=0.5 \
  frequency:=0.2 \
  publish_rate_hz:=100
```

## Using Parameter Files

### Custom Configuration File

Create a custom YAML file (`my_config.yaml`):

```yaml
ghost_publisher:
  ros__parameters:
    linear_scale: 0.7
    angular_scale: 1.2
    frequency: 0.4
    publish_rate_hz: 60
```

Launch with custom configuration:

```bash
ros2 run ghost_follower ghost_publisher \
  --ros-args --params-file my_config.yaml
```

## Multi-Robot Scenarios

### Two Robots with Different Patterns

**Terminal 1 - Robot 1 (slow):**
```bash
ros2 run ghost_follower ghost_publisher \
  --ros-args \
  -r cmd_vel:=/robot1/cmd_vel \
  -p linear_scale:=0.3 \
  -p frequency:=0.3
```

**Terminal 2 - Robot 2 (fast):**
```bash
ros2 run ghost_follower ghost_publisher \
  --ros-args \
  -r cmd_vel:=/robot2/cmd_vel \
  -p linear_scale:=0.8 \
  -p frequency:=0.7
```

## Topic Remapping

### Publishing to a Custom Topic

```bash
ros2 run ghost_follower ghost_publisher \
  --ros-args -r cmd_vel:=/my_robot/velocity_commands
```

### Publishing to Namespaced Topic

```bash
ros2 run ghost_follower ghost_publisher \
  --ros-args -r cmd_vel:=/turtlebot3/cmd_vel
```

## Monitoring and Visualization

### View Published Messages

```bash
ros2 topic echo /cmd_vel
```

### Monitor Publishing Rate

```bash
ros2 topic hz /cmd_vel
```

### Record Data for Analysis

```bash
ros2 bag record /cmd_vel
```

### Visualize in PlotJuggler

```bash
ros2 run plotjuggler plotjuggler
# Then drag /cmd_vel/linear/x and /cmd_vel/angular/z to plot
```

### Visualize in RViz2

```bash
ros2 run rviz2 rviz2
# Add TF display and configure to visualize robot movement
```

## Integration with Isaac Sim

### Turtlebot Example

1. **Start Isaac Sim**
   ```bash
   ~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh
   ```

2. **Load Turtlebot**
   - Isaac Examples → ROS2 → Turtlebot → Load
   - Press Play

3. **Start Ghost Publisher**
   ```bash
   ros2 launch ghost_follower ghost_publisher.launch.py \
     linear_scale:=0.5 \
     frequency:=0.5
   ```

### Carter Robot Example

1. Load Carter in Isaac Sim
2. Adjust parameters for larger robot:
   ```bash
   ros2 launch ghost_follower ghost_publisher.launch.py \
     linear_scale:=0.8 \
     angular_scale:=0.7 \
     frequency:=0.4
   ```

### Custom Robot Example

For a custom differential drive robot, tune parameters based on:
- Robot size: Larger robots may need smaller angular_scale
- Wheel base: Wider wheel base may need larger angular_scale
- Max velocity: Adjust linear_scale to stay within limits

Example for a large robot:
```bash
ros2 launch ghost_follower ghost_publisher.launch.py \
  linear_scale:=1.5 \
  angular_scale:=0.5 \
  frequency:=0.3
```

## Testing and Debugging

### Check if Node is Running

```bash
ros2 node list
# Should show: /ghost_publisher
```

### Check Node Info

```bash
ros2 node info /ghost_publisher
```

### List Parameters

```bash
ros2 param list /ghost_publisher
```

### Get Parameter Value

```bash
ros2 param get /ghost_publisher linear_scale
```

### Set Parameter at Runtime

```bash
ros2 param set /ghost_publisher linear_scale 0.8
```

Note: Runtime parameter changes require node restart to take effect.

## Common Parameter Combinations

### Conservative (Safe for testing)
```bash
linear_scale: 0.3
angular_scale: 0.7
frequency: 0.3
publish_rate_hz: 50
```

### Balanced (Default)
```bash
linear_scale: 0.5
angular_scale: 1.0
frequency: 0.5
publish_rate_hz: 50
```

### Aggressive (For capable robots)
```bash
linear_scale: 1.0
angular_scale: 1.5
frequency: 0.7
publish_rate_hz: 100
```

### Demo Mode (Impressive for presentations)
```bash
linear_scale: 0.8
angular_scale: 1.2
frequency: 0.6
publish_rate_hz: 60
```

## Tips and Tricks

1. **Start Conservative**: Begin with small values and increase gradually
2. **Match Robot Capabilities**: Don't exceed your robot's max velocities
3. **Tune Frequency**: Lower frequency = larger figure-8, higher = smaller
4. **Higher Publish Rate**: Use for smoother motion (at cost of CPU usage)
5. **Monitor CPU**: High publish rates may strain older systems
6. **Check Safety**: Always test in simulation before real hardware

## Troubleshooting

**Robot moves but not in figure-8:**
- Check parameter values aren't too extreme
- Verify robot's differential drive controller is working
- Ensure angular velocity isn't being clamped

**Robot moves too slowly:**
- Increase `linear_scale`
- Decrease `frequency` for a larger pattern

**Robot turns too sharply:**
- Decrease `angular_scale`
- Increase `frequency` for gentler curves

**Choppy movement:**
- Increase `publish_rate_hz`
- Check CPU usage isn't too high

## Additional Resources

- [Main README](../README.md)
- [Isaac Sim Setup Guide](../docs/ISAAC_SIM_SETUP.md)
- [Quick Start Guide](../docs/QUICK_START.md)
- [ROS 2 Documentation](https://docs.ros.org/)
