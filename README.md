# my_bot - ROS 2 Navigation Robot

A differential drive robot with autonomous navigation using SLAM Toolbox and Nav2.

## Setup

### 1. Create Workspace and Clone Repository
```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone <your-repo-url>
```

### 2. Install Dependencies
```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
```

### 3. Build and Source
```bash
cd ~/dev_ws
colcon build
source install/setup.bash
```

## Usage

### Phase 1: Create Map (SLAM Mapping)

**Terminal 1: Launch Simulation**
```bash
source ~/dev_ws/install/setup.bash
ros2 launch my_bot launch_sim.launch.py world:=$(ros2 pkg prefix my_bot)/share/my_bot/worlds/obstacles.world
```

**Terminal 2: Open RViz**
```bash
source ~/dev_ws/install/setup.bash
rviz2 -d $(ros2 pkg prefix my_bot)/share/my_bot/config/view_bot.rviz
```

**Terminal 3: Start SLAM**
```bash
source ~/dev_ws/install/setup.bash
ros2 launch my_bot online_async_launch.py
```

**In RViz:**
1. Change Fixed Frame: `odom` → `map`
2. Add → By topic → Map → `/map`
3. Drive robot around with joystick
4. Save map using SlamToolbox panel: filename `my_map_save`

**Close all terminals when done**

---

### Phase 2: Autonomous Navigation

**Terminal 1: Launch Simulation**
```bash
source ~/dev_ws/install/setup.bash
ros2 launch my_bot launch_sim.launch.py world:=$(ros2 pkg prefix my_bot)/share/my_bot/worlds/obstacles.world
```

**Terminal 2: Launch Navigation**
```bash
source ~/dev_ws/install/setup.bash
ros2 launch my_bot nav2_combined.launch.py
```

**In RViz (opens automatically):**
1. Click "2D Pose Estimate" → Set robot's initial position
2. Click "Nav2 Goal" → Set destination
3. Robot navigates autonomously!

## Map Files Location

Maps are saved in: `~/dev_ws/my_map_save.yaml` and `~/dev_ws/my_map_save.pgm`

## Quick Commands

```bash
# Mapping
ros2 launch my_bot launch_sim.launch.py world:=$(ros2 pkg prefix my_bot)/share/my_bot/worlds/obstacles.world
rviz2 -d $(ros2 pkg prefix my_bot)/share/my_bot/config/view_bot.rviz
ros2 launch my_bot online_async_launch.py

# Navigation
ros2 launch my_bot launch_sim.launch.py world:=$(ros2 pkg prefix my_bot)/share/my_bot/worlds/obstacles.world
ros2 launch my_bot nav2_combined.launch.py
```


# Real Robot Usage

## Hardware Setup

### Prerequisites
```bash
# Install additional dependencies
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
```

### Hardware Checklist
- ✅ ESP32 powered and connected to WiFi (check IP in serial monitor)
- ✅ Update ESP32 IP in `description/ros2_control.xacro` if changed
- ✅ RPLidar connected via USB
- ✅ Motors and encoders wired to ESP32
- ✅ Raspberry Pi on same network as ESP32

---

## Quick Start

### Phase 1: SLAM Mapping

**Terminal 1: Launch Robot**
```bash
ros2 launch my_bot launch_robot.launch.py
```
*Wait for: "Successfully activated!" and "RPLidar start"*

**Terminal 2: Start SLAM**
```bash
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=./src/my_bot/config/mapper_params_online_async.yaml \
  use_sim_time:=false
```

**Terminal 3: Visualize & Drive**
```bash
# Open RViz
rviz2

# In RViz: Set Fixed Frame to 'map', Add /map and /scan topics

# Drive robot (keyboard)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

**Save Map:**
```bash
cd ~/dev_ws
ros2 run nav2_map_server map_saver_cli -f my_map
```
*Close all terminals*

---

### Phase 2: Autonomous Navigation

**Terminal 1: Launch Robot**
```bash
ros2 launch my_bot launch_robot.launch.py
```

**Terminal 2: Launch Navigation**
```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  map:=~/dev_ws/my_map.yaml
```

**Terminal 3: RViz Navigation**
```bash
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**In RViz:**
1. Click **"2D Pose Estimate"** → Set robot's current position
2. Click **"Nav2 Goal"** → Set destination
3. Robot navigates autonomously!

---

## Troubleshooting

**ESP32 Connection Issues:**
```bash
# Check connectivity
ping <ESP32_IP>

# Test manually
nc <ESP32_IP> 8888
e   # Should return encoder values: "0 0"
```

**TF Transform Errors:**
```bash
# Check delays (should be < 0.01s)
ros2 run tf2_ros tf2_monitor odom laser_frame
```

**Robot Not Moving:**
```bash
# Test command
ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}}" --once

# Check encoders updating
ros2 topic echo /joint_states --field position
```

**Notes:**
- Occasional "Empty response from ESP32" warnings (<10%) are normal with WiFi
- Ensure all nodes show "Successfully activated" before proceeding
- For best mapping: drive slowly, close loops, cover all areas

---

## Key Differences from Simulation

| Aspect | Simulation | Real Robot |
|--------|-----------|------------|
| `use_sim_time` | `true` | `false` |
| Hardware Interface | Gazebo | ESP32 TCP |
| Launch Lidar | Automatic | `launch_robot.launch.py` includes it |
| Communication | Instant | WiFi (occasional delays) |
| Encoder Accuracy | Perfect | Real-world noise |

---

**Real robot is now operational!** 🤖

For detailed debugging and advanced features, see full documentation above.