# BonicBot A2 — Manual Control Reference

Practical `ros2 topic pub`/`echo` commands for driving, moving joints, and reading
sensors directly, bypassing `robot_app`. Everything here was verified against real
A2 Pro hardware on 2026-08-24. Prerequisite for all sections below:

```bash
cd ~/bonic/bonicbot-a2-ros
ros2 launch bonicbot_a2_hardware hardware.launch.py use_lidar:=false use_camera:=false
```

Wait for `Activated` and `ros2 control list_controllers` to show all 7 controllers
`active` before sending anything.

---

## 1. Base / drive

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once
ros2 topic echo /diff_cont/odom --once
```

`linear.x` in m/s, `angular.z` in rad/s. Max wheel speed is `max_velocity` (10.47
rad/s) × `wheel_radius` (0.06 m) ≈ 0.63 m/s — stay well under that on the bench.
A single `--once` publish decelerates back to stop almost immediately; for a
sustained move, publish repeatedly (e.g. `ros2 topic pub -r 10 ...`) or use
teleop/joystick.

---

## 2. Joints — arms, grippers, neck

All arm/gripper/neck controllers take `std_msgs/msg/Float64MultiArray` on
`/<controller>/commands`, in **radians**, and the array **must contain every joint
in that controller's group, every publish** — partial arrays are silently rejected
(no joint names carried; array position = joint identity).

| Controller | Topic | Array order | Range (rad) | Range (deg) |
|---|---|---|---|---|
| Right arm | `/right_arm_controller/commands` | `[shoulder_pitch, elbow]` | shoulder: −0.785…3.14 · elbow: −0.873…0 | −45°…180° · −50°…0° |
| Left arm | `/left_arm_controller/commands` | `[shoulder_pitch, elbow]` | same as right | same as right |
| Right gripper | `/right_gripper_controller/commands` | `[finger1]` | −0.785…1.047 | −45°…60° |
| Left gripper | `/left_gripper_controller/commands` | `[finger1]` | −0.785…1.047 | −45°…60° |
| Head/neck | `/head_controller/commands` | `[neck_yaw]` | −1.571…1.571 | −90°…90° |

Examples:

```bash
# Right arm: shoulder +10°, elbow -10°
ros2 topic pub /right_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.1745, -0.1745]}" --once

# Left arm
ros2 topic pub /left_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.1745, -0.1745]}" --once

# Right gripper open ~10°
ros2 topic pub /right_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.1745]}" --once

# Left gripper
ros2 topic pub /left_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.1745]}" --once

# Neck yaw +10°
ros2 topic pub /head_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.1745]}" --once
```

Torque auto-engages on the first position command sent to a servo — no separate
enable step needed. To release all servos (arms go loose):

```bash
./stop_session.sh
```

### Reading joint positions

```bash
ros2 topic echo /joint_states --once
```

`position` is radians, in the same order as the `name` array (which includes both
wheels alongside the 7 servos). `effort` is always `nan` — CDC servo feedback is
position-only, no load/torque data. `/joint_states` updates continuously at 50 Hz;
`ros2 topic hz /joint_states` should read ~50.

---

## 3. Battery

```bash
ros2 topic echo /battery_state --once
```

Polled once per second internally. Fields that are meaningful: `voltage` (V),
`current` (A, negative while discharging), `percentage` (0.0–1.0 SOC). Everything
else (`temperature`, `charge`, `capacity`, cell-level fields) is unavailable over
CDC and stays at defaults.

---

## 4. IMU

```bash
ros2 topic echo /imu/data --once
ros2 topic hz /imu/data
```

Polled at ~25 Hz (decimated from the 50 Hz control loop). Linear acceleration in
m/s², angular velocity in rad/s (converted from the wire's deg/s).

---

## 5. Face / LED matrix display

Raw pass-through topic — publish the exact `CMD_MATRIX_ACTION` byte payload
(action code + that action's own fields) as a `std_msgs/msg/UInt8MultiArray`. ROS
does not interpret or name any of this; it's a dumb pipe straight to the ESP.
Fire-and-forget — nothing is sent unless you publish, so this costs nothing at
idle (unlike the polled sensors above).

```bash
ros2 topic pub /face/matrix_action std_msgs/msg/UInt8MultiArray "{data: [<action_code>, <payload...>]}" --once
```

| Code | Action | Payload | Notes |
|---|---|---|---|
| 1 | `SET_TEXT` | ASCII bytes | e.g. `[1, 72, 73]` = "HI" |
| 2 | `SET_COLOR` | R, G, B | **Sets the text/animation tint only — does NOT fill the whole matrix by itself.** See note below. |
| 3 | `SET_ANIMATION` | 1B anim ID | ID → animation mapping isn't documented anywhere available to this repo; ask firmware owner |
| 4 | `SET_BRIGHTNESS` | 1B (or 4B) 0–255 | |
| 5 | `SET_SPEED` | 1B (or 4B) | |
| 9 | `PLAY` | — | resume animation |
| 10 | `PAUSE` | — | **stop the animation loop before manual pixel control, or your pixels get overwritten on the next animation frame** |
| 11 | `GET_STATUS` | — | triggers `RESP_MATRIX_STATUS` (0x58) reply — **not parsed/exposed in ROS**, command sends fine but the response goes nowhere |
| 12 | `SET_PIXEL` | X, Y, R, G, B | verified working — single-pixel writes land correctly |
| 13 | `CLEAR` | — | |
| 14 | `SET_FRAME` | 384B RGB (16×8 grid) | too large to hand-type via `ros2 topic pub`; needs a small script |

```bash
# Pause any running animation first
ros2 topic pub /face/matrix_action std_msgs/msg/UInt8MultiArray "{data: [10]}" --once

# Set a single pixel to red (X=0, Y=0)
ros2 topic pub /face/matrix_action std_msgs/msg/UInt8MultiArray "{data: [12, 0, 0, 255, 0, 0]}" --once

# Clear
ros2 topic pub /face/matrix_action std_msgs/msg/UInt8MultiArray "{data: [13]}" --once
```

> **Full-display color change — important finding:** `SET_COLOR` (action 2) does
> **not** paint the whole matrix a solid color. Bench-testing on 2026-08-24 showed
> it has no visible effect on its own — sending it changes nothing on the display.
> This matches `RESP_MATRIX_STATUS`'s field naming, which calls the equivalent
> state **"Text Color"**: `SET_COLOR` is almost certainly just the tint used when
> text or an animation is actively rendering, not a fill command. `SET_PIXEL` was
> confirmed working (individual pixels update correctly), so the transport and
> firmware are both fine — to get an actual full-red screen, use `SET_FRAME` with
> all 128 pixels set to the same color in one 384-byte packet, or drive `SET_PIXEL`
> across every coordinate.

---

## 6. Recovery

- **Controllers wedged / commands not landing:** `./stop_session.sh` then relaunch
  — `Ctrl+C` alone leaves orphaned processes.
- **ESP unplugged/replugged mid-session:** no action needed — reconnects
  automatically on a backoff (1s → 2s → 4s → 5s cap) once `/dev/esp` reappears.
  Watch the launch terminal for `Reconnected; re-running handshake`.
