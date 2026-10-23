# Lab 1: ROS 2, Turtlesim, Actions & Services

**Course:** Design of Mechatronics Systems, Faculty of Engineering, Alexandria University
**Instructors:** Eng. Abdallah Elsayed & Eng. Mohamed Hussein
**ROS 2 distribution:** Jazzy (Ubuntu 24.04)

A small ROS 2 system that controls the turtlesim simulator. It provides a custom message, a `goToPose` action server, a reset service, a speed-limit service with live ROS parameters, and a square action server that draws a square by calling `goToPose` four times. The whole system starts from one launch file.

## Team

| Name | ID |
|------|----|
| Ahmed Mohamed Hammad | 9287 |
| Eyad Sakr | 9352 |
| Omar Mohab | 9479 |

## Repository contents

This repository contains only the two ROS 2 packages (no `build/`, `install/` or `log/` folders).

```
turtlesim-lab1/
├── README.md
├── .gitignore
├── turtle_interfaces/          # interface definitions only
│   ├── msg/TargetPose.msg
│   ├── action/GoToPose.action
│   ├── action/DrawSquare.action
│   └── srv/SetSpeedLimits.srv
└── turtle_control/             # nodes, launch file, parameters
    ├── turtle_control/
    │   ├── go_to_pose_server.py
    │   ├── reset_service.py
    │   └── square_server.py
    ├── launch/system.launch.py
    ├── config/params.yaml
    ├── setup.py
    └── package.xml
```

## Package descriptions

### `turtle_interfaces`

| Interface | Type | Fields |
|-----------|------|--------|
| `TargetPose` | msg | `float32 x`, `float32 y`, `float32 yaw` |
| `GoToPose` | action | Goal: `TargetPose target`. Result: `bool success`, `float32 final_distance_error`. Feedback: `float32 distance_remaining` |
| `DrawSquare` | action | Goal: `float32 side_length`. Result: `bool success`. Feedback: `uint8 current_side` |
| `SetSpeedLimits` | srv | Request: `float32 max_linear_speed`, `float32 max_angular_speed`. Response: `bool success`, `string message` |

### `turtle_control`

| Node | Provides | Description |
|------|----------|-------------|
| `go_to_pose_server` | action `/go_to_pose`, service `/set_speed_limits`, parameters `max_linear_speed`, `max_angular_speed` | Subscribes to `/turtle1/pose`, publishes `/turtle1/cmd_vel`. Drives to (x, y) with a proportional controller (turning toward the target first if the heading error is large), then rotates to the final yaw. Publishes `distance_remaining` as feedback. Reads the speed parameters on every control step, so changes take effect while the turtle is moving. |
| `reset_service` | service `/reset_turtle` (`std_srvs/srv/Empty`) | Uses only turtlesim's own services: lifts the pen (`/turtle1/set_pen`), teleports to the start pose (`/turtle1/teleport_absolute`), clears the screen (`/clear`), then puts the pen back down. No line is drawn while teleporting. |
| `square_server` | action `/draw_square` | Calls `/reset_turtle` first, then sends exactly 4 separate goals to `/go_to_pose` (one per corner), waiting for each to finish. Publishes the current side as feedback. |

### Behavior details

- **goToPose limits:** x and y must be in `[0, 11]`, otherwise the goal is rejected. A new goal is rejected while another one is running. A goal is aborted after a 60 s timeout. Tolerances are 0.05 for position and 0.03 rad for yaw.
- **Speed limits:** both must be greater than zero. Invalid values are refused by the service (`success: false`) and by `ros2 param set`.
- **Square:** `side_length` must be in `(0, 4.5]` so the whole square stays inside the window. The square starts from the turtle's start pose (about 5.54, 5.54).

## Requirements covered

| Req. | Requirement | Where |
|------|-------------|-------|
| R1 | Run turtlesim | Started by `system.launch.py` |
| R2 | Custom message | `turtle_interfaces/msg/TargetPose.msg` |
| R3 | goToPose action server | `go_to_pose_server` |
| R4 | Reset service | `reset_service` (`/reset_turtle`) |
| R5 | Square action server | `square_server` (`/draw_square`) |
| R6 | Launch files | `turtle_control/launch/system.launch.py` |
| R7 | ROS parameters + service to change them | `max_linear_speed`, `max_angular_speed`, `/set_speed_limits` |
| R8 | Packages | `turtle_interfaces`, `turtle_control` |

## Prerequisites

- Ubuntu 24.04 with ROS 2 Jazzy installed
- The turtlesim package and colcon:

```bash
sudo apt install ros-jazzy-turtlesim python3-colcon-common-extensions
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Hammad-911/turtlesim-lab1.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

> Every new terminal needs `source ~/ros2_ws/install/setup.bash` before running any `ros2` command that uses our interfaces.

## Run

Start the whole system with one command (turtlesim, all three nodes, and the parameters from `config/params.yaml`):

```bash
ros2 launch turtle_control system.launch.py
```

Open a second sourced terminal for the commands below. Check that everything started:

```bash
ros2 node list    # /turtlesim, /go_to_pose_server, /reset_service, /square_server
ros2 action list  # /go_to_pose, /draw_square
```

To run the nodes by hand instead of with the launch file, start each in its own sourced terminal:

```bash
ros2 run turtlesim turtlesim_node
ros2 run turtle_control go_to_pose_server
ros2 run turtle_control reset_service
ros2 run turtle_control square_server
```

## Try every requirement

### Interfaces (R2)

```bash
ros2 interface list | grep turtle_interfaces
ros2 interface show turtle_interfaces/msg/TargetPose
ros2 interface show turtle_interfaces/action/GoToPose
ros2 interface show turtle_interfaces/action/DrawSquare
ros2 interface show turtle_interfaces/srv/SetSpeedLimits
```

### Parameters and speed service (R7)

```bash
ros2 param list
ros2 param get /go_to_pose_server max_linear_speed
ros2 param set /go_to_pose_server max_linear_speed 1.0
ros2 service call /set_speed_limits turtle_interfaces/srv/SetSpeedLimits "{max_linear_speed: 1.0, max_angular_speed: 1.0}"
ros2 param get /go_to_pose_server max_linear_speed     # now 1.0
```

Invalid values are refused:

```bash
ros2 service call /set_speed_limits turtle_interfaces/srv/SetSpeedLimits "{max_linear_speed: -1.0, max_angular_speed: 1.0}"
# success=False, message='Speeds must be greater than zero'
ros2 param set /go_to_pose_server max_linear_speed -1.0   # refused
```

### goToPose action (R3)

Yaw reference: `0` = right, `1.57` = up, `3.14` = left, `-1.57` = down.

```bash
ros2 action send_goal /go_to_pose turtle_interfaces/action/GoToPose "{target: {x: 8.0, y: 3.0, yaw: 1.57}}" --feedback
ros2 topic echo /turtle1/pose --once      # about x 8.0, y 3.0, theta 1.57
```

Feedback shows `distance_remaining` shrinking toward zero, and the result is `success: true` with a small `final_distance_error`. A goal outside the window is rejected:

```bash
ros2 action send_goal /go_to_pose turtle_interfaces/action/GoToPose "{target: {x: 20.0, y: 3.0, yaw: 0.0}}"
```

To see the speed limits act live, send a long goal and run `ros2 param set /go_to_pose_server max_linear_speed 0.5` while the turtle moves.

### Reset service (R4)

```bash
ros2 service call /reset_turtle std_srvs/srv/Empty
ros2 topic echo /turtle1/pose --once      # about x 5.54, y 5.54, theta 0
```

The screen is cleared and the turtle returns to the start with no line drawn. It can be called repeatedly, including when the turtle is already at the start.

### Square action (R5)

```bash
ros2 action send_goal /draw_square turtle_interfaces/action/DrawSquare "{side_length: 3.0}" --feedback
```

Feedback shows `current_side` 1 to 4, and the result is `success: true`. The `go_to_pose_server` log shows exactly 4 accepted goals. Running it again starts clean because of the reset call. A side length that is too large is rejected:

```bash
ros2 action send_goal /draw_square turtle_interfaces/action/DrawSquare "{side_length: 8.0}" --feedback
```

### Launch file and parameters (R6, R7)

The launch file loads `config/params.yaml`:

```yaml
go_to_pose_server:
  ros__parameters:
    max_linear_speed: 2.0
    max_angular_speed: 2.0
```

Change the values, run `colcon build`, relaunch, and confirm with `ros2 param get /go_to_pose_server max_linear_speed`.

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `package not found` or `The passed ... type is invalid` | Run `source ~/ros2_ws/install/setup.bash` in this terminal. |
| `No executable found` | Check `console_scripts` in `turtle_control/setup.py`, rebuild, and source again. |
| Goal rejected or nothing happens | Check `ros2 action list` and that the nodes are running. Targets must be inside 0 to 11. |
| Changes do not show up | Run `colcon build`, then source in a new terminal. |
| Every node appears twice in `ros2 node list` | Old terminals are still running the nodes. Stop them all and start the system once. |
