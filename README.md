# mercury
offical repo for ICMTC UGVC-2026

# How to run docker

```
sudo docker compose build 
sudo docker compose run ros
```

# How to run simulation

```
echo 'export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix simulation)/share/simulation/models:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
colcon build
ros2 launch bringup bringup_sim.launch.py
```

# Add this to your bashrc/zshrc
```
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
```


# watchdog_monitor

A non-intrusive ROS2 monitoring and observability package for the Mercury robot.
Runs alongside the existing stack without modifying any control logic.

---

# Nodes

| Node                     | Publishes                               | Rate         | Description                                                    |
| ------------------------ | --------------------------------------- | ------------ | -------------------------------------------------------------- |
| `system_monitor_node`    | `/system_status`                        | 2s           | Tracks running vs expected nodes, publishes JSON health report |
| `watchdog_node`          | `/system_alerts`                        | 3s           | Detects node crashes, topic silences, TF failures              |
| `waypoint_detector_node` | `/waypoint_reached`, `/waypoint_status` | 10Hz / 1Hz   | Detects robot arrival at predefined waypoints                  |
| `control_listener_node`  | —                                       | event-driven | Passive observer, logs all monitoring events                   |
| `monitoring_dashboard`   | —                                       | 1Hz          | Live terminal dashboard                                   |

---

# Quick Start

```bash
# Build
colcon build --packages-select watchdog_monitor
source install/setup.bash

# Launch everything
ros2 launch watchdog_monitor monitoring_all.launch.py

# Launch dashboard (separate terminal)
ros2 launch watchdog_monitor dashboard.launch.py
```

---

# Launch Arguments

| Argument           | Default                       | Description                              |
| ------------------ | ----------------------------- | ---------------------------------------- |
| `monitor_interval` | `2.0`                         | Node health check interval (s)           |
| `topic_timeout`    | `5.0`                         | Topic silence threshold before alert (s) |
| `arrival_radius`   | `0.5`                         | Waypoint detection radius (m)            |
| `odom_topic`       | `/diff_drive_controller/odom` | Odometry source topic                    |

---

# Waypoint Configuration

Edit `config/waypoints.yaml` to define waypoints in your map frame:

```yaml
waypoint_detector_node:
  ros__parameters:
    waypoints: [2.0, 0.0, 2.0, 4.0, 0.0, 4.0]  # [x1,y1, x2,y2, ...]
    waypoint_names: ["WP-1", "WP-2", "WP-3"]
    arrival_radius: 0.5
```

---

# Topic Reference

| Topic               | Type                     | Published by             |
| ------------------- | ------------------------ | ------------------------ |
| `/system_status`    | `std_msgs/String` (JSON) | `system_monitor_node`    |
| `/system_alerts`    | `std_msgs/String` (JSON) | `watchdog_node`          |
| `/waypoint_reached` | `std_msgs/String` (JSON) | `waypoint_detector_node` |
| `/waypoint_status`  | `std_msgs/String` (JSON) | `waypoint_detector_node` |

