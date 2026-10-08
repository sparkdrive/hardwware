# Clue Chain Hunt — Inter IIT Bootcamp, Phase 2 (Hardware PS)

Autonomous clue hunt in ROS 2 Humble + Gazebo Fortress. A **leader** (LiDAR + camera) solves a chain of
ArUco + QR clue boards to reach the treasure; a **camera-only follower** keeps 0.6–2.0 m behind it.

```
clue_hunt_description/   robot URDF/xacro (provided starter)
clue_hunt_gazebo/        practice world, board models, sim.launch.py (provided starter)
clue_hunt_navigation/    slam_toolbox + Nav2 configs and launch files (provided starter)
clue_hunt_solver/        OUR SOLUTION: hunt_node, follower_node, hunt.launch.py, saved map (maps/arena.*)
docs/                    technical report + screenshots
```

## 1. Setup (Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress)
```bash
sudo apt install -y ros-humble-ros-gz ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard ros-humble-tf2-tools \
  ros-humble-xacro ros-humble-robot-state-publisher python3-opencv python3-numpy

mkdir -p ~/hunt_ws/src ~/hunt_ws/maps
git clone <THIS_REPO_URL> ~/hunt_ws/src/clue_hunt      # all four packages are at the repo root
cd ~/hunt_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash
```
> If colcon does not find the packages, clone into a folder and run `colcon build` from a workspace whose
> `src/` contains the four package folders (the repo root *is* that folder).

## 2. Map (already saved — only redo if you want to)
The saved map is `clue_hunt_solver/maps/arena.yaml` (+ `arena.pgm`), installed with the package.
To rebuild it:
```bash
ros2 launch clue_hunt_navigation mapping.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard          # drive slowly round the arena
ros2 run nav2_map_server map_saver_cli -f ~/hunt_ws/maps/arena
cp ~/hunt_ws/maps/arena.* ~/hunt_ws/src/clue_hunt/clue_hunt_solver/maps/
```

## 3. Autonomous run (3 terminals, no human input)
```bash
# T1 – simulation, both robots
ros2 launch clue_hunt_gazebo sim.launch.py
# T2 – Nav2 (AMCL) on the saved map
ros2 launch clue_hunt_navigation navigation.launch.py \
  map:=$(ros2 pkg prefix clue_hunt_solver)/share/clue_hunt_solver/maps/arena.yaml
# T3 – leader + follower
ros2 launch clue_hunt_solver hunt.launch.py
```
Watch T3 for `valid clue N: ...`; the last line is `DONE`.
Check outputs: `ros2 topic echo /hunt/clues`, `/hunt/boards`, `/hunt/treasure`, `/follower/cmd_vel`.

## 4. Published topics
| topic | type | content |
|---|---|---|
| `/hunt/clues` | std_msgs/String | full text of each valid clue, in order |
| `/hunt/boards` | std_msgs/String | `"id x y"` board position in `map` |
| `/hunt/treasure` | geometry_msgs/PoseStamped | treasure position in `map` |
| `/follower/cmd_vel` | geometry_msgs/Twist | follower velocity commands |
| `/leader/status` | std_msgs/String | MOVING / SEARCHING / READING / DONE (extra) |

## 5. Nodes
| node | role |
|---|---|
| `hunt_node` | ArUco (DICT_4X4_50, 0.24 m) + QR detection, `solvePnP` → TF2 → `map`, chain-token check (rejects look-alikes), clue parser (GOTO / REL / PILLAR / BETWEEN / TREASURE), ring + grid search, Nav2 goals, run-time pillar finder (colour blob + LiDAR range) |
| `follower_node` | tracks tag id 49 (0.12 m), leader pose in `follower/odom`, breadcrumb pure-pursuit, speed feed-forward + gap P-term, tag-lost recovery. Uses only follower camera + odometry |
| `common.py` | image conversion, ArUco wrapper (old/new OpenCV API), PnP, QR crop + decode |

Parameters: hunt `standoff` (1.2 m), `search_radius` (1.3 m), `tag_size`; follower `d_stop`, `d_des`, `kp_gap`, `lookahead`, `v_max`, `w_max`.

## 6. Known limitations
- No live-SLAM (no-pre-built-map) mode and no RViz markers yet (bonus items not implemented).
- Follower has no obstacle sensing; it relies on following the leader's path.
- See `docs/REPORT.pdf` for the system description.
