---
title: "Clue Chain Hunt — Technical Report"
subtitle: "Inter IIT Bootcamp Phase 2 · Hardware PS · IIT Guwahati"
---

## 1. System overview

| Node / file | Package | Role |
|---|---|---|
| `hunt_node` | clue_hunt_solver | leader autonomy: perception, clue logic, Nav2 client |
| `follower_node` | clue_hunt_solver | follower autonomy (camera + wheel odometry only) |
| `hunt.launch.py` | clue_hunt_solver | single command that starts both nodes (`use_sim_time:=True`) |
| `sim.launch.py` | clue_hunt_gazebo | Gazebo Fortress, both robots, `ros_gz_bridge` |
| `navigation.launch.py` | clue_hunt_navigation | map_server + AMCL + Nav2 + RViz2 on the saved map |

**Data flow.** Leader: `/camera/image_raw`, `/scan`, TF → `hunt_node` → Nav2 `navigate_to_pose` → `/cmd_vel`.
Follower: `/follower/camera/image_raw`, `follower/odom` TF → `follower_node` → `/follower/cmd_vel`.
Published: `/hunt/clues`, `/hunt/boards`, `/hunt/treasure` (+ `/leader/status`).
The leader sets its own AMCL initial pose at (0,0) (it spawns at the origin).

## 2. Clue detection, pose estimation, frames

- **ArUco:** `DICT_4X4_50`, side 0.24 m; wrapper supports both the old and new OpenCV ArUco API.
- **solvePnP:** `solvePnPGeneric` with IPPE_SQUARE and ITERATIVE; the candidate with the lowest reprojection error and positive depth is kept. Poses behind the camera, farther than 5.5 m, or facing away (flipped PnP solution) are discarded.
- **Frames:** marker → `cam_optical_link` → (TF2, at the image timestamp) → `map`. The board frame (+X out of the face, +Y reader's right, +Z up) is related to the OpenCV marker frame by a fixed rotation `R_MB`. Each board keeps all observations; its position and normal are the **median of the 10 closest views** (close views are the most accurate). Board position is published on `/hunt/boards` as `id x y`.
- **QR decoding:** the region 0.30 m right of the marker is projected into the image with the estimated pose, cropped, upscaled (1×/2×/3×) and decoded with `cv2.QRCodeDetector`, with and without CLAHE.
- **Validation:** a clue is `HUNT:<n>:<token>:<command>`; it is valid only if `n` equals the expected index and `token` equals SHA-1(previous clue text)[:4]. The first token is derived from `START`.

## 3. Clue-solving and search strategy

1. Wait for Nav2, the map and the camera; publish the initial pose.
2. For board *n*: if a candidate with id *n* is already seen, approach it; otherwise go to the hinted area, **spin 360°**, then visit a **ring of 6 waypoints** (r = 1.3 m) around the hint; if still not found, run a **grid exploration** over free map cells (greedy nearest-neighbour tour).
3. **Approach:** drive to a stand-off (1.2 m, then 0.9 m and 1.6 m as retries) along the board normal, face the board, re-estimate from close range, decode the QR.
4. **Reject:** decoy boards are ignored by id; look-alike boards (right id, wrong token) are rejected and the next nearest candidate is tried.
5. **Commands:** `GOTO x y` → search around the point; `REL a b` → point relative to the board frame; `PILLAR <colour>` → pillar located at run time; `BETWEEN A B f` → point at fraction *f* between two pillars; `TREASURE` prefix → drive onto the point, publish `/hunt/treasure`, log `DONE`.
6. **Pillar finder:** HSV colour blob in the camera gives the bearing; the LiDAR range at that bearing (+0.2 m radius) gives the centre; observations are clustered with a median and outlier rejection. Nothing is hard-coded.
7. Goals are snapped to the nearest free cell of the inflated map (unknown cells count as blocked).

## 4. Follower design

- Detects **tag 49** (0.12 m) in the follower camera; leader centre = tag position 0.21 m ahead along the tag normal; transformed to `follower/odom` (wheel odometry only).
- **Breadcrumbs:** leader positions are stored every 0.12 m; the follower runs **pure pursuit on the trail** (lookahead 0.7 m), so it follows the leader's path rather than cutting corners into walls.
- **Speed:** leader speed estimate (low-passed feed-forward) + P-term on the gap (`kp_gap` 1.5) around `d_des` = 1.2 m; stops at `d_stop` = 0.9 m while still facing the leader; speed is scaled by cos(heading error) on turns.
- **Tag lost:** keeps following crumbs for 3 s, then spins to re-acquire. It uses no data from the leader (no LiDAR, map or leader topics except the status string `/leader/status`).
