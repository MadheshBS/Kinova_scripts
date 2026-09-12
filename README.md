# kinova_ws — Terminal Commands Reference

**Terminal 1 — Launch Gazebo with the robot spawned:**
```bash
ros2 launch kortex_bringup kortex_sim_control.launch.py robot_type:=gen3 dof:=7 gripper:=robotiq_2f_85
```

**Terminal 2 — Launch MoveIt2's move_group node:**
```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config move_group.launch.py
```

**Terminal 3 — Launch MoveIt2 RViz GUI:**
```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config moveit_rviz.launch.py
```

**Terminal 4 — Spawn the table:**
```bash
ros2 run ros_gz_sim create -file ~/kinova_ws/src/kinova_scripts/models/simple_table/model.sdf -name my_table -x 0.4 -y 0.0 -z 0.2
```

**Terminal 5 — Spawn the overhead camera and start the bridge:**
```bash
ros2 run ros_gz_sim create -file ~/kinova_ws/src/kinova_scripts/models/overhead_camera/model.sdf -name overhead_camera -x 0.4 -y 0.0 -z 1.5 -R 0 -P 1.5708 -Y 0
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$HOME/kinova_ws/src/kinova_scripts/config/camera_bridge.yaml
```
Run these only once per session — running them twice leaves duplicate camera/bridge processes behind, which silently corrupts detector output. Verify before re-running:
```bash
ps aux | grep -E "overhead_camera|parameter_bridge" | grep -v grep
```

**Terminal 6 — Publish the static camera → base_link transform (leave running, required for Stage 3+):**
```bash
ros2 run tf2_ros static_transform_publisher --x 0.4 --y 0.0 --z 1.5 --qx 0.7071 --qy -0.7071 --qz 0 --qw 0 --frame-id base_link --child-frame-id overhead_camera/camera_link/overhead_rgbd
```

**Terminal 7 — Spawn the cubes:**
```bash
ros2 run ros_gz_sim create -file ~/kinova_ws/src/kinova_scripts/models/simple_cube/model.sdf       -name cube_red   -x 0.4 -y 0.1  -z 0.42
ros2 run ros_gz_sim create -file ~/kinova_ws/src/kinova_scripts/models/simple_cube_green/model.sdf -name cube_green -x 0.4 -y 0.0  -z 0.42
ros2 run ros_gz_sim create -file ~/kinova_ws/src/kinova_scripts/models/simple_cube_blue/model.sdf  -name cube_blue  -x 0.4 -y -0.1 -z 0.42
```

**Terminal 8 — View the live RGB camera feed:**
```bash
QT_QPA_PLATFORM=xcb ros2 run rqt_image_view rqt_image_view
```

**Get camera intrinsics (one-off, any terminal):**
```bash
ros2 topic echo /camera/camera_info --once
```

**Save a single frame to disk:**
```bash
ros2 run image_view image_saver --ros-args -r image:=/camera/image_raw
```

**Click a saved frame to read pixel coordinates:**
```bash
QT_QPA_PLATFORM=xcb python3 -c "
import cv2
img = cv2.imread('left0020.jpg')
cv2.imshow('frame', img)
cv2.setMouseCallback('frame', lambda event, x, y, flags, param: print(x, y) if event == cv2.EVENT_LBUTTONDOWN else None)
cv2.waitKey(0)
"
```

**Check HSV values at known pixel coordinates:**
```bash
python3 ~/check_hsv.py
```

**Rebuild after editing kinova_scripts source:**
```bash
cd ~/kinova_ws
colcon build --packages-select kinova_scripts
source install/setup.bash
```

**Terminal 9 — Run the color detector node:**
```bash
ros2 run kinova_scripts cube_color_detector
```

**Verify detector output (any terminal):**
```bash
ros2 topic echo /detected_cube/red
ros2 topic echo /detected_cube/green
ros2 topic echo /detected_cube/blue
```

**Git — commit and push:**
```bash
git status
git add .
git commit -m "<message>"
git pull origin main --no-rebase
git push
```