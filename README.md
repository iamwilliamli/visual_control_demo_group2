## EVC demo Group 2

Dependencies
pyzbar:
```bash
sudo apt-get install libzbar0
python3 -m pip install pyzbar
```

To run
 - `roslaunch coordinator everything.launch` on jetbot
 - `roslaunch paul_alarm paul_everything.launch` in docker
 - `roslaunch jetson_camera ocr_node.launch` in docker
 - `roslaunch lane_detection final_assignment_local.launch` in docker