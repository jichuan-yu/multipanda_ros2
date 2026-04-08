

```
docker start multipanda-container
```

```
docker exec -it --user developer -e DISPLAY=$DISPLAY multipanda-container bash 
```

```
source ./install/setup.bash
```

Run real-robot controller:
```
ros2 launch franka_bringup franka_control.launch.py \
  robot_ip:=172.16.0.3 \
  load_gripper:=true \
  use_rviz:=true
```

Run simulation:
```
```
