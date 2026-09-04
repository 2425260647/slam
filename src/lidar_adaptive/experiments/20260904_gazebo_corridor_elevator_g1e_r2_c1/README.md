# Gazebo corridor elevator C1

- World: `/home/slam/slam_ws/install_isolated/share/lidar_adaptive/worlds/corridor_elevator.world`
- Motion: Scout Mini starts at x=-2 m and receives 0.35 m/s forward velocity, crossing the elevator opening.
- Duration: 60.000 s
- C0: baseline pointcloud_to_laserscan; C1: confidence projection; C2: external scan-thinning.
- Gazebo `/gazebo/model_states` is continuous truth; ATE/RPE are computed only for this simulated run.
