# Formal Corridor Bag Audit

| Bag | Readable | Duration (s) | Cloud Hz | Odom Hz | Path (m) | Net displacement (m) | Result |
|---|---:|---:|---:|---:|---:|---:|---|
| corridor_repeat_01.bag | True | 565.5 | 10.000 | 50.000 | 173.215 | 61.465 | PASS |
| corridor_repeat_02.bag | True | 555.4 | 10.000 | 50.000 | 174.548 | 61.089 | PASS |
| corridor_repeat_03.bag | True | 578.8 | 10.000 | 50.000 | 173.720 | 60.898 | PASS |

## corridor_repeat_01.bag

Warnings: missing recommended topic: /cmd_vel

```json
{
  "accepted_for_slam": true,
  "duration_s": 565.5273666381836,
  "end_time": 1784864246.554219,
  "errors": [],
  "file_size_bytes": 1440682570,
  "odometry": {
    "maximum_step_rotation_rad": 0.010003921883754319,
    "maximum_step_translation_m": 0.014637569832452882,
    "net_displacement_m": 61.465314113628025,
    "net_yaw_rad": 1.8717108604992776,
    "p99_step_rotation_rad": 0.008155090521635436,
    "p99_step_translation_m": 0.01198010331898302,
    "step_rotation_over_0_1_rad": 0,
    "step_translation_over_0_1_m": 0,
    "total_path_m": 173.215367302528
  },
  "path": "/home/slam/slam_ws/bags/corridor_repeat_01.bag",
  "point_cloud": {
    "longest_sparse_interval_s": 0.09984397888183594,
    "maximum_points": 21303,
    "median_points": 15111,
    "minimum_points": 2965,
    "p01_points": 3886.54,
    "p05_points": 5020.799999999999,
    "sparse_frame_count": 2,
    "sparse_limit_points": 3022.2000000000003
  },
  "readable": true,
  "start_time": 1784863681.0268524,
  "stationary_check": {
    "final_10s": {
      "max_abs_angular_radps": 0.3720000088214874,
      "max_abs_linear_mps": 0.05299999937415123,
      "mean_abs_angular_radps": 0.04165599965304136,
      "mean_abs_linear_mps": 0.0012739999843761325
    },
    "initial_10s": {
      "max_abs_angular_radps": 0.0,
      "max_abs_linear_mps": 0.0,
      "mean_abs_angular_radps": 0.0,
      "mean_abs_linear_mps": 0.0
    }
  },
  "tf": {
    "pairs_and_publishers": {
      "base_link -> base_footprint": [
        "/robot_state_publisher"
      ],
      "base_link -> camera_link": [
        "/camera2base"
      ],
      "base_link -> front_left_wheel_link": [
        "/robot_state_publisher"
      ],
      "base_link -> front_right_wheel_link": [
        "/robot_state_publisher"
      ],
      "base_link -> inertial_link": [
        "/robot_state_publisher"
      ],
      "base_link -> laser_link": [
        "/newscan2base"
      ],
      "base_link -> rear_left_wheel_link": [
        "/robot_state_publisher"
      ],
      "base_link -> rear_right_wheel_link": [
        "/robot_state_publisher"
      ],
      "laser_link -> scan_link": [
        "/laserscan2lasercenter"
      ],
      "odom -> base_link": [
        "/scout_base_node"
      ]
    },
    "static_transforms": [
      {
        "caller": "/camera2base",
        "child": "camera_link",
        "parent": "base_link",
        "translation": [
          0.08,
          0.0,
          0.12
        ],
        "yaw_rad": 0.0
      },
      {
        "caller": "/newscan2base",
        "child": "laser_link",
        "parent": "base_link",
        "translation": [
          0.0,
          0.0,
          0.17
        ],
        "yaw_rad": -1.57
      },
      {
        "caller": "/laserscan2lasercenter",
        "child": "scan_link",
        "parent": "laser_link",
        "translation": [
          0.0,
          0.0,
          0.0
        ],
        "yaw_rad": 1.571592653589793
      },
      {
        "caller": "/robot_state_publisher",
        "child": "base_footprint",
        "parent": "base_link",
        "translation": [
          0.0,
          0.0,
          -0.23479
        ],
        "yaw_rad": 0.0
      },
      {
        "caller": "/robot_state_publisher",
        "child": "inertial_link",
        "parent": "base_link",
        "translation": [
          0.0,
          0.0,
          0.0
        ],
        "yaw_rad": 0.0
      }
    ]
  },
  "timing": {
    "/odom": {
      "count": 28277,
      "frequency_hz": 49.99999584900775,
      "max_gap_s": 0.020254850387573242,
      "non_monotonic_count": 0,
      "p95_gap_s": 0.020066022872924805
    },
    "/tf": {
      "count": 33931,
      "frequency_hz": 59.9999910450754,
      "max_gap_s": 0.020799636840820312,
      "non_monotonic_count": 0,
      "p95_gap_s": 0.020147085189819336
    },
    "/tf_static": {
      "count": 4,
      "frequency_hz": 128.02083672472733,
      "max_gap_s": 0.008783340454101562,
      "non_monotonic_count": 0,
      "p95_gap_s": 0.008780169486999513
    },
    "/velodyne_points": {
      "count": 5655,
      "frequency_hz": 9.99995555330701,
      "max_gap_s": 0.10070490837097168,
      "non_monotonic_count": 0,
      "p95_gap_s": 0.1001291275024414
    }
  },
  "topics": {
    "/odom": {
      "messages": 28277,
      "type": "nav_msgs/Odometry"
    },
    "/scout_status": {
      "messages": 28277,
      "type": "scout_msgs/ScoutStatus"
    },
    "/tf": {
      "messages": 33931,
      "type": "tf2_msgs/TFMessage"
    },
    "/tf_static": {
      "messages": 4,
      "type": "tf2_msgs/TFMessage"
    },
    "/velodyne_points": {
      "messages": 5655,
      "type": "sensor_msgs/PointCloud2"
    }
  },
  "warnings": [
    "missing recommended topic: /cmd_vel"
  ]
}
```

## corridor_repeat_02.bag

Warnings: missing recommended topic: /cmd_vel; point cloud stays below 20% of median for more than 1 s

```json
{
  "accepted_for_slam": true,
  "duration_s": 555.3834307193756,
  "end_time": 1784864895.934312,
  "errors": [],
  "file_size_bytes": 1329514528,
  "odometry": {
    "maximum_step_rotation_rad": 0.00863476508789418,
    "maximum_step_translation_m": 0.012718745890754711,
    "net_displacement_m": 61.0894683942134,
    "net_yaw_rad": 1.8448600576950434,
    "p99_step_rotation_rad": 0.007308873235322788,
    "p99_step_translation_m": 0.011846116599229366,
    "step_rotation_over_0_1_rad": 0,
    "step_translation_over_0_1_m": 0,
    "total_path_m": 174.5476486718512
  },
  "path": "/home/slam/slam_ws/bags/corridor_repeat_02.bag",
  "point_cloud": {
    "longest_sparse_interval_s": 2.8999950885772705,
    "maximum_points": 21027,
    "median_points": 14319,
    "minimum_points": 1557,
    "p01_points": 2486.36,
    "p05_points": 3452.8,
    "sparse_frame_count": 110,
    "sparse_limit_points": 2863.8
  },
  "readable": true,
  "start_time": 1784864340.5508814,
  "stationary_check": {
    "final_10s": {
      "max_abs_angular_radps": 0.009999999776482582,
      "max_abs_linear_mps": 0.004000000189989805,
      "mean_abs_angular_radps": 9.980039697088405e-05,
      "mean_abs_linear_mps": 3.992016157674456e-05
    },
    "initial_10s": {
      "max_abs_angular_radps": 0.0,
      "max_abs_linear_mps": 0.0,
      "mean_abs_angular_radps": 0.0,
      "mean_abs_linear_mps": 0.0
    }
  },
  "tf": {
    "pairs_and_publishers": {
      "base_link -> base_footprint": [
        "/robot_state_publisher"
      ],
      "base_link -> camera_link": [
        "/camera2base"
      ],
      "base_link -> front_left_wheel_link": [
        "/robot_state_publisher"
      ],
      "base_link -> front_right_wheel_link": [
        "/robot_state_publisher"
      ],
      "base_link -> inertial_link": [
        "/robot_state_publisher"
      ],
      "base_link -> laser_link": [
        "/newscan2base"
      ],
      "base_link -> rear_left_wheel_link": [
        "/robot_state_publisher"
      ],
      "base_link -> rear_right_wheel_link": [
        "/robot_state_publisher"
      ],
      "laser_link -> scan_link": [
        "/laserscan2lasercenter"
      ],
      "odom -> base_link": [
        "/scout_base_node"
      ]
    },
    "static_transforms": [
      {
        "caller": "/camera2base",
        "child": "camera_link",
        "parent": "base_link",
        "translation": [
          0.08,
          0.0,
          0.12
        ],
        "yaw_rad": 0.0
      },
      {
        "caller": "/newscan2base",
        "child": "laser_link",
        "parent": "base_link",
        "translation": [
          0.0,
          0.0,
          0.17
        ],
        "yaw_rad": -1.57
      },
      {
        "caller": "/laserscan2lasercenter",
        "child": "scan_link",
        "parent": "laser_link",
        "translation": [
          0.0,
          0.0,
          0.0
        ],
        "yaw_rad": 1.571592653589793
      },
      {
        "caller": "/robot_state_publisher",
        "child": "base_footprint",
        "parent": "base_link",
        "translation": [
          0.0,
          0.0,
          -0.23479
        ],
        "yaw_rad": 0.0
      },
      {
        "caller": "/robot_state_publisher",
        "child": "inertial_link",
        "parent": "base_link",
        "translation": [
          0.0,
          0.0,
          0.0
        ],
        "yaw_rad": 0.0
      }
    ]
  },
  "timing": {
    "/odom": {
      "count": 27769,
      "frequency_hz": 49.99999792731768,
      "max_gap_s": 0.021005868911743164,
      "non_monotonic_count": 0,
      "p95_gap_s": 0.02006840705871582
    },
    "/tf": {
      "count": 33323,
      "frequency_hz": 60.00071694200604,
      "max_gap_s": 0.021184206008911133,
      "non_monotonic_count": 0,
      "p95_gap_s": 0.020147550106048583
    },
    "/tf_static": {
      "count": 4,
      "frequency_hz": 119.06165550130578,
      "max_gap_s": 0.010436058044433594,
      "non_monotonic_count": 0,
      "p95_gap_s": 0.010277295112609863
    },
    "/velodyne_points": {
      "count": 5553,
      "frequency_hz": 9.999796905162617,
      "max_gap_s": 0.10172390937805176,
      "non_monotonic_count": 0,
      "p95_gap_s": 0.10011887550354004
    }
  },
  "topics": {
    "/odom": {
      "messages": 27769,
      "type": "nav_msgs/Odometry"
    },
    "/scout_status": {
      "messages": 27769,
      "type": "scout_msgs/ScoutStatus"
    },
    "/tf": {
      "messages": 33323,
      "type": "tf2_msgs/TFMessage"
    },
    "/tf_static": {
      "messages": 4,
      "type": "tf2_msgs/TFMessage"
    },
    "/velodyne_points": {
      "messages": 5553,
      "type": "sensor_msgs/PointCloud2"
    }
  },
  "warnings": [
    "missing recommended topic: /cmd_vel",
    "point cloud stays below 20% of median for more than 1 s"
  ]
}
```

## corridor_repeat_03.bag

Warnings: missing recommended topic: /cmd_vel; point cloud stays below 20% of median for more than 1 s

```json
{
  "accepted_for_slam": true,
  "duration_s": 578.8144693374634,
  "end_time": 1784865596.6257927,
  "errors": [],
  "file_size_bytes": 1306380042,
  "odometry": {
    "maximum_step_rotation_rad": 0.00883935242582723,
    "maximum_step_translation_m": 0.013142422463923952,
    "net_displacement_m": 60.89842623103738,
    "net_yaw_rad": 1.9019498441775085,
    "p99_step_rotation_rad": 0.006847670261075391,
    "p99_step_translation_m": 0.01125839099419097,
    "step_rotation_over_0_1_rad": 0,
    "step_translation_over_0_1_m": 0,
    "total_path_m": 173.72046677799239
  },
  "path": "/home/slam/slam_ws/bags/corridor_repeat_03.bag",
  "point_cloud": {
    "longest_sparse_interval_s": 4.400175094604492,
    "maximum_points": 20973,
    "median_points": 13719.0,
    "minimum_points": 1116,
    "p01_points": 2181.74,
    "p05_points": 3061.7000000000003,
    "sparse_frame_count": 158,
    "sparse_limit_points": 2743.8
  },
  "readable": true,
  "start_time": 1784865017.8113234,
  "stationary_check": {
    "final_10s": {
      "max_abs_angular_radps": 0.2919999957084656,
      "max_abs_linear_mps": 0.024000000208616257,
      "mean_abs_angular_radps": 0.07092814325185831,
      "mean_abs_linear_mps": 0.0020099800431398218
    },
    "initial_10s": {
      "max_abs_angular_radps": 0.0,
      "max_abs_linear_mps": 0.0,
      "mean_abs_angular_radps": 0.0,
      "mean_abs_linear_mps": 0.0
    }
  },
  "tf": {
    "pairs_and_publishers": {
      "base_link -> base_footprint": [
        "/robot_state_publisher"
      ],
      "base_link -> camera_link": [
        "/camera2base"
      ],
      "base_link -> front_left_wheel_link": [
        "/robot_state_publisher"
      ],
      "base_link -> front_right_wheel_link": [
        "/robot_state_publisher"
      ],
      "base_link -> inertial_link": [
        "/robot_state_publisher"
      ],
      "base_link -> laser_link": [
        "/newscan2base"
      ],
      "base_link -> rear_left_wheel_link": [
        "/robot_state_publisher"
      ],
      "base_link -> rear_right_wheel_link": [
        "/robot_state_publisher"
      ],
      "laser_link -> scan_link": [
        "/laserscan2lasercenter"
      ],
      "odom -> base_link": [
        "/scout_base_node"
      ]
    },
    "static_transforms": [
      {
        "caller": "/camera2base",
        "child": "camera_link",
        "parent": "base_link",
        "translation": [
          0.08,
          0.0,
          0.12
        ],
        "yaw_rad": 0.0
      },
      {
        "caller": "/newscan2base",
        "child": "laser_link",
        "parent": "base_link",
        "translation": [
          0.0,
          0.0,
          0.17
        ],
        "yaw_rad": -1.57
      },
      {
        "caller": "/laserscan2lasercenter",
        "child": "scan_link",
        "parent": "laser_link",
        "translation": [
          0.0,
          0.0,
          0.0
        ],
        "yaw_rad": 1.571592653589793
      },
      {
        "caller": "/robot_state_publisher",
        "child": "base_footprint",
        "parent": "base_link",
        "translation": [
          0.0,
          0.0,
          -0.23479
        ],
        "yaw_rad": 0.0
      },
      {
        "caller": "/robot_state_publisher",
        "child": "inertial_link",
        "parent": "base_link",
        "translation": [
          0.0,
          0.0,
          0.0
        ],
        "yaw_rad": 0.0
      }
    ]
  },
  "timing": {
    "/odom": {
      "count": 28940,
      "frequency_hz": 50.00000224750652,
      "max_gap_s": 0.02048516273498535,
      "non_monotonic_count": 0,
      "p95_gap_s": 0.02007126808166504
    },
    "/tf": {
      "count": 34729,
      "frequency_hz": 59.99999011889438,
      "max_gap_s": 0.02147698402404785,
      "non_monotonic_count": 0,
      "p95_gap_s": 0.020143663883209227
    },
    "/tf_static": {
      "count": 4,
      "frequency_hz": 202.24234533969815,
      "max_gap_s": 0.006054878234863281,
      "non_monotonic_count": 0,
      "p95_gap_s": 0.006038808822631836
    },
    "/velodyne_points": {
      "count": 5788,
      "frequency_hz": 9.99978619744727,
      "max_gap_s": 0.10140681266784668,
      "non_monotonic_count": 0,
      "p95_gap_s": 0.10012006759643555
    }
  },
  "topics": {
    "/odom": {
      "messages": 28940,
      "type": "nav_msgs/Odometry"
    },
    "/scout_status": {
      "messages": 28942,
      "type": "scout_msgs/ScoutStatus"
    },
    "/tf": {
      "messages": 34729,
      "type": "tf2_msgs/TFMessage"
    },
    "/tf_static": {
      "messages": 4,
      "type": "tf2_msgs/TFMessage"
    },
    "/velodyne_points": {
      "messages": 5788,
      "type": "sensor_msgs/PointCloud2"
    }
  },
  "warnings": [
    "missing recommended topic: /cmd_vel",
    "point cloud stays below 20% of median for more than 1 s"
  ]
}
```
