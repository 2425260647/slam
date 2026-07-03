# A-LOAM mapping_01 Evaluation

## Conclusion

- Verdict: `fail_or_needs_calibration`
- Main issues: trajectory has discontinuous jumps, large Z drift for a ground robot, large roll/pitch variation, revisited positions are inconsistent in height

## Back-end Mapped Pose `/aft_mapped_to_init`

- Samples: 2496
- Duration: 249.50 s
- XY path length: 4879.813 m
- Z range: 211.163 m
- Z final-start: -165.159 m
- Max XY step: 12.250 m
- Max Z step: 15.373 m
- Roll max abs: 179.876 deg
- Pitch max abs: 87.105 deg
- Jump counts: xy>1m=1526, z>0.3m=1594, yaw>20deg=1228

## Front-end Odom `/laser_odom_to_init`

- XY path length: 6311.380 m
- Z range: 0.000 m
- Z final-start: 0.000 m
- Roll max abs: 0.000 deg
- Pitch max abs: 0.000 deg

## Revisit Consistency

- Pair count: 5
- XY offset P95: 0.718 m
- Z offset P95: 19.436 m
- Yaw offset P95: 169.734 deg

## Final Map Cloud `/laser_cloud_map`

- Points: 276492
- X/Y/Z range: 238.699 / 532.799 / 217.244 m
- Z 1%-99%: -199.199 to 0.562 m
- Occupied 0.2m cells: 117033
- Dense cell ratio >=3 pts: 0.300
