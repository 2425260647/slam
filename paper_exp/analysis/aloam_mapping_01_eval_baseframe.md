# A-LOAM mapping_01 Evaluation

## Conclusion

- Verdict: `pass_with_caution`
- Main issues: none obvious from numeric checks

## Back-end Mapped Pose `/aft_mapped_to_init`

- Samples: 2496
- Duration: 249.50 s
- XY path length: 29.247 m
- Z range: 0.483 m
- Z final-start: 0.021 m
- Max XY step: 0.048 m
- Max Z step: 0.033 m
- Roll max abs: 7.824 deg
- Pitch max abs: 6.791 deg
- Jump counts: xy>1m=0, z>0.3m=0, yaw>20deg=0

## Front-end Odom `/laser_odom_to_init`

- XY path length: 22.178 m
- Z range: 6.338 m
- Z final-start: 3.261 m
- Roll max abs: 160.606 deg
- Pitch max abs: 88.952 deg

## Revisit Consistency

- Pair count: 5056
- XY offset P95: 0.963 m
- Z offset P95: 0.111 m
- Yaw offset P95: 179.496 deg

## Final Map Cloud `/laser_cloud_map`

- Points: 2322
- X/Y/Z range: 9.267 / 10.160 / 1.616 m
- Z 1%-99%: -0.818 to 0.481 m
- Occupied 0.2m cells: 578
- Dense cell ratio >=3 pts: 0.716
