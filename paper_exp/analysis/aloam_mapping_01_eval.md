# A-LOAM mapping_01 Evaluation

## Conclusion

- Verdict: `pass_with_caution`
- Main issues: none obvious from numeric checks

## Back-end Mapped Pose `/aft_mapped_to_init`

- Samples: 2495
- Duration: 249.40 s
- XY path length: 30.685 m
- Z range: 0.689 m
- Z final-start: -0.008 m
- Max XY step: 0.050 m
- Max Z step: 0.035 m
- Roll max abs: 7.706 deg
- Pitch max abs: 9.229 deg
- Jump counts: xy>1m=0, z>0.3m=0, yaw>20deg=0

## Front-end Odom `/laser_odom_to_init`

- XY path length: 24.076 m
- Z range: 7.425 m
- Z final-start: 4.939 m
- Roll max abs: 179.419 deg
- Pitch max abs: 76.568 deg

## Revisit Consistency

- Pair count: 5002
- XY offset P95: 0.955 m
- Z offset P95: 0.071 m
- Yaw offset P95: 179.590 deg

## Final Map Cloud `/laser_cloud_map`

- Points: 2634
- X/Y/Z range: 10.099 / 9.236 / 1.707 m
- Z 1%-99%: -1.208 to 0.417 m
- Occupied 0.2m cells: 631
- Dense cell ratio >=3 pts: 0.707
