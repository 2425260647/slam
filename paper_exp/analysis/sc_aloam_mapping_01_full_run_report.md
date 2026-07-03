# SC-A-LOAM mapping_01 Full Run Report

## Run

- Bag: `paper_exp/bags/mapping_01.bag`
- Output: `paper_exp/sc_aloam/mapping_01_full_run/`
- Keyframes: 161
- Pose graph vertices: 161
- Pose graph edges: 161
- Loop closure edges: 1
- Loop edge: keyframe `8 -> 155`

## Trajectory Metrics

### Original A-LOAM Back-End Keyframe Trajectory

- End XYZ: `(1.856720, 0.239985, 0.019926)` m
- XY displacement from start to end: `1.872165` m
- Z range: `0.467552` m
- Final Z offset: `0.019926` m

### SC-A-LOAM Optimized Trajectory

- End XYZ: `(1.853760, 0.239809, 0.019826)` m
- XY displacement from start to end: `1.869207` m
- Z range: `0.467579` m
- Final Z offset: `0.019826` m

## Optimization Effect

- Max 3D correction: `0.002967` m
- Mean 3D correction: `0.001404` m
- Keyframes corrected by more than 1 cm: `0`
- Keyframes corrected by more than 5 cm: `0`
- Keyframes corrected by more than 10 cm: `0`

## Judgement

SC-A-LOAM successfully detected and inserted one loop closure edge, but the correction is only millimeter-level. For `mapping_01.bag`, this means the original A-LOAM back-end trajectory is already close to the accepted loop constraint, so Scan Context did not produce a meaningful drift reduction in this run.
