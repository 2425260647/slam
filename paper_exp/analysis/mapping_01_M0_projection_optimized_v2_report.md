# Mapping 01 M0 Projection Optimized V2

## Output
- Map: `paper_exp/maps/mapping_01/M0_projection_optimized_v2/my_map.pgm`
- YAML: `paper_exp/maps/mapping_01/M0_projection_optimized_v2/my_map.yaml`
- Metrics: `paper_exp/analysis/mapping_01_M0_projection_optimized_v2_metrics.json`
- Log: `/tmp/ros_logs/innovation1_m0_v2.log`

## Algorithm Change
- Keep the nearest valid obstacle point for each scan angle bin.
- Bridge only short angular gaps when neighboring ranges are close.
- Keep the original ground filtering and voxel downsampling pipeline.

## Metrics
- Wall continuity: `0.9429124334784712`
- Free connectivity: `0.9965067681367351`
- Noise density: `2.6702880859375e-05`
- Boundary sharpness: `238.13693237304688`
- Unknown ratio: `0.9309616088867188`
- Occupied cells: `2067`
- Free cells: `16031`

## Comparison
- Baseline wall continuity: `0.9267831837505904`
- V1 wall continuity: `0.814954276492738`
- V2 wall continuity: `0.9429124334784712`
- V1 occupied cells: `1859`
- V2 occupied cells: `2067`

## Interpretation
V2 improves wall continuity and restores more occupied structure than V1. This supports the diagnosis that V1 removed too many sparse but valid wall and obstacle points.
