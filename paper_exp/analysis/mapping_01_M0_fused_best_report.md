# Mapping 01 M0 Fused Best

## Output
- Map: `paper_exp/maps/mapping_01/M0_fused_best/my_map.pgm`
- YAML: `paper_exp/maps/mapping_01/M0_fused_best/my_map.yaml`
- Metrics: `paper_exp/analysis/mapping_01_M0_fused_best_metrics.json`
- Fusion script: `src/pointcloud_to_grid/scripts/fuse_projection_maps.py`

## Fusion Goal
This map is a fused result from three M0 variants:

- `M0_projection_optimized`: preferred for obstacle shape preservation.
- `M0_projection_optimized_v2`: preferred for wall continuity.
- `M0_baseline`: preferred for retaining all visible obstacles.

## Fusion Rule
- Use the largest connected occupied structure from `M0_projection_optimized_v2` as the main wall structure.
- Add medium and small obstacle components from `M0_projection_optimized` to preserve obstacle shapes.
- Add missing medium and small obstacle components from `M0_baseline` to avoid removing real obstacles.
- Remove tiny occupied components below 3 cells as noise.
- Use majority voting across the three maps for free space.

## Metrics
- Wall continuity: `0.910833704859563`
- Free connectivity: `0.9972495842394781`
- Noise density: `0.0`
- Boundary sharpness: `222.01502990722656`
- Unknown ratio: `0.9318046569824219`
- Occupied cells: `2243`
- Free cells: `15634`

## Interpretation
The fused map intentionally keeps more independent obstacle components than `M0_projection_optimized_v2`. Because the current `wall_continuity` metric divides the largest occupied component by all occupied cells, retaining real independent obstacles can reduce the numeric continuity score. Therefore, this fused map should be judged by both visual inspection and metrics: it is designed to preserve obstacle shape, retain baseline obstacles, and keep the v2 continuous wall structure.
