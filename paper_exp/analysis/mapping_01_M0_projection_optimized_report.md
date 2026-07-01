# Mapping 01 M0 Projection Optimized

## Output
- Map: `paper_exp/maps/mapping_01/M0_projection_optimized/my_map.pgm`
- YAML: `paper_exp/maps/mapping_01/M0_projection_optimized/my_map.yaml`
- Metrics: `paper_exp/analysis/mapping_01_M0_projection_optimized_metrics.json`

## Metrics
- Wall continuity: `0.814954276492738`
- Free connectivity: `0.9980631052796001`
- Noise density: `4.57763671875e-05`
- Boundary sharpness: `234.84619140625`
- Unknown ratio: `0.931854248046875`

## Comparison With Baseline
- Occupied cells: `2117 -> 1859`
- Free cells: `15959 -> 16005`
- Wall continuity: `0.9267831837505904 -> 0.814954276492738`
- Free connectivity: `0.9957390813960775 -> 0.9980631052796001`
- Noise density: `1.52587890625e-05 -> 4.57763671875e-05`

## Interpretation
This M0 is the source-side projection optimized map. It reduces some occupied clutter and slightly improves free-space connectivity, but wall continuity drops, so it still needs follow-up structure repair before it can serve as the strongest paper result.
