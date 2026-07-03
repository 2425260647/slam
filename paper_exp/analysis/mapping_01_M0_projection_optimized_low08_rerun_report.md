# Mapping 01 M0 Projection Optimized Low08 Rerun

## Output
- Map: `paper_exp/maps/mapping_01/M0_projection_optimized_low08_rerun/my_map.pgm`
- YAML: `paper_exp/maps/mapping_01/M0_projection_optimized_low08_rerun/my_map.yaml`
- Metrics: `paper_exp/analysis/mapping_01_M0_projection_optimized_low08_rerun_metrics.json`
- Compare: `paper_exp/analysis/mapping_01_M0_projection_optimized_low08_rerun_compare.png`

## Metrics
- Wall continuity: `0.814954276492738`
- Free connectivity: `0.9980631052796001`
- Noise density: `4.57763671875e-05`
- Boundary sharpness: `234.84619140625`
- Unknown ratio: `0.931854248046875`
- Occupied cells: `1859`

## Comparison With Original v1
- The rerun map is byte-identical to the original `M0_projection_optimized` map.
- `sha256` of both `.pgm` files is the same.
- Metrics are also identical.

## Interpretation
This rerun confirms that the original `0.08 m` chain is reproducible. The current `0.08 m` result is not caused by a broken rerun or save error. It matches the original v1 output exactly, so any difference you see comes from changing thresholds, not from nondeterministic replay in this run.
