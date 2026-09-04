# Experiment aggregate

This report aggregates the JSON summaries in the listed run directories.
Gazebo runs include continuous model-state truth; ATE/RPE are computed from the recorded truth.

| Case | Runs | Scan messages (mean) | Selection fraction (mean) | ATE RMSE (mean, m) | 1 s RPE RMSE (mean, m) |
| --- | ---: | ---: | ---: | ---: | ---: |
| C0 | 3 | 189.666667 | n/a | 0.046488 | 0.017972 |
| C1 | 3 | 184.000000 | 0.309771 | 0.062473 | 0.018558 |
| C2 | 3 | 53.333333 | 0.298325 | 0.145330 | 0.091728 |

Relative changes use `(case - C0) / abs(C0)` on the means. A negative
value is lower than C0 for that metric; it is not by itself evidence of
statistical significance or generalization.

```json
{
  "C1": {
    "ate_rotation_rmse_rad": 0.29128261997194455,
    "ate_translation_p95": 0.11673542785695354,
    "ate_translation_rmse": 0.34383165211843647,
    "rpe_rotation_rmse_rad": 0.5309276982086967,
    "rpe_translation_rmse": 0.032565019612016896
  },
  "C2": {
    "ate_rotation_rmse_rad": 1.1140927239359117,
    "ate_translation_p95": 2.977950182625496,
    "ate_translation_rmse": 2.1261519150280512,
    "rpe_rotation_rmse_rad": 1.619913633297178,
    "rpe_translation_rmse": 4.103803933086247
  }
}
```
