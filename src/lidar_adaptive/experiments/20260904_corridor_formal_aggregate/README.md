# Experiment aggregate

This report aggregates the JSON summaries in the listed run directories.
Real Scout corridor bags have no continuous external truth; ATE/RPE are intentionally unavailable.

| Case | Runs | Scan messages (mean) | Selection fraction (mean) | ATE RMSE (mean, m) | 1 s RPE RMSE (mean, m) |
| --- | ---: | ---: | ---: | ---: | ---: |
| C0 | 3 | 599.666667 | n/a | n/a | n/a |
| C1 | 3 | 599.666667 | 0.393683 | n/a | n/a |
| C2 | 3 | 235.666667 | 0.393128 | n/a | n/a |

Relative changes use `(case - C0) / abs(C0)` on the means. A negative
value is lower than C0 for that metric; it is not by itself evidence of
statistical significance or generalization.

```json
{
  "C1": {
    "ate_rotation_rmse_rad": null,
    "ate_translation_p95": null,
    "ate_translation_rmse": null,
    "rpe_rotation_rmse_rad": null,
    "rpe_translation_rmse": null
  },
  "C2": {
    "ate_rotation_rmse_rad": null,
    "ate_translation_p95": null,
    "ate_translation_rmse": null,
    "rpe_rotation_rmse_rad": null,
    "rpe_translation_rmse": null
  }
}
```
