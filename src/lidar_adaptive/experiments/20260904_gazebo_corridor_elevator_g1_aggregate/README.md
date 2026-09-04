# Experiment aggregate

This report aggregates the JSON summaries in the listed run directories.
Real Scout corridor bags have no continuous external truth; this report
does not contain ATE/RPE claims for those bags.

| Case | Runs | Scan messages (mean) | Selection fraction (mean) | ATE RMSE (mean, m) | 1 s RPE RMSE (mean, m) |
| --- | ---: | ---: | ---: | ---: | ---: |
| C0 | 3 | 186.333333 | n/a | 0.099453 | 0.020209 |
| C1 | 3 | 188.000000 | 0.521242 | 0.032309 | 0.016383 |
| C2 | 3 | 92.333333 | 0.493808 | 0.059332 | 0.029111 |

Relative changes use `(case - C0) / abs(C0)` on the means. A negative
value is lower than C0 for that metric; it is not by itself evidence of
statistical significance or generalization.

```json
{
  "C1": {
    "ate_rotation_rmse_rad": 1.9931374679486826,
    "ate_translation_p95": -0.6276731423475471,
    "ate_translation_rmse": -0.6751354598297714,
    "rpe_rotation_rmse_rad": 0.44524924349154404,
    "rpe_translation_rmse": -0.1892921023663011
  },
  "C2": {
    "ate_rotation_rmse_rad": 1.7087789373620739,
    "ate_translation_p95": -0.40775706170461196,
    "ate_translation_rmse": -0.40341204882590254,
    "rpe_rotation_rmse_rad": 0.9847275398383945,
    "rpe_translation_rmse": 0.4405284954626841
  }
}
```
