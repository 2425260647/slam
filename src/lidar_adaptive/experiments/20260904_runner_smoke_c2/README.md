# runner smoke C2 (failed post-processing)

The case runner replayed and recorded the 5 s C2 bag, but the first version
looked for installed Python files below the package `share` directory and
failed during post-processing. The run directory is retained as a failure
record. The corrected runner is validated in
`20260904_runner_smoke_c2_retry`.
