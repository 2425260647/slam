# Retained failed attempt

The initial C0 command set only `use_confidence=false`. Because the launch
condition for the baseline node used an invalid string expression, no
`pointcloud_to_laserscan` process started and Cartographer waited for `/scan`.
This run is intentionally retained as a failure record. The launch now uses
the explicit `use_baseline_projection=true` argument.
