# Gazebo corridor elevator smoke

This run starts `corridor_elevator.world`, publishes a constant forward
velocity, and records the research chain together with Gazebo model truth.
The fixed bag contains 74 LiDAR frames, 370 controller odometry messages,
7397 `/gazebo/model_states` messages, and 100% timestamp-matched selection
quality. The selector forwarded 17 of 74 scans (22.97%) with the current
threshold configuration.

This is a data-path and truth-capture smoke test, not a completed accuracy
experiment. The model states make ATE/RPE computation permissible for this bag,
but those metrics have not yet been computed. The four missing PID-gain
warnings are emitted by the shared Gazebo controller and did not stop the
simulation; they must be recorded and treated consistently in later runs.

The first trajectory computation on `gazebo_smoke_fixed.bag` produced an ATE
translation RMSE of 0.0588 m (P95 0.0777 m), ATE rotation RMSE of 0.00150 rad,
and 1 s RPE translation RMSE of 0.0258 m. These are single-run smoke values,
not a comparison or a statistical claim; the full experiment requires fixed
motion scripts, C0/C1/C2 repeats, and mean/std reporting.
