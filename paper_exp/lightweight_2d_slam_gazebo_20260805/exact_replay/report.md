# Lightweight 2D SLAM exact-input loop-closure audit

## Purpose

This audit isolates loop closure from Gazebo trajectory variation. One Gazebo
run was recorded once and replayed into two fresh SLAM processes. The only
changed parameter was `enable_loop_closure`.

## Frozen input

- Bag: `square_inputs.bag`
- Duration: 70.55 s
- `/scan`: 705 messages
- `/scout_mini_velocity_controller/odom`: 3528 messages
- `/gazebo/model_states`: 7057 messages
- `/tf_static`: 1 message
- `/clock`: 7058 messages
- Bag size: 37.4 MB

Both scored replays used paused startup and covered the same interval from
approximately 1342.43 s to 1412.89 s. Each trajectory CSV contains 1455
synchronized samples. Earlier replays that missed the beginning of the bag
were excluded.

## Results

| Metric | Loop off | Loop on | Interpretation |
| --- | ---: | ---: | --- |
| Translation RMSE vs Gazebo | 0.037047 m | 0.036825 m | 0.60% lower; negligible |
| Translation maximum | 0.055078 m | 0.054376 m | 1.27% lower |
| Yaw RMSE vs Gazebo | 0.021564 rad | 0.021573 rad | 0.04% higher; negligible |
| Yaw maximum | 0.097830 rad | 0.097809 rad | effectively unchanged |
| Final translation error | 0.046839 m | 0.046814 m | effectively unchanged |
| Final yaw error | 0.001192 rad | 0.001182 rad | effectively unchanged |
| Mean scan processing | 13.090 ms | 13.253 ms | both real-time at 10 Hz |
| P95 scan processing | 20.603 ms | 20.696 ms | both real-time at 10 Hz |
| Maximum scan processing | 24.620 ms | 24.824 ms | no loop-search stall |
| Dropped scans | 0 | 0 | passed |
| Keyframes / submaps | 161 / 4 | 161 / 4 | identical |
| Accepted loop constraints | 0 | 2 | two confirmed submap-pair constraints |

The confirmed constraints were `1 -> 109` and `0 -> 159`. They were
consistent with the existing sequential trajectory, so they neither caused a
meaningful correction nor degraded tracking.

## Failure found and fixed

Before the fix, loop matching ran synchronously inside the scan callback and
accepted 12 repeated constraints in one short run. Its worst scan callback
reached 738 ms and yaw RMSE reached 0.1367 rad. The corrected implementation:

1. moves wide-window loop matching to a background worker;
2. uses a coarser bounded loop-search grid;
3. requires independent consistent confirmations;
4. allows only one constraint per submap pair and enforces an acceptance
   cooldown;
5. requires wheel odometry to confirm motion before creating a keyframe, so
   stationary scan jitter does not create an endless loop stream.

After the fix, the exact-input loop-on run had a maximum scan callback of
24.824 ms and no dropped scans.

## Map check

Both groups produced non-empty global occupancy grids at 0.05 m resolution.
Visual inspection of `no_loop/map.png` and `loop_enabled/map.png` found no new
wall duplication or incoherent deformation in the loop-on map. Occupied-cell
component counts changed from 269 to 211, but this is only a raster structure
proxy because map origins and dimensions differ slightly; it is not a
trajectory-accuracy metric.

## Strict conclusion

The minimal system now passes the ordinary-scene safety and real-time checks.
The repaired loop closure is no longer harmful on this input, but this short
small-area loop does not demonstrate a meaningful global-consistency gain.
A larger ordinary loop with accumulated drift, followed by a fair
Cartographer/GMapping comparison, is still required before claiming that the
new system matches mature SLAM algorithms. No degeneracy-robustness claim is
supported by this Gazebo audit.
