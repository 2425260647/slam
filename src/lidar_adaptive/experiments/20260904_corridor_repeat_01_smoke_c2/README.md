# corridor_repeat_01 smoke C2

This is a short real-bag external scan-thinning smoke test using the first
15 s of `bags/corridor_repeat_01.bag` with `rosbag play --clock`. It uses
confidence projection, `require_ring=true`, and
`forward_selected_scans=true`; the explicit command is in `logs/command.txt`.

The run verifies that selected scans can be forwarded to Cartographer without
node failure and that every decision remains timestamped in `/scan_selection`.
The bag has no continuous external pose ground truth, so the 10.7% selection
fraction is an implementation observation, not an accuracy or generalization
claim. No ATE/RPE is reported.
