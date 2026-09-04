# runner smoke C2 retry

This 5 s C2 case-runner smoke test uses `corridor_repeat_01.bag`,
`require_ring=true`, `forward_selected_scans=true`, and `selection_score=0.76`.
The corrected runner generated 49 projection/selection decisions, forwarded
6 scans (12.24%), and achieved 100% quality timestamp synchronization. Mean
projection and selector processing times were 1.95 ms and 0.059 ms,
respectively. This is an interface/reproducibility smoke result, not an
accuracy claim; the real bag has no continuous external pose truth.
