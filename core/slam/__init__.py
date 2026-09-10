"""
core/slam/

2D lidar SLAM: the pose estimate becomes a consequence of what the robot did.

Epic 03, chunks 21-23. Built bottom-up and each piece is testable alone:

    scan.py        what a scan IS -- endpoints in the robot frame.
    likelihood.py  a blurred occupancy likelihood field, precomputed per submap.
    matcher.py     correlative scan matching: two scans -> a relative pose.
    submap.py      N consecutive scans in one relocatable frame.
    loop.py        loop closure: proposing it, and mostly refusing it.
    tracker.py     noisy odometry, corrected by matching. The front end.
    graph.py       the pose graph, and the Gauss-Newton that applies a closure.
    localiser.py   SlamLocaliser -- the assembly, under chunk 19's protocol.

⚠️ **Every algorithmic choice here is made in favour of DETERMINISM over
accuracy**, which is epic 03's stated rule. Same-machine bitwise determinism is
a committed property of this project (`scripts/check_determinism.sh`), and SLAM
is exactly where it goes wrong: convergence tolerances, variable iteration
counts, RANSAC, dict iteration order, threading and BLAS parallelism. So there
is no convergence tolerance in this package, no random restart, no iterate-until-
small, and every tie is broken by a stated rule rather than by whatever the
array happened to contain.

SIM-FREE. No isaacsim/omni/torch/pxr/carb, enforced by
core/tests/test_import_guard.py.
"""
