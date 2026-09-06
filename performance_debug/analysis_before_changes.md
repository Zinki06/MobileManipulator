# Performance analysis before editing (robot motion prohibited)

Pipeline: project_bringup starts OpenCR ros2_control100Hz, camera, C++ ArUco
localization30Hz TF, Nav2 controller20Hz/planner1Hz BT/local map5Hz, motion_executor
Python50Hz action supervision/encoder turns, smoother -> collision_monitor ->
C++motion_guard50Hz -> /cmd_vel -> drive. Cleanup C++ state machine handles station
scan, request-driven Python YOLO/SAM, Python Gemini HTTP, C++ reachability/pick/place.
Perception, control, diagnostics and video are separate processes. Arm services are
reentrant/MultiThreaded; motion executor6 threads. Python sensor/perception operations
mostly call native NumPy/OpenCV/Torch; blanket C++ conversion is not warranted.

No robot nodes are currently running. Per-node live CPU cannot be measured honestly.
Stored telemetry has4Hz sampling and cannot establish full sensor publication rates.
CPU/process rates and callback timing will be instrumented for the next user-run task.

Measured baseline (performance_debug/baseline.json):
- Latest run183108:16 turns, mean45deg5.0497s;16 captures mean1.7229s.
- Two empty eight-heading stations:54.66s/60.02s (mean57.34s).
- Measured translation median.07698m/s, p90.08271; commandedStationPath .08.
- Motion spin policy hardcaps .30 and uses proportional gain1 near goal; guard/hardware
  independently cap .35rad/s and .10m/s with accel .4rad/s2/.15m/s2.
- Offline saved1280x720 RGB-D: NumPy depth projection median7.32msCPU7.28ms;
  body candidate generation716ms/812CPUms; planner subprocess94.5mswall/2msparentCPU.
- body_candidates recalculates full-frame grids/erode/deprojection/depth conversion and
  local plane masks for every body seed; high payoff from cache and bounded ROI.
- depth safety publishes at~5Hz due180ms gate, independently receiving30Hz images.
  C++ conversion merits trial benchmarking: reduce allocations/serialization and improve
  safety source cadence, preserving topic/encoding/strides/validity gates.
- ArUco detects every RGB frame in one executor and copies1280x720 BGR before detection;
  can backlog timer/odom, while correction already rejects rapid turning frames.
- Fixed3s park even when already parked;500ms startup wait;200ms pub->Trigger delay;
  turn readiness300ms plussettle250ms plusmanager350ms. Replace redundant waits with
  fresh state/result conditions, retain stabilization and timeout failures.
- Linear slow-down begins30cm before goal; corners already curvature/cost regulated.
- No evidence that missing dynamic costmap avoidance or changing mission scan coverage
  improves safety: keep static-map test policy, collision checks,8 headings,5negative frames.
- Gemini HTTP~3s in prior logs but task semantics require planner; retain external decision.

Order: coherent speed/accel/approach profile + braking validation; turn convergence and
wait duplication; bounded joint timing/no-op; vectorized candidate caches; benchmarked
C++depth feed; latest-only QoS/rate-limited ArUco; passive profiling and regression tests.
All hardware/camera/segmentation/vendor manipulation sources remain untouched.
