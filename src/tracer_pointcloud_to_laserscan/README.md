# Tracer cloud-to-scan transport fix

Based on ros-perception/pointcloud_to_laserscan **2.0.1**, commit
`67e158ef4042ba05022acc765a6f71b32c207ac7`. Original BSD notices are retained
in the source and headers. Only the cloud-to-scan component is included.

Local changes:
- Separate package, namespace and library; the system package is unchanged.
- RELIABLE cloud subscription and scan publisher, depth 1. The cloud input must
  come from a RELIABLE publisher (the configured Velodyne publisher does).
- Queue depth defaults to 1 instead of CPU core count, avoiding old-cloud queues.
- Guard the angle-bin array bound, including an exact upper-angle endpoint.

Height, angle, range, target-frame and timestamp behavior are inherited from
upstream. The sensor launch retains its existing filtering parameters. This
change does not restore points removed upstream by Velodyne's min_range.

Build with `colcon build --packages-select tracer_pointcloud_to_laserscan tracer_base path_planner_server`
and source the workspace before restarting sensors and navigation. Building
alone does not replace an already running sensor converter.
