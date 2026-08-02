# Slice 1 Curated Evidence

Raw command output is written under `nav2_apps/runtime_logs/` and ignored by
Git. Copy only a bounded, reviewed summary into this directory.

Each summary must record:

- exact command and environment/profile;
- UTC timestamp, commit SHA, and `git status --short`;
- relevant lifecycle, topic/type, and parameter evidence;
- goal acceptance and result timeline;
- key errors, final pose, and cleanup state;
- whether the result is local engineering evidence or official The Construct
  acceptance.

Do not commit credentials, private data, cloud secrets, raw rosbags, or
uncontrolled logs.

`collect_motion_evidence` creates a bounded raw run under the ignored
`nav2_apps/runtime_logs/motion_<UTC>/` directory. It samples map/odom poses,
velocity commands, Nav2 feedback/recovery count, path sizes, and selected ROS
events while wrapping one mission command. Review `summary.md`, `motion.csv`,
`events.csv`, and the process logs; copy only a compact reviewed conclusion
into this evidence directory.
