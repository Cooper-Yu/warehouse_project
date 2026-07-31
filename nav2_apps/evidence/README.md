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
