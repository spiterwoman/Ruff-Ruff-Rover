## Info
- `setup_dev.sh`: Source ROS 2, build the workspace with `colcon`, and print the command needed to source the overlay in your current shell.
- `record_bag.sh`: Record either a default rover topic set or all topics into `data/bags/<timestamp>`.
- `replay_bag.sh`: Replay a chosen bag or automatically pick the newest one in `data/bags`.

## Usage
Build the workspace:

```bash
bash ./scripts/setup_dev.sh
```

Build selected packages only:

```bash
bash ./scripts/setup_dev.sh --packages-select rover_base rover_behavior
```

Record a focused rover bag:

```bash
bash ./scripts/record_bag.sh
```

Record every topic:

```bash
bash ./scripts/record_bag.sh --all-topics
```

Replay the newest bag:

```bash
bash ./scripts/replay_bag.sh
```

Replay a specific bag faster and in a loop:

```bash
bash ./scripts/replay_bag.sh --bag-path ./data/bags/20260322_103000 --rate 1.5 --loop
