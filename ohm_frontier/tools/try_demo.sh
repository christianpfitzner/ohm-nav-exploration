#!/usr/bin/env bash
# One demo, one hall, one measurement — the whole experiment in one command.
#
#   tools/try_demo.sh <launch file> <robot> <domain id> [seconds] [extra launch args…]
#
#     tools/try_demo.sh reactive_wall_follow.launch.py wally 61 45 world:=production
#
# Six tool calls become one: start the stack headless, give it 15 s to spawn, measure it with
# `measure_drive.py`, stop the whole process group, and print the last thing the node said — which is the
# pair of numbers a demo is judged on: what it did (metres) and what it thought it was doing (its own line).
#
# The domain id is an argument rather than an environment variable because two people can then verify two
# demos on one machine at the same time without their DDS discovery crossing; one domain per experiment,
# and a robot name nobody else is using. `SIM_DIR` overrides the simulator's home; `HALL_SECONDS` the 15 s
# start-up wait, which is generous for `production` (a 120 × 40 m map takes a few seconds to appear).
#
# Exit status is the measurement's, not the stack's: killing a launch on purpose is not a failure.
set -uo pipefail

if [ $# -lt 3 ]; then
  echo "usage: $0 <launch file> <robot> <domain id> [seconds] [extra launch args…]" >&2
  exit 2
fi

launch_file=$1
robot=$2
export ROS_DOMAIN_ID=$3
seconds=${4:-45}
shift 4

sim_dir=${SIM_DIR:-/home/pfitzner/git/mecanum-lab}
start_wait=${HALL_SECONDS:-15}
tools=$(cd "$(dirname "$0")" && pwd)
log=/tmp/try_${robot}.log
pidfile=/tmp/try_${robot}.pid

# Sourced here so the command at the top works in a shell that has nothing loaded: ROS itself, then this
# checkout's own overlay if someone has built it. A caller that sourced something else first keeps what it
# sourced — this only runs when `ros2` is missing, which is the difference between "wrong tree" and "no tree".
if ! command -v ros2 >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  . "/opt/ros/${ROS_DISTRO:-kilted}/setup.bash" 2>/dev/null || true
  workspace=$(cd "$tools/../.." && pwd)        # the checkout that holds this file, which is where its install/ is
  # shellcheck disable=SC1091
  [ -f "$workspace/install/setup.bash" ] && . "$workspace/install/setup.bash"
fi
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 is not on PATH and /opt/ros/${ROS_DISTRO:-kilted} did not provide it — nothing was started" >&2
  exit 1
fi

echo "## $(date -u +%H:%M:%S) ${launch_file} as ${robot} on domain ${ROS_DOMAIN_ID} for ${seconds}s"
echo "## ohm_frontier from: $(ros2 pkg prefix ohm_frontier 2>/dev/null || echo 'NOT IN THIS WORKSPACE — build it first')"
echo "## extra args: ${*:-none}   full log: ${log}"

setsid bash -c "echo \$\$ > '$pidfile'; exec ros2 launch ohm_frontier '$launch_file' \
    robot:='$robot' headless:=true sim_dir:='$sim_dir' $*" >"$log" 2>&1 &
for _ in $(seq 40); do [ -s "$pidfile" ] && break; sleep 0.25; done
pgid=$(cat "$pidfile" 2>/dev/null); rm -f "$pidfile"

sleep "$start_wait"
if ! grep -q "$robot/scan" <(timeout 5 ros2 topic list 2>/dev/null); then
  echo "!! the stack never came up — last 15 lines:"; tail -15 "$log"
  [ -n "$pgid" ] && kill -KILL -"$pgid" 2>/dev/null
  exit 1
fi

python3 "$tools/measure_drive.py" "$robot" "$seconds" "/tmp/drive_${robot}.csv"
status=$?

echo "## what the node said while that was happening (last 10 of its own lines)"
grep -vE '^\[(launch|rviz2|slam_toolbox|nav2_|robot_state_publisher)' "$log" | grep -vE '^\[INFO\] \[' | tail -10

[ -n "$pgid" ] && { kill -TERM -"$pgid" 2>/dev/null; sleep 2; kill -KILL -"$pgid" 2>/dev/null; }
exit $status
