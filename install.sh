#!/usr/bin/env bash
# What this machine needs before "ros2 launch ohm_frontier explore.launch.py" does anything.
#
# Checking is the interesting half, because on a lab machine everything is installed and every failure is
# a *sourced* problem rather than an installed one. Two of those, both measured here: ROS 2 sitting under
# /opt/ros that the terminal has never sourced, so ros2 is not on the PATH and the package "does not
# exist"; and the simulator, which is not an installed ROS package at all but a checkout that
# explore.launch.py falls back to — so "not built" is a note, not a failure.
#
# Nothing here installs by itself. --apt and --pip print one command each, and only --yes runs them, so a
# student can read what would happen first. sudo is never called on a check run.
#
# Exit codes, so a teacher's wrapper can tell the classes apart. When several classes fail the script
# exits with the most fundamental one, because that is the only one worth fixing first:
#   0  everything needed is there         4  a ROS package one of the launch files includes is missing
#   2  bad option                         5  the python side (interpreter, numpy, pytest, pygame)
#   3  no ROS 2, or rclpy will not import 6  the simulator checkout missing, or older than this repo needs
#                                          7  the build did not work (--build)
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$here"

usage() {
  cat <<'TEXT'
usage: ./install.sh [option]...

  --check           look and explain, install nothing — this is also what happens with no option
  --apt             print the one sudo apt command for the ROS packages this repo's launch files need
  --pip             print the command that installs requirements.txt with pip --user
  --build           build the package: colcon build --symlink-install
  --yes             with --apt or --pip: run the command instead of only printing it
  --sim-dir=PATH    where the mecanum-lab checkout is (the same as MECANUM_LAB=..., default ~/git/mecanum-lab)

exit 0 ready · 2 bad option · 3 no ROS 2 · 4 ROS packages missing · 5 python side missing
     6 simulator missing · 7 the build failed
TEXT
}

do_apt=0; do_pip=0; do_build=0; assume_yes=0
sim_dir="${MECANUM_LAB:-$HOME/git/mecanum-lab}"
for arg in "$@"; do
  case "$arg" in
    --check) ;;
    --apt) do_apt=1 ;;
    --pip) do_pip=1 ;;
    --build) do_build=1 ;;
    --yes) assume_yes=1 ;;
    --sim-dir=*) sim_dir="${arg#*=}" ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $arg — ./install.sh --help" >&2
       exit 2 ;;
  esac
done

# Colour for a terminal only: this report is meant to be pasted into a message, and escape codes in a
# pasted report are worse than no report.
if [[ -t 1 ]]; then
  green=$'\033[32m'; red=$'\033[31m'; yellow=$'\033[33m'; off=$'\033[0m'
else
  green=""; red=""; yellow=""; off=""
fi

broken=0                                    # exit code of the most fundamental failure seen so far
classes=""                                  # every class that failed, so the summary can name the rest
ok() { printf '  %sok%s      %s\n' "$green" "$off" "$1"; }
missing() { printf '  %smissing%s %s\n' "$red" "$off" "$1"; }
# The three labels are two, four and seven letters wide, and the second line of an entry starts under the
# first one's text — a report that is not lined up in columns cannot be scanned in a lecture hall.
note() {
  printf '  %snote%s    %s\n' "$yellow" "$off" "$1"
  if [[ -n "${2:-}" ]]; then printf '          %s\n' "$2"; fi
}
# fail <code of this class> <what is missing> <what to do about it, on its own line>
fail() {
  missing "$2"
  if [[ -n "${3:-}" ]]; then printf '          %s\n' "$3"; fi
  case " $classes " in *" $1 "*) ;; *) classes="$classes $1" ;; esac
  if [[ $broken -eq 0 || $broken -gt $1 ]]; then broken=$1; fi
}
# The classes by name, for the line at the bottom: several things are missing at once more often than not,
# and the exit code can only carry the one worth fixing first.
class_name() {
  case "$1" in
    3) echo "ROS 2 itself" ;;
    4) echo "the ROS packages the launch files include" ;;
    5) echo "the python side" ;;
    6) echo "the simulator" ;;
    7) echo "the build" ;;
  esac
}

echo "ohm-nav-exploration — what this machine needs"
echo "------------------------------------------------------------------"

# ---------------------------------------------------------------- python: frontiers.py is numpy and nothing else
if python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' 2>/dev/null; then
  ok "python3 $(python3 -c 'import platform; print(platform.python_version())')"
else
  fail 5 "python3 is missing or older than 3.10" "sudo apt install python3"
fi

if python3 -c 'import numpy' >/dev/null 2>&1; then
  ok "numpy $(python3 -c 'import numpy; print(numpy.__version__)') — frontiers.py counts the cells with it"
else
  fail 5 "numpy is missing, and every frontier rule in frontiers.py is a numpy expression" \
       "sudo apt install python3-numpy      (or: ./install.sh --pip)"
fi

if python3 -m pytest --version >/dev/null 2>&1; then
  ok "pytest $(python3 -m pytest --version 2>/dev/null | head -1 | awk '{print $2}') — for the rules in ohm_frontier/test"
else
  fail 5 "pytest is missing — only the tests need it, never the robot" \
       "sudo apt install python3-pytest     (or: ./install.sh --pip)"
fi

# ---------------------------------------------------------------- ROS 2: sourced, or invisible
# ROS_DISTRO is set by a shell that sourced a setup file, so an empty one does not mean "no ROS", it
# means "not in this terminal" — which for the student fails the same way: ros2: command not found.
asked_for="${ROS_DISTRO:-}"
setup=""
for candidate in "${ROS_SETUP:-}" "/opt/ros/${ROS_DISTRO:-none}/setup.bash"; do
  if [[ -n "$candidate" && -f "$candidate" ]]; then setup="$candidate"; break; fi
done
if [[ -z "$setup" ]]; then                    # a machine with a distro nobody guessed the name of
  for candidate in /opt/ros/*/setup.bash; do
    if [[ -f "$candidate" ]]; then setup="$candidate"; break; fi
  done
fi
distro="none"
if [[ -n "$setup" ]]; then distro="$(basename "$(dirname "$setup")")"; fi

if [[ -z "$setup" ]]; then
  fail 3 "no ROS 2 under /opt/ros — nothing that speaks /map, /scan or an action will ever answer" \
       "install it per docs.ros.org; this repo is written for Kilted and measured on Kilted"
else
  if [[ -z "$asked_for" ]]; then
    note "this terminal has no ROS sourced — these checks sourced $setup themselves" \
         "and your own shell needs that line too, before ros2 means anything"
  fi
  # set +u because the ROS setup files read AMENT_* variables that are unset on a first source: with -u
  # this script would stop here without printing the reason.
  set +u; source "$setup"; set -u
  distro="${ROS_DISTRO:-$distro}"
  if python3 -c 'import rclpy' >/dev/null 2>&1; then
    ok "ROS 2 $distro at $(dirname "$setup") — rclpy importable, ros2 on the PATH"
    # ROS advertises pytest plugins (launch_testing, launch_ros) whose hook signature is from pytest 8. A
    # newer pytest from pip then refuses to start at all in a shell that has ROS sourced — measured here:
    # "Plugin 'launch_testing' for hook 'pytest_pycollect_makemodule'". The tests are fine, it is the
    # autoload that dies, so one environment variable is the whole answer. Asked about here rather than
    # with the python checks because it only appears once ROS is in the picture.
    if python3 -c 'import launch_testing' >/dev/null 2>&1 && python3 -m pytest --version >/dev/null 2>&1; then
      printf '          the tests, from a shell with ROS sourced: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q\n'
    fi
  else
    fail 3 "ROS 2 $distro is on disk but rclpy does not import — that is a ros-base without a client library" \
         "sudo apt install ros-$distro-desktop"
  fi
fi

# ---------------------------------------------------------------- what the launch files actually include
# Not "is the metapackage installed" but "is the file explore.launch.py includes there". Those two files,
# plus the action type the node is a client of, are the whole dependency of a run.
share_prefix="/opt/ros/${distro}/share"
if [[ -n "$setup" ]]; then
  if [[ -f "$share_prefix/slam_toolbox/launch/online_async_launch.py" ]]; then
    ok "slam_toolbox — the mapper explore.launch.py includes"
  else
    fail 4 "$share_prefix/slam_toolbox/launch/online_async_launch.py is not there — the mapper" \
         "sudo apt install ros-$distro-slam-toolbox"
  fi

  if [[ -f "$share_prefix/nav2_bringup/launch/navigation_launch.py" ]]; then
    ok "nav2_bringup — the navigation servers explore.launch.py includes"
  else
    fail 4 "$share_prefix/nav2_bringup/launch/navigation_launch.py is not there — the navigation servers" \
         "sudo apt install ros-$distro-nav2-bringup"
  fi

  if python3 -c 'import nav2_msgs' >/dev/null 2>&1; then
    ok "nav2_msgs — the NavigateToPose action this node is a client of"
  else
    fail 4 "nav2_msgs does not import, so the node can only publish its goals and nothing drives" \
         "sudo apt install ros-$distro-navigation2"
  fi

  # The view is part of the default run now: explore.launch.py opens RViz unless rviz:=false is asked for.
  if command -v rviz2 >/dev/null 2>&1 || [[ -x "/opt/ros/$distro/bin/rviz2" ]]; then
    ok "rviz2 — the view the default run opens"
  else
    fail 4 "rviz2 is missing, so the default run opens no view — the map, the frontiers and the goal stay unseen" \
         "sudo apt install ros-$distro-rviz2      (or run it without the view: rviz:=false)"
  fi

  # The three weights are ordinary declared parameters, which means the panel the owner asked for is a
  # package of its own and nothing more: without it the weights are still live over `ros2 param`, and with it
  # somebody at the front of a lecture hall can move them with a mouse. Optional, so a note and not a failure.
  if [[ -d "/opt/ros/$distro/lib/rqt_reconfigure" ]] || command -v rqt_reconfigure >/dev/null 2>&1; then
    ok "rqt_reconfigure — the panel the frontier weights are declared for"
  else
    note "rqt_reconfigure is not installed, so the ranking can be tuned but not with a panel" \
         "sudo apt install ros-$distro-rqt-reconfigure      (ros2 param set and describe work without it)"
  fi

  apt_command=(sudo apt install -y "ros-$distro-navigation2" "ros-$distro-nav2-bringup"
               "ros-$distro-slam-toolbox" "ros-$distro-rviz2")
  if [[ $do_apt == 1 ]]; then
    echo
    if [[ $assume_yes == 1 ]]; then
      echo "  running this — it will ask for your password:"
      echo "  ${apt_command[*]}"
      "${apt_command[@]}"
    else
      echo "  the ROS packages in one command (add --yes to have this script run it):"
      echo "  ${apt_command[*]}"
    fi
  fi
fi

if [[ $do_pip == 1 ]]; then
  echo
  pip_command=(python3 -m pip install --user -r "$here/requirements.txt")
  if [[ $assume_yes == 1 ]]; then
    echo "  running: ${pip_command[*]}"
    "${pip_command[@]}" || python3 -m pip install --user --break-system-packages -r "$here/requirements.txt"
  else
    echo "  the python requirements in one command (add --yes to have this script run it):"
    echo "  ${pip_command[*]}"
  fi
fi

# ---------------------------------------------------------------- the simulator: a checkout, not an apt package
if [[ ! -f "$sim_dir/launch/lab.launch.py" ]]; then
  fail 6 "no simulator at $sim_dir — explore.launch.py has nothing to include and no hall to map" \
       "git clone git@github.com:christianpfitzner/mecanum-lab.git ~/git/mecanum-lab   (or --sim-dir=PATH)"
else
  ok "simulator at $sim_dir — its launch/lab.launch.py is there"
  # The hall of the default run has to exist too; a typo in a world name costs a launch, not a warning.
  if [[ -f "$sim_dir/worlds/rooms.txt" ]]; then
    ok "the world rooms — the hall explore.launch.py starts in"
  else
    fail 6 "$sim_dir/worlds/rooms.txt is missing, which is the hall the default run asks for" \
         "git -C $sim_dir pull"
  fi
  # Two of the arguments this repo passes to that launch file are recent additions. An older checkout does
  # not refuse them loudly: the argument is dropped, the run then has two publishers on one tf edge, and
  # nav2's costmap answers that with "frame does not exist" instead of naming the cause.
  for argument in "tf_tree:the tf tree whose top edge belongs to the mapper" \
                  "lidar_no_echo:the lidar that says \"no echo\" instead of its own range"; do
    name="${argument%%:*}"; why="${argument#*:}"
    if grep -q "$name" "$sim_dir/launch/lab.launch.py"; then
      ok "the simulator takes $name — $why"
    else
      fail 6 "the simulator checkout is older than the $name argument this repo passes it — $why" \
           "git -C $sim_dir pull       (without it: two publishers on map → <robot>/odom, and no map)"
    fi
  done
  if python3 -c 'import pygame' >/dev/null 2>&1; then
    ok "pygame — the simulator's own window"
  else
    fail 5 "pygame is missing, so the simulator only runs with --headless and the default run stops" \
         "sudo apt install python3-pygame   (or: cd $sim_dir && ./install.sh)"
  fi
  if ros2 pkg prefix mecanum_lab >/dev/null 2>&1; then
    ok "mecanum_lab installed as a ROS package, so its launch files are found by package name"
  else
    note "the simulator is not built as a ROS package — that is fine, explore.launch.py takes the checkout" \
         "to build it: cd $sim_dir && ./install.sh"
  fi
fi

# ---------------------------------------------------------------- the build, only when asked for
if [[ $do_build == 1 ]]; then
  echo
  if ! command -v colcon >/dev/null 2>&1; then
    fail 5 "colcon is missing, so the package cannot be built and ros2 launch cannot find it" \
         "sudo apt install ros-$distro-dev-tools"
  elif colcon build --symlink-install; then
    ok "built — in every new shell after this one: source install/setup.bash"
  else
    fail 7 "colcon build did not succeed — the line above this one says why" \
         "with ROS sourced: colcon build --symlink-install"
  fi
fi

# ---------------------------------------------------------------- what the checks are worth
echo "------------------------------------------------------------------"
if [[ $broken -eq 0 ]]; then
  echo "result: ready. From this directory,"
  echo "  once per terminal:"
  echo "    source $setup"
  echo "  once, and after every change to setup.py:"
  echo "    colcon build --symlink-install && source install/setup.bash"
  echo "  a run:"
  echo "    ros2 launch ohm_frontier explore.launch.py"
  echo "  and the frontier rules, without a robot and without the build:"
  echo "    cd ohm_frontier && python3 -m pytest test -q"
else
  case "$broken" in
    2) echo "result: bad option — ./install.sh --help" ;;
    3) echo "result: ROS 2 first — nothing else on this list is reachable without it." ;;
    4) echo "result: ROS 2 is there; the packages its launch files include are not. See the missing lines." ;;
    5) echo "result: the python side is incomplete. See the missing lines." ;;
    6) echo "result: no simulator, so there is nothing to map. See the missing lines." ;;
    7) echo "result: the environment is there but the package does not build. See the line above." ;;
  esac
  # One exit code, but usually several classes missing at once: name the rest, so nobody fixes the ROS
  # packages, runs this again and finds the simulator was never there.
  others=""
  for code in $classes; do
    if [[ "$code" -ne "$broken" ]]; then others="$others, $(class_name "$code")"; fi
  done
  if [[ -n "$others" ]]; then
    echo "        this run also found nothing usable in: ${others#", "} — the exit code names what to fix first."
  fi
fi
exit "$broken"
