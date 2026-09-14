"""A hall, a grid, and the places in it that a lidar has not seen."""

from glob import glob
import os

from setuptools import setup

setup(
    name="ohm_frontier",
    version="0.1.0",
    description="Frontier finding and prioritisation for the mecanum-lab simulator",
    packages=["ohm_frontier"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/ohm_frontier"]),
        ("share/ohm_frontier", ["package.xml"]),
        (os.path.join("share", "ohm_frontier", "launch"), glob("launch/*.py")),
        (os.path.join("share", "ohm_frontier", "config"), glob("config/*.yaml")),
        # The RViz view lives beside the parameters but is not a parameter file: `config/*.yaml` would
        # install it and `rviz:=true` would then look for it under `config/` and find nothing.
        (os.path.join("share", "ohm_frontier", "rviz"), glob("config/*.rviz")),
    ],
    # The three reactive demos are executables of this same package: each launch file names one in
    # `executable=`, and a name that is not installed here is a launch-time error no import catches — which
    # is why test_reactive.py compares the launch files against this list.
    entry_points={"console_scripts": [
        "frontier_node = ohm_frontier.frontier_node:main",
        "wall_following = ohm_frontier.wall_following:main",
        "obstacle_avoidance = ohm_frontier.obstacle_avoidance:main",
        "turn_and_move = ohm_frontier.turn_and_move:main",
    ]},
)
