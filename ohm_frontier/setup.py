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
    ],
    entry_points={"console_scripts": ["frontier_node = ohm_frontier.frontier_node:main"]},
)
