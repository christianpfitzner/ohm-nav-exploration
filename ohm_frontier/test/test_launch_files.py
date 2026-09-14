"""The launch files are imported here — which is the point of this file.

A launch file is Python that nothing imports until someone types `ros2 launch`, so a wrong import line in
one is a bug that survives the whole test suite and meets the user as
`ImportError: cannot import name 'FindPackageShare' from 'launch_ros.actions'`. That happened. So both
files are imported, run, and their described processes counted.

    python3 -m pytest test          # skipped here if `launch` is not installed
"""
import importlib.util
import pathlib

import pytest

# guarded one after the other, in the order a machine without ROS 2 fails on them
pytest.importorskip("ament_index_python", reason="these tests import the launch files, which need ROS 2")
pytest.importorskip("launch", reason="these tests import the launch files, which need ROS 2")
pytest.importorskip("launch_ros", reason="these tests import the launch files, which need ROS 2")
from ament_index_python.packages import PackageNotFoundError  # noqa: E402
from launch.actions import IncludeLaunchDescription  # noqa: E402
from launch_ros.actions import Node  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parents[1]
SHARES = {"ohm_frontier": "/opt/fake/ohm_frontier", "mecanum_lab": "/opt/fake/mecanum_lab",
          "nav2_bringup": "/opt/fake/nav2_bringup", "slam_toolbox": "/opt/fake/slam_toolbox"}


def launch_file(name, monkeypatch, installed=()):
    """The launch file as a module, with the packages it looks up pointed at fake directories.

    `installed` names the packages that are supposed to be there; the others answer with the same
    `PackageNotFoundError` a machine without them gives.
    """
    spec = importlib.util.spec_from_file_location(name.replace(".launch.py", "_launch"),
                                                 HERE / "launch" / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)             # this is the check; the rest is what it describes

    def fake_share(package):
        if package not in installed:
            raise PackageNotFoundError(package)
        return SHARES[package]

    if hasattr(module, "get_package_share_directory"):    # only explore.launch.py looks packages up
        monkeypatch.setattr(module, "get_package_share_directory", fake_share)
    return module


def test_the_launch_files_import_at_all():
    """No `FindPackageShare` from the wrong module, no typo in a name: this alone is what broke on a
    machine that had everything else installed."""
    for name in ("explore.launch.py", "frontier.launch.py"):
        source = (HERE / "launch" / name).read_text()
        offenders = [line for line in source.splitlines()
                     if line.startswith(("from launch_ros", "import launch_ros"))
                     and "FindPackageShare" in line]
        assert not offenders, f"{name} imports {offenders}"


def test_the_explore_launch_file_describes_four_processes(monkeypatch):
    module = launch_file("explore.launch.py", monkeypatch,
                         installed=("ohm_frontier", "nav2_bringup"))
    entities = module.generate_launch_description().entities
    included = [e for e in entities if isinstance(e, IncludeLaunchDescription)]
    nodes = [e for e in entities if isinstance(e, Node)]
    declared = [e for e in entities if type(e).__name__ == "DeclareLaunchArgument"]
    assert len(declared) >= 6, "robot, world, sim_dir, use_sim_time, rviz, slam_params, nav2_params"
    assert len(included) == 2, f"the simulator and nav2 are included, got {len(included)}"
    assert len(nodes) == 2, f"slam_toolbox and the frontier node are started, got {len(nodes)}"


def test_the_frontier_launch_file_starts_one_node_alone(monkeypatch):
    module = launch_file("frontier.launch.py", monkeypatch, installed=("ohm_frontier",))
    entities = module.generate_launch_description().entities
    assert len([e for e in entities if isinstance(e, Node)]) == 1
    assert not [e for e in entities if isinstance(e, IncludeLaunchDescription)], \
        "this one is for a run where the rest is already up"


def test_a_package_that_is_not_installed_is_named_with_what_to_do_about_it(monkeypatch):
    """`ImportError` from a launch file tells a student nothing. A missing apt package must say so."""
    module = launch_file("explore.launch.py", monkeypatch, installed=("mecanum_lab",))
    with pytest.raises(Exception) as error:
        module.generate_launch_description()
    message = str(error.value)
    assert "ohm_frontier" in message, message
    assert "apt install" in message, f"the message does not say what to install: {message}"
