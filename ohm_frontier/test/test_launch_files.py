"""The launch files are imported here — which is the point of this file.

A launch file is Python that nothing imports until someone types `ros2 launch`, so a wrong import line in
one is a bug that survives the whole test suite and meets the user as
`ImportError: cannot import name 'FindPackageShare' from 'launch_ros.actions'`. That happened. So both
files are imported, run, and their described processes counted.

    python3 -m pytest test          # skipped here if `launch` is not installed
"""
import importlib.util
import os
import pathlib

import pytest

# guarded one after the other, in the order a machine without ROS 2 fails on them
pytest.importorskip("ament_index_python", reason="these tests import the launch files, which need ROS 2")
pytest.importorskip("launch", reason="these tests import the launch files, which need ROS 2")
pytest.importorskip("launch_ros", reason="these tests import the launch files, which need ROS 2")
from ament_index_python.packages import PackageNotFoundError  # noqa: E402
from launch.actions import IncludeLaunchDescription  # noqa: E402
from launch.launch_context import LaunchContext  # noqa: E402
from launch_ros.actions import Node  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parents[1]
SHARES = {"ohm_frontier": "/opt/fake/ohm_frontier", "mecanum_lab": "/opt/fake/mecanum_lab",
          "nav2_bringup": "/opt/fake/nav2_bringup", "slam_toolbox": "/opt/fake/slam_toolbox"}


def context_with(configurations):
    """A launch context that answers launch arguments, for the part of a launch file that runs later."""
    context = LaunchContext(argv=[])
    context.launch_configurations.update(configurations)      # as `robot:=carlo` would have done
    return context


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


def test_the_explore_launch_file_describes_the_whole_run(monkeypatch):
    module = launch_file("explore.launch.py", monkeypatch,
                         installed=("ohm_frontier", "nav2_bringup", "slam_toolbox"))
    entities = module.generate_launch_description().entities
    declared = [e for e in entities if type(e).__name__ == "DeclareLaunchArgument"]
    assert len(declared) >= 7, "robot, world, sim_dir, use_sim_time, rviz, slam_params, nav2_params"
    assert len([e for e in entities if isinstance(e, Node)]) == 1, "the frontier node is the only node here"
    # the other three processes come as included launch files; two of them only after the robot's name is
    # known, because their parameters live in a file the name has to be written into
    assert len([e for e in entities if isinstance(e, IncludeLaunchDescription)]) == 1
    assert len([e for e in entities if type(e).__name__ == "OpaqueFunction"]) == 1
    slam_template, nav2_template = (os.path.join(str(HERE), "config", n)
                                    for n in ("slam_toolbox.yaml", "nav2_rooms.yaml"))
    built = module.started_by_name(context_with({"robot": "carlo"}), slam_template, nav2_template, "true")
    assert len([e for e in built if isinstance(e, IncludeLaunchDescription)]) == 2, \
        "slam_toolbox and nav2 have to arrive once the name is known, too"


def test_the_simulator_is_asked_for_the_tree_that_leaves_the_top_edge_to_the_mapper(monkeypatch):
    """`map → <robot>/odom` may have one publisher. That the simulator is the one standing down is the
    whole arrangement, so a launch file that stops asking for it would silently break the map."""
    module = launch_file("explore.launch.py", monkeypatch,
                         installed=("ohm_frontier", "nav2_bringup", "slam_toolbox"))
    simulator = [e for e in module.generate_launch_description().entities
                 if isinstance(e, IncludeLaunchDescription)][0]
    assert dict(simulator.launch_arguments).get("tf_tree") == "slam", dict(simulator.launch_arguments)


def test_the_robot_name_reaches_the_parameter_files_it_has_to_be_in(monkeypatch):
    """nav2 and slam_toolbox are given a file, not parameters, and name frames absolutely: `robot:=carlo`
    is worth nothing unless the file they read says carlo too."""
    module = launch_file("explore.launch.py", monkeypatch, installed=("ohm_frontier",))
    for name in ("nav2_rooms.yaml", "slam_toolbox.yaml"):
        template = os.path.join(str(HERE), "config", name)
        assert "<robot>" in open(template).read(), f"{name} has no <robot> to substitute"
        assert not [line for line in open(template) if "muster/" in line and not line.lstrip().startswith("#")], \
            f"{name} carries a baked-in robot name"
        filled = open(module.written(template, "carlo")).read()
        assert "<robot>" not in filled and "carlo/base_link" in filled, name


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
