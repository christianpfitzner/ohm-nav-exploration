"""The launch files and the RViz view are imported and read here — which is the point of this file.

A launch file is Python that nothing imports until someone types `ros2 launch`, so a wrong import line in
one is a bug that survives the whole test suite and meets the user as
`ImportError: cannot import name 'FindPackageShare' from 'launch_ros.actions'`. That happened. So every
launch file is imported, run, and its described processes counted; the defaults a lecture sees are read off
the declared arguments rather than trusted from the docstring; and `config/explore.rviz` is read as the YAML
it is, because a display class spelled wrong costs one display and comes with no error message at all.

Nothing is launched. These tests ask what a launch file *describes* and what a config *names*; the machine
that runs them needs no simulator, no navigation stack and no RViz.

    python3 -m pytest test          # skipped here if `launch` is not installed
"""
import glob
import importlib.util
import os
import pathlib
import re

import pytest
import yaml

# guarded one after the other, in the order a machine without ROS 2 fails on them
pytest.importorskip("ament_index_python", reason="these tests import the launch files, which need ROS 2")
pytest.importorskip("launch", reason="these tests import the launch files, which need ROS 2")
pytest.importorskip("launch_ros", reason="these tests import the launch files, which need ROS 2")
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory  # noqa: E402
from launch.actions import IncludeLaunchDescription, LogInfo  # noqa: E402
from launch.launch_context import LaunchContext  # noqa: E402
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions  # noqa: E402
from launch_ros.actions import Node  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parents[1]
SHARES = {"ohm_frontier": "/opt/fake/ohm_frontier", "mecanum_lab": "/opt/fake/mecanum_lab",
          "nav2_bringup": "/opt/fake/nav2_bringup", "slam_toolbox": "/opt/fake/slam_toolbox"}
VIEW = HERE / "config" / "explore.rviz"

#: the hall each scenario file is named after, and whatever else it fixes beyond the hall
SCENARIOS = {"explore_rooms.launch.py": ("rooms", {}),
             "explore_maze.launch.py": ("maze", {}),
             "explore_open.launch.py": ("open", {}),
             "explore_arena.launch.py": ("arena", {}),
             "explore_no_nav2.launch.py": ("rooms", {"nav2": "false"})}


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

    if hasattr(module, "get_package_share_directory"):    # the scenario files look one package up, the
        monkeypatch.setattr(module, "get_package_share_directory", fake_share)   # main file four
    return module


def said(value, context):
    """A launch argument, an argument default, a command word or a plain string, as the text launch uses.

    Anything a launch file hands over may be a substitution or a string, and the two mean the same thing to
    the person typing the command. Reading only the strings would let a file pass by being fixed where it
    should be typed and vice versa, so everything goes through the substitution machinery here.
    """
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def spoken(action, context):
    """The sentence a `LogInfo` puts on the screen.

    `LogInfo` keeps its message in a private attribute, so this is the only way to read what a student would
    actually see. The mangled name is asserted rather than assumed: if the release moves it, this says so.
    """
    given = getattr(action, "_LogInfo__msg", None)
    assert given is not None, "LogInfo keeps its message somewhere else now — read it from there"
    return perform_substitutions(context, given)


def pointed_at(source, context):
    """The launch file an `IncludeLaunchDescription` points at, as text.

    The `location` property hands back the repr of a substitution object — useful to a human debugging a
    launch file, useless here — so the substitutions are performed from where launch keeps them.
    """
    given = getattr(source, "_LaunchDescriptionSource__location", None)
    assert given is not None, "a launch description source keeps its location elsewhere now — read it there"
    return said(given, context)


def qos_blocks(node, out=None):
    """Every topic block of a parsed RViz config, `Update Topic:` included.

    The Map display carries two of them, and the latched `/map_updates` is in the second one, so reading
    only `Topic:` would leave the durability of the map's own announcements untested.
    """
    out = out if out is not None else []
    if isinstance(node, dict):
        for key in ("Topic", "Update Topic"):
            block = node.get(key)
            if isinstance(block, dict):
                out.append(block)
        for value in node.values():
            qos_blocks(value, out)
    elif isinstance(node, list):
        for value in node:
            qos_blocks(value, out)
    return out


def declared(described):
    """The declared arguments by name, with the text their defaults carry."""
    plain = context_with({})
    return {a.name: said(a.default_value, plain)
            for a in described.entities if type(a).__name__ == "DeclareLaunchArgument"}


def includes(described):
    return [e for e in described.entities if isinstance(e, IncludeLaunchDescription)]


def survey(node, found=None):
    """Every display class, every topic and every key of a parsed RViz config, however deeply nested.

    `Topic:` is not the only place a config names a topic — the Map display has an `Update Topic:` beside it
    — and a display may sit in a group, so the walk is recursive rather than a fixed shape.
    """
    found = found if found is not None else {"topics": [], "classes": [], "keys": set()}
    if isinstance(node, dict):
        if isinstance(node.get("Class"), str):
            found["classes"].append(node["Class"])
        for key in ("Topic", "Update Topic"):
            block = node.get(key)
            if isinstance(block, dict) and isinstance(block.get("Value"), str):
                found["topics"].append((node.get("Class"), node.get("Name"), block["Value"]))
        found["keys"].update(node)
        for value in node.values():
            survey(value, found)
    elif isinstance(node, list):
        for value in node:
            survey(value, found)
    return found


def view():
    return yaml.safe_load(open(VIEW))


def test_every_launch_file_avoids_the_import_that_does_not_exist():
    """No `FindPackageShare` from the wrong module, no typo in a name: this alone is what broke on a
    machine that had everything else installed. Every file in `launch/`, the scenario files included."""
    for path in sorted(glob.glob(str(HERE / "launch" / "*.py"))):
        offenders = [line for line in pathlib.Path(path).read_text().splitlines()
                     if line.startswith(("from launch_ros", "import launch_ros"))
                     and "FindPackageShare" in line]
        assert not offenders, f"{os.path.basename(path)} imports {offenders}"


def test_the_explore_launch_file_describes_the_whole_run(monkeypatch):
    module = launch_file("explore.launch.py", monkeypatch,
                         installed=("ohm_frontier", "nav2_bringup", "slam_toolbox"))
    described = module.generate_launch_description()
    assert len(declared(described)) >= 10, \
        "robot, world, sim_dir, use_sim_time, headless, rviz, rviz_config, nav2, slam_params, nav2_params"
    assert len([e for e in described.entities if isinstance(e, Node)]) == 1, \
        "the frontier node is the only node started directly"
    assert len(includes(described)) == 1, "the simulator is the only process included up front"
    # the mapper and nav2 come later because their parameter files have to carry the robot's name, and the
    # view comes later for the same reason in the other direction: its topics carry the name
    assert len([e for e in described.entities if type(e).__name__ == "OpaqueFunction"]) == 2, \
        "one for the two parameter files, one for the view"


def test_the_defaults_are_the_run_a_lecture_asks_for_without_typing_anything(monkeypatch):
    """`ros2 launch ohm_frontier explore.launch.py`, with nothing after it, is the command in the README, so
    the defaults are documentation: the hall the documentation describes, a window people can see, and a
    view that shows the decisions rather than only the robot."""
    module = launch_file("explore.launch.py", monkeypatch,
                         installed=("ohm_frontier", "nav2_bringup", "slam_toolbox"))
    defaults = declared(module.generate_launch_description())
    assert defaults["world"] == "rooms", defaults["world"]
    assert defaults["headless"] == "false", "a default that starts headless shows a lecture nothing"
    assert defaults["rviz"] == "true", "the frontier view is on unless someone asks for it off"
    assert defaults["nav2"] == "true", "the stack is in; explore_no_nav2.launch.py is how to leave it out"
    assert defaults["use_sim_time"] == "true"
    assert defaults["rviz_config"].endswith(os.path.join("rviz", "explore.rviz")), \
        "the view is installed under share/ohm_frontier/rviz, which is what setup.py promises"


def test_the_simulator_is_asked_for_the_tree_that_leaves_the_top_edge_to_the_mapper(monkeypatch):
    """`map → <robot>/odom` may have one publisher. That the simulator is the one standing down is the
    whole arrangement, so a launch file that stopped asking for it would silently break the map."""
    module = launch_file("explore.launch.py", monkeypatch,
                         installed=("ohm_frontier", "nav2_bringup", "slam_toolbox"))
    simulator = includes(module.generate_launch_description())[0]
    assert dict(simulator.launch_arguments).get("tf_tree") == "slam", dict(simulator.launch_arguments)


def test_the_simulator_is_asked_for_no_view_of_its_own(monkeypatch):
    """Two RViz 2 processes with fixed frame `map` is one too many, and the one a student then blames is
    the empty one. So the lab's own RViz is asked off whatever this package does with `rviz`, while the
    simulator's window stays a separate question called `headless`."""
    module = launch_file("explore.launch.py", monkeypatch,
                         installed=("ohm_frontier", "nav2_bringup", "slam_toolbox"))
    given = dict(includes(module.generate_launch_description())[0].launch_arguments)
    plain = context_with({"headless": "false"})
    assert said(given["rviz"], plain) == "false", "the lab's RViz is off; this package owns the one view"
    assert said(given["headless"], plain) == "false", "the window question is passed through, not fixed here"
    assert said(given["lidar_no_echo"], plain) == "inf", "a mapper has to hear that a beam did not come back"


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


def test_the_mapper_and_the_stack_arrive_once_the_name_is_known(monkeypatch):
    module = launch_file("explore.launch.py", monkeypatch,
                         installed=("ohm_frontier", "nav2_bringup", "slam_toolbox"))
    templates = (os.path.join(str(HERE), "config", "slam_toolbox.yaml"),
                 os.path.join(str(HERE), "config", "nav2_rooms.yaml"))
    built = module.started_by_name(context_with({"robot": "carlo", "nav2": "true"}), *templates, "true")
    assert len([e for e in built if isinstance(e, IncludeLaunchDescription)]) == 2, \
        "slam_toolbox and nav2 have to arrive once the name is known, too"


def test_leaving_the_stack_out_keeps_the_mapper_and_says_so_on_the_screen(monkeypatch):
    """`nav2:=false` is `explore_no_nav2.launch.py`: the map and the decisions stay, the driving goes. A
    silent omission would read as a stack that failed to start, so the answer is a line saying what is off."""
    module = launch_file("explore.launch.py", monkeypatch,
                         installed=("ohm_frontier", "nav2_bringup", "slam_toolbox"))
    templates = (os.path.join(str(HERE), "config", "slam_toolbox.yaml"),
                 os.path.join(str(HERE), "config", "nav2_rooms.yaml"))
    built = module.started_by_name(context_with({"robot": "muster", "nav2": "false"}), *templates, "true")
    assert len([e for e in built if isinstance(e, IncludeLaunchDescription)]) == 1, "the mapper stays in"
    lines = [e for e in built if isinstance(e, LogInfo)]
    assert len(lines) == 1, "the run says which half it left out"
    assert "/frontier_goal" in spoken(lines[0], context_with({})), \
        "the line has to say where the goals go instead, or it reads as an error"


def test_the_view_starts_on_the_robots_name_in_its_own_topics(monkeypatch, tmp_path):
    """RViz cannot put a name into a display's topic itself and every topic of a robot here carries one, so
    the view goes through the same `written()` as the parameter files. What comes out has to be a config a
    window can be started on: parseable, name substituted, and on this robot's topics."""
    module = launch_file("explore.launch.py", monkeypatch,
                         installed=("ohm_frontier", "nav2_bringup", "slam_toolbox"))
    monkeypatch.setattr(module.shutil, "which", lambda name: f"/opt/ros/fake/bin/{name}")
    context = context_with({"rviz": "true", "rviz_config": str(VIEW), "robot": "carlo",
                            "use_sim_time": "true"})
    started = module.started_by_view(context, "true", str(VIEW), "true")
    assert len(started) == 1 and type(started[0]).__name__ == "ExecuteProcess", started

    command = [said(word, context) for word in started[0].cmd]
    assert command[0] == "rviz2" and "--display-config" in command, command
    written = yaml.safe_load(open(command[command.index("--display-config") + 1]))
    assert "<robot>" not in str(written), "the name did not reach the file RViz was handed"
    named = {topic for _, _, topic in survey(written)["topics"]}
    assert {"/map", "/carlo/scan", "/carlo/odom", "/frontiers", "/frontier_goal", "/plan"} <= named, named
    assert "--ros-args" in command, "a viewer on the wall clock draws a sim-timed map 1.7 billion seconds ago"


def test_no_view_is_started_when_none_was_asked_for(monkeypatch):
    """`rviz:=false` is a recorded run and a CI machine. Silence is the right answer there, not an
    explanation of a choice somebody made on purpose."""
    module = launch_file("explore.launch.py", monkeypatch,
                         installed=("ohm_frontier", "nav2_bringup", "slam_toolbox"))
    assert module.started_by_view(context_with({"rviz": "false"}), "false", str(VIEW), "true") == []


def test_a_machine_without_rviz_is_told_what_to_type(monkeypatch):
    """rviz2 is not part of the ROS base install, and a launch file that starts it anyway ends the screen
    with `process has died` — which reads as a broken explorer rather than as a missing apt package, and is
    the wrong lesson twice over."""
    module = launch_file("explore.launch.py", monkeypatch,
                         installed=("ohm_frontier", "nav2_bringup", "slam_toolbox"))
    monkeypatch.setattr(module.shutil, "which", lambda name: None)
    answered = module.started_by_view(context_with({"rviz": "true", "rviz_config": str(VIEW),
                                                    "robot": "muster"}), "true", str(VIEW), "true")
    assert len(answered) == 1 and isinstance(answered[0], LogInfo), answered
    message = spoken(answered[0], context_with({}))
    assert "rviz2" in message and "apt install" in message, message


@pytest.mark.parametrize("name,hall,extra", [(n, h, x) for n, (h, x) in SCENARIOS.items()])
def test_each_scenario_file_asks_for_the_hall_it_is_named_after(monkeypatch, name, hall, extra):
    """A file whose `world:=` outbids the hall its own name promises is how a demo starts in the wrong
    hall, so the hall is the first thing worth asserting about these five. Nothing is run."""
    module = launch_file(name, monkeypatch, installed=("ohm_frontier",))
    described = module.generate_launch_description()
    plain = context_with({"robot": "muster", "headless": "false", "rviz": "true"})
    assert {"robot", "headless", "rviz"} <= set(declared(described)), \
        f"{name} does not let a lecture ask for another robot, a windowless run or no view"

    included = includes(described)
    assert len(included) == 1, "a scenario file is a hall and an include, not a copied graph"
    assert pointed_at(included[0].launch_description_source, plain).endswith("explore.launch.py"), \
        "everything else — simulator, mapper, stack, node, tf arrangement — comes from the one file"
    given = dict(included[0].launch_arguments)
    assert said(given["world"], plain) == hall, f"{name} does not ask for the {hall} hall"
    for argument, expected in extra.items():
        assert said(given[argument], plain) == expected, f"{name} does not set {argument}:={expected}"
    for passed in ("robot", "headless", "rviz"):
        assert passed in given, f"{name} does not pass {passed} through, so it could not be typed"


def test_the_frontier_launch_file_starts_one_node_alone(monkeypatch):
    module = launch_file("frontier.launch.py", monkeypatch, installed=("ohm_frontier",))
    entities = module.generate_launch_description().entities
    assert len([e for e in entities if isinstance(e, Node)]) == 1
    assert not [e for e in entities if isinstance(e, IncludeLaunchDescription)], \
        "this one is for a run where the rest is already up"


def test_the_view_is_yaml_and_shows_the_four_things_a_run_has_to_show():
    """The map, where the robot has been, every frontier there is, and the one it chose. A display class
    spelled wrong loses one of those four and says nothing while it is missing."""
    by_topic = {topic.replace("<robot>/", ""): cls for cls, _, topic in survey(view())["topics"]
                if cls and cls.startswith("rviz_default_plugins/")}
    assert by_topic["/map"] == "rviz_default_plugins/Map", by_topic
    assert by_topic["/odom"] == "rviz_default_plugins/Odometry", "the trajectory: the poses it kept"
    assert by_topic["/frontiers"] == "rviz_default_plugins/MarkerArray", "every candidate, both namespaces"
    assert by_topic["/frontier_goal"] == "rviz_default_plugins/Pose", "the one it drives to"
    assert by_topic["/scan"] == "rviz_default_plugins/LaserScan", "what the mapper is working from"
    assert by_topic["/plan"] == "rviz_default_plugins/Path", "what the stack is trying to do about it"
    assert view()["Visualization Manager"]["Global Options"]["Fixed Frame"] == "map", \
        "from tf_tree:=slam on, map is the top of the tree and the frame the map message names"


def test_the_robots_name_reaches_the_view_it_has_to_be_in(monkeypatch):
    """`/carlo/scan` is not `/muster/scan`, and RViz has no way to substitute a name into a topic itself."""
    assert "<robot>" in open(VIEW).read(), "the view is a template; a baked-in name is the bug, not the fix"
    module = launch_file("explore.launch.py", monkeypatch, installed=("ohm_frontier",))
    filled = open(module.written(str(VIEW), "carlo")).read()
    assert "<robot>" not in filled
    assert "/carlo/scan" in filled and "/carlo/odom" in filled
    yaml.safe_load(filled)                          # still a config, not merely still a string


def test_the_view_names_only_display_classes_this_rviz_declares():
    """A `Class:` no plugin declares is dropped at start-up, quietly, and the window then shows one thing
    too few. The names are checked against what the installed plugin library declares, not against memory."""
    try:
        share = get_package_share_directory("rviz_default_plugins")
    except PackageNotFoundError:                    # `launch` here, RViz not: nothing to check against
        pytest.skip("no rviz_default_plugins on this machine")
    declared_names = set(re.findall(r'name="([^"]+)"',
                                    open(os.path.join(share, "plugins_description.xml")).read()))
    used = {c for c in survey(view())["classes"] if c.startswith("rviz_default_plugins/")}
    missing = sorted(c for c in used if c not in declared_names)
    assert not missing, f"no installed RViz plugin declares {missing}"


def test_the_view_carries_no_key_rviz_two_does_not_read():
    """`Unreliable:` and a display's `Description:` are RViz 1 keys. Neither string exists in
    `librviz_default_plugins.so` on this release — reliability is the topic's `Reliability Policy` there and
    a display is named by `Name` — and an unknown key is read in silence, so the window simply lacks the
    thing the config believed it had set."""
    keys = survey(view())["keys"]
    for stale in ("Unreliable", "Description"):
        assert stale not in keys, f"{stale}: is an RViz 1 key that does nothing here"


def test_the_view_asks_for_the_qos_that_matches_what_this_stack_publishes():
    """The simulator publishes standard (reliable) QoS everywhere — `mecanum_lab/ros_bridge.py` says so in
    its own header — and slam_toolbox latches `/map`. So the scan asks for Reliable rather than the
    best-effort default an RViz LaserScan arrives with, and the map asks for Transient Local: a volatile
    subscriber that joins after the run is over never sees a latched topic at all."""
    by_value = {block["Value"]: block for block in qos_blocks(view()) if "Value" in block}
    assert by_value["/<robot>/scan"]["Reliability Policy"] == "Reliable"
    assert by_value["/map"]["Durability Policy"] == "Transient Local", "/map is latched"
    assert by_value["/map_updates"]["Reliability Policy"] == "Reliable"


def test_a_package_that_is_not_installed_is_named_with_what_to_do_about_it(monkeypatch):
    """`ImportError` from a launch file tells a student nothing. A missing apt package must say so."""
    module = launch_file("explore.launch.py", monkeypatch, installed=("mecanum_lab",))
    with pytest.raises(Exception) as error:
        module.generate_launch_description()
    message = str(error.value)
    assert "ohm_frontier" in message, message
    assert "apt install" in message, f"the message does not say what to install: {message}"
