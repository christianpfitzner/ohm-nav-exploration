



UI: 
- is good


RVIZ: 
- Exploration remove the numbers flying around



Reactive Navigation: 
- all reactive Navigation stuff does not work and the robot just moves strange
- obstacle avoidance can come close to the robot and should use the lidar
- wall following should try to find a wall first and then follow it in a fixed distance, e.g. 0.5 meters from the kinematic center


README: 
- One command to copy from the code environment!! not multiple calls of roslaunch in one code environment
- reduce the information and its detail. This should be a quick start. 



---

## Resolved, with the run that says so

| the line above | what happened | where to see it |
| --- | --- | --- |
| RVIZ: remove the numbers flying around | the ranking arithmetic is off by default; `scores:=true` or `ros2 param set /frontier_node show_scores true` brings it back, and the switch was watched on the wire — the default `/frontiers` namespaces are `{frontiers, selected, approach}`, enabling adds `scores` and `clock` with 21 text labels on the `rooms` map | `8df4eb8`, [docs/verification.md](docs/verification.md) |
| reactive navigation moves strange | all four demos had a real cause and each is now measured: see the two rows below and [docs/control-demos.md](docs/control-demos.md) | `263bd7c`, `9da0b71`, `c15e62c`, `644e3e3` |
| obstacle avoidance comes too close, use the lidar | it reads all 360 beams and keeps a 0.40 m ring: in two `rooms` runs, **0 readings inside 0.30 m out of 77 220**, nearest echo 0.41 m. Every state including `STOPPED` still publishes a heading, so a blocked robot turns towards the widest gap instead of freezing | `c15e62c`, [ohm_frontier/obstacle_avoidance.py](ohm_frontier/ohm_frontier/obstacle_avoidance.py) |
| wall following: find a wall first, then hold 0.5 m from the kinematic centre | there is a `SEARCHING` phase that gives up after `search_timeout` and a `NO_WALL` that says so; the gap is projected onto the lateral beam, so it is 0.50 m from the centre and not along a diagonal. `rooms`: 5.68 m of net and a 7.1 s unbroken hold; `maze`: 8.33 m and 19.5 s | `263bd7c`, `9da0b71` |
| README: one command per block, less detail | one copy-paste block per thing, detail moved to `docs/` (troubleshooting, control-demos, verification, frontier-exploration). The demos' launch files publish their own goal so the copied line moves the robot | `917641a`, `03b26ac` |

136 tests pass (`cd ohm_frontier && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q`) and
`./install.sh --check` is clean. What the four rules still cannot do is a list at the end of
[docs/control-demos.md](docs/control-demos.md) rather than a surprise at the front of one.
