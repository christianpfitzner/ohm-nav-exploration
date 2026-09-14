


General: 
- create commits for each task and to the commits at the end
- test everything which is possible


UI: 
- default map should be rooms
- load the rviz visualization by default showing the map, the trajectory, all available frontiers, as well as the current selected frontier
- default should not be headless
- provide a possible rqt_reconfigure for the priorization of the frontiers (size, orientation, euclidean distance, ...)

code: 
- create multiple demo launch files for different maps and scenarios
- create furth navigation algorithms for reactive navigation (wall following, simple obstacle avoidance, turn and move to a certain goal). These should be examples for the lecture to be displayed. create the code. 
- clean up unused code
- apply code hygene
- create an install script for missing requirements



Strange behaviour: 
- the frontier flipped every second and the robot did not reach the frontiers, while every time a new one was selected as goal. 
- a new frontier should be selected after reaching a previous one or after a timeout (10 s e.g.)

README: 
- simplyfy the readme documentation
- explain what frontier exploration is on student level
- link the frontier based exploration algorithm by yamauchi in the repo
