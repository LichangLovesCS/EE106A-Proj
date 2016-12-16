# EE106a Project Note 4 by Lingyao Zhang

Tag： Robotic

---

1. roslaunch baxter_moveit_config move_group.launch
If the problem is: cannot find trajectory controller.... (anything related to trajectory)
then just run
```
rosrun baxter_interface joint_trajectory_action_server.py
```

2. If anything unexpected happens and you don't why, just restart the robot!!!
The restart button is behind the robot and on its back.

3. Why can't I move the right arm??!!!

***error message for the right arm***
```
[ INFO] [1479608404.094791702]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
[ INFO] [1479608404.095210874]: Found a contact between 'right_hand_camera' (type 'Robot link') and 'right_gripper_base' (type 'Robot link'), which constitutes a collision. Contact information is not stored.
[ INFO] [1479608404.095246935]: Collision checking is considered complete (collision was found and 0 contacts are stored)
[ INFO] [1479608404.095284581]: Start state appears to be in collision with respect to group right_arm
[ WARN] [1479608404.230347624]: Unable to find a valid state nearby the start state (using jiggle fraction of 0.050000 and 100 sampling attempts). Passing the original planning request to the planner.
[ INFO] [1479608404.230925980]: Planner configuration 'right_arm[RRTConnectkConfigDefault]' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[ WARN] [1479608404.231234927]: right_arm[RRTConnectkConfigDefault]: Skipping invalid start state (invalid state)
[ERROR] [1479608404.231283362]: right_arm[RRTConnectkConfigDefault]: Motion planning start tree could not be initialized!
[ INFO] [1479608404.231308830]: No solution found after 0.000275 seconds
[ WARN] [1479608404.241035096]: Goal sampling thread never did any work.
[ INFO] [1479608404.241197558]: Unable to solve the planning problem
[ INFO] [1479608404.344888775]: Received new trajectory execution service request...
[ WARN] [1479608404.344949260]: The trajectory to execute is empty
```

***success message for moving the left arm***
```
[ INFO] [1479608258.236395455]: Received new trajectory execution service request...
[ INFO] [1479608259.083919401]: Execution completed: SUCCEEDED
[ INFO] [1479608300.317742444]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
[ INFO] [1479608300.318732568]: Planner configuration 'left_arm[RRTConnectkConfigDefault]' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[ INFO] [1479608300.319080218]: left_arm[RRTConnectkConfigDefault]: Starting planning with 1 states already in datastructure
[ INFO] [1479608300.340852911]: left_arm[RRTConnectkConfigDefault]: Created 5 states (2 start + 3 goal)
[ INFO] [1479608300.340906384]: Solution found in 0.022066 seconds
[ INFO] [1479608300.341165367]: SimpleSetup: Path simplification took 0.000211 seconds and changed from 4 to 2 states
```






