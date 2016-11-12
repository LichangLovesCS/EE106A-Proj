# EE106a Project Notes 1 by Lingyao Zhang

标签（空格分隔）： Robotics 

---

[What is end-point?](http://sdk.rethinkrobotics.com/wiki/Gripper_Customization#Endpoints)
endpoint_state = transform from **base** to **side_gripper**  (side = left or right)

I need to test :
```
1. rostopic echo /left_gripper
2. fixed orientation for gripper to hold the cube
3. Test if MoveIt can only move arm based on base frame
```

Then check if the following is correct 
```python
    #AR_marker specifies which face to grab. Gripper specifies which gripper to use.
    def Grab_Cube(self, arm, AR_marker, Accuracy=0.03):
      	(trans,rot) = self.listener.lookupTransform('/base', AR_marker,rospy.Time(0))#base or gripper?

        Goal_Pose = Pose()
        Goal = []
        #Our orientation here should be fixed. We need to test it.
        Goal_Pose.orientation.x = 
        Goal_Pose.orientation.y = 
        Goal_Pose.orientation.z = 
        Goal_Pose.orientation.w = 

        #we need to add some constant offset here
        Goal_Pose.position.x = trans[0]
        Goal_Pose.position.y = trans[1]
        Goal_Pose.position.z = trans[2]
        Goal.append(Goal_Pose)
        (plan, fraction) = arm.compute_cartesian_path(
                               Goal,   # waypoints to follow
                               Accuracy,        # eef_step
                               0.0)     
        arm.execute(plan)
```






