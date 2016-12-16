 def IK_MoveIt(Move_arm,rot,StartPosition, MiddlePosition,EndPosition , Accuracy):
    	waypoints = []  

	    wpose = Pose()
	    wpose.orientation.x = rot[0]
	    wpose.orientation.y = rot[1]
	    wpose.orientation.z = rot[2]
	    wpose.orientation.w = rot[3]

	    # first point
	    wpose.position.x = StartPosition[0]
	    wpose.position.y = StartPosition[1]
	    wpose.position.z = StartPosition[2]
	    waypoints.append(copy.deepcopy(wpose))

	    # Middle Point (if existed)
	    if MiddlePosition != False :
	        Number = len(MiddlePosition)/3 - 1
	        for i in range(Number):
	            wpose.position.x = MiddlePosition[i*3]
	            wpose.position.y = MiddlePosition[i*3+1]
	            wpose.position.z = MiddlePosition[i*3+2]
	            waypoints.append(copy.deepcopy(wpose))

	    # End point
	    wpose.position.x = EndPosition[0]
	    wpose.position.y = EndPosition[1]
	    wpose.position.z = EndPosition[2]
	    waypoints.append(copy.deepcopy(wpose))

	    (plan, fraction) = Move_arm.compute_cartesian_path(
	                               waypoints,   # waypoints to follow
	                               Accuracy,        # eef_step
	                               0.0)         # jump_threshold
	    
	    # Execute the plan         
	    #raw_input('Press Enter to go')            
	    Move_arm.execute(plan) 