# EE106a Project Notes 2 by Lingyao Zhang

标签（空格分隔）： robotic

---
## Some Errors

1.  Start move_group launch file before running main function or we will get an error!  

```
Traceback (most recent call last):
  File "main.py", line 160, in <module>
    PC = PlayCube()
  File "main.py", line 49, in __init__
    self.Right_Hand_Camera = baxter_interface.CameraController('right_hand_camera')
  File "/var/local/home/baxter/ros_ws/src/baxter_interface/src/baxter_interface/camera.py", line 81, in __init__
    "Close a different camera first and try again.".format(self._id)))
AttributeError: Cannot locate a service for camera name 'right_hand_camera'. Close a different camera first and try again.
terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
  what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
Aborted (core dumped)
```
To avoid this , type in :
```
roslaunch baxter_moveit_config move_group.launch 
```

2.  Find how to start camera service or we will get an error:
```
Traceback (most recent call last):
  File "main.py", line 160, in <module>
    PC = PlayCube()
  File "main.py", line 49, in __init__
    self.Right_Hand_Camera = baxter_interface.CameraController('right_hand_camera')
  File "/var/local/home/baxter/ros_ws/src/baxter_interface/src/baxter_interface/camera.py", line 81, in __init__
    "Close a different camera first and try again.".format(self._id)))
AttributeError: Cannot locate a service for camera name 'right_hand_camera'. Close a different camera first and try again.
terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
  what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
Aborted (core dumped)
```
we need to start the camera service first!!!
The following is the init function for camera.
```python

    def __init__(self, name):
        """
        Constructor.

        @param name: camera identifier.  You can get a list of valid
                     identifiers by calling the ROS service /cameras/list.

                     Expected names are right_hand_camera, left_hand_camera
                     and head_camera.  However if the cameras are not
                     identified via the parameter server, they are simply
                     indexed starting at 0.
        """
        self._id = name

        list_svc = rospy.ServiceProxy('/cameras/list', ListCameras)
        rospy.wait_for_service('/cameras/list', timeout=10)
        if not self._id in list_svc().cameras:
            raise AttributeError(
                ("Cannot locate a service for camera name '{0}'. "
                "Close a different camera first and try again.".format(self._id)))

        self._open_svc = rospy.ServiceProxy('/cameras/open', OpenCamera)
        self._close_svc = rospy.ServiceProxy('/cameras/close', CloseCamera)

        self._settings = CameraSettings()
        self._settings.width = 320
        self._settings.height = 200
        self._settings.fps = 20
        self._open = False
```






