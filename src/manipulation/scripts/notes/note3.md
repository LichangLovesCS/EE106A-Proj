# EE106a Project Notes 3 by Lingyao Zhang

Tag： Robotics 

---

1. image message is different between OpenCV and ROS. We need to transform them.
[how do we transfrom them?](http://sdk.rethinkrobotics.com/wiki/Display_Image_-_Code_Walkthrough)
I found another useful reference [here](http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython).

2. Network is down. Cannot connect to the core of Baxter.
I have gone inside Baxter by typing the command.
```
./baxter.sh
```
When I am trying :
```
rosrun baxter_tools enable_robot.py -e
```
It says
```
Unable to register with master node [http://robotbaxter.local:11311]: master may not be running yet. Will keep trying.
```
Then I try:
```
roscore
```

It tells me
```
Unable to contact my own server at [http://m92p-bax.local:40049/].
This usually means that the network is not configured properly.

A common cause is that the machine cannot ping itself.  Please check
for errors by running:

	ping m92p-bax.local

For more tips, please see

	http://www.ros.org/wiki/ROS/NetworkSetup
```
Then I try
```
ping m92p-bax.local
```
It says
```
unknown host
```





