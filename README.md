# Pupil tracking package for ROS
**Please drop a star ⭐ if you like the project.**

[Click here to watch the package in action.](https://www.youtube.com/watch?v=4N1E9Mdwwsg)


  *It should be noted that though most commits are from [suryachereddy](https://github.com/OUR-PALS/OUR-PALS-BOT), 3 others ([darshan1215](https://github.com/darshan1215), [Amulya Ganti](https://github.com/ganti0907) and [Sriya027](https://github.com/Sriya027)) have also contributed for the project (including parts of the code). We thank [Dr. Chakravarthula Kiran](https://github.com/kirandotc) (our faculty guide) and [Srujan Panuganti](https://github.com/srujanpanuganti) for their valuable inputs to the project.*

This package integrates [pupil tracking code](https://github.com/OUR-PALS/Vision_Tracking) with ROS to control a robot by publishing '/cmd_vel' message corresponding to the pupils' position. 

# Building the package
 We only tested this package on ROS Melodic. You can follow the [official documentation](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) for building the package.
# To run the OUR-PALS pupil tracking node

`rosrun ourpals_iris iris_realtime.py`

Notes:
1. We haven't configured ROS Params, and the node publishes */cmd_vel* by default. This topic can be changed manually from the 55th line of *iris_realtime.py*. 
2. step_size, linear_vel and angular vel can be manually be modified from the line 41:43 *iris_realtime.py*.
# For the robot side of the project
*You can use this package with any robot that subscribes a Twist message. However you may use/refer our robot's firmware below.* 

Click [here](https://github.com/OUR-PALS/OUR-PALS-BOT) for the Teensy (robot controller) and Arduino (ultrasonic sensor) firmwares for the robot. 
**This repository also contains the instructions and pin bindings to run the project. It also includes basic instructions to install and setup the RPi 4 for the project**



*For any doubts regarding the project feel free to ask in "issues".*

