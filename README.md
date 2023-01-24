## Example codes written at NIT Silchar ROS hands on workshop

There are two main scripts:   
 - [two\_pt\_ctrl.py](test_pub_sub/scripts/two_pt_ctrl.py) : subscribes to **_/current\_goal_** topic and publishes odometry to **_/goal\_reached_** topic  
 - [goal\_generator.py](test_pub_sub/scripts/goal_generator.py) : sends waypoints to the **two\_pt\_ctrl.py** script  




#### Bonus:   
visualization script : [_live\_plot.py_](test_pub_sub/scripts/live_plot.py)

![Visualization of navigation](myimage.gif "Two point navigation in action")
