# robot_controllers

cv_pid.py is a state machine.

<h1>States</h1>

0: rospy.get_time() seems unreliable during the first few seconds until the world loads. Wait for 10s (not actually 10 in simulation)<br>
1: position the robot parallel to the outer line by turning for 3s.<br>
2: PID control<br>
3: Red line has been detected. Stop and detect for pedestrian.<br>
4: Pedestrian has crossed and the robot is moving straight. When the red line is detected go back to pid control state<br>
