Detailed instructions to run the code:

### Step 1: 
Installing Dependencies-
Install Turtlebot3 in ubuntu

### Step 2:
Perform the following steps in the terminal prior to running the main launch files to run the package successfully
export TURTLEBOT3_MODEL=waffle

### Step 3:
Copy the launch files from the launch folder in control to turtlebot3_simulations/turtlebot3_gazebo/launch

### Additional Information
1) in scene#c#.py change line 78 to update the location of test.txt. #can be 1/2/3/4 depending on what cases is to be run.
2) Using c.py, c1.py and func.py. The collision avoidance system is demonstrated using 4 different scenarios simulated on ROS Gazebo. 


## Running the test cases:

-Test Case 1 (Wrong Lane):
roslaunch turtlebot3_gazebo scene1.launch

-Test Case 2 (Swerving into oncoming traffic lane):
roslaunch turtlebot3_gazebo scene2.launch

-Test Case 3 (Sudden Braking):
roslaunch turtlebot3_gazebo scene3.launch

-Test Case 4 (Crossroads):
roslaunch turtlebot3_gazebo scene4.launch







