DO NOT modify the example.txt file!
DO NOT modify the example.txt file!
DO NOT modify the example.txt file!
----------------------------------------------------------------------
THis is the instruction for connecting KUKA base and arm (separately)
through commands in the terminal. 



run KUKA base. 
0. Turn on application, on the controller, select ROSKmrriwaController
    Connect pc to wifi (wifiname :  KMP_200_1041344 )

1. Open a new terminal, copy and paste the command between dashed lines
    to copy and paste into terminal: CTRL + SHIFT + C/V
----------------------------------------------------------------------
source devel/setup.bash
roscore -p 30001
----------------------------------------------------------------------

2. At this point, check is appication is connceted on the controller 
    message on controller : all nodes connected to ros 

3. Open a new terminal, copy and paste the command between dashed lines
-----------------------------------------------------------------------
roslaunch kmriiwa_bringup kmriiwa_bringup.launch
rostopic pub -r 10 /kmriiwa/base/command/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
------------------------------------------------------------------------
note : instead of returnning, copy the first line  then double click TAB





run KUKA arm.
0. Turn on application (on the controller, select ROSKmrriwaController)
    Connect pc to wifi (wifiname :  KMP_200_1041344 )

1. Open a new terminal, copy and paste the command between dashed lines
----------------------------------------------------------------------
source devel/setup.bash
roscore -p 30001
----------------------------------------------------------------------

2. At this point, check is appication is connceted on the controller 
    message on controller : all nodes connected to ros 

3. Open a new terminal, copy and paste the command between dashed lines
-----------------------------------------------------------------------
rostopic pub -r 10 /kmriiwa/arm/command/JointPosition kmriiwa_msgs/JointPosition "header:
  se: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
a1: 0.0
a2: 0.0
a3: 0.0
a4: 0.0
a5: 0.0
a6: 0.0
a7: 0.0" 
-----------------------------------------------------------------------
note : instead of returnning, copy the first line then double click TAB
       the maximum number accepted  for each joint is 2





# KUKA Connection Instructions


**DO NOT modify the example.txt file!**
**DO NOT modify the example.txt file!**
**DO NOT modify the example.txt file!**
----------------------------------------------------------------------
This is the instruction for connecting KUKA base and arm (separately) through commands in the terminal. 



## Connect and run KUKA base

### Step 0. Turn on application
    - Turn on application, on the controller, select ROSKmrriwaController
    - Connect pc to wifi (wifiname : KMP_200_1041344 )

### Step 1. Open a new terminal
    - Open a new terminal, copy and paste the command between dashed lines
    - to copy and paste into terminal: CTRL + SHIFT + C/V
----------------------------------------------------------------------
    source devel/setup.bash
    roscore -p 30001
----------------------------------------------------------------------

### Step 2. Check connection
    - At this point, check if appication is connected on the controller 
    - message on controller : all nodes connected to ros master

### Step 3. Open a new terminal
    - Open a new terminal, copy and paste the command between dashed lines
----------------------------------------------------------------------
    roslaunch kmriiwa_bringup kmriiwa_bringup.launch
    rostopic pub -r 10 /kmriiwa/base/command/cmd_vel geometry_msgs/Twist "linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0" 
------------------------------------------------------------------------
Note : - Instead of pressing Enter after pasting the command, copy the first line and then press TAB to complete the command





## Connect and run KUKA arm

### Step 0. Turn on application
    - Turn on application, on the controller, select ROSKmrriwaController
    - Connect pc to wifi (wifiname : KMP_200_1041344 )

### Step 1. Open a new terminal
    - Open a new terminal, copy and paste the command between dashed lines
    - to copy and paste into terminal: CTRL + SHIFT + C/V
----------------------------------------------------------------------
    source devel/setup.bash
    roscore -p 30001
----------------------------------------------------------------------

### Step 2. Check connection
    - At this point, check if appication is connected on the controller 
    - message on controller : all nodes connected to ros master

### Step 3. Open a new terminal
    - Open a new terminal, copy and paste the command between dashed lines
----------------------------------------------------------------------
    roslaunch kmriiwa_bringup kmriiwa_bringup.launch
    rostopic pub -r 10 /kmriiwa/arm/command/JointPosition kmriiwa_msgs/JointPosition "header:
      se: 0
      stamp: {secs: 0, nsecs: 0}
      frame_id: ''
    a1: 0.0
    a2: 0.0
    a3: 0.0
    a4: 0.0
    a5: 0.0
    a6: 0.0
    a7: 0.0" 
-----------------------------------------------------------------------
Note : - Instead of pressing Enter after pasting the command, copy the first line and then press TAB to complete the command
       - The maximum number accepted for each joint is 2



## KUKA rosrun
### Step 0. 
    - When the goal is to execute the command based on the python file, the command for step 3 should be:
----------------------------------------------------------------------
    rosrun replace_with_your_package_name replace_with_your_python_file_name.py
----------------------------------------------------------------------
### Step 1. 
    - If the pythin file is not executable, use the chmod +x command:
----------------------------------------------------------------------
    chmod +x replace_with_your_python_file_name.py
----------------------------------------------------------------------









End -10/4/2024
----------------------------------------------------------------------
