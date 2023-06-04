# Turtlebot Setup
Login details for both turtlebots: <br />
`username`: turtlebot3 <br />
`password`: turtlebot

## Instructions to connect and operate Turtlebot3 Waffle Pi

This setup is applicable to ROS noetic, and the steps provided apply for the waffle pi model of the Turtlebot3 series. To install ROS1 Noetic and the Turtlebot3 packages associated with it, refer to the [ROS1 Noetic installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu) and the [Turtlebot3 PC setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) respectively. <br />
In connecting to it, it is assumed that a remote PC (host) is used over an already configured network (UTS Wifi in this case). The host PC and the Turtlebot must both be connected to this network. When that is set up, the following steps are taken to configure both ROS networks. For further reference refer to the [ROBOTIS GUIDE](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup).

## Configuring the host PC's ROS Network

1. Open a new terminal in the host PC. 
2. Use `ifconfig` to record the host PC's IP address. The IP address is usually under a WLP (`wlp0s20f3`) connection given as `inet`.
3. Use `nano ~/.bashrc` to modify the `ROS_MASTER_URI` and `ROS_HOSTNAME` to the host PC's IP, like: <br />
  `export ROS_MASTER_URI=http://{IP_ADDRESS_OF_REMOTE_PC}:11311` <br />
  `export ROS_HOSTNAME={IP_ADDRESS_OF_REMOTE_PC}` <br />
  `export TURTLEBOT3_MODEL=waffle_pi` (Change this if using a different turtlebot model) <br />
   **Note:** The last step is done to prevent exporting the model anytime a new process is launched.
4. Save the bashrc file and implement it using the command `source ~/.bashrc`. 

## Configuring the Turtlebot's ROS Network

5. Connect the Intel Joule 570x board to a monitor using a microSD to HDMI cable and to a keyboard and mouse using a USB-A ports in the USB adapter.
7. Power the turtlebot via the OpenCR using the provided charger/battery and use the OpenCR switch to turn the turtlebot on. When the Intel board boots, a terminal will open directly.   
8. If prompted to a login page,use the login `turtlebot3` and password `turtlebot`, for the turtlebot3 waffle pi model. 
9. Once login is successful, open a new terminal, and use `ifconfig` to record the Turtlebot's IP address (Refer to step 2).
10. Use `nano ~/.bashrc` to modify the `ROS_MASTER_URI` and `ROS_HOSTNAME` to the PC's IP and the Turtlebot's IP respectively, like: <br />
  `export ROS_MASTER_URI=http://{IP_ADDRESS_OF_REMOTE_PC}:11311` <br />
  `export ROS_HOSTNAME={IP_ADDRESS_OF_INTEL_570x}` <br />
  `export TURTLEBOT3_MODEL=waffle_pi` <br />
11. Save and implement the bashrc file using the command `source ~/.bashrc`.

## Preparing the Turtlebot for ROS Operation

With the turtlebot on, and its network configured: <br />
12. Run `roscore` on the host PC. <br />
13. Connect to the Turtlebot using `ssh turtlebot3@{IP_ADDRESS_OF_570x}` and then the password `turtlebot` (characters are case sensitive). <br />
14. Using the same terminal, [bringup basic packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup) to start Turtlebot3 applications for remote control using: <br />
  `roslaunch turtlebot3_bringup turtlebot3_robot.launch`
  
The Turtlebot should now able to communicate with ROS on the host PC. To logout of `ssh`, press **Ctrl + D**.
