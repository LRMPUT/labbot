# labbot
Repository branch containing complete ROS2 workspace for LabBot robot as a part of Master’s degree thesis at year 2025

Title: Implementation of the Labbot mobile robot controller in the ROS2 system

Workspace contains package source files for custom labbot_interfaces, Orbbec Astra Camera ROS2 driver adapted to ROS2 Jazzy Jalisco and micro-ROS agent, to make work easier and more consistent. 

In order to use the workspace firstly ROS2 Jazzy Jallisco is necessary to be installed (Ubuntu Noble Numbat 24.04 is recommended operating system as stated by Tier 1 platform list of ROS2).

# LabBot driver topics
The topics used in micro-ROS robot driver are described as follows:
- **/cmd_vel** – `geometry_msgs/msg/Twist.msg`, subscribed, linear and angular velocity used for mobile robot control 
- **/labbot/battery_state** – `labbot_interfaces/msg/BatteryArray.msg`, published, custom battery state containing  identification, type, voltage, current, remaining charge, tresholds and flags
- **/labbot/bumper_state** – `labbot_interfaces/msg/BumperArray.msg`, published, bumper array flags and activation times
- **/labbot/controller_state** – `labbot_interfaces/msg/ControllerStateAray.msg`, published, array of LabBot PID controllers state described in **labbot_interfaces** package
- **/labbot/mcu_temperature** – `std_msgs/msg/Float32.msg`, published current MCU temperature
- **/labbot/motor_state** – `sensor_msgs/msg/JointState.msg`, published, message containing array of motor identification, position, speed and current, used in **robot_state_publisher**  
- **/odom** – `nav_msgs/msg/Odometry.msg`, published, odometry message containing **odom-base_footprint** relations and estimated position, orientation and robot velocities   

# LabBot driver services
Services prepared for robot hardware interaction and their call examples are described as follows:
- **/labbot/get_controller** – `labbot_interfaces/srv/GetController.srv`, used for reading choosen PID controller settings
```
ros2 service call /labbot/get_controller labbot_interfaces/srv/GetController "{controller_id: 0}"
```
- **/labbot/set_controller** – `labbot_interfaces/srv/SetController.srv`, service used for changing choosen PID controller settings
```
ros2 service call /labbot/set_controller labbot_interfaces/srv/SetController  
  "{controller_id: 0, controller_setting: {active: false, k_p: 0.0, k_i: 0.0, k_d: 0.0, val_max: 0.0, val_min: 0.0, multiplier: 0.0}}"
```
- **/labbot/isolate_bumper** – `labbot_interfaces/srv/IsolateBumper.srv`, service allowing deactivation of choosen bumper in safety functions
```
ros2 service call /labbot/isolate_bumper labbot_interfaces/srv/IsolateBumper "{bumper_id: 0, active: True}"
```
- **/labbot/reset_bumper** – `labbot_interfaces/srv/ResetAlarm.srv`, service resetting bumper activation state
```
ros2 service call /labbot/reset_bumper labbot_interfaces/srv/ResetAlarm
```
- **/labbot/reset_overcurrent** – `labbot_interfaces/srv/ResetAlarm.srv`, service resetting overcurrent state for robot drive motors  
```
ros2 service call /labbot/reset_overcurrent labbot_interfaces/srv/ResetAlarm
```

# ROS2 Jazzy Jalisco installation 
To install using official instruction is recommended (consider installing ros-jazzy-desktop):
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

After ROS2 installation it is worth to source ROS2 environment permanently:  
```
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

# ROS2 workspace preparation
In order to provide seamless workspace integration it is recommended to clone this branch into ``/home/user-name/`` and rename the root folder:
```
git clone https://github.com/LRMPUT/labbot  --branch ROS2-Jazzy
mv labbot labbot_ws
cd labbot_ws
```

# Camera libuvc installation
Camera installation according to official repository https://github.com/orbbec/ros2_astra_camera. Scripts wraps official instructions to install dependencies, build the libuvc library and apply USB rules to the camera.
```
bash install_dependencies.sh
```

# Nav2 (Navigation Stack) package installation
According to official instructions:
```
sudo apt update
sudo apt install \
  ros-$ROS_DISTRO-navigation2 \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-nav2-minimal-tb*
```
# slam-toolbox installation
Simple slam-toolbox installation with apt:
```
sudo apt update
sudo apt install ros-jazzy-slam-toolbox
```
# Building the workspace
Inside the workspace root folder, use `colcon build` tool and source the installation (remember to change "directory_ws" in the command):
```
colcon build
echo "source /home/$USER/directory_ws/install/setup.sh" >> ~/.bashrc
```
# Logging to LabBot computer
In order to connect to LabBot computer, firstly remote station must be connected to **jerry-hotspot** network:
- SSID: **jerry-hotspot**
- PASS: **lrm-jerry**

To log remotely to LabBot use SSH tool and the following:
- IPV4: **10.42.0.1**
- USER: **jerry**
- PASS: **jerry**

# Start LabBot robot driver base
This launch file is started automatically with system initialization. In order to prepare LabBot for work this launch file starts the following driver modules:
- **micro_ros_agent** -- communication base for microcontroller-computer using USB-CDC, communication config should not be changed, but if necessary use ``labbot_package/launch/local_start_drivers.launch.xml``
- **robot_state_publisher** -- publishing transform tree (TF) and wheel position, tool uses URDF robot config ``labbot_package/urdf/labbot.xacro``
- **robot_localization** -- linking **odom** frame with robot **base_footprint** frame, config can be edited in ``labbot_package/config/localization_config.yaml``
- **Orbbec Astra RGB-D** camera driver -- starting camera ROS2 driver with predefined configuration in Astra camera launch file
- **depthimage_to_laserscan** -- converting camera data to **LaserScan**
- **slam_toolbox** -- generating environment map in real time, SLAM config is located in ``labbot_package/config/slam_config.yaml``

To start it manually:
```
ros2 launch labbot_package local_start_drivers.launch.xml
```
# Start LaBot navigation type locally
In order to allow robot movement two navigation scripts are available. Only one script should be used at once.

To provide manual control of the robot with teleoperation node use the local script starting **teleop_twist_joy** that should be paired with **joy** started on remote computer:
```
ros2 launch local_start_teleop.launch.xml  # started on LabBot
```
Another navigation script uses **Nav2** package for autonomous navigation that reads **Nav2 Goal** poses and perform the motion. Nav2 config is placed in ``labbot_package/config/nav2_config.yaml``, the package uses **slam_toolbox**.
```
ros2 launch local_start_nav2.launch.xml  # started on LabBot
```
# Start robot visualization and necessary controllers remotelly
In order to interact with the robot two Rviz2 launch files exist in the package, files should be matched with launched navigation type.

To start **joy** and **rviz2** config showing robot state, localization and SLAM map generation use:
```
ros2 launch remote_rviz2_teleop.launch.xml  # started on remote station
```
To start Rviz2 config containing necessary **Nav2** controllers use:
```
ros2 launch ros2 launch remote_rviz2_navigation.launch.xml  # started on remote station
```
# Extending LabBot startup functionality
In order to provide automatic startup of chosen launch files Ubuntu **systemctl** can be used. Simple usage will be described using **local_start_drivers.launch.xml**, which is started with LabBot computer system. 

To create necessary system service, a text file should be created first.
```
sudo vim /etc/systemd/system/labbot_drivers.service
```
Example service file using desired launch file is described in the following way:
```
[Unit]
Description=LabBot drivers startup
After=network-online.target
Wants=network-online.target

[Service]
User=jerry
WorkingDirectory=/home/jerry
Environment=ROS_DOMAIN_ID=7
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && source /home/jerry/labbot_ws/install/setup.bash && exec ros2 launch labbot_package local_start_drivers.launch.xml'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```
It is important to source ROS2 installation and LabBot workspace before launching driver start script. After preparing service file the service boot startup should be enabled:
```
sudo systemctl enable labbot_drivers.service
```
Service can be started and stopped manually:
```
sudo systemctl start labbot_drivers.service
sudo systemctl stop labbot_drivers.service
```
To check the status of prepared service:
```
journalctl -u labbot_drivers.service -f
```

# Netplan config example for connecting LabBot to wider network
To provide alternative WiFi connection Ubuntu **netplan** example can be used. LabBot computer is prepared to initialize this connection first, and if connection fails **jerry_hotspot** is activated.
```
network:
  version: 2
  wifis:
    NM-ae0cdb0a-5b4e-448e-8882-a76bd0b636ae:
      renderer: NetworkManager
      match:
        name: "wlp2s0"
      addresses:
      - "STATIC_IP/SUBNET_MASK"
      dhcp6: true
      ipv6-address-generation: "stable-privacy"
      access-points:
        "NETWORK_SSID":
          auth:
            key-management: "psk"
            password: "NETWORK-PASS"
          networkmanager:
            uuid: "ae0cdb0a-5b4e-448e-8882-a76bd0b636ae"
            name: "NETWORK-SSID"
            passthrough:
              connection.autoconnect-priority: "2"
              connection.autoconnect-retries: "5"
              connection.timestamp: "1757276036"
              ipv4.address1: "STATIC_IP/SUBNET_MASK,GATEWAY"
              ipv4.method: "manual"
              ipv6.ip6-privacy: "-1"
              proxy._: ""
      networkmanager:
        uuid: "ae0cdb0a-5b4e-448e-8882-a76bd0b636ae"
        name: "NETWORK-SSID"
```
# Additional time synchronization if necessary
```
sudo date -s "$(ssh jerry@10.42.0.1 'date --utc +%Y-%m-%d\ %H:%M:%S')"
```
