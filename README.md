A ROS2 workspace for labbot robot.


### How to install ROS2 TurtleBot3 packages from sources? 
Based on https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html

1. Download turtlebot3.repos into workspace directory (it should containt at least `src` directory):

```bash 
wget https://raw.githubusercontent.com/LRMPUT/labbot/ROS2-Humble/turtlebot3.repos
```

2. Get packages using `vcstools`:

```bash
vcs import src<turtlebot3.repos
```

3. Get dependiences using `rosdep`:

```bash
rosdep update
rosdep install --from-paths src --ignore-src  -y --rosdistro humble
```

4. Source the workspace:
- with the global ROS2 installation (only if building the workspace for the first time):

```bash
source /opt/ros/humble/setup.bash
```
- locally (otherwise):

```bash
source install/setup.bash
```

5. Install packages:

```bash
colcon build --symlink-install
```

thanks to `--symlink-install` changes made in python nodes will take effect without rebuilding the workspace. 

### Odometry
Migration from http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
to https://navigation.ros.org/setup_guides/odom/setup_odom.html

- use `ros2_control`:
"An alternative to manually publishing this information that we recommend is through the ros2_control framework. The ros2_control framework contains various packages for real-time control of robots in ROS 2."
