<div id="top"></div>

## About The Project
<div align="center">
  <a href="https://www.turtlebot.com/">
    <img src="appendix/turtlebot3_with_logo.png" alt="Images" width="500" height="200">
  </a>
</div>

### Built With
* [Turtlebot3 - Burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
* [ROS](http://wiki.ros.org/)
* [Python](https://www.python.org)

## Run Code (Gazebo)

### Karte aufnehmen
#### Simultaneous Localization and Mapping (SLAM)
   ```sh
   roslaunch rtc_project ps4_slam.launch gazebo:=true controller_layout:=2 map_file:=/home/lennart/catkin_ws/src/rtc_project/maps/house_map
   ```
über die SHARE - Taste wird die Karte unter `map_file` mit `map_saver` gespeichert.

### Navigationsziele anfahren
   ```sh
   roslaunch rtc_project ps4_set_navigation_points.launch gazebo:=true controller_layout:=2 map_file:=/home/lennart/catkin_ws/src/rtc_project/maps/house_map.yaml
   ```
   
wenn `gazebo:=true`:
   ```sh
   rosnode kill /robot_state_publisher
   ```
da `/tf` sonst von zu vielen nodes gepublished wird!

<!-- 
(muss per Hand ausgeführt werden, da es im launch-file zu früh ausgeführt wird und die Pose dann nicht an rviz gepublished wird)
-->


### Navigation
#### Advanced Monte Carlo Localization (AMCL)
   ```sh
   roslaunch rtc_project navigation.launch gazebo:=true map_file:=/home/lennart/catkin_ws/src/rtc_project/maps/house_map.yaml
   ```
#### Sonar-Point-Cloud in Costmap eintragen
   ```sh
   rosrun rtc sonar_to_costmap.py
   ```
#### Ziele mit Action-Server anfahren 
   ```sh
   rosrun rtc_project turtlebot3_move_base_action_client.py /home/lennart/catkin_ws/src/rtc_project/maps/house_map_path.txt
   ```

## Clone Repository
   ```sh
   git clone https://github.com/lennart2810/rtc_project.git
   ```


### Dependencies 
* [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
* [ds4_driver](https://github.com/naoki-mizuno/ds4_driver)

## Roadmap
- :heavy_check_mark: ReadMe anlegen
- :soon: avoid Obstacles

<div align="center">
  <a href="https://www.turtlebot.com/">
    <img src="appendix/Sicherheitsfunktionen.png" alt="Images" width="500" height="420">
  </a>
</div>


## Credits
* [ProfJust](https://github.com/ProfJust/rtc)
* [ROBOTIS](https://github.com/ROBOTIS-GIT/turtlebot3)
*  Navigation Tuning
* [naoki-mizuno](https://github.com/naoki-mizuno/ds4_driver)

## Contact
_Lennart Fuhrig_ - [GitHub](https://github.com/lennart2810) 

<p align="right"><a href="#top">back to top</a></p>
