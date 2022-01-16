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
* [DualShock 4 Wireless-Controller](https://www.playstation.com/de-de/accessories/dualshock-4-wireless-controller/)
<br />

## Run Code

### Karte aufnehmen (Steuerung mit DualShock 4 Wireless-Controller)
#### Simultaneous Localization and Mapping (SLAM)
   ```sh
   roslaunch rtc_project slam.launch gazebo:=true controller_layout:=2 map_file:=/home/lennart/catkin_ws/src/rtc_project/maps/default_map
   ```
* für das Arbeiten mit dem realen Turtlebot muss `gazebo:=false` sein.
* `controller_layout:=2` sorgt dafür, dass der Turtlebot mit den beiden Joysticks gesteuert wird.
* die SHARE - Taste führt den [`map_saver`](http://wiki.ros.org/map_server) aus, sodass die Karte unter `map_file` gespeichert wird.
<br />

### Navigationsziele setzen
   ```sh
   roslaunch rtc_project set_navigation_points.launch points_via_robot:=true gazebo:=true controller_layout:=2 map_file:=/home/lennart/catkin_ws/src/rtc_project/maps/default_map.yaml
   ```
#### RViz
* mit `points_via_robot:=false` werden die Ziele über den *2D Nav Goal* - Pfeil gesetzt.
* unter *Tool Properties* muss dafür das Topic **move_base_simple/set_goal** eingestell werden.
#### Turtlebot
* mit `points_via_robot:=true` können die Navigationsziele mit dem Turtlebot *angefahren* werden.
* mit der SHARE - Taste wird die aktuelle Position und Orientierung des Turtlebots in eine **.txt** gespeichert [publish_pose_2_file](https://github.com/ProfJust/rtc/blob/master/nodes/ue07_navigation_amcl/publish_pose_2_file.py). 
<br />

### Navigation
#### Advanced Monte Carlo Localization (AMCL)
   ```sh
   roslaunch rtc_project navigation.launch gazebo:=true map_file:=/home/lennart/catkin_ws/src/rtc_project/maps/default_map.yaml
   ```
#### Ziele mit Action-Server anfahren 
   ```sh
   rosrun rtc_project turtlebot3_move_base_action_client.py /home/lennart/catkin_ws/src/rtc_project/maps/default_map_path.txt
   ```
* mit [turtlebot3_move_base_action_client](https://github.com/ProfJust/rtc/blob/master/nodes/ue07_navigation_amcl/turtlebot3_move_base_action_client.py) werden alle gesetzen Navigationsziele nacheinander angefahren.
<br />

## Sicherheitsfunktion, falls Marcel wieder nur am Rasen ist ;)
<div align="center">
  <a href="https://github.com/lennart2810/rtc_project/blob/master/scripts/ObjectWarnings.ipynb">
    <img src="appendix/Sicherheitsfunktionen.png" alt="Images" width="500" height="420">
  </a>
</div>
<br />


## Clone Repository
   ```sh
   git clone https://github.com/lennart2810/rtc_project.git
   ```

### Dependencies 
* [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
* [ds4_driver](https://github.com/naoki-mizuno/ds4_driver)

## Credits
* [ProfJust](https://github.com/ProfJust/rtc)
* [ROBOTIS](https://github.com/ROBOTIS-GIT/turtlebot3)
*  Navigation Tuning
* [naoki-mizuno](https://github.com/naoki-mizuno/ds4_driver)

## Contact
* _Lennart Fuhrig_ - [GitHub](https://github.com/lennart2810) 
* _Marcel Heinen_
* _Jonas Klinker_

<p align="right"><a href="#top">back to top</a></p>
