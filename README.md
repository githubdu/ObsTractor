# Simulation Of Environment Modeling

Refresh the Grid-base Map of a manipulator workspace in real-time

- [lwr_description]
   urdf model of kuka iiwa robot for gazebo
 
- [lwr_gazebo]
   Simulated workstation in gazebo including kuka/kinects/obstacles
   
- [offline mapping]
   Projective relation between grids of workspace and pixels of sensors
   
- [iiwa_7_filter]
   filter out robot itself from the point clouds
   
- [kinects_obstacle_extractor]
   Environment Modeling





1. devide the workspace to grids

![](https://github.com/githubdu/ObsTractor/blob/master/pictures/1.png)




2. offline mapping between grids and pixels of sensors
	save the result to ./kinects_obstacle_extractor/data/kinect_depth_optical_frame.txt

![](https://github.com/githubdu/ObsTractor/blob/master/pictures/2.png)




3. free space. no obstacle and can be seen by sensors

![](https://github.com/githubdu/ObsTractor/blob/master/pictures/3.png)





4. unknown space. grids in workspace that can not be seen by  sensors

![](https://github.com/githubdu/ObsTractor/blob/master/pictures/4.png)





5. point clouds from sensors. obstacles and robot surface seen by sensors.

![](https://github.com/githubdu/ObsTractor/blob/master/pictures/5.png)





6. occlusion space. grids in workspace that blocked from sensors by obstacles.

![](https://github.com/githubdu/ObsTractor/blob/master/pictures/6.png)





7. filter out robot itself from the point clouds

![](https://github.com/githubdu/ObsTractor/blob/master/pictures/7.png)





8. final grid map. include free space obstacles surface and occlusion space.

![](https://github.com/githubdu/ObsTractor/blob/master/pictures/8.png)






How to launch the Simulation

0, compile source code using catkin

1, load robot and two kinects models in gazebo
    roslaunch lwr_gazebo lwr_kinect_sim_room.launch

2, launch the obstacle extrator
	roslaunch kinects_obstacle_extractor.launch
	
3, if needed, control human in gazebo to move
	roslaunch lwr_gazebo gazebo_human_control.launch
	
4, if needed, run rqt to move the manipulator
	
