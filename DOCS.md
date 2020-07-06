# Docs

## Map

* Occcupancy grid represented as an integer in the range 0-100
    * 0 free
    * 100 occupied
    * -1 unknown

* map is a static map - does not change

## Hector Slam

Creating the map launch:

```
roslaunch hector_slam_launch tutorial.launch
```
Save the map:
```
rosrun map_server map_saver -f $NAME
```

Config params for the **hector_slam** are configured in the
**/mapping_default.launch** file. It's crucial to set these params right:

```
<arg name="odom_frame" default="base_link"/>
<arg name="base_frame" default="base_link"/>
<arg name="map_size" default="1024"/>
<node pkg="tf" type="static_transform_publisher" name="base_to_laser_broadcaster" args="0 0 0 0 0 0 base_link laser 100"/>
```


## Navigation

**move_base** node is responsible to to move the robot from it's current
position to a goal position. It's a **SimpleActionServer** which take a goal
pose with message type *geometry_msgs/PoseStamped*:

```
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp:
now, frame_id: "map"}, pose: {position: {x: 4.34, y: -0.59, z: 2.30}, orientation: {w: 1.0}}}'
```

The Action Server provides the topic **move_base/goal** which is the input of
the Navigation Stack.

### Global Planner

Is in charge of calculating a safe path in order to arrive at the goal pose.
This path is calculated before the robot starts moving, so it does not takes
into account the readings of the sensors while moving.

### Costmap

Represents places that are safe for the robot to be in a grid of cells. Each
cell in a costmap has an integer value in the range 0-255:

* 255 No Information
* 254 Lethal obstacle - a collision-causing obstacle is sensed in the cell
* 0 Free space

The local costsmap is created from the robot's sensor readings.

### The Local Planner

Given a path to follow provided by the global planner and a map, the local
planner provide velocity commands in order to move the robot. Unlike the global
planner, the local planner monitors the odometry and the laser data, and
chooses a collision-free local plan. Can recompute the robot's path on the fly
in order to keep the robott from striking objects.

Local planner uses the local costmap in order to calculate local plans.

### Recovery Behaviors

It might happen that the robot gets stuck while performing the trajectory,
these issue are handled by the **recovery behaviors.*

* clear costmap
* rotate recovery

### Rotate Recovery

Attempts to clear out space by rotating the robot 360 degrees.
