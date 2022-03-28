# Send a GPS goal

## The following 

In one terminal run :

```bash
roslaunch agribot gps_goal.launch
```

In another terminal run the following command to set the initial gps coordinates :

```bash
rostopic pub /local_xy_origin geometry_msgs/PoseStamped '{ header: { frame_id: "/map" }, pose: { position: { x: 6.659000, y: 46.779961 } } }' -1
```

Where x is longitude and y is latitude. 

The source position, here, is the approximate location here :

![src_cadastre](../images/cadastre_point_src_2021-11-04_10-25-02.png)

The destination position, here, is the approximate location here :

![dst_cadastre](../images/cadastre_point_dest_2021-11-04_10-25-52.png)

Then send the goal and run :

```bash
rostopic pub /gps_goal_fix sensor_msgs/NavSatFix "{latitude: 46.780018, longitude: 6.650107}" -1
```

Or with :

```bash
rosrun agribot gps_goal.py --lat 46.780018 --long 6.650107
```


## Sources

- http://wiki.ros.org/gps_goal
- 


