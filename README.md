# tuw_laserscan_features
Feature extractor for laserscan message such as lines and line segments

![](res/linedetection_and_stage.png)

## nodes
The package holds currently the following nodes

* linedetection 
* feature_viz
* composed_node

## linedetection
Line detection based on a divide and conquer algoritm and detects lines within a `sensor_msgs/LaserScan` msg and publishes line segments `tuw_geometry_msgs/LineSegments` 

* Input: 
  * Topic: scan 
  * Type: `sensor_msgs/LaserScan`
* Output:
  * Topic line_segments
  * Type: `tuw_geometry_msgs/LineSegments` 
* Parameter: 
  * threshold_split_neighbor, bool, default: false
  * threshold_split, double, default: 0.05, split threshold [m]
  * min_length, double,  default: 0.1,  min line length [m]
  * min_points_per_line, int , default: 20, minimal number of points supporting a line
  * min_points_per_unit, int,  default: 10, minimal number of points supporting a unit

[![Linedetection](res/linedetection_and_stage_youtube.png)](https://www.youtube.com/watch?v=5FfFnKGjazg )
[![Linedetection](https://img.youtube.com/vi/5FfFnKGjazg/0.jpg)](https://www.youtube.com/watch?v=5FfFnKGjazg )

## feature_viz
It utilizes the tuw::figure class for a visualization

* Input: 
  * Topic: scan 
    * Type: `sensor_msgs/LaserScan`
  * Topic line_segments
    * Type: `tuw_geometry_msgs/LineSegments` 

## composed_node
A demo which hows how to combine multiple nodes into one process. In this case the linedetection and feature_viz node are fused into one process.