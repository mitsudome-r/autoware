# autoware_map package

## opendrive2autoware_converter

opendrive2autoware_converter is a rosnode that convert OpenDRIVE map format (.xodr) to autoware map format (list of .csv files).

### How to use: 
```
rosrun autoware_map opendrive2autoware_converter param1 param2/ param3/ param4

```

+ param1: file path, source OpenDRIVE map (.xodr) file. 
+ param2: folder path, country code definition files, for traffic signs and road marks.
+ param3: folder path, destination folder to write the autoware map .csv files.
+ param4: float, optional, waypoints resolution, default is 0.5

## autoware2vectormap_converter
### Feature
converts autoware_map_msgs to vector_map_msgs

### Subscribed Topics
+ **autoware_map_info/lane** autoware_map_msgs::LaneArray    
+ **autoware_map_info/lane_attr_relation** autoware_map_msgs::LaneAttrRelationArray   
+ **autoware_map_info/lane_relation** autoware_map_msgs::LaneRelationArray   
+ **autoware_map_info/lane_signal_light_relation** autoware_map_msgs::LaneSignalLightRelationArray  
+ **autoware_map_info/lane_change_relation** autoware_map_msgs::LaneChangeRelationArray  
+ **autoware_map_info/opposite_lane_relation** autoware_map_msgs::OppositeLaneRelationArray     
+ **autoware_map_info/point** autoware_map_msgs::PointArray    
+ **autoware_map_info/area** autoware_map_msgs::AreaArray   
+ **autoware_map_info/route** autoware_map_msgs::RouteArray   
+ **autoware_map_info/signal** autoware_map_msgs::SignalArray   
+ **autoware_map_info/signal_light** autoware_map_msgs::SignalLightArray   
+ **autoware_map_info/wayarea** autoware_map_msgs::WayareaArray    
+ **autoware_map_info/waypoint** autoware_map_msgs::WaypointArray    
+ **autoware_map_info/waypoint_lane_relation** autoware_map_msgs::WaypointLaneRelationArray   
+ **autoware_map_info/waypoint_relation** autoware_map_msgs::WaypointRelationArray   
+ **autoware_map_info/waypoint_signal_relation** autoware_map_msgs::WaypointSignalRelationArray

### Published Topics
+ **vector_map** visualization_msgs::MarkerArray  
visualization of vector_map_info
+ **vmap_stat** std_msgs::Bool   
publish true after vector_map_info messages are published
+ **vector_map_info/point** vector_map_msgs::PointArray  
mainly converted from `autoware_map_info/point` topic. Some points are virtually inserted for stop_lines, which does not contain longitdude/latitude information.
+ **vector_map_info/node** vector_map_msgs::NodeArray   
Converted `autoware_map_info/wayponit` topic
+ **vector_map_info/vector** vector_map_msgs::VectorArray   
converted from `autoware_map_info/signal_light` and other related topics.
+ **vector_map_info/line** vector_map_msgs::LineArray    
converted from `autoware_map_info/area`, `autoware_map_info/waypoint`, and other related topics. Contains line information for areas and stop lines
+ **vector_map_info/dtlane** vector_map_msgs::DTLaneArray
Converted from `autoware_map_info/waypoint_relations` and other related topics.   
+ **vector_map_info/lane** vector_map_msgs::LaneArray   
Converted from `autoware_map_info/waypoint_relations` and other related topic.   
+ **vector_map_info/area** vector_map_msgs::AreaArray   
Converted from `autoware_map_info/area` topic.
+ **vector_map_info/way_area** vector_map_msgs::WayAreaArray   
Converted from `autoware_map_info/wayarea` topic. Not tested yet.
+ **vector_map_info/stop_line** vector_map_msgs::StopLineArray   
Converted from `autoware_map_info/waypoint` topic. Creates virtual line from yaw in waypoint_relations and also creates dummy road sign to be linked.
+ **vector_map_info/cross_walk** vector_map_msgs::CrossWalkArray   
Converted from `autoware_map_info/lane_attr_relation` topic.
+ **vector_map_info/signal** vector_map_msgs::SignalArray   
Converted from `autoware_map_info/signal` topic.
+ **vector_map_info/pole** vector_map_msgs::PoleArray   
Dummy pole is created for `vector_map_info/signal`
+ **vector_map_info/cross_road** vector_map_msgs::CrossRoadArray   
Converted using `autoware_map_info/lane_attr_relation`
+ **vector_map_info/road_sign** vector_map_msgs::RoadSignArray   
Dummy road signs are created for stop line

#### Usage
NOTE: autoware_map messages must be published before running this node with current implementation.  
`rosrun map_file autoware_map_loader [FILE]`  
`rosrun autoware_map autoware2vectormap_converter`

#### Related Packages
+ *autoware_map_msgs package*: Defines the message types for Autoware Map Format
+ *map_file package*: Loads semantic map written as Autoware Map Format.


## Code API
This package provides library to access to autoware_map messages, similar to library provided by vector_map package.


### Sample Code
```
#include <autoware_map/autoware_map.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "autwoare_map_subscriber");
    ros::NodeHandle nh;

    autoware_map::AutowareMap awm;

    autoware_map::category_t awm_required_category =  autoware_map::Category::POINT |
                                                      autoware_map::Category::AREA;

    //awm.subscribe(nh, awm_required_category);
    awm.subscribe(nh, awm_required_category, ros::Duration(5));

    //find object from id
    int point_id = 1;
    autoware_map_msgs::Point point = awm.findByKey(autoware_map::Key<autoware_map_msgs::Point>(point_id));
    if(point.point_id == point_id)
    {
        std::cout << point << std::endl;
    }
    else
    {
        std::cerr << "failed to find a point with id: " << point_id << std::endl;
    }

    //find multiple object that satisfies desired conditions
    std::vector<autoware_map_msgs::Area> area_arrays;
    area_arrays = awm.findByFilter( [](const autoware_map_msgs::Area){return true;} );
    for(auto area: area_arrays)
    {
        std::cout << area << std::endl;
    }
}
```
### Code Explanation
```
    autoware_map::category_t awm_required_category =  autoware_map::Category::POINT |
                                                      autoware_map::Category::AREA;
```
Above code enables to specify topics that users would like to subscribe.   
Category is set for each autoware_map_msgs type as follwing:
+ autoware_map::Category::NONE
+ autoware_map::Category::LANE
+ autoware_map::Category::LANE_ATTR_RELATION
+ autoware_map::Category::LANE_RELATION
+ autoware_map::Category::LANE_SIGNAL_LIGHT_RELATION
+ autoware_map::Category::LANE_CHANGE_RELATION
+ autoware_map::Category::OPPOSITE_LANE_RELATION
+ autoware_map::Category::POINT
+ autoware_map::Category::AREA
+ autoware_map::Category::ROUTE
+ autoware_map::Category::SIGNAL
+ autoware_map::Category::SIGNAL_LIGHT
+ autoware_map::Category::WAYAREA
+ autoware_map::Category::WAYPOINT
+ autoware_map::Category::WAYPOINT_LANE_RELATION
+ autoware_map::Category::WAYPOINT_RELATION
+ autoware_map::Category::WAYPOINT_SIGNAL_RELATION
+ autoware_map::Category::ALL

```
//awm.subscribe(nh, awm_required_category);
awm.subscribe(nh, awm_required_category, ros::Duration(5));
```
Above code actually subscribes to specified topics.
The function in the comment blocks the process until messages are recieved from all specified categories,  
whereas the second function blocks for user specified duration.

```
awm.findByKey(autoware_map::Key<autoware_map_msgs::Point>(point_id));
awm.findByFilter( [](const autoware_map_msgs::Area){return true;});
```
The above code allows to retrieve user specified object.
