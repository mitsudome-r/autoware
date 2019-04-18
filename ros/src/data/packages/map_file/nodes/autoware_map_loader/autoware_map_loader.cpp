/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/console.h>
#include <std_msgs/UInt64.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_map/map_handler.hpp>
#include <autoware_map/util.h>
#include <autoware_map/visualization.h>

#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <autoware_map_msgs/PointArray.h>


namespace
{
void printUsage()
{
    ROS_ERROR_STREAM("Usage:");
    ROS_ERROR_STREAM("rosrun map_file autoware_map_loader [CSV]...");
}

template <class T>
std::vector<T> parse(const std::string& csv_file)
{
    std::ifstream ifs(csv_file.c_str());
    std::string line;
    std::getline(ifs, line); // remove first line
    std::vector<T> objs;
    while (std::getline(ifs, line))
    {
        T obj;
        std::istringstream iss(line);
        iss >> obj;
        objs.push_back(obj);
    }
    return objs;
}

template <class T, class U>
U createObjectArray(const std::string& file_path)
{
    U obj_array;
    // NOTE: Autoware want to use map messages with or without /use_sim_time.
    // Therefore we don't set obj_array.header.stamp.
    // obj_array.header.stamp = ros::Time::now();
    obj_array.header.frame_id = "map";
    obj_array.data = parse<T>(file_path);
    return obj_array;
}


} // namespace

void split(const std::string from, std::vector<std::string> &to,const char delim = ','){
  std::stringstream ss;
  ss << from;
  std::string splitted;
  while (std::getline(ss, splitted, delim))
  {
    to.push_back(splitted);
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "autoware_map_loader");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string file_path_param;

    if (argc < 2 && !pnh.getParam("map_files", file_path_param))
    {
      printUsage();
      return EXIT_FAILURE;
    }

    ros::Publisher lane_pub = nh.advertise<autoware_map_msgs::LaneArray>("autoware_map_info/lane", 1, true);
    ros::Publisher lane_attribute_relation_pub = nh.advertise<autoware_map_msgs::LaneAttributeRelationArray>("autoware_map_info/lane_attribute_relation", 1, true);
    ros::Publisher lane_relation_pub = nh.advertise<autoware_map_msgs::LaneRelationArray>("autoware_map_info/lane_relation", 1, true);
    ros::Publisher lane_signal_light_relation_pub = nh.advertise<autoware_map_msgs::LaneSignalLightRelationArray>("autoware_map_info/lane_signal_light_relation", 1, true);
    ros::Publisher lane_change_relation_pub = nh.advertise<autoware_map_msgs::LaneChangeRelationArray>("autoware_map_info/lane_change_relation", 1, true);
    ros::Publisher opposite_lane_relation_pub = nh.advertise<autoware_map_msgs::OppositeLaneRelationArray>("autoware_map_info/opposite_lane_relation", 1, true);
    ros::Publisher point_pub = nh.advertise<autoware_map_msgs::PointArray>("autoware_map_info/point", 1, true);
    ros::Publisher area_pub = nh.advertise<autoware_map_msgs::AreaArray>("autoware_map_info/area", 1, true);
    ros::Publisher signal_pub = nh.advertise<autoware_map_msgs::SignalArray>("autoware_map_info/signal", 1, true);
    ros::Publisher signal_light_pub = nh.advertise<autoware_map_msgs::SignalLightArray>("autoware_map_info/signal_light", 1, true);
    ros::Publisher wayarea_pub = nh.advertise<autoware_map_msgs::WayareaArray>("autoware_map_info/wayarea", 1, true);
    ros::Publisher waypoint_pub = nh.advertise<autoware_map_msgs::WaypointArray>("autoware_map_info/waypoint", 1, true);
    ros::Publisher waypoint_lane_relation_pub = nh.advertise<autoware_map_msgs::WaypointLaneRelationArray>("autoware_map_info/waypoint_lane_relation", 1, true);
    ros::Publisher waypoint_relation_pub = nh.advertise<autoware_map_msgs::WaypointRelationArray>("autoware_map_info/waypoint_relation", 1, true);
    ros::Publisher waypoint_signal_relation_pub = nh.advertise<autoware_map_msgs::WaypointSignalRelationArray>("autoware_map_info/waypoint_signal_relation", 1, true);
    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("autoware_map", 1, true);
    ros::Publisher stat_pub = nh.advertise<std_msgs::UInt64>("awmap_stat", 1, true);

    std::vector<std::string> file_paths;
    for (int i = 1; i < argc; ++i)
    {
        std::string file_path(argv[i]);
        file_paths.push_back(file_path);
    }
    //add file list from param
    std::vector<std::string> splitted_param;
    split(file_path_param, splitted_param);
    for(auto file_name: splitted_param){
       file_paths.push_back(file_name);
       std::cout << file_name << std::endl;
    }

    autoware_map::category_t category = autoware_map::Category::NONE;
    for (const auto& file_path : file_paths)
    {
        std::string file_name(basename(file_path.c_str()));
        if (file_name == "points.csv")
        {
            autoware_map_msgs::PointArray points = createObjectArray<autoware_map_msgs::Point, autoware_map_msgs::PointArray>(file_path);
            if(!points.data.empty())
            {
                point_pub.publish(points);
                category |= autoware_map::Category::POINT;
            }
        }
        else if(file_name == "lanes.csv")
        {
            autoware_map_msgs::LaneArray lanes = createObjectArray<autoware_map_msgs::Lane, autoware_map_msgs::LaneArray>(file_path);
            if(!lanes.data.empty())
            {
                lane_pub.publish(lanes);
                category |= autoware_map::Category::LANE;
            }
        }
        else if(file_name == "lane_attribute_relations.csv")
        {
            autoware_map_msgs::LaneAttributeRelationArray lane_attribute_relations = createObjectArray<autoware_map_msgs::LaneAttributeRelation, autoware_map_msgs::LaneAttributeRelationArray>(file_path);
            if(!lane_attribute_relations.data.empty())
            {
                lane_attribute_relation_pub.publish(lane_attribute_relations);
                category |= autoware_map::Category::LANE_ATTRIBUTE_RELATION;
            }
        }
        else if(file_name == "lane_relations.csv")
        {
            autoware_map_msgs::LaneRelationArray lane_relations = createObjectArray<autoware_map_msgs::LaneRelation, autoware_map_msgs::LaneRelationArray>(file_path);
            if(!lane_relations.data.empty())
            {
                lane_relation_pub.publish(lane_relations);
                category |= autoware_map::Category::LANE_RELATION;
            }
        }
        else if(file_name == "lane_signal_light_relations.csv")
        {
            autoware_map_msgs::LaneSignalLightRelationArray lane_signal_relations = createObjectArray<autoware_map_msgs::LaneSignalLightRelation, autoware_map_msgs::LaneSignalLightRelationArray>(file_path);
            if(!lane_signal_relations.data.empty())
            {
                lane_signal_light_relation_pub.publish(lane_signal_relations);
                category |= autoware_map::Category::LANE_SIGNAL_LIGHT_RELATION;
            }
        }
        else if(file_name == "lane_change_relations.csv" )
        {
            autoware_map_msgs::LaneChangeRelationArray lane_change_relations = createObjectArray<autoware_map_msgs::LaneChangeRelation, autoware_map_msgs::LaneChangeRelationArray>(file_path);
            if(!lane_change_relations.data.empty())
            {
                lane_change_relation_pub.publish(lane_change_relations);
                category |= autoware_map::Category::LANE_CHANGE_RELATION;
            }
        }
        else if(file_name == "opposite_lane_relations.csv")
        {
            autoware_map_msgs::OppositeLaneRelationArray opposite_lane_relations = createObjectArray<autoware_map_msgs::OppositeLaneRelation, autoware_map_msgs::OppositeLaneRelationArray>(file_path);
            if(!opposite_lane_relations.data.empty())
            {
                opposite_lane_relation_pub.publish(opposite_lane_relations);
                category |= autoware_map::Category::OPPOSITE_LANE_RELATION;
            }
        }
        else if(file_name == "areas.csv")
        {
            autoware_map_msgs::AreaArray areas = createObjectArray<autoware_map_msgs::Area, autoware_map_msgs::AreaArray>(file_path);

            if(!areas.data.empty())
            {
                area_pub.publish(areas);
                category |= autoware_map::Category::AREA;
            }
        }
        else if(file_name == "signals.csv")
        {
            autoware_map_msgs::SignalArray signals = createObjectArray<autoware_map_msgs::Signal, autoware_map_msgs::SignalArray>(file_path);
            if(!signals.data.empty()) {
                signal_pub.publish(signals);
                category |= autoware_map::Category::SIGNAL;
            }
        }
        else if(file_name == "signal_lights.csv")
        {
            autoware_map_msgs::SignalLightArray signal_lights = createObjectArray<autoware_map_msgs::SignalLight, autoware_map_msgs::SignalLightArray>(file_path);
            if(!signal_lights.data.empty())
            {
                signal_light_pub.publish(signal_lights);
                category |= autoware_map::Category::SIGNAL_LIGHT;
            }
        }
        else if(file_name == "wayareas.csv")
        {
            autoware_map_msgs::WayareaArray wayareas = createObjectArray<autoware_map_msgs::Wayarea, autoware_map_msgs::WayareaArray>(file_path);
            if(!wayareas.data.empty())
            {
                wayarea_pub.publish(wayareas);
                category |= autoware_map::Category::WAYAREA;
            }
        }
        else if(file_name == "waypoints.csv")
        {

            autoware_map_msgs::WaypointArray waypoints= createObjectArray<autoware_map_msgs::Waypoint, autoware_map_msgs::WaypointArray>(file_path);
            if(!waypoints.data.empty())
            {
                waypoint_pub.publish(waypoints);
                category |= autoware_map::Category::WAYPOINT;
            }
        }
        else if(file_name == "waypoint_lane_relations.csv")
        {
            autoware_map_msgs::WaypointLaneRelationArray waypoint_lane_relations= createObjectArray<autoware_map_msgs::WaypointLaneRelation, autoware_map_msgs::WaypointLaneRelationArray>(file_path);
            if(!waypoint_lane_relations.data.empty())
            {
                waypoint_lane_relation_pub.publish(waypoint_lane_relations);
                category |= autoware_map::Category::WAYPOINT_LANE_RELATION;
            }
        }
        else if(file_name == "waypoint_relations.csv")
        {
            autoware_map_msgs::WaypointRelationArray waypoint_relations = createObjectArray<autoware_map_msgs::WaypointRelation, autoware_map_msgs::WaypointRelationArray>(file_path);
            if(!waypoint_relations.data.empty())
            {
                waypoint_relation_pub.publish(waypoint_relations);
                category |= autoware_map::Category::WAYPOINT_RELATION;
            }
        }
        else if(file_name == "waypoint_signal_relations.csv")
        {
            autoware_map_msgs::WaypointSignalRelationArray waypoint_signal_relations = createObjectArray<autoware_map_msgs::WaypointSignalRelation, autoware_map_msgs::WaypointSignalRelationArray>(file_path);
            if(!waypoint_signal_relations.data.empty())
            {
                waypoint_signal_relation_pub.publish(waypoint_signal_relations);
                category |= autoware_map::Category::WAYPOINT_SIGNAL_RELATION;
            }
        }
        else
            ROS_ERROR_STREAM("unknown csv file: " << file_path);
    }

    std_msgs::UInt64 stat;
    stat.data = category;
    stat_pub.publish(stat);

    autoware_map::AutowareMapHandler autoware_map;
    autoware_map.subscribe(nh, category);

    visualization_msgs::MarkerArray marker_array;
    insertMarkerArray(marker_array, createPointMarkerArray(autoware_map, Color::YELLOW));
    insertMarkerArray(marker_array, createWaypointMarkerArray(autoware_map, Color::GREEN));
    insertMarkerArray(marker_array, createWaypointSignalRelationMarkerArray(autoware_map, Color::RED));
    insertMarkerArray(marker_array, createWaypointRelationMarkerArray(autoware_map, Color::YELLOW));
    insertMarkerArray(marker_array, createSignalMarkerArray(autoware_map));
    insertMarkerArray(marker_array, createAreaMarkerArray(autoware_map, Color::WHITE));

    marker_array_pub.publish(marker_array);


    ros::spin();

    return EXIT_SUCCESS;
}
