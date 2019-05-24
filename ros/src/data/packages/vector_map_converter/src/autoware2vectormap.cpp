/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <vector_map_converter/autoware2vectormap.hpp>

template <class T>
std::vector<T> parse(const std::string& csv_file)
{
    std::vector<T> objs;    
    std::ifstream ifs(csv_file.c_str());
    if( !ifs.good() )
    {
        ROS_WARN_STREAM("could not find file: " << csv_file);
        return objs;        
    }

    std::string line;
    std::getline(ifs, line); // remove first line
    while (std::getline(ifs, line))
    {
        T obj;
        std::istringstream iss(line);
        iss >> obj;
        objs.push_back(obj);
    }
    return objs;
}

void readFiles( const std::string directory_path,
                std::vector<autoware_map_msgs::Area> &areas,
                std::vector<autoware_map_msgs::Lane> &lanes,
                std::vector<autoware_map_msgs::LaneAttributeRelation> &lane_attribute_relations,
                std::vector<autoware_map_msgs::LaneRelation> &lane_relations,
                std::vector<autoware_map_msgs::LaneSignalLightRelation> &lane_signal_relations,
                std::vector<autoware_map_msgs::LaneChangeRelation> &lane_change_relations,
                std::vector<autoware_map_msgs::OppositeLaneRelation> &opposite_lane_relations,
                std::vector<autoware_map_msgs::Point> &points,
                std::vector<autoware_map_msgs::Signal> &signals,
                std::vector<autoware_map_msgs::SignalLight> &signal_lights,
                std::vector<autoware_map_msgs::Wayarea> &wayareas,
                std::vector<autoware_map_msgs::Waypoint> &waypoints,
                std::vector<autoware_map_msgs::WaypointLaneRelation> &waypoint_lane_relations,
                std::vector<autoware_map_msgs::WaypointRelation> &waypoint_relations,
                std::vector<autoware_map_msgs::WaypointSignalRelation> &waypoint_signal_relations
                )
{
    std::string file_path;

    file_path = directory_path + "/points.csv";
    points = parse<autoware_map_msgs::Point>(file_path);
    
    file_path = directory_path + "/lanes.csv";
    lanes = parse<autoware_map_msgs::Lane>(file_path);
    
    file_path = directory_path + "/lane_attribute_relations.csv";
    lane_attribute_relations = parse<autoware_map_msgs::LaneAttributeRelation>(file_path);
    
    file_path = directory_path + "/lane_relations.csv";
    lane_relations = parse<autoware_map_msgs::LaneRelation>(file_path);
    
    file_path = directory_path + "/lane_signal_light_relations.csv";
    lane_signal_relations = parse<autoware_map_msgs::LaneSignalLightRelation>(file_path);
    
    file_path = directory_path + "/lane_change_relations.csv";
    lane_change_relations = parse<autoware_map_msgs::LaneChangeRelation>(file_path);
    
    file_path = directory_path + "/opposite_lane_relations.csv";
    opposite_lane_relations = parse<autoware_map_msgs::OppositeLaneRelation>(file_path);
    
    file_path = directory_path + "/areas.csv";
    areas = parse<autoware_map_msgs::Area>(file_path);
    
    file_path = directory_path + "/signals.csv";
    signals = parse<autoware_map_msgs::Signal>(file_path);
    
    file_path = directory_path + "/signal_lights.csv";
    signal_lights = parse<autoware_map_msgs::SignalLight>(file_path);
    
    file_path = directory_path + "/wayareas.csv";
    wayareas = parse<autoware_map_msgs::Wayarea>(file_path);
    
    file_path = directory_path + "/waypoints.csv";
    waypoints = parse<autoware_map_msgs::Waypoint>(file_path);
    
    file_path = directory_path + "/waypoint_lane_relations.csv";
    waypoint_lane_relations = parse<autoware_map_msgs::WaypointLaneRelation>(file_path);
    
    file_path = directory_path + "/waypoint_relations.csv";
    waypoint_relations = parse<autoware_map_msgs::WaypointRelation>(file_path);
    
    file_path = directory_path + "/waypoint_signal_relations.csv";
    waypoint_signal_relations = parse<autoware_map_msgs::WaypointSignalRelation>(file_path);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "awm_vmap_converter");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    autoware_map::category_t category = autoware_map::Category::NONE;

    std::vector<std::string> file_paths;
    for (int i = 1; i < argc; ++i)
    {
        std::string file_path(argv[i]);
        file_paths.push_back(file_path);
    }
    
    std::string map_dir;
    pnh.param<std::string>("map_dir", map_dir, "./");
    
    std::string save_dir;
    pnh.param<std::string>("save_dir", save_dir, "./");

    bool create_whitelines;
    bool wayareas_from_lanes;
    pnh.param<bool>("create_whitelines", create_whitelines, false);
    pnh.param<bool>("wayareas_from_lanes", wayareas_from_lanes, false);


    std::vector<autoware_map_msgs::Point> points;
    std::vector<autoware_map_msgs::Lane> lanes;
    std::vector<autoware_map_msgs::LaneAttributeRelation> lane_attribute_relations;
    std::vector<autoware_map_msgs::LaneRelation> lane_relations;
    std::vector<autoware_map_msgs::LaneSignalLightRelation> lane_signal_light_relations;
    std::vector<autoware_map_msgs::LaneChangeRelation> lane_change_relations;
    std::vector<autoware_map_msgs::OppositeLaneRelation> opposite_lane_relations;
    std::vector<autoware_map_msgs::Area> areas;
    std::vector<autoware_map_msgs::Signal> signals;
    std::vector<autoware_map_msgs::SignalLight> signal_lights;
    std::vector<autoware_map_msgs::Wayarea> wayareas;
    std::vector<autoware_map_msgs::Waypoint> waypoints;
    std::vector<autoware_map_msgs::WaypointLaneRelation> waypoint_lane_relations;
    std::vector<autoware_map_msgs::WaypointRelation> waypoint_relations;
    std::vector<autoware_map_msgs::WaypointSignalRelation> waypoint_signal_relations;

    std::vector<vector_map_msgs::Area> vmap_areas;
    std::vector<vector_map_msgs::CrossRoad> vmap_cross_roads;
    std::vector<vector_map_msgs::CrossWalk> vmap_cross_walks;
    std::vector<vector_map_msgs::DTLane> vmap_dtlanes;
    std::vector<vector_map_msgs::Lane> vmap_lanes;
    std::vector<vector_map_msgs::Line> vmap_lines;
    std::vector<vector_map_msgs::Node> vmap_nodes;
    std::vector<vector_map_msgs::Point> vmap_points;
    std::vector<vector_map_msgs::Pole> vmap_dummy_poles;
    std::vector<vector_map_msgs::RoadSign> vmap_road_signs;
    std::vector<vector_map_msgs::Signal> vmap_signals;
    std::vector<vector_map_msgs::StopLine> vmap_stop_lines;
    std::vector<vector_map_msgs::UtilityPole> vmap_dummy_utility_poles;
    std::vector<vector_map_msgs::Vector> vmap_vectors;
    std::vector<vector_map_msgs::WayArea> vmap_way_areas;
    std::vector<vector_map_msgs::WhiteLine> vmap_white_lines;

    readFiles(map_dir,
              areas,
              lanes,
              lane_attribute_relations,
              lane_relations,
              lane_signal_light_relations,
              lane_change_relations,
              opposite_lane_relations,
              points,
              signals,
              signal_lights,
              wayareas,
              waypoints,
              waypoint_lane_relations,
              waypoint_relations,
              waypoint_signal_relations
              );

    autoware_map::AutowareMapHandler map_handler;

    map_handler.setFromAutowareMapMsgs(areas,
                                       lanes,
                                       lane_attribute_relations,
                                       lane_change_relations,
                                       lane_relations,
                                       lane_signal_light_relations,
                                       opposite_lane_relations,
                                       points,
                                       signals,
                                       signal_lights,
                                       wayareas,
                                       waypoints,
                                       waypoint_lane_relations,
                                       waypoint_relations,
                                       waypoint_signal_relations
                                       );
    map_handler.resolveRelations();

    getVectorMapMsgs( map_handler,
                      vmap_areas,
                      vmap_cross_roads,
                      vmap_cross_walks,
                      vmap_dtlanes,
                      vmap_lanes,
                      vmap_lines,
                      vmap_nodes,
                      vmap_points,
                      vmap_dummy_poles,
                      vmap_road_signs,
                      vmap_signals,
                      vmap_stop_lines,
                      vmap_dummy_utility_poles,
                      vmap_vectors,
                      vmap_way_areas,
                      vmap_white_lines
                      );

   if( create_whitelines )
   {
     createWhitelines(map_handler, vmap_points, vmap_lines, vmap_white_lines);
   }

   if( wayareas_from_lanes )
   {
     createWayAreasFromLanes(map_handler, vmap_way_areas, vmap_areas, vmap_lines, vmap_points);
   }

    writeVectorMapMsgs( save_dir,
                        vmap_areas,
                        vmap_cross_roads,
                        vmap_cross_walks,
                        vmap_dtlanes,
                        vmap_lanes,
                        vmap_lines,
                        vmap_nodes,
                        vmap_points,
                        vmap_dummy_poles,
                        vmap_road_signs,
                        vmap_signals,
                        vmap_stop_lines,
                        vmap_dummy_utility_poles,
                        vmap_vectors,
                        vmap_way_areas,
                        vmap_white_lines
                        );
}
