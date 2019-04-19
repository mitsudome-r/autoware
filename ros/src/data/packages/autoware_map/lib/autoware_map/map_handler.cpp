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
#include <autoware_map/map_handler.hpp>
namespace autoware_map {

template<typename T>
using shared_vector = std::vector<std::shared_ptr<T> >;
template<typename T>
using weak_vector = std::vector<std::weak_ptr<T> >;
template<typename U,typename T>
using shared_unordered_map = std::unordered_map<U,std::shared_ptr<T> >;

template<> Area AutowareMapHandler::findById<Area>(const int id) const
{
    return *(areas_.at(id));
}
template<> Lane AutowareMapHandler::findById<Lane>(const int id) const
{
    return *(lanes_.at(id));
}
template<> Point AutowareMapHandler::findById<Point>(const int id) const
{
    return *(points_.at(id));
}
template<> Signal AutowareMapHandler::findById<Signal>(const int id) const
{
    return *(signals_.at(id));
}
template<> SignalLight AutowareMapHandler::findById<SignalLight>(const int id) const
{
    return *(signal_lights_.at(id));
}
template<> Wayarea AutowareMapHandler::findById<Wayarea>(const int id) const
{
    return *(wayareas_.at(id));
}
template<> Waypoint AutowareMapHandler::findById<Waypoint>(const int id) const
{
    return *(waypoints_.at(id));
}

template<> AreaMsg AutowareMapHandler::findById<AreaMsg>(const int id) const
{
    return *(areas_.at(id));
}
template<> LaneMsg AutowareMapHandler::findById<LaneMsg>(const int id) const
{
    return *(lanes_.at(id));
}
template<> PointMsg AutowareMapHandler::findById<PointMsg>(const int id) const
{
    return *(points_.at(id));
}
template<> SignalMsg AutowareMapHandler::findById<SignalMsg>(const int id) const
{
    return *(signals_.at(id));
}
template<> SignalLightMsg AutowareMapHandler::findById<SignalLightMsg>(const int id) const
{
    return *(signal_lights_.at(id));
}
template<> WayareaMsg AutowareMapHandler::findById<WayareaMsg>(const int id) const
{
    return *(wayareas_.at(id));
}
template<> WaypointMsg AutowareMapHandler::findById<WaypointMsg>(const int id) const
{
    return *(waypoints_.at(id));
}




std::vector<Area> AutowareMapHandler::findByFilter(const Filter<Area>& filter) const
{
    std::vector<Area> vector;
    for (const auto& pair : areas_)
    {
        if (filter(*(pair.second)))
            vector.push_back(*(pair.second));
    }
    return vector;
}

std::vector<Lane> AutowareMapHandler::findByFilter(const Filter<Lane>& filter) const
{
    std::vector<Lane> vector;
    for (const auto& pair : lanes_)
    {
        if (filter(*(pair.second)))
            vector.push_back(*(pair.second));
    }
    return vector;
}
std::vector<Point> AutowareMapHandler::findByFilter(const Filter<Point>& filter) const
{
    std::vector<Point> vector;
    for (const auto& pair : points_)
    {
        if (filter(*(pair.second)))
            vector.push_back(*(pair.second));
    }
    return vector;
}
std::vector<Signal> AutowareMapHandler::findByFilter(const Filter<Signal>& filter) const
{
    std::vector<Signal> vector;
    for (const auto& pair : signals_)
    {
        if (filter(*(pair.second)))
            vector.push_back(*(pair.second));
    }
    return vector;
}
std::vector<SignalLight> AutowareMapHandler::findByFilter(const Filter<SignalLight>& filter) const
{
    std::vector<SignalLight> vector;
    for (const auto& pair : signal_lights_)
    {
        if (filter(*(pair.second)))
            vector.push_back(*(pair.second));
    }
    return vector;
}
std::vector<Wayarea> AutowareMapHandler::findByFilter(const Filter<Wayarea>& filter) const
{
    std::vector<Wayarea> vector;
    for (const auto& pair : wayareas_)
    {
        if (filter(*(pair.second)))
            vector.push_back(*(pair.second));
    }
    return vector;
}
std::vector<Waypoint> AutowareMapHandler::findByFilter(const Filter<Waypoint>& filter) const
{
    std::vector<Waypoint> vector;
    for (const auto& pair : waypoints_)
    {
        if (filter(*(pair.second)))
            vector.push_back(*(pair.second));
    }
    return vector;
}
std::vector<LaneAttributeRelation> AutowareMapHandler::findByFilter(const Filter<LaneAttributeRelation>& filter) const
{
    std::vector<LaneAttributeRelation> vector;
    for (const auto& relation : lane_attribute_relations_)
    {
        if (filter(*relation))
            vector.push_back(*relation);
    }
    return vector;
}
std::vector<LaneChangeRelation> AutowareMapHandler::findByFilter(const Filter<LaneChangeRelation>& filter) const
{
    std::vector<LaneChangeRelation> vector;
    for (const auto& relation : lane_change_relations_)
    {
        if (filter(*relation))
            vector.push_back(*relation);
    }
    return vector;
}
std::vector<LaneRelation> AutowareMapHandler::findByFilter(const Filter<LaneRelation>& filter) const
{
    std::vector<LaneRelation> vector;
    for (const auto& relation : lane_relations_)
    {
        if (filter(*relation))
            vector.push_back(*relation);
    }
    return vector;
}
std::vector<LaneSignalLightRelation> AutowareMapHandler::findByFilter(const Filter<LaneSignalLightRelation>& filter) const
{
    std::vector<LaneSignalLightRelation> vector;
    for (const auto& relation : lane_signal_light_relations_)
    {
        if (filter(*relation))
            vector.push_back(*relation);
    }
    return vector;
}
std::vector<OppositeLaneRelation> AutowareMapHandler::findByFilter(const Filter<OppositeLaneRelation>& filter) const
{
    std::vector<OppositeLaneRelation> vector;
    for (const auto& relation : opposite_lane_relations_)
    {
        if (filter(*relation))
            vector.push_back(*relation);
    }
    return vector;
}
std::vector<WaypointLaneRelation> AutowareMapHandler::findByFilter(const Filter<WaypointLaneRelation>& filter) const
{
    std::vector<WaypointLaneRelation> vector;
    for (const auto& relation : waypoint_lane_relations_)
    {
        if (filter(*relation))
            vector.push_back(*relation);
    }
    return vector;
}
std::vector<WaypointRelation> AutowareMapHandler::findByFilter(const Filter<WaypointRelation>& filter) const
{
    std::vector<WaypointRelation> vector;
    for (const auto& relation : waypoint_relations_)
    {
        if (filter(*relation))
            vector.push_back(*relation);
    }
    return vector;
}
std::vector<WaypointSignalRelation> AutowareMapHandler::findByFilter(const Filter<WaypointSignalRelation>& filter) const
{
    std::vector<WaypointSignalRelation> vector;
    for (const auto& relation : waypoint_signal_relations_)
    {
        if (filter(*relation))
            vector.push_back(*relation);
    }
    return vector;
}


void AutowareMapHandler::resolveRelations()
{
    for (auto relation : lane_attribute_relations_)
    {
        std::shared_ptr<Lane> lane = lanes_.at(relation->lane_id);
        lane->lane_attribute_relations.push_back(relation);
        if(relation->area_id != 0) {
            std::shared_ptr<Area> area = areas_.at(relation->area_id);
            relation->area = area;
        }
        relation->lane = lane;
    }

    for (auto relation : lane_change_relations_)
    {
        std::shared_ptr<Lane> lane = lanes_.at(relation->lane_id);
        std::shared_ptr<Lane> next_lane = lanes_.at(relation->next_lane_id);
        lane->lane_change_relations.push_back(relation);
    }

    for (auto relation : lane_relations_)
    {
        std::shared_ptr<Lane> lane = lanes_.at(relation->lane_id);
        std::shared_ptr<Lane> next_lane = lanes_.at(relation->next_lane_id);
        relation->next_lane = next_lane;
        relation->lane = lane;
        lane->lane_relations.push_back(relation);
    }

    for (auto relation : lane_signal_light_relations_)
    {
        std::shared_ptr<Lane> lane = lanes_.at(relation->lane_id);
        std::shared_ptr<SignalLight> signal_light = signal_lights_.at(relation->signal_light_id);
        relation->lane = lane;
        relation->signal_light = signal_light;
        lane->signal_light_relations.push_back(relation);
        signal_light->signal_light_relations.push_back(relation);
    }

    for (auto relation : opposite_lane_relations_)
    {
        std::shared_ptr<Lane> lane = lanes_.at(relation->lane_id);
        std::shared_ptr<Lane> opposite_lane = lanes_.at(relation->opposite_lane_id);
        lane->opposite_lane_relations.push_back(relation);
        opposite_lane->opposite_lane_relations.push_back(relation);
        relation->lane = lane;
        relation->opposite_lane = opposite_lane;
    }

    for (auto relation : waypoint_lane_relations_)
    {
        std::shared_ptr<Lane> lane = lanes_.at(relation->lane_id);
        std::shared_ptr<Waypoint> waypoint = waypoints_.at(relation->waypoint_id);
        waypoint->waypoint_lane_relations.push_back(relation);
        lane->waypoint_lane_relations.push_back(relation);
        relation->lane = lane;
        relation->waypoint = waypoint;
    }

    for( auto relation : waypoint_relations_)
    {
        std::shared_ptr<Waypoint> waypoint = waypoints_.at(relation->waypoint_id);
        std::shared_ptr<Waypoint> next_waypoint = waypoints_.at(relation->next_waypoint_id);
        waypoint->waypoint_relations.push_back(relation);
        next_waypoint->waypoint_relations.push_back(relation);
        relation->waypoint = waypoint;
        relation->next_waypoint = next_waypoint;
    }

    for(auto relation : waypoint_signal_relations_)
    {
        std::shared_ptr<Waypoint> waypoint = waypoints_.at(relation->waypoint_id);
        std::shared_ptr<Signal> signal = signals_.at(relation->signal_id);
        waypoint->waypoint_signal_relations.push_back(relation);
        signal->waypoint_signal_relations.push_back(relation);
        relation->waypoint = waypoint;
        relation->signal = signal;
    }

    for(auto item : areas_)
    {
        std::shared_ptr<Area> area = item.second;
        for(auto id : area->point_ids )
        {
            area->points.push_back(points_.at(id));
        }
    }

    for (auto item : lanes_)
    {
        std::shared_ptr<Lane> lane = item.second;
        lane->start_waypoint = waypoints_.at(lane->start_waypoint_id);
        lane->end_waypoint = waypoints_.at(lane->end_waypoint_id);
        auto waypoint = lane->start_waypoint.lock();
        while(waypoint->id != lane->end_waypoint_id) {
            lane->waypoints.push_back(waypoint);
            waypoint = waypoint->getNextWaypoint(lane->id);
        }
        lane->waypoints.push_back(waypoint);
    }

    for ( auto item : signal_lights_)
    {
        std::shared_ptr<SignalLight> signal_light = item.second;
        std::shared_ptr<Signal> signal= signals_.at(signal_light->signal_id);
        signal->signal_lights.push_back(signal_light);
        signal_light->point = points_.at(signal_light->point_id); 
        signal_light->signal = signal;
    }

    for ( auto item : wayareas_)
    {
        std::shared_ptr<Wayarea> wayarea = item.second;
        wayarea->area = areas_.at(wayarea->area_id);
    }

    for ( auto item : waypoints_)
    {
        std::shared_ptr<Waypoint> waypoint = item.second;
        waypoint->point = points_.at(waypoint->point_id);
    }
}

void AutowareMapHandler::setFromAutowareMapMsgs(const std::vector<autoware_map_msgs::Area> areas,
                                        const std::vector<autoware_map_msgs::Lane> lanes,
                                        const std::vector<autoware_map_msgs::LaneAttributeRelation> lane_attribute_relations,
                                        const std::vector<autoware_map_msgs::LaneChangeRelation> lane_change_relations,
                                        const std::vector<autoware_map_msgs::LaneRelation> lane_relations,
                                        const std::vector<autoware_map_msgs::LaneSignalLightRelation> lane_signal_light_relations,
                                        const std::vector<autoware_map_msgs::OppositeLaneRelation> opposite_lane_relations,
                                        const std::vector<autoware_map_msgs::Point> points,
                                        const std::vector<autoware_map_msgs::Signal> signals,
                                        const std::vector<autoware_map_msgs::SignalLight> signal_lights,
                                        const std::vector<autoware_map_msgs::Wayarea> wayareas,
                                        const std::vector<autoware_map_msgs::Waypoint> waypoints,
                                        const std::vector<autoware_map_msgs::WaypointLaneRelation> waypoint_lane_relations,
                                        const std::vector<autoware_map_msgs::WaypointRelation> waypoint_relations,
                                        const std::vector<autoware_map_msgs::WaypointSignalRelation> waypoint_signal_relations
                                        )
{
    //Objects
    for (auto area : areas)
    {
        areas_[area.area_id] = std::make_shared<Area>(area);
    }
    for (auto lane : lanes)
    {
        lanes_[lane.lane_id] = std::make_shared<Lane>(lane);
    }
    for (auto signal_light : signal_lights)
    {
        signal_lights_[signal_light.signal_light_id] = std::make_shared<SignalLight>(signal_light);
    }
    for (auto point : points)
    {
        points_[point.point_id] = std::make_shared<Point>(point);
    }
    for (auto signal : signals)
    {
        signals_[signal.signal_id] = std::make_shared<Signal>(signal);
    }
    for (auto wayarea : wayareas)
    {
        wayareas_[wayarea.wayarea_id] = std::make_shared<Wayarea>(wayarea);
    }
    for (auto waypoint : waypoints)
    {
        waypoints_[waypoint.waypoint_id] = std::make_shared<Waypoint>(waypoint);
    }

    //Relations
    for (auto relation : lane_attribute_relations)
    {
        lane_attribute_relations_.push_back(std::make_shared<LaneAttributeRelation>(relation));
    }
    for (auto relation : lane_change_relations)
    {
        lane_change_relations_.push_back(std::make_shared<LaneChangeRelation>(relation));
    }
    for (auto relation : lane_relations)
    {
        lane_relations_.push_back(std::make_shared<LaneRelation>(relation));
    }
    for (auto relation : lane_signal_light_relations)
    {
        lane_signal_light_relations_.push_back(std::make_shared<LaneSignalLightRelation>(relation));
    }
    for (auto relation : opposite_lane_relations)
    {
        opposite_lane_relations_.push_back(std::make_shared<OppositeLaneRelation>(relation));
    }
    for (auto relation : waypoint_lane_relations)
    {
        waypoint_lane_relations_.push_back(std::make_shared<WaypointLaneRelation>(relation));
    }
    for (auto relation : waypoint_relations)
    {
        waypoint_relations_.push_back(std::make_shared<WaypointRelation>(relation));
    }
    for (auto relation : waypoint_signal_relations)
    {
        waypoint_signal_relations_.push_back(std::make_shared<WaypointSignalRelation>(relation));
    }
    // resolveRelations();
}

void AutowareMapHandler::subscribe(ros::NodeHandle& nh, category_t category)
{
    registerSubscriber(nh, category);
    ros::Rate rate(10);
    while (ros::ok() && !hasSubscribed(category))
    {
        ros::spinOnce();
        rate.sleep();
    }
    resolveRelations();
}

void AutowareMapHandler::subscribe(ros::NodeHandle& nh, category_t category, const ros::Duration& timeout)
{
    registerSubscriber(nh, category);
    ros::Rate rate(10);
    ros::Time end = ros::Time::now() + timeout;
    while (ros::ok() && !hasSubscribed(category) && ros::Time::now() < end)
    {
        ros::spinOnce();
        rate.sleep();
    }
    resolveRelations();
}

void AutowareMapHandler::subscribe(ros::NodeHandle& nh, category_t category, const size_t max_retries)
{
    size_t tries = 0;
    registerSubscriber(nh, category);
    ros::Rate rate(10);
    while (ros::ok() && !hasSubscribed(category) && tries++ < max_retries)
    {
        ros::spinOnce();
        rate.sleep();
    }
    resolveRelations();
}


void AutowareMapHandler::registerSubscriber(ros::NodeHandle& nh, category_t category)
{
  if (category & LANE)
  {
    subs_[LANE] = nh.subscribe( "/autoware_map_info/lane", 1, &AutowareMapHandler::updateLane, this);
  }
  if (category & LANE_ATTRIBUTE_RELATION)
  {
    subs_[LANE_ATTRIBUTE_RELATION] = nh.subscribe( "/autoware_map_info/lane_attribute_relation", 1, &AutowareMapHandler::updateLaneAttributeRelation,this );
  }
  if (category & LANE_RELATION)
  {
    subs_[LANE_RELATION] = nh.subscribe( "/autoware_map_info/lane_relation", 1, &AutowareMapHandler::updateLaneRelation,this );
  }
  if (category & LANE_SIGNAL_LIGHT_RELATION)
  {
    subs_[LANE_SIGNAL_LIGHT_RELATION] = nh.subscribe( "/autoware_map_info/lane_signal_light_relation", 1, &AutowareMapHandler::updateLaneSignalLightRelation,this );
  }
  if (category & LANE_CHANGE_RELATION)
  {
    subs_[LANE_CHANGE_RELATION] = nh.subscribe( "/autoware_map_info/lane_change_relation", 1, &AutowareMapHandler::updateLaneChangeRelation,this );
  }
  if (category & OPPOSITE_LANE_RELATION)
  {
    subs_[OPPOSITE_LANE_RELATION] = nh.subscribe( "/autoware_map_info/opposite_lane_relation", 1, &AutowareMapHandler::updateOppositeLaneRelation,this );
  }
  if (category & POINT)
  {
    subs_[POINT] = nh.subscribe( "/autoware_map_info/point", 1, &AutowareMapHandler::updatePoint,this );
  }
  if (category & AREA)
  {
    subs_[AREA] = nh.subscribe( "/autoware_map_info/area", 1, &AutowareMapHandler::updateArea,this );
  }
  if (category & SIGNAL)
  {
    subs_[SIGNAL] = nh.subscribe( "/autoware_map_info/signal", 1, &AutowareMapHandler::updateSignal,this );
  }
  if (category & SIGNAL_LIGHT)
  {
    subs_[SIGNAL_LIGHT] = nh.subscribe( "/autoware_map_info/signal_light", 1, &AutowareMapHandler::updateSignalLight,this );
  }
  if (category & WAYAREA)
  {
    subs_[WAYAREA] = nh.subscribe( "/autoware_map_info/wayarea", 1, &AutowareMapHandler::updateWayarea,this );
  }
  if (category & WAYPOINT)
  {
    subs_[WAYPOINT] = nh.subscribe( "/autoware_map_info/waypoint", 1, &AutowareMapHandler::updateWaypoint,this );
  }
  if (category & WAYPOINT_LANE_RELATION)
  {
    subs_[WAYPOINT_LANE_RELATION] = nh.subscribe( "/autoware_map_info/waypoint_lane_relation", 1, &AutowareMapHandler::updateWaypointLaneRelation,this );
  }
  if (category & WAYPOINT_RELATION)
  {
    subs_[WAYPOINT_RELATION] = nh.subscribe( "/autoware_map_info/waypoint_relation", 1, &AutowareMapHandler::updateWaypointRelation,this );
  }
  if (category & WAYPOINT_SIGNAL_RELATION)
  {
    subs_[WAYPOINT_SIGNAL_RELATION] = nh.subscribe( "/autoware_map_info/waypoint_signal_relation", 1, &AutowareMapHandler::updateWaypointSignalRelation,this );
  }
}

bool AutowareMapHandler::hasSubscribed(category_t category) const
{
    if (category & LANE)
    {
        if (lanes_.empty())
            return false;
    }
    if (category & LANE_ATTRIBUTE_RELATION)
    {
        if (lane_attribute_relations_.empty())
            return false;
    }
    if (category & LANE_RELATION)
    {
        if (lane_relations_.empty())
            return false;
    }
    if (category & LANE_SIGNAL_LIGHT_RELATION)
    {
        if (lane_signal_light_relations_.empty())
            return false;
    }
    if (category & LANE_CHANGE_RELATION)
    {
        if (lane_change_relations_.empty())
            return false;
    }
    if (category & OPPOSITE_LANE_RELATION)
    {
        if (opposite_lane_relations_.empty())
            return false;
    }
    if (category & POINT)
    {
        if (points_.empty())
            return false;
    }
    if (category & AREA)
    {
        if (areas_.empty())
            return false;
    }
    if (category & SIGNAL)
    {
        if (signals_.empty())
            return false;
    }
    if (category & SIGNAL_LIGHT)
    {
        if (signal_lights_.empty())
            return false;
    }
    if (category & WAYAREA)
    {
        if (wayareas_.empty())
            return false;
    }
    if (category & WAYPOINT)
    {
        if (waypoints_.empty())
            return false;
    }
    if (category & WAYPOINT_LANE_RELATION)
    {
        if (waypoint_lane_relations_.empty())
            return false;
    }
    if (category & WAYPOINT_RELATION)
    {
        if (waypoint_relations_.empty())
            return false;
    }
    if (category & WAYPOINT_SIGNAL_RELATION)
    {
        if (waypoint_signal_relations_.empty())
            return false;
    }
    return true;
}
category_t AutowareMapHandler::hasSubscribed() const
{
    category_t category=NONE;
    if (!lanes_.empty())
        category |= LANE;
    if (!lane_attribute_relations_.empty())
        category |= LANE_ATTRIBUTE_RELATION;
    if (!lane_relations_.empty())
        category |= LANE_RELATION;
    if (!lane_signal_light_relations_.empty())
        category |= LANE_SIGNAL_LIGHT_RELATION;
    if (!lane_change_relations_.empty())
        category |= LANE_CHANGE_RELATION;
    if (!opposite_lane_relations_.empty())
        category |= OPPOSITE_LANE_RELATION;
    if (!points_.empty())
        category |= POINT;
    if (!areas_.empty())
        category |= AREA;
    if (!signals_.empty())
        category |= SIGNAL;
    if (!signal_lights_.empty())
        category |= SIGNAL_LIGHT;
    if (!wayareas_.empty())
        category |= WAYAREA;
    if (!waypoints_.empty())
        category |= WAYPOINT;
    if (!waypoint_lane_relations_.empty())
        category |= WAYPOINT_LANE_RELATION;
    if (!waypoint_relations_.empty())
        category |= WAYPOINT_RELATION;
    if (!waypoint_signal_relations_.empty())
        category |= WAYPOINT_SIGNAL_RELATION;

    return category;
}


void AutowareMapHandler::updateLane(const LaneArrayMsg::ConstPtr &msg)
{
  lanes_.clear();
  for (const auto& data : msg->data)
  {
    lanes_[data.lane_id] = std::make_shared<Lane>(data);
  }
}

void AutowareMapHandler::updateLaneAttributeRelation(const LaneAttributeRelationArrayMsg::ConstPtr &msg)
{
  lane_attribute_relations_.clear();
  for (const auto& data : msg->data)
  {
    std::shared_ptr<LaneAttributeRelation> item = std::make_shared<LaneAttributeRelation>(data);
    lane_attribute_relations_.push_back(item);
  }
}

void AutowareMapHandler::updateLaneRelation(const LaneRelationArrayMsg::ConstPtr &msg)
{
  lane_relations_.clear();
  for (const auto& data : msg->data)
  {
    std::shared_ptr<LaneRelation> item = std::make_shared<LaneRelation>(data);
    lane_relations_.push_back(item);
  }
}
void AutowareMapHandler::updateLaneSignalLightRelation(const LaneSignalLightRelationArrayMsg::ConstPtr &msg)
{
  lane_signal_light_relations_.clear();
  for (const auto& data : msg->data)
  {
    std::shared_ptr<LaneSignalLightRelation> item = std::make_shared<LaneSignalLightRelation>(data);
    lane_signal_light_relations_.push_back(item);
  }
}
void AutowareMapHandler::updateLaneChangeRelation(const LaneChangeRelationArrayMsg::ConstPtr &msg)
{
  lane_change_relations_.clear();
  for (const auto& data : msg->data)
  {
    std::shared_ptr<LaneChangeRelation> item = std::make_shared<LaneChangeRelation>(data);
    lane_change_relations_.push_back(item);
  }
}
void AutowareMapHandler::updateOppositeLaneRelation(const OppositeLaneRelationArrayMsg::ConstPtr &msg)
{
  opposite_lane_relations_.clear();
  for (const auto& data : msg->data)
  {
    std::shared_ptr<OppositeLaneRelation> item = std::make_shared<OppositeLaneRelation>(data);
    opposite_lane_relations_.push_back(item);
  }
}
void AutowareMapHandler::updatePoint(const PointArrayMsg::ConstPtr &msg)
{
  points_.clear();
  for (const auto& data : msg->data)
  {
    points_[data.point_id] = std::make_shared<Point>(data);
  }
}
void AutowareMapHandler::updateArea(const AreaArrayMsg::ConstPtr &msg)
{
  areas_.clear();
  for (const auto& data : msg->data)
  {
    areas_[data.area_id] = std::make_shared<Area>(data);
  }
}

void AutowareMapHandler::updateSignal(const SignalArrayMsg::ConstPtr &msg)
{
  signals_.clear();
  for (const auto& data : msg->data)
  {
    signals_[data.signal_id] = std::make_shared<Signal>(data);
  }
}
void AutowareMapHandler::updateSignalLight(const SignalLightArrayMsg::ConstPtr &msg)
{
  signal_lights_.clear();
  for (const auto& data : msg->data)
  {
    signal_lights_[data.signal_light_id] = std::make_shared<SignalLight>(data);
  }
}
void AutowareMapHandler::updateWayarea(const WayareaArrayMsg::ConstPtr &msg)
{
  wayareas_.clear();
  for (const auto& data : msg->data)
  {
    wayareas_[data.wayarea_id] = std::make_shared<Wayarea>(data);
  }
}
void AutowareMapHandler::updateWaypoint(const WaypointArrayMsg::ConstPtr &msg)
{
  waypoints_.clear();
  for (const auto& data : msg->data)
  {
    waypoints_[data.waypoint_id] = std::make_shared<Waypoint>(data);
  }
}
void AutowareMapHandler::updateWaypointRelation(const WaypointRelationArrayMsg::ConstPtr &msg)
{
  waypoint_relations_.clear();
  for (const auto& data : msg->data)
  {
    std::shared_ptr<WaypointRelation> item = std::make_shared<WaypointRelation>(data);
    waypoint_relations_.push_back(item);
  }
}
void AutowareMapHandler::updateWaypointLaneRelation(const WaypointLaneRelationArrayMsg::ConstPtr &msg)
{
  waypoint_lane_relations_.clear();
  for (const auto& data : msg->data)
  {
    std::shared_ptr<WaypointLaneRelation> item = std::make_shared<WaypointLaneRelation>(data);
    waypoint_lane_relations_.push_back(item);
  }
}
void AutowareMapHandler::updateWaypointSignalRelation(const WaypointSignalRelationArrayMsg::ConstPtr &msg)
{
  waypoint_signal_relations_.clear();
  for (const auto& data : msg->data)
  {
    std::shared_ptr<WaypointSignalRelation> item = std::make_shared<WaypointSignalRelation>(data);
    waypoint_signal_relations_.push_back(item);
  }
}


}
