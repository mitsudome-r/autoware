/*
 * opendrive_road.cpp
 *
 *  Created on: Feb 11, 2019
 *      Author: hatem
 */

#include "opendrive2autoware_converter/opendrive_road.h"
#include <fstream>

namespace autoware_map
{

OpenDriveRoad::OpenDriveRoad(TiXmlElement* main_element)
{
	name_ = XmlHelpers::GetStringAttribute(main_element, "name", "");
	id_ = XmlHelpers::GetIntAttribute(main_element, "id", 0);
	junction_id_ = XmlHelpers::GetIntAttribute(main_element, "junction", -1);
	length_ = XmlHelpers::GetDoubleAttribute(main_element, "length", 0.0);

	//Read Links
	std::vector<TiXmlElement*> sub_elements;
	std::vector<TiXmlElement*> lane_elements;
	std::vector<TiXmlElement*> elements;

	XmlHelpers::FindFirstElement("link", main_element, elements);
	if(elements.size() > 0)
	{
		std::vector<TiXmlElement*> pred_elements, succ_elements;

		XmlHelpers::FindElements("predecessor", elements.at(0)->FirstChildElement(), pred_elements);
		for(unsigned int j=0; j < pred_elements.size(); j++)
		{
			predecessor_road_.push_back(FromRoadLink(pred_elements.at(j)));
		}

		XmlHelpers::FindElements("successor", elements.at(0)->FirstChildElement(), succ_elements);
		for(unsigned int j=0; j < succ_elements.size(); j++)
		{
			successor_road_.push_back(ToRoadLink(succ_elements.at(j)));
		}
	}

	elements.clear();
	XmlHelpers::FindFirstElement("planView", main_element, elements);
	if(elements.size() > 0)
	{
		//Get Geometries
		std::vector<TiXmlElement*> geom_elements;
		XmlHelpers::FindElements("geometry", elements.at(0), geom_elements);
		for(unsigned int j=0; j < geom_elements.size(); j++)
		{
			geometries_.push_back(Geometry(geom_elements.at(j)));
		}
	}

	elements.clear();
	XmlHelpers::FindFirstElement("elevationProfile", main_element, elements);
	if(elements.size() > 0)
	{
		//Get Geometries
		std::vector<TiXmlElement*> elev_elements;
		XmlHelpers::FindElements("elevation", elements.at(0), elev_elements);
		for(unsigned int j=0; j < elev_elements.size(); j++)
		{
			elevations_.push_back(Elevation(elev_elements.at(j)));
		}
	}

	int lanes_index_count = 1;
	elements.clear();
	XmlHelpers::FindFirstElement("lanes", main_element, elements);
	if(elements.size()>0)
	{
		//laneOffsets
		std::vector<TiXmlElement*> offsets;
		XmlHelpers::FindElements("laneOffsets", elements.at(0), offsets);
		for(unsigned int j=0; j < offsets.size(); j++)
		{
			laneOffsets_.push_back(LaneOffset(offsets.at(j)));
		}

		//std::cout << "LaneOffsets Loaded with: " << laneOffsets_.size() <<" offsets."<<std::endl;

		//laneSections and lanes
		std::vector<TiXmlElement*> sections;
		XmlHelpers::FindElements("laneSection", elements.at(0), sections);

		for(unsigned int j=0; j < sections.size(); j++)
		{
			double curr_section_s = XmlHelpers::GetDoubleAttribute(sections.at(j), "s", 0.0);
			double next_section_s = 0.0;
			double section_length = 0.0;

			if(j+1 < sections.size())
			{
				next_section_s = XmlHelpers::GetDoubleAttribute(sections.at(j+1), "s", 0.0);
				section_length = next_section_s - curr_section_s;
			}
			else
			{
				section_length = length_ - curr_section_s;
			}

			if(section_length <= 0)
			{
				std::cout << "Load From OpenDrive with Section length is 0.0 !!" << std::endl;
				continue;
			}

			sections_.push_back(RoadSection(sections.at(j),curr_section_s, section_length, j));

			if(j > 0)
			{
				sections_.at(sections_.size()-1).p_prev_section = &sections_.at(sections_.size()-2);
				sections_.at(sections_.size()-2).p_next_section = &sections_.at(sections_.size()-1);
			}
		}

		//std::cout << "LaneSections Loaded with: " << sections.size() <<" Sections , And: " << lanes_.size() << " Lanes" <<std::endl;
	}

	elements.clear();
	XmlHelpers::FindFirstElement("signals", main_element, elements);
	if(elements.size()>0)
	{
		sub_elements.clear();
		XmlHelpers::FindElements("signal", main_element, sub_elements);
		for(unsigned int j=0; j < sub_elements.size(); j++)
		{
			road_signals_.push_back(Signal(sub_elements.at(j)));
		}

		sub_elements.clear();
		XmlHelpers::FindElements("signalReference", main_element, sub_elements);
		for(unsigned int j=0; j < sub_elements.size(); j++)
		{
			road_signals_references_.push_back(SignalRef(sub_elements.at(j)));
		}
	}

	elements.clear();
	XmlHelpers::FindFirstElement("objects", main_element, elements);
	if(elements.size()>0)
	{
		sub_elements.clear();
		XmlHelpers::FindElements("object", main_element, sub_elements);
		for(unsigned int j=0; j < sub_elements.size(); j++)
		{
			road_objects_.push_back(RoadObject(sub_elements.at(j)));
		}

		sub_elements.clear();
		XmlHelpers::FindElements("objectReference", main_element, sub_elements);
		for(unsigned int j=0; j < sub_elements.size(); j++)
		{
			road_objects_references_.push_back(RoadObjectRef(sub_elements.at(j)));
		}

		sub_elements.clear();
		XmlHelpers::FindElements("tunnel", main_element, sub_elements);
		for(unsigned int j=0; j < sub_elements.size(); j++)
		{
			road_objects_tunnels_.push_back(RoadObjectTunnel(sub_elements.at(j)));
		}

		sub_elements.clear();
		XmlHelpers::FindElements("object", main_element, sub_elements);
		for(unsigned int j=0; j < sub_elements.size(); j++)
		{
			road_objects_bridges_.push_back(RoadObjectBridge(sub_elements.at(j)));
		}
	}

	//std::cout << "Road Loaded With ID: " << id_ << " , Length: " << length_  << std::endl;
}

bool OpenDriveRoad::CreateSingleCenterPoint(const double& _ds, PlannerHNS::WayPoint& _p)
{
	for(unsigned int i=0; i < geometries_.size(); i++)
	{
		if(_ds >= geometries_.at(i).s && _ds <= (geometries_.at(i).s+geometries_.at(i).length))
		{
			if(geometries_.at(i).GetPoint(_ds, _p))
			{
				Elevation* p_elv = GetMatchingElevations(_ds);
				if(p_elv != nullptr)
				{
					_p.pos.z = p_elv->GetHeigh(_ds);
				}
				return true;
			}
		}
	}

	return false;
}

void OpenDriveRoad::CreateRoadCenterInfo(std::vector<RoadCenterInfo>& points_list, const double& resolution)
{
	PlannerHNS::WayPoint p;
	RoadCenterInfo inf;
	points_list.clear();
	for(unsigned int i=0; i < geometries_.size(); i++)
	{
		int n_waypoints = floor(geometries_.at(i).length / resolution);
		double s_inc = geometries_.at(i).s;
		for(int j=0; j< n_waypoints; j++)
		{
			if(geometries_.at(i).GetPoint(s_inc, p))
			{
				Elevation* p_elv = GetMatchingElevations(s_inc);
				if(p_elv != nullptr)
				{
					p.pos.z = p_elv->GetHeigh(s_inc);
				}

				LaneOffset* p_lane_off = GetMatchingLaneOffset(s_inc);
				if(p_lane_off != nullptr)
				{
					inf.offset_width_ = p_lane_off->GetOffset(s_inc);
				}

				inf.ds_ = s_inc;
				inf.center_p_ = p.pos;
				points_list.push_back(inf);
			}
			s_inc+=resolution;
		}

		double remaining_distance = geometries_.at(i).s+geometries_.at(i).length - s_inc;
		if(remaining_distance > 0)
		{
			s_inc += remaining_distance;
			if(geometries_.at(i).GetPoint(s_inc, p))
			{
				Elevation* p_elv = GetMatchingElevations(s_inc);
				if(p_elv != nullptr)
				{
					p.pos.z = p_elv->GetHeigh(s_inc);
				}

				LaneOffset* p_lane_off = GetMatchingLaneOffset(s_inc);
				if(p_lane_off != nullptr)
				{
					inf.offset_width_ = p_lane_off->GetOffset(s_inc);
				}

				inf.ds_ = s_inc;
				inf.center_p_ = p.pos;
				points_list.push_back(inf);
			}
		}
	}
}

void OpenDriveRoad::InsertUniqueFromSectionIds(const int& from_section_id, const OpenDriveLane* curr_lane, PlannerHNS::Lane& _l)
{
	for(unsigned int i=0; i < curr_lane->from_lane_.size(); i++)
	{
		int from_lane_id = curr_lane->from_lane_.at(i).from_lane_id;
		int from_gen_id = (id_*100000 + 50000) + from_section_id * 1000 + from_lane_id * 100;

		if(Exists(_l.fromIds, from_gen_id))
		{
			std::cout << "Redundant Connection, from road: " << id_ <<", to road: " << id_
						<< ", to section: " << from_section_id+1 << ", GenLaneID: " << from_gen_id << std::endl;
		}
		else
		{
			_l.fromIds.push_back(from_gen_id);
		}
	}
}

void OpenDriveRoad::InsertUniqueToSectionIds(const int& to_section_id, const OpenDriveLane* curr_lane, PlannerHNS::Lane& _l)
{
	for(unsigned int i=0; i < curr_lane->to_lane_.size(); i++)
	{
		int to_lane_id = curr_lane->to_lane_.at(i).to_lane_id;
		int to_gen_id = (id_*100000 + 50000) + to_section_id *1000 + to_lane_id * 100;

		if(Exists(_l.toIds, to_gen_id))
		{
			std::cout << "Redundant Connection, from road: " << id_ <<", to road: " << id_
						<< ", from section: " << to_section_id - 1 << ", GenLaneID: " << to_gen_id << std::endl;
		}
		else
		{
			_l.toIds.push_back(to_gen_id);
		}
	}
}

void OpenDriveRoad::InsertUniqueFromRoadIds(const int& curr_section_id, const int& curr_lane_id, PlannerHNS::Lane& _l)
{
	for(unsigned int i=0; i < from_roads.size(); i++)
	{
		int from_lane_id = from_roads.at(i).GetFromLane(curr_lane_id);
		if(from_lane_id != 0)
//			std::cout << "Bad Connection, from road: " << from_roads.at(i).incoming_road_ <<", to road: " << id_ << ", to section: " << curr_section_id << std::endl;
//		else
		{
			//std::cout << "Good Connection, from road: " << from_roads.at(i).incoming_road_ <<", to road: " << id_ << ", to section: " << curr_section_id << std::endl;
			if(from_roads.at(i).outgoing_road_ != id_)
				std::cout << " >>>  Something Very Bad Happened in InsertUniqueFromRoadIds, outgoing_road doesn't match current_road, " << from_roads.at(i).outgoing_road_ << ", " << id_ << std::endl;

			int from_gen_id = (from_roads.at(i).incoming_road_*100000 + 50000) + from_roads.at(i).incoming_section_*1000 + from_lane_id * 100;
			if(Exists(_l.fromIds, from_gen_id))
			{
				std::cout << "Redundant Connection, from road: " << from_roads.at(i).incoming_road_ <<", to road: " << id_
							<< ", to section: " << curr_section_id << ", GenLaneID: " << from_gen_id << std::endl;
			}
			else
			{
				_l.fromIds.push_back(from_gen_id);
			}
		}
	}
}

void OpenDriveRoad::InsertUniqueToRoadIds(const int& curr_section_id, const int& curr_lane_id, PlannerHNS::Lane& _l)
{
	for(unsigned int i=0; i < to_roads.size(); i++)
	{
		int to_lane_id = to_roads.at(i).GetToLane(curr_lane_id);
		if(to_lane_id != 0)
//			std::cout << "Bad Connection, to road: " << to_roads.at(i).outgoing_road_ <<", from road: " << id_ << ", from section: " << curr_section_id << std::endl;
//		else
		{
			//std::cout << "Good Connection, to road: " << to_roads.at(i).outgoing_road_ <<", from road: " << id_ << ", from section: " << curr_section_id << std::endl;
			if(to_roads.at(i).incoming_road_ != id_)
				std::cout << " >>>  Something Very Bad Happened in InsertUniqueToRoadIds, incoming_road doesn't match current_road, " << to_roads.at(i).incoming_road_ << ", " << id_ << std::endl;

			int to_gen_id = (to_roads.at(i).outgoing_road_*100000 + 50000) + 0*1000 + to_lane_id * 100;
			if(Exists(_l.toIds, to_gen_id))
			{
				std::cout << "Redundant Connection, to road: " << to_roads.at(i).outgoing_road_ <<", from road: " << id_
							<< ", from section: " << curr_section_id << ", GenLaneID: " << to_gen_id << std::endl;
			}
			else
			{
				_l.toIds.push_back(to_gen_id);
			}
		}
	}
}

void OpenDriveRoad::InsertUniqueToConnection(const Connection& _connection)
{

	bool bFound = false;
	for(unsigned int j=0; j < to_roads.size(); j++)
	{
		if(to_roads.at(j).incoming_road_== _connection.incoming_road_ && to_roads.at(j).outgoing_road_ == _connection.outgoing_road_)
		{
			for(unsigned int k=0; k < to_roads.at(j).lane_links.size(); k++)
			{
				for(unsigned int lk=0; lk < _connection.lane_links.size(); lk++)
				{
					if(to_roads.at(j).lane_links.at(k).first == _connection.lane_links.at(lk).first && to_roads.at(j).lane_links.at(k).second == _connection.lane_links.at(lk).second)
					{
						bFound = true;
						break;
					}
				}

				if(bFound == true)
					break;
			}

			if(bFound == true)
				break;
		}
	}

	if(!bFound)
	{
		to_roads.push_back(_connection);
	}
	else
	{
		//std::cout << "To Connection already exists, From : " << _connection.incoming_road_ << ", To: " << _connection.outgoing_road_ << std::endl;
	}
}

void OpenDriveRoad::InsertUniqueFromConnection(const Connection& _connection)
{
	bool bFound = false;
	for(unsigned int j=0; j < from_roads.size(); j++)
	{
		if(from_roads.at(j).incoming_road_== _connection.incoming_road_ && from_roads.at(j).outgoing_road_ == _connection.outgoing_road_)
		{
			for(unsigned int k=0; k < from_roads.at(j).lane_links.size(); k++)
			{
				for(unsigned int lk=0; lk < _connection.lane_links.size(); lk++)
				{
					if(from_roads.at(j).lane_links.at(k).first == _connection.lane_links.at(lk).first && from_roads.at(j).lane_links.at(k).second == _connection.lane_links.at(lk).second)
					{
						bFound = true;
						break;
					}
				}

				if(bFound == true)
					break;
			}

			if(bFound == true)
				break;
		}
	}

	if(!bFound)
	{
		from_roads.push_back(_connection);
	}
	else
	{
		//std::cout << "From Connection already exists, From : " << _connection.incoming_road_ << ", To: " << _connection.outgoing_road_ << std::endl;
	}
}

void OpenDriveRoad::CreateRoadLanes(std::vector<PlannerHNS::Lane>& lanes_list)
{
	for(unsigned int i=0; i < sections_.size(); i++)
	{
		RoadSection* p_sec = & sections_.at(i);

		for(unsigned int lj=0; lj < p_sec->left_lanes_.size(); lj++)
		{
			PlannerHNS::Lane op_lane;
			OpenDriveLane* p_l_l = &p_sec->left_lanes_.at(lj);

			if(p_l_l->type == BORDER_LANE)
				continue;

			op_lane.id = (this->id_*100000 + 50000) + p_sec->id_*1000 + p_l_l->id * 100;
			op_lane.num = p_l_l->id;
			op_lane.roadId = id_;

			if(sections_.size() == 1) //handle special case of link to previous and following lane at once
			{
				InsertUniqueFromRoadIds(p_sec->id_, p_l_l->id, op_lane);
				InsertUniqueToRoadIds(p_sec->id_, p_l_l->id, op_lane);
			}
			else if(i==0) //handle special case of link to previous road, and more than one sections exists
			{
				InsertUniqueFromRoadIds(p_sec->id_, p_l_l->id, op_lane);
				InsertUniqueToSectionIds(p_sec->id_+1, p_l_l, op_lane);
			}
			else if(i==sections_.size()-1) //handle special case of link to Next road, and more than one section exists
			{
				InsertUniqueFromSectionIds(p_sec->id_-1, p_l_l, op_lane);
				InsertUniqueToRoadIds(p_sec->id_, p_l_l->id, op_lane);
			}
			else
			{
				InsertUniqueFromSectionIds(p_sec->id_-1, p_l_l, op_lane);
				InsertUniqueToSectionIds(p_sec->id_+1, p_l_l, op_lane);
			}

			lanes_list.push_back(op_lane);
		}

		for(unsigned int rj=0; rj < p_sec->right_lanes_.size(); rj++)
		{
			PlannerHNS::Lane op_lane;
			OpenDriveLane* p_r_l = &p_sec->right_lanes_.at(rj);

			if(p_r_l->type == BORDER_LANE)
				continue;

			op_lane.id = (this->id_*100000 + 50000) + p_sec->id_*1000 + p_r_l->id * 100;
			op_lane.num = p_r_l->id;
			op_lane.roadId = id_;

			if(sections_.size() == 1) //handle special case of link to previous and following lane at once
			{
				InsertUniqueFromRoadIds(p_sec->id_, p_r_l->id, op_lane);
				InsertUniqueToRoadIds(p_sec->id_, p_r_l->id, op_lane);
			}
			else if(i==0) //handle special case of link to previous road, and more than one sections exists
			{
				InsertUniqueFromRoadIds(p_sec->id_, p_r_l->id, op_lane);
				InsertUniqueToSectionIds(p_sec->id_+1, p_r_l, op_lane);
			}
			else if(i==sections_.size()-1) //handle special case of link to Next road, and more than one section exists
			{
				InsertUniqueFromSectionIds(p_sec->id_-1, p_r_l, op_lane);
				InsertUniqueToRoadIds(p_sec->id_, p_r_l->id, op_lane);
			}
			else
			{
				InsertUniqueFromSectionIds(p_sec->id_-1, p_r_l, op_lane);
				InsertUniqueToSectionIds(p_sec->id_+1, p_r_l, op_lane);
			}

			lanes_list.push_back(op_lane);
		}
	}
}

void OpenDriveRoad::GetRoadLanes(std::vector<PlannerHNS::Lane>& lanes_list, const double& resolution)
{
	std::vector<RoadCenterInfo> ref_info;
	CreateRoadCenterInfo(ref_info, resolution);
	CreateRoadLanes(lanes_list);
	int wp_id_seq = 1;

	for(unsigned int i=0; i < ref_info.size(); i++)
	{
		RoadSection* p_sec = GetMatchingSection(ref_info.at(i).ds_);
		if(p_sec != nullptr)
		{
			double accum_offset_width = ref_info.at(i).offset_width_;

			for(unsigned int lj=0; lj < p_sec->left_lanes_.size(); lj++)
			{
				OpenDriveLane* p_l_l = &p_sec->left_lanes_.at(lj);

				int combined_lane_id = (this->id_*100000 + 50000) + p_sec->id_*1000 + p_l_l->id * 100;
				PlannerHNS::Lane* p_op_lane = GetLaneById(combined_lane_id, lanes_list);

				if(p_op_lane != nullptr)
				{
					double lane_width = p_l_l->GetLaneWidth(ref_info.at(i).ds_ - p_sec->s_);
					double center_point_margin = accum_offset_width + (lane_width / 2.0);

					PlannerHNS::WayPoint p;
					p.pos = ref_info.at(i).center_p_;
					double a = p.pos.a+M_PI_2;
					p.pos.x += center_point_margin * cos(a);
					p.pos.y += center_point_margin * sin(a);
					p.collisionCost = lane_width;
					p.id = wp_id_seq++;
					//TODO apply super elevation later

					p_op_lane->width = lane_width;
					p_op_lane->points.push_back(p);

					accum_offset_width += lane_width;
				}
			}

			accum_offset_width = ref_info.at(i).offset_width_;

			for(unsigned int rj=0; rj < p_sec->right_lanes_.size(); rj++)
			{
				OpenDriveLane* p_r_l = &p_sec->right_lanes_.at(rj);
				int combined_lane_id = (this->id_*100000 + 50000) + p_sec->id_*1000 + p_r_l->id * 100;
				PlannerHNS::Lane* p_op_lane = GetLaneById(combined_lane_id, lanes_list);

				if(p_op_lane != nullptr)
				{
					double lane_width = p_r_l->GetLaneWidth(ref_info.at(i).ds_ - p_sec->s_);
					double center_point_margin = accum_offset_width + (lane_width / 2.0);

					PlannerHNS::WayPoint p;
					p.pos = ref_info.at(i).center_p_;
					double a = p.pos.a-M_PI_2;
					p.pos.x += center_point_margin * cos(a);
					p.pos.y += center_point_margin * sin(a);
					p.collisionCost = lane_width;
					p.id = wp_id_seq++;
					//TODO apply super elevation later

					p_op_lane->width = lane_width;
					p_op_lane->points.push_back(p);

					accum_offset_width += lane_width;
				}
			}
		}
	}
}

std::vector<Connection> OpenDriveRoad::GetLastSectionConnections()
{
	RoadSection* p_l_sec =  GetLastSection();
	std::vector<Connection> connections_list;
	if(p_l_sec != nullptr)
	{
		Connection conn;
		conn.incoming_road_ = id_;
		conn.incoming_section_ = p_l_sec->id_;

		for(unsigned int i =0 ; i < p_l_sec->left_lanes_.size(); i++)
		{
			if(p_l_sec->left_lanes_.at(i).to_lane_.size() > 0)
			{
				int _to_id = p_l_sec->left_lanes_.at(i).to_lane_.at(0).to_lane_id;
				conn.lane_links.push_back(std::make_pair(p_l_sec->left_lanes_.at(i).id, _to_id));
			}

			if(conn.lane_links.size() > 0)
			{
				connections_list.push_back(conn);
				conn.lane_links.clear();
			}
		}

		for(unsigned int i =0 ; i < p_l_sec->right_lanes_.size(); i++)
		{
			if(p_l_sec->right_lanes_.at(i).to_lane_.size() > 0)
			{
				int _to_id = p_l_sec->right_lanes_.at(i).to_lane_.at(0).to_lane_id;
				conn.lane_links.push_back(std::make_pair(p_l_sec->right_lanes_.at(i).id, _to_id));
			}

			if(conn.lane_links.size() > 0)
			{
				connections_list.push_back(conn);
				conn.lane_links.clear();
			}
		}
	}

	return connections_list;
}

std::vector<Connection> OpenDriveRoad::GetFirstSectionConnections()
{
	RoadSection* p_f_sec =  GetFirstSection();
	std::vector<Connection> connections_list;
	if(p_f_sec != nullptr)
	{
		Connection conn;
		conn.outgoing_road_ = id_;

		for(unsigned int i =0 ; i < p_f_sec->left_lanes_.size(); i++)
		{
			if(p_f_sec->left_lanes_.at(i).from_lane_.size() > 0)
			{
				int _from_id = p_f_sec->left_lanes_.at(i).from_lane_.at(0).from_lane_id;
				conn.lane_links.push_back(std::make_pair(_from_id, p_f_sec->left_lanes_.at(i).id));
			}

			if(conn.lane_links.size() > 0)
			{
				connections_list.push_back(conn);
				conn.lane_links.clear();
			}
		}

		for(unsigned int i =0 ; i < p_f_sec->right_lanes_.size(); i++)
		{
			if(p_f_sec->right_lanes_.at(i).from_lane_.size() > 0)
			{
				int _from_id = p_f_sec->right_lanes_.at(i).from_lane_.at(0).from_lane_id;
				conn.lane_links.push_back(std::make_pair(_from_id, p_f_sec->right_lanes_.at(i).id));
			}

			if(conn.lane_links.size() > 0)
			{
				connections_list.push_back(conn);
				conn.lane_links.clear();
			}
		}
	}

	return connections_list;
}

void OpenDriveRoad::GetTrafficLights(std::vector<PlannerHNS::TrafficLight>& all_lights)
{
	for(unsigned int i=0; i<road_signals_.size(); i++)
	{
		if(road_signals_.at(i).type_ == TRAFFIC_LIGHT_1 || road_signals_.at(i).type_ == TRAFFIC_LIGHT_2 || road_signals_.at(i).type_ == TRAFFIC_LIGHT_3)
		{
			PlannerHNS::TrafficLight tl;
			tl.id = (this->id_*1000000) + road_signals_.at(i).id_ * 10;
			PlannerHNS::WayPoint p;
			if(CreateSingleCenterPoint(road_signals_.at(i).s_, p))
			{
				double a = p.pos.a+M_PI_2;
				p.pos.x += road_signals_.at(i).t_ * cos(a);
				p.pos.y += road_signals_.at(i).t_ * sin(a);
				tl.pos = p.pos;
				tl.laneIds = road_signals_.at(i).valid_lanes_ids_;
				all_lights.push_back(tl);
			}
		}
	}
}

void OpenDriveRoad::GetTrafficSigns(std::vector<PlannerHNS::TrafficSign>& all_signs)
{
//	for(unsigned int i=0; i<road_signals_.size(); i++)
//	{
//		PlannerHNS::TrafficLight tl;
//		tl.id = (this->id_*1000000) + road_signals_.at(i).id_ * 10;
//		PlannerHNS::WayPoint p;
//		if(CreateSingleCenterPoint(road_signals_.at(i).s_, p))
//		{
//			double a = p.pos.a+M_PI_2;
//			p.pos.x += road_signals_.at(i).t_ * cos(a);
//			p.pos.y += road_signals_.at(i).t_ * sin(a);
//			tl.pos = p.pos;
//			tl.laneIds = road_signals_.at(i).valid_lanes_ids_;
//			all_lights.push_back(tl);
//		}
//	}
}

void OpenDriveRoad::GetStopLines(std::vector<PlannerHNS::StopLine>& all_stop_lines)
{
	for(unsigned int i=0; i<road_signals_.size(); i++)
	{
		if(road_signals_.at(i).type_ == STOP_LINE_SIGNAL || road_signals_.at(i).type_ == WAIT_LINE_SIGNAL)
		{
			PlannerHNS::StopLine sl;
			sl.id = (this->id_*1000000) + road_signals_.at(i).id_ * 10;
			PlannerHNS::WayPoint p, p1, p2;
			if(CreateSingleCenterPoint(road_signals_.at(i).s_, p))
			{
				//assume fixed 2 meter stop line length
				p1 = p;
				double a = p.pos.a+M_PI_2;
				p1.pos.x += (road_signals_.at(i).t_ - 1.0 ) * cos(a);
				p1.pos.y += (road_signals_.at(i).t_ - 1.0 ) * sin(a);

				p2 = p;
				p2.pos.x += (road_signals_.at(i).t_ + 1.0 ) * cos(a);
				p2.pos.y += (road_signals_.at(i).t_ + 1.0 ) * sin(a);
				sl.points.push_back(p1.pos);
				sl.points.push_back(p2.pos);

				if(road_signals_.at(i).valid_lanes_ids_.size() > 0)

				for(unsigned int j=0; j<road_signals_.at(i).valid_lanes_ids_.size(); j++)
				{
					sl.laneId = road_signals_.at(i).valid_lanes_ids_.at(j);
					all_stop_lines.push_back(sl);
					sl.id += 1;
				}
			}
		}
	}
}
}
