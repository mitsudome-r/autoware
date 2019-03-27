/*
 *  Copyright (c) 2019, Nagoya University
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

#ifndef OPENDRIVE_ROAD
#define OPENDRIVE_ROAD

#include "opendrive_objects.h"


namespace autoware_map
{

class FromRoadLink
{
public:
	LINK_TYPE link_type;
	int from_road_id;
	CONTACT_POINT contact_point;

	FromRoadLink(TiXmlElement* main_element)
	{
		if(XmlHelpers::GetStringAttribute(main_element, "elementType", "").compare("road") == 0)
			link_type = ROAD_LINK;
		else
			link_type = JUNCTION_LINK;

		from_road_id = XmlHelpers::GetIntAttribute(main_element, "elementId", 0);

		if(XmlHelpers::GetStringAttribute(main_element, "contactPoint", "").compare("start") == 0)
			contact_point = START_POINT;
		else if(XmlHelpers::GetStringAttribute(main_element, "contactPoint", "").compare("end") == 0)
			contact_point = END_POINT;
		else
			contact_point = EMPTY_POINT;
	}
};

class ToRoadLink
{
public:
	LINK_TYPE link_type;
	int to_road_id;
	CONTACT_POINT contact_point;

	ToRoadLink(TiXmlElement* main_element)
	{
		if(XmlHelpers::GetStringAttribute(main_element, "elementType", "").compare("road") == 0)
			link_type = ROAD_LINK;
		else
			link_type = JUNCTION_LINK;

		to_road_id = XmlHelpers::GetIntAttribute(main_element, "elementId", 0);

		if(XmlHelpers::GetStringAttribute(main_element, "contactPoint", "").compare("start") == 0)
			contact_point = START_POINT;
		else if(XmlHelpers::GetStringAttribute(main_element, "contactPoint", "").compare("end") == 0)
			contact_point = END_POINT;
		else
			contact_point = EMPTY_POINT;
	}
};

class OpenDriveRoad
{
public:
	std::string name_;
	int id_;
	int junction_id_;
	double length_;
	std::vector<FromRoadLink> predecessor_road_;
	std::vector<ToRoadLink> successor_road_;
	std::vector<Geometry> geometries_;
	std::vector<Elevation> elevations_;
	std::vector<RoadSection> sections_;
	std::vector<LaneOffset> laneOffsets_;
	std::vector<Signal> road_signals_;
	std::vector<SignalRef> road_signals_references_;
	std::vector<RoadObject> road_objects_;
	std::vector<RoadObjectRef> road_objects_references_;
	std::vector<RoadObjectTunnel> road_objects_tunnels_;
	std::vector<RoadObjectBridge> road_objects_bridges_;
	std::vector<Connection> to_roads;
	std::vector<Connection> from_roads;


	std::vector<Connection> GetFirstSectionConnections();
	std::vector<Connection> GetLastSectionConnections();
	void GetRoadLanes(std::vector<PlannerHNS::Lane>& lanes_list, const double& resolution = 0.5);
	void GetTrafficLights(std::vector<PlannerHNS::TrafficLight>& all_lights);
	void GetTrafficSigns(std::vector<PlannerHNS::TrafficSign>& all_signs);
	void GetStopLines(std::vector<PlannerHNS::StopLine>& all_stop_lines);
	void InsertUniqueToConnection(const Connection& _connection);
	void InsertUniqueFromConnection(const Connection& _connection);

	OpenDriveRoad()
	{
		id_ = 0;
		junction_id_ = 0;
		length_ = 0;

	}

	OpenDriveRoad(TiXmlElement* main_element);

private:
	Geometry* GetMatchingGeometry(const double& sOffset)
	{
		for(unsigned int i=0; i < geometries_.size(); i++)
		{
			if(sOffset >= geometries_.at(i).s && sOffset < (geometries_.at(i).s+geometries_.at(i).length))
			{
				return &geometries_.at(i);
			}
		}

		return nullptr;
	}

	Elevation* GetMatchingElevations(const double& sOffset)
	{
		if(elevations_.size() == 0)
			return nullptr;

		if(elevations_.size() == 1)
			return &elevations_.at(0);

		for(int i=1; i < elevations_.size(); i++)
		{
			if(sOffset >= elevations_.at(i-1).s && sOffset < elevations_.at(i).s)
			{
				return &elevations_.at(i-1);
			}
		}

		return &elevations_.at(elevations_.size()-1);
	}

	LaneOffset* GetMatchingLaneOffset(const double& sOffset)
	{
		if(laneOffsets_.size() == 0)
			return nullptr;

		if(laneOffsets_.size() == 1)
			return &laneOffsets_.at(0);

		for(int i=1; i < laneOffsets_.size(); i++)
		{
			if(sOffset >= laneOffsets_.at(i-1).s && sOffset < laneOffsets_.at(i).s)
			{
				return &laneOffsets_.at(i-1);
			}
		}

		return &laneOffsets_.at(laneOffsets_.size()-1);
	}

	RoadSection* GetMatchingSection(const double& sOffset)
	{
		if(sections_.size() == 0)
			return nullptr;

		if(sections_.size() == 1)
			return &sections_.at(0);

		for(int i=1; i < sections_.size(); i++)
		{
			if(sOffset >= sections_.at(i-1).s_ && sOffset < sections_.at(i).s_)
			{
				return &sections_.at(i-1);
			}
		}

		return &sections_.at(sections_.size()-1);
	}

	bool CreateSingleCenterPoint(const double& _ds, PlannerHNS::WayPoint& _p);

	void InsertUniqueFromSectionIds(const int& from_section_id, const OpenDriveLane* curr_lane, PlannerHNS::Lane& _l);
	void InsertUniqueToSectionIds(const int& to_section_id, const OpenDriveLane* curr_lane, PlannerHNS::Lane& _l);

	void InsertUniqueFromRoadIds(const int& curr_section_id, const int& curr_lane_id, PlannerHNS::Lane& _l);
	void InsertUniqueToRoadIds(const int& curr_section_id, const int& curr_lane_id, PlannerHNS::Lane& _l);




	void CreateRoadLanes(std::vector<PlannerHNS::Lane>& lanes_list);
	void CreateRoadCenterInfo(std::vector<RoadCenterInfo>& points_list, const double& resolution = 0.5);

	PlannerHNS::Lane* GetLaneById(const int& _l_id, std::vector<PlannerHNS::Lane>& _lanes_list)
	{
		for(unsigned int i=0; i < _lanes_list.size(); i++)
		{
			if(_lanes_list.at(i).id == _l_id)
				return &_lanes_list.at(i);
		}

		return nullptr;
	}

	RoadSection* GetFirstSection()
	{
		if(sections_.size() == 0)
			return nullptr;

		return &sections_.at(0);
	}

	RoadSection* GetLastSection()
	{
		if(sections_.size() == 0)
			return nullptr;

		return &sections_.at(sections_.size()-1);
	}

	bool Exists(const std::vector<int>& _list, const int& _val)
	{
		for(unsigned int j=0; j< _list.size(); j++)
		{
			if(_list.at(j) == _val)
			{
				return true;
			}
		}

		return false;
	}

};

}

#endif // OPENDRIVE2AUTOWARE_CONVERTER
