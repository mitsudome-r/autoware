/*
 * opendrive2autoware_converter_core.cpp
 *
 *  Created on: Feb 11, 2019
 *      Author: hatem
 */

#include "opendrive2autoware_converter/opendrive_loader.h"
#include <fstream>

namespace autoware_map
{



OpenDriveLoader::OpenDriveLoader()
{
}

OpenDriveLoader::~OpenDriveLoader()
{
}

void OpenDriveLoader::loadOpenDRIVE(const std::string& xodr_file, PlannerHNS::RoadNetwork& map, const double& resolution)
{
	std::ifstream f(xodr_file.c_str());
	if(!f.good())
	{
		std::cout << "Can't Open OpenDRIVE Map File: (" << xodr_file << ")" << std::endl;
		return;
	}

	std::cout << " >> Loading OpenDRIVE Map file ... " << std::endl;

	TiXmlDocument doc(xodr_file);
	try
	{
		doc.LoadFile();
	}
	catch(std::exception& e)
	{
		std::cout << "OpenDRIVE Custom Reader Error, Can't Load .xodr File, path is: "<<  xodr_file << std::endl;
		std::cout << e.what() << std::endl;
		return;
	}


	std::cout << " >> Reading Header Data from OpenDRIVE map file ... " << std::endl;
	std::vector<TiXmlElement*> elements;
	XmlHelpers::FindFirstElement("header", doc.FirstChildElement(), elements);

	std::cout << "Final Results, Num:" << elements.size() <<std::endl;

	if(elements.size() > 0)
	{
		OpenDriveHeader header(elements.at(0));
		std::cout << "Final Results, Num:" << elements.size() << ", main element: " <<  elements.at(0)->Value() << std::endl;
	}

	std::cout << " >> Reading Data from OpenDRIVE map file ... " << std::endl;
	elements.clear();
	XmlHelpers::FindElements("road", doc.FirstChildElement(), elements);
	std::cout << "Final Results Roads, Num:" << elements.size() << std::endl;

	roads_list_.clear();
	for(unsigned int i=0; i < elements.size(); i++)
	{
		roads_list_.push_back(OpenDriveRoad(elements.at(i)));
	}


	elements.clear();
	XmlHelpers::FindElements("junction", doc.FirstChildElement(), elements);

	std::cout << "Final Results Junctions, Num:" << elements.size() << std::endl;

	junctions_list_.clear();
	for(unsigned int i=0; i < elements.size(); i++)
	{
		junctions_list_.push_back(Junction(elements.at(i)));
	}

	//Connect Roads
	ConnectRoads();

	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		std::cout << "Road ID: " << roads_list_.at(i).id_ << std::endl;
		 std::cout << "From: ";
		for(unsigned int j=0; j < roads_list_.at(i).from_roads.size(); j++)
		{
			std::cout << "("  << roads_list_.at(i).from_roads.at(j).incoming_road_ << "|";
			for(unsigned int k=0; k < roads_list_.at(i).from_roads.at(j).lane_links.size(); k++)
			{
				std::cout << roads_list_.at(i).from_roads.at(j).lane_links.at(k).first << ", " << roads_list_.at(i).from_roads.at(j).lane_links.at(k).second << " ; ";
			}
			std::cout << ")";
		}

		std::cout << std::endl;
		std::cout << "To : " ;

		for(unsigned int j=0; j < roads_list_.at(i).to_roads.size(); j++)
		{
			std::cout << "("  << roads_list_.at(i).to_roads.at(j).outgoing_road_ <<"|";
			for(unsigned int k=0; k < roads_list_.at(i).to_roads.at(j).lane_links.size(); k++)
			{
				std::cout << roads_list_.at(i).to_roads.at(j).lane_links.at(k).first << ", " << roads_list_.at(i).to_roads.at(j).lane_links.at(k).second << " ; ";
			}
			std::cout << ")";
		}

		std::cout << std::endl <<std::endl;
	}

	std::cout << "Finish Linking Road Network .. " << std::endl;

	PlannerHNS::RoadSegment segment;
	segment.id = 1;
	map.roadSegments.push_back(segment);
	GetMapLanes(map.roadSegments.at(0).Lanes, resolution);
	std::cout << "Finish Extracting Lanes: " << map.roadSegments.at(0).Lanes.size() << std::endl;

	GetTrafficLights(map.trafficLights);
	GetTrafficSigns(map.signs);
	GetStopLines(map.stopLines);

	std::cout << "Finish Extracting Traffic Objects ... " << std::endl;

	LinkWayPoints(map);

	std::cout << "Finish Linking Way Points ... " << std::endl;

}

std::vector<OpenDriveRoad*> OpenDriveLoader::GetRoadsBySuccId(const int& _id)
{
	std::vector<OpenDriveRoad*> _ret_list;
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		for(unsigned int j=0; j < roads_list_.at(i).successor_road_.size(); j++)
		{
			if(roads_list_.at(i).successor_road_.at(j).to_road_id == _id)
			{
				_ret_list.push_back(&roads_list_.at(i));
			}
		}
	}

	return _ret_list;
}

std::vector<OpenDriveRoad*> OpenDriveLoader::GetRoadsByPredId(const int& _id)
{
	std::vector<OpenDriveRoad*> _ret_list;
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		for(unsigned int j=0; j < roads_list_.at(i).predecessor_road_.size(); j++)
		{
			if(roads_list_.at(i).predecessor_road_.at(j).from_road_id == _id)
			{
				_ret_list.push_back(&roads_list_.at(i));
			}
		}
	}

	return _ret_list;
}

OpenDriveRoad* OpenDriveLoader::GetRoadById(const int& _id)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		if(roads_list_.at(i).id_ == _id)
		{
			return &roads_list_.at(i);
		}
	}

	return nullptr;
}

Junction* OpenDriveLoader::GetJunctionById(const int& _id)
{
	for(unsigned int i=0; i < junctions_list_.size(); i++)
	{
		if(junctions_list_.at(i).id_ == _id)
		{
			return &junctions_list_.at(i);
		}
	}

	return nullptr;
}

void OpenDriveLoader::ConnectRoads()
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		if(roads_list_.at(i).predecessor_road_.size() > 0)
		{
			//connect normal roads , junctions will be handeled alone
			if(roads_list_.at(i).predecessor_road_.at(0).link_type == ROAD_LINK)
			{
				OpenDriveRoad* p_pre_road = GetRoadById(roads_list_.at(i).predecessor_road_.at(0).from_road_id);
				if(p_pre_road != nullptr)
				{
					std::vector<Connection> pre_conn_list = p_pre_road->GetLastSectionConnections();
					for(unsigned k=0; k < pre_conn_list.size(); k++)
					{
						pre_conn_list.at(k).outgoing_road_ = roads_list_.at(i).id_;
						roads_list_.at(i).InsertUniqueFromConnection(pre_conn_list.at(k));
					}

					std::vector<Connection> my_conn_list = roads_list_.at(i).GetFirstSectionConnections();
					for(unsigned k=0; k < my_conn_list.size(); k++)
					{
						my_conn_list.at(k).incoming_road_ = p_pre_road->id_;
						roads_list_.at(i).InsertUniqueFromConnection(my_conn_list.at(k));
					}
				}
			}
		}

		if(roads_list_.at(i).successor_road_.size() > 0)
		{
			//connect normal roads , junctions will be handeled alone
			if(roads_list_.at(i).successor_road_.at(0).link_type == ROAD_LINK)
			{
				OpenDriveRoad* p_suc_road = GetRoadById(roads_list_.at(i).successor_road_.at(0).to_road_id);
				if(p_suc_road != nullptr)
				{
					std::vector<Connection> suc_conn_list = p_suc_road->GetFirstSectionConnections();
					for(unsigned k=0; k < suc_conn_list .size(); k++)
					{
						suc_conn_list.at(k).incoming_road_ = roads_list_.at(i).id_;
						roads_list_.at(i).InsertUniqueToConnection(suc_conn_list .at(k));
					}

					std::vector<Connection> my_conn_list = roads_list_.at(i).GetLastSectionConnections();
					for(unsigned k=0; k < my_conn_list.size(); k++)
					{
						my_conn_list.at(k).outgoing_road_ = p_suc_road->id_;
						roads_list_.at(i).InsertUniqueToConnection(my_conn_list .at(k));
					}
				}
			}
		}
	}

	//Link Junctions
	for(unsigned int i=0; i < junctions_list_.size(); i++)
	{
		for(unsigned int j=0; j < junctions_list_.at(i).connections_.size(); j++)
		{
			//std::cout << "J_ID: " << junctions_list_.at(i).id_ << ", (" << junctions_list_.at(i).connections_.at(j).incoming_road_ << ", " << junctions_list_.at(i).connections_.at(j).outgoing_road_ << " )" <<std::endl;
			OpenDriveRoad* p_from_road = GetRoadById(junctions_list_.at(i).connections_.at(j).incoming_road_);
			if(p_from_road != nullptr)
			{
				p_from_road->InsertUniqueToConnection(junctions_list_.at(i).connections_.at(j));
			}

			OpenDriveRoad* p_to_road = GetRoadById(junctions_list_.at(i).connections_.at(j).outgoing_road_);
			if(p_to_road != nullptr)
			{
				p_to_road->InsertUniqueFromConnection(junctions_list_.at(i).connections_.at(j));
			}
		}
	}

	//Link Missing successors that are linked to junctions
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		if(roads_list_.at(i).predecessor_road_.size() > 0)
		{
			if(roads_list_.at(i).predecessor_road_.at(0).link_type != ROAD_LINK)
			{
				std::vector<OpenDriveRoad*> pred_list = GetRoadsBySuccId(roads_list_.at(i).id_);
				for(unsigned int j=0; j < pred_list.size(); j++)
				{
					for(unsigned int k=0; k < pred_list.at(j)->to_roads.size(); k++)
					{
						if(pred_list.at(j)->to_roads.at(k).outgoing_road_ == roads_list_.at(i).id_)
						{
							roads_list_.at(i).InsertUniqueFromConnection(pred_list.at(j)->to_roads.at(k));
						}
					}
				}
			}
		}
	}
}

void OpenDriveLoader::GetMapLanes(std::vector<PlannerHNS::Lane>& all_lanes, const double& resolution)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		roads_list_.at(i).GetRoadLanes(all_lanes, resolution);
	}
}

void OpenDriveLoader::GetTrafficLights(std::vector<PlannerHNS::TrafficLight>& all_lights)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		roads_list_.at(i).GetTrafficLights(all_lights);
	}
}

void OpenDriveLoader::GetTrafficSigns(std::vector<PlannerHNS::TrafficSign>& all_signs)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		roads_list_.at(i).GetTrafficSigns(all_signs);
	}
}

void OpenDriveLoader::GetStopLines(std::vector<PlannerHNS::StopLine>& all_stop_lines)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		roads_list_.at(i).GetStopLines(all_stop_lines);
	}
}

void OpenDriveLoader::LinkWayPoints(PlannerHNS::RoadNetwork& map)
{
	PlannerHNS::MappingHelpers::LinkLanesPointers(map);

	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i = 0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			PlannerHNS::Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			for(unsigned int iwp= 0; iwp < pL->points.size(); iwp++)
			{
				if(iwp < pL->points.size()-1)
				{
					pL->points.at(iwp).toIds.push_back(pL->points.at(iwp+1).id);
				}
				else
				{
					for(unsigned int k=0; k< pL->toLanes.size(); k++)
					{
						if(pL->toLanes.at(k) != nullptr && pL->toLanes.at(k)->points.size()>0)
						{
							pL->points.at(iwp).toIds.push_back(pL->toLanes.at(k)->points.at(0).id);
						}
					}
				}
			}
		}
	}
}

}
