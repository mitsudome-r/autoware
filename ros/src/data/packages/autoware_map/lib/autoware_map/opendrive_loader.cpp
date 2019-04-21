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

void OpenDriveLoader::loadOpenDRIVE(const std::string& xodr_file, PlannerHNS::RoadNetwork& map, double resolution)
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
	XmlHelpers::findFirstElement("header", doc.FirstChildElement(), elements);

	std::cout << "Final Results, Num:" << elements.size() <<std::endl;

	if(elements.size() > 0)
	{
		OpenDriveHeader header(elements.at(0));
		std::cout << "Final Results, Num:" << elements.size() << ", main element: " <<  elements.at(0)->Value() << std::endl;
	}

	std::cout << " >> Reading Data from OpenDRIVE map file ... " << std::endl;
	elements.clear();
	XmlHelpers::findElements("road", doc.FirstChildElement(), elements);
	std::cout << "Final Results Roads, Num:" << elements.size() << std::endl;

	roads_list_.clear();
	for(unsigned int i=0; i < elements.size(); i++)
	{
		roads_list_.push_back(OpenDriveRoad(elements.at(i)));
	}


	elements.clear();
	XmlHelpers::findElements("junction", doc.FirstChildElement(), elements);

	std::cout << "Final Results Junctions, Num:" << elements.size() << std::endl;

	junctions_list_.clear();
	for(unsigned int i=0; i < elements.size(); i++)
	{
		junctions_list_.push_back(Junction(elements.at(i)));
	}

	//Connect Roads
	connectRoads();

	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		std::cout << "Road ID: " << roads_list_.at(i).id_ << std::endl;
		 std::cout << "From: ";
		for(unsigned int j=0; j < roads_list_.at(i).from_roads_.size(); j++)
		{
			std::cout << "("  << roads_list_.at(i).from_roads_.at(j).incoming_road_ << "|";
			for(unsigned int k=0; k < roads_list_.at(i).from_roads_.at(j).lane_links.size(); k++)
			{
				std::cout << roads_list_.at(i).from_roads_.at(j).lane_links.at(k).first << ", " << roads_list_.at(i).from_roads_.at(j).lane_links.at(k).second << " ; ";
			}
			std::cout << ")";
		}

		std::cout << std::endl;
		std::cout << "To : " ;

		for(unsigned int j=0; j < roads_list_.at(i).to_roads_.size(); j++)
		{
			std::cout << "("  << roads_list_.at(i).to_roads_.at(j).outgoing_road_ <<"|";
			for(unsigned int k=0; k < roads_list_.at(i).to_roads_.at(j).lane_links.size(); k++)
			{
				std::cout << roads_list_.at(i).to_roads_.at(j).lane_links.at(k).first << ", " << roads_list_.at(i).to_roads_.at(j).lane_links.at(k).second << " ; ";
			}
			std::cout << ")";
		}

		std::cout << std::endl <<std::endl;
	}

	std::cout << "Finish Linking Road Network .. " << std::endl;

	PlannerHNS::RoadSegment segment;
	segment.id = 1;
	map.roadSegments.push_back(segment);
	getMapLanes(map.roadSegments.at(0).Lanes, resolution);
	std::cout << "Finish Extracting Lanes: " << map.roadSegments.at(0).Lanes.size() << std::endl;

	getTrafficLights(map.trafficLights);
	getTrafficSigns(map.signs);
	getStopLines(map.stopLines);

	std::cout << "Finish Extracting Traffic Objects ... " << std::endl;

	linkWayPoints(map);

	std::cout << "Finish Linking Way Points ... " << std::endl;

}

std::vector<OpenDriveRoad*> OpenDriveLoader::getRoadsBySuccId(int _id)
{
	std::vector<OpenDriveRoad*> _ret_list;
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		for(unsigned int j=0; j < roads_list_.at(i).successor_road_.size(); j++)
		{
			if(roads_list_.at(i).successor_road_.at(j).to_road_id_ == _id)
			{
				_ret_list.push_back(&roads_list_.at(i));
			}
		}
	}

	return _ret_list;
}

std::vector<OpenDriveRoad*> OpenDriveLoader::getRoadsByPredId(int _id)
{
	std::vector<OpenDriveRoad*> _ret_list;
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		for(unsigned int j=0; j < roads_list_.at(i).predecessor_road_.size(); j++)
		{
			if(roads_list_.at(i).predecessor_road_.at(j).from_road_id_ == _id)
			{
				_ret_list.push_back(&roads_list_.at(i));
			}
		}
	}

	return _ret_list;
}

OpenDriveRoad* OpenDriveLoader::getRoadById(int _id)
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

Junction* OpenDriveLoader::getJunctionById(int _id)
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

void OpenDriveLoader::connectRoads()
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		if(roads_list_.at(i).predecessor_road_.size() > 0)
		{
			//connect normal roads , junctions will be handeled alone
			if(roads_list_.at(i).predecessor_road_.at(0).link_type_ == ROAD_LINK)
			{
				OpenDriveRoad* p_pre_road = getRoadById(roads_list_.at(i).predecessor_road_.at(0).from_road_id_);
				if(p_pre_road != nullptr)
				{
					std::vector<Connection> pre_conn_list = p_pre_road->getLastSectionConnections();
					for(unsigned k=0; k < pre_conn_list.size(); k++)
					{
						pre_conn_list.at(k).outgoing_road_ = roads_list_.at(i).id_;
						roads_list_.at(i).insertUniqueFromConnection(pre_conn_list.at(k));
					}

					std::vector<Connection> my_conn_list = roads_list_.at(i).getFirstSectionConnections();
					for(unsigned k=0; k < my_conn_list.size(); k++)
					{
						my_conn_list.at(k).incoming_road_ = p_pre_road->id_;
						roads_list_.at(i).insertUniqueFromConnection(my_conn_list.at(k));
					}
				}
			}
		}

		if(roads_list_.at(i).successor_road_.size() > 0)
		{
			//connect normal roads , junctions will be handeled alone
			if(roads_list_.at(i).successor_road_.at(0).link_type_ == ROAD_LINK)
			{
				OpenDriveRoad* p_suc_road = getRoadById(roads_list_.at(i).successor_road_.at(0).to_road_id_);
				if(p_suc_road != nullptr)
				{
					std::vector<Connection> suc_conn_list = p_suc_road->getFirstSectionConnections();
					for(unsigned k=0; k < suc_conn_list .size(); k++)
					{
						suc_conn_list.at(k).incoming_road_ = roads_list_.at(i).id_;
						roads_list_.at(i).insertUniqueToConnection(suc_conn_list .at(k));
					}

					std::vector<Connection> my_conn_list = roads_list_.at(i).getLastSectionConnections();
					for(unsigned k=0; k < my_conn_list.size(); k++)
					{
						my_conn_list.at(k).outgoing_road_ = p_suc_road->id_;
						roads_list_.at(i).insertUniqueToConnection(my_conn_list .at(k));
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
			OpenDriveRoad* p_from_road = getRoadById(junctions_list_.at(i).connections_.at(j).incoming_road_);
			if(p_from_road != nullptr)
			{
				p_from_road->insertUniqueToConnection(junctions_list_.at(i).connections_.at(j));
			}

			OpenDriveRoad* p_to_road = getRoadById(junctions_list_.at(i).connections_.at(j).outgoing_road_);
			if(p_to_road != nullptr)
			{
				p_to_road->insertUniqueFromConnection(junctions_list_.at(i).connections_.at(j));
			}
		}
	}

	//Link Missing successors that are linked to junctions
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		if(roads_list_.at(i).predecessor_road_.size() > 0)
		{
			if(roads_list_.at(i).predecessor_road_.at(0).link_type_ != ROAD_LINK)
			{
				std::vector<OpenDriveRoad*> pred_list = getRoadsBySuccId(roads_list_.at(i).id_);
				for(unsigned int j=0; j < pred_list.size(); j++)
				{
					for(unsigned int k=0; k < pred_list.at(j)->to_roads_.size(); k++)
					{
						if(pred_list.at(j)->to_roads_.at(k).outgoing_road_ == roads_list_.at(i).id_)
						{
							roads_list_.at(i).insertUniqueFromConnection(pred_list.at(j)->to_roads_.at(k));
						}
					}
				}
			}
		}
	}
}

void OpenDriveLoader::getMapLanes(std::vector<PlannerHNS::Lane>& all_lanes, double resolution)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		roads_list_.at(i).getRoadLanes(all_lanes, resolution);
	}
}

void OpenDriveLoader::getTrafficLights(std::vector<PlannerHNS::TrafficLight>& all_lights)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		roads_list_.at(i).getTrafficLights(all_lights);
	}
}

void OpenDriveLoader::getTrafficSigns(std::vector<PlannerHNS::TrafficSign>& all_signs)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		roads_list_.at(i).getTrafficSigns(all_signs);
	}
}

void OpenDriveLoader::getStopLines(std::vector<PlannerHNS::StopLine>& all_stop_lines)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		roads_list_.at(i).getStopLines(all_stop_lines);
	}
}

void OpenDriveLoader::linkWayPoints(PlannerHNS::RoadNetwork& map)
{
	linkLanesPointers(map);

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

void OpenDriveLoader::linkLanesPointers(PlannerHNS::RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		//Link Lanes
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			PlannerHNS::Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			for(unsigned int j = 0 ; j < pL->fromIds.size(); j++)
			{
				for(unsigned int l= 0; l < map.roadSegments.at(rs).Lanes.size(); l++)
				{
					if(map.roadSegments.at(rs).Lanes.at(l).id == pL->fromIds.at(j))
					{
						pL->fromLanes.push_back(&map.roadSegments.at(rs).Lanes.at(l));
					}
				}
			}

			for(unsigned int j = 0 ; j < pL->toIds.size(); j++)
			{
				for(unsigned int l= 0; l < map.roadSegments.at(rs).Lanes.size(); l++)
				{
					if(map.roadSegments.at(rs).Lanes.at(l).id == pL->toIds.at(j))
					{
						pL->toLanes.push_back(&map.roadSegments.at(rs).Lanes.at(l));
					}
				}
			}

			for(unsigned int j = 0 ; j < pL->points.size(); j++)
			{
				pL->points.at(j).pLane  = pL;
			}
		}
	}
}

}
