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

#ifndef OPENDRIVE2OP_CONVERTER
#define OPENDRIVE2OP_CONVERTER

#include "opendrive2autoware_converter/opendrive_road.h"

namespace autoware_map
{
class MapWriter
{

public:
	MapWriter();
    ~MapWriter();
    void writeAutowareMap(const std::string& folder_name, PlannerHNS::RoadNetwork& map);


private:

	template <class T>
	void WriteCSVFile(const std::string& folder, const std::string& title, const std::string& header, const std::vector<T>& data_list);
	PlannerHNS::Lane* GetLaneFromID(PlannerHNS::RoadNetwork& map, int _l_id)
	{
		for(unsigned int i=0; i<map.roadSegments.size(); i++)
		{
			for(unsigned int j=0; j<map.roadSegments.at(i).Lanes.size(); j++)
			{
				if(map.roadSegments.at(i).Lanes.at(j).points.size() > 0 &&  map.roadSegments.at(i).Lanes.at(j).id == _l_id)
				{
					return &map.roadSegments.at(i).Lanes.at(j);
				}
			}
		}

		return nullptr;
	}
};

}

#endif // OPENDRIVE2AUTOWARE_CONVERTER
