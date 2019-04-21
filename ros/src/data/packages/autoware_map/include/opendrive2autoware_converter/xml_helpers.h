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

#ifndef XML_HELPERS_H
#define XML_HELPERS_H

#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include <limits>
#include <algorithm>

#include "op_planner/RoadNetwork.h"
#include "tinyxml.h"

namespace autoware_map
{
class XmlHelpers
{

public:

	static void findElements(std::string name, TiXmlElement* parent_element, std::vector<TiXmlElement*>& element_list);
	static void findFirstElement(std::string name, TiXmlElement* parent_element, std::vector<TiXmlElement*>& element_list);
	static int getIntAttribute(TiXmlElement* p_elem, std::string name, int def_val = 0);
	static double getDoubleAttribute(TiXmlElement* p_elem, std::string name, double def_val = 0.0);
	static std::string getStringAttribute(TiXmlElement* p_elem, std::string name, std::string def_val);
	static std::string getStringValue(TiXmlElement* p_elem, std::string def_val);
};

class Mat3
{
	double m[3][3];

public:
	Mat3()
	{
		for(unsigned int i=0;i<3;i++)
		{
			for(unsigned int j=0;j<3;j++)
			{
				m[i][j] = 0;
			}
		}

		m[0][0] = m[1][1] = m[2][2] = 1;
	}

	Mat3(double transX, double transY)
	{
		m[0][0] = 1; m[0][1] =  0; m[0][2] =  transX;
		m[1][0] = 0; m[1][1] =  1; m[1][2] =  transY;
		m[2][0] = 0; m[2][1] =  0; m[2][2] =  1;
	}

	Mat3(double rotation_angle)
	{
		double c = cos(rotation_angle);
		double s = sin(rotation_angle);
		m[0][0] = c; m[0][1] = -s; m[0][2] =  0;
		m[1][0] = s; m[1][1] =  c; m[1][2] =  0;
		m[2][0] = 0; m[2][1] =  0; m[2][2] =  1;
	}

	Mat3(PlannerHNS::GPSPoint rotationCenter)
	{
		double c = cos(rotationCenter.a);
		double s = sin(rotationCenter.a);
		double u = rotationCenter.x;
		double v = rotationCenter.y;
		m[0][0] = c; m[0][1] = -s; m[0][2] = -u*c + v*s + u;
		m[1][0] = s; m[1][1] =  c; m[1][2] = -u*s - v*c + v;
		m[2][0] = 0; m[2][1] =  0; m[2][2] =  1;
	}


	PlannerHNS::GPSPoint operator * (PlannerHNS::GPSPoint v)
	{
		PlannerHNS::GPSPoint _v = v;
		v.x = m[0][0]*_v.x + m[0][1]*_v.y + m[0][2]*1;
		v.y = m[1][0]*_v.x + m[1][1]*_v.y + m[1][2]*1;
		return v;
	}
};

}

#endif
