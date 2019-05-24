/*
 * opendrive2op_map_converter_core.cpp
 *
 *  Created on: Feb 11, 2019
 *      Author: hatem
 */

#include "opendrive2autoware_converter/xml_helpers.h"
#include <fstream>

namespace opendrive_converter
{
void XmlHelpers::findElements(std::string name, TiXmlElement* parent_element, std::vector<TiXmlElement*>& element_list)
{
	if(parent_element == nullptr)
		return;
	else if(name.compare(parent_element->Value()) == 0)
	{
		element_list.push_back(parent_element);
	}

	findElements(name, parent_element->FirstChildElement(), element_list);
	findElements(name, parent_element->NextSiblingElement(), element_list);

}

void XmlHelpers::findFirstElement(std::string name, TiXmlElement* parent_element, std::vector<TiXmlElement*>& element_list)
{
	if(parent_element == nullptr  || element_list.size()>0)
		return;
	else if(name.compare(parent_element->Value()) == 0)
	{
		element_list.push_back(parent_element);
		return;
	}

	findFirstElement(name, parent_element->FirstChildElement(), element_list);
	findFirstElement(name, parent_element->NextSiblingElement(), element_list);
}

int XmlHelpers::getIntAttribute(TiXmlElement* p_elem, std::string name, int def_val )
{
	if(p_elem != nullptr && p_elem->Attribute(name) != nullptr)
		return strtol(p_elem->Attribute(name.c_str()), NULL, 10);
	else
		return def_val;
}

double XmlHelpers::getDoubleAttribute(TiXmlElement* p_elem, std::string name, double def_val )
{
	if(p_elem != nullptr && p_elem->Attribute(name) != nullptr)
		return strtod(p_elem->Attribute(name.c_str()), NULL);
	else
		return def_val;
}

std::string XmlHelpers::getStringAttribute(TiXmlElement* p_elem, std::string name, std::string def_val)
{
	if(p_elem != nullptr && p_elem->Attribute(name) != nullptr)
		return std::string(p_elem->Attribute(name.c_str()));
	else
		return def_val;
}

std::string XmlHelpers::getStringValue(TiXmlElement* p_elem, std::string def_val)
{
	if(p_elem != nullptr && p_elem->Value() != nullptr)
		return p_elem->ValueStr();
	else
		return def_val;
}
}
