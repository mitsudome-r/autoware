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

#include "op_planner/MappingHelpers.h"

namespace autoware_map
{
class XmlHelpers
{

public:

	static void FindElements(const std::string& name, TiXmlElement* parent_element, std::vector<TiXmlElement*>& element_list);
	static void FindFirstElement(const std::string& name, TiXmlElement* parent_element, std::vector<TiXmlElement*>& element_list);
	static int GetIntAttribute(TiXmlElement* p_elem, std::string name, int def_val = 0);
	static double GetDoubleAttribute(TiXmlElement* p_elem, std::string name, double def_val = 0.0);
	static std::string GetStringAttribute(TiXmlElement* p_elem, std::string name, std::string def_val);
	static std::string GetStringValue(TiXmlElement* p_elem, std::string def_val);
};

}

#endif // XML_HELPERS_H
