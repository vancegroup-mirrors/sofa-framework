/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#include <sofa/simulation/tree/xml/AttributeElement.h>
#include <sofa/simulation/tree/xml/Element.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace simulation
{

namespace tree
{

namespace xml
{

using namespace sofa::defaulttype;
using helper::Creator;

//template class Factory< std::string, objectmodel::BaseObject, Node<objectmodel::BaseObject*>* >;

AttributeElement::AttributeElement(const std::string& name, const std::string& type, BaseElement* parent)
: Element<core::objectmodel::BaseObject>(name, type, parent)
{
}

AttributeElement::~AttributeElement()
{
}

bool AttributeElement::init()
{
  int i=0;
  for (child_iterator<> it = begin(); it != end(); ++it)
  {
    i++;
    it->initNode();
  } 
  return initNode(); 
}

bool AttributeElement::initNode()
{
    std::string info;
    std::string name = getAttribute( "type", "");
    info = getAttribute( "name", ""); 
    getParentElement()->setAttribute(name, value.c_str());
    return true;
}

SOFA_DECL_CLASS(Attribute)

Creator<BaseElement::NodeFactory, AttributeElement> AttributeNodeClass("Attribute");

const char* AttributeElement::getClass() const
{
	return AttributeNodeClass.c_str();
}

} // namespace xml

} // namespace tree

} // namespace simulation

} // namespace sofa

