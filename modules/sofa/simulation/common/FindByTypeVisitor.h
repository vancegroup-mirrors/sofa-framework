/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 3      *
*                (c) 2006-2008 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_SIMULATION_TREE_FINDBYTYPE_VISITOR_H
#define SOFA_SIMULATION_TREE_FINDBYTYPE_VISITOR_H

#include <sofa/simulation/common/Node.h>
#include <sofa/simulation/common/Visitor.h>
#include <sofa/core/VisualModel.h>
#include <sofa/helper/system/gl.h>
#include <iostream>

using std::cerr;
using std::endl;

namespace sofa
{

namespace simulation
{


/** Find all components of a given type and store pointers in a list.
*/
template<class T>
class FindByTypeVisitor : public Visitor
{
public:
    std::vector<T*> found; ///< The result of the search: contains pointers to all components of the given type found.

    /// For each component, if it is of the given type, then put it in the list
    virtual Result processNodeTopDown(simulation::Node* node)
    {
        for( simulation::Node::Sequence<core::objectmodel::BaseObject>::iterator i=node->object.begin(), iend=node->object.end(); i!=iend; i++ )
        {
            if( T* obj= dynamic_cast<T*>(*i) )
                found.push_back(obj);
        }
        return RESULT_CONTINUE;
    }

};


} // namespace simulation

} // namespace sofa

#endif
