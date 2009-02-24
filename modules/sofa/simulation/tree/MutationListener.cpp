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
#include <sofa/simulation/tree/MutationListener.h>
#include <sofa/simulation/tree/GNode.h>

namespace sofa
{

namespace simulation
{

namespace tree
{

MutationListener::~MutationListener()
{
}

void MutationListener::addChild(GNode* /*parent*/, GNode* child)
{
	child->addListener(this);
	for(GNode::ObjectIterator it = child->object.begin(); it != child->object.end(); ++it)
		addObject(child, *it);
	for(GNode::ChildIterator it = child->child.begin(); it != child->child.end(); ++it)
		addChild(child, *it);
}

void MutationListener::removeChild(GNode* /*parent*/, GNode* child)
{
	for(GNode::ObjectIterator it = child->object.begin(); it != child->object.end(); ++it)
		removeObject(child, *it);
	for(GNode::ChildIterator it = child->child.begin(); it != child->child.end(); ++it)
		removeChild(child, *it);
	child->removeListener(this);
}

void MutationListener::addObject(GNode* /*parent*/, core::objectmodel::BaseObject* /*object*/)
{
}

void MutationListener::removeObject(GNode* /*parent*/, core::objectmodel::BaseObject* /*object*/)
{
}

void MutationListener::moveChild(GNode* previous, GNode* parent, GNode* child)
{
	removeChild(previous, child);
	addChild(parent, child);
}

void MutationListener::moveObject(GNode* previous, GNode* parent, core::objectmodel::BaseObject* object)
{
	removeObject(previous, object);
	addObject(parent, object);
}

} // namespace tree

} // namespace simulation

} // namespace sofa

