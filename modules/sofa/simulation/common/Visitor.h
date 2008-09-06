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
#ifndef SOFA_SIMULATION_VISITOR_H
#define SOFA_SIMULATION_VISITOR_H

#include <sofa/simulation/common/Node.h>
#include <sofa/simulation/common/LocalStorage.h>
#include <sofa/core/componentmodel/behavior/BaseMechanicalState.h>
#include <iostream>

namespace sofa
{

namespace simulation
{


class LocalStorage;

/// Base class for visitors propagated recursively through the scenegraph
class Visitor
{
public:
    virtual ~Visitor() {}

    enum Result { RESULT_CONTINUE, RESULT_PRUNE };

    /// Callback method called when decending to a new node. Recursion will stop if this method returns RESULT_PRUNE
    virtual Result processNodeTopDown(simulation::Node* /*node*/) { return RESULT_CONTINUE; }

    /// Callback method called after child node have been processed and before going back to the parent node.
    virtual void processNodeBottomUp(simulation::Node* /*node*/) {}

    /// Return a category name for this visitor
    /// Only used for debugging / profiling purposes
    virtual const char* getCategoryName() const { return "default"; }

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    virtual const char* getClassName() const { return "Visitor"; }

#ifdef SOFA_VERBOSE_TRAVERSAL
    void debug_write_state_before( core::objectmodel::BaseObject* obj ) ;
    void debug_write_state_after( core::objectmodel::BaseObject* obj ) ;
#else
    inline void debug_write_state_before( core::objectmodel::BaseObject*  ) {}
    inline void debug_write_state_after( core::objectmodel::BaseObject*  ) {}
#endif


    /// Helper method to enumerate objects in the given list. The callback gets the pointer to node
    template < class Visit, class Container, class Object >
    void for_each(Visit* visitor, simulation::Node* node, const Container& list, void (Visit::*fn)(simulation::Node*, Object*))
    {

        if (node->getLogTime())
        {
            const std::string category = getCategoryName();
            ctime_t t0 = node->startTime();
            for (typename Container::iterator it=list.begin(); it != list.end(); ++it)
            {
                debug_write_state_before(*it);
                (visitor->*fn)(node, *it);
                debug_write_state_after(*it);
                t0 = node->endTime(t0, category, *it);
            }
        }
        else
        {
            for (typename Container::iterator it=list.begin(); it != list.end(); ++it)
            {
                debug_write_state_before(*it);
                (visitor->*fn)(node, *it);
                debug_write_state_after(*it);
            }
        }

    }

    /// Helper method to enumerate objects in the given list. The callback gets the pointer to node
    template < class Visit, class Container, class Object >
    Visitor::Result for_each_r(Visit* visitor, simulation::Node* node, const Container& list, Visitor::Result (Visit::*fn)(simulation::Node*, Object*))
    {

        Visitor::Result res = Visitor::RESULT_CONTINUE;
        if (node->getLogTime())
        {
            const std::string category = getCategoryName();
            ctime_t t0 = node->startTime();
            for (typename Container::iterator it=list.begin(); it != list.end(); ++it)
            {
                debug_write_state_before(*it);
                res = (visitor->*fn)(node, *it);
                debug_write_state_after(*it);
                t0 = node->endTime(t0, category, *it);
            }
        }
        else
        {
            for (typename Container::iterator it=list.begin(); it != list.end(); ++it)
            {
                debug_write_state_before(*it);
                res = (visitor->*fn)(node, *it);
                debug_write_state_after(*it);
            }
        }
        return res;

    }


    //template < class Visit, class Container, class Object >
    //void for_each(Visit* visitor, const Container& list, void (Visit::*fn)(Object))
    //{
    //	for (typename Container::iterator it=list.begin(); it != list.end(); ++it)
    //	{
    //		(visitor->*fn)(*it);
    //	}
    //}

    typedef simulation::Node::ctime_t ctime_t;

    /// Optional helper method to call before handling an object if not using the for_each method.
    /// It currently takes care of time logging, but could be extended (step-by-step execution for instance)
    ctime_t begin(simulation::Node* node, core::objectmodel::BaseObject* /*obj*/)
    {
        return node->startTime();
    }

    /// Optional helper method to call after handling an object if not using the for_each method.
    /// It currently takes care of time logging, but could be extended (step-by-step execution for instance)
    void end(simulation::Node* node, core::objectmodel::BaseObject* obj, ctime_t t0)
    {
        node->endTime(t0, getCategoryName(), obj);
    }

    /// Alias for context->executeVisitor(this)
    void execute(core::objectmodel::BaseContext*);


    /// Specify whether this visitor can be parallelized.
    virtual bool isThreadSafe() const { return false; }

    /// Callback method called when decending to a new node. Recursion will stop if this method returns RESULT_PRUNE
    /// This version is offered a LocalStorage to store temporary data
    virtual Result processNodeTopDown(simulation::Node* node, LocalStorage*) { return processNodeTopDown(node); }

    /// Callback method called after child node have been processed and before going back to the parent node.
    /// This version is offered a LocalStorage to store temporary data
    virtual void processNodeBottomUp(simulation::Node* node, LocalStorage*) { processNodeBottomUp(node); }
};

} // namespace simulation

} // namespace sofa

#endif
