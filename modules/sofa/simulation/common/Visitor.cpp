/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
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
#include <sofa/simulation/common/Visitor.h>
#include <sofa/simulation/common/VisualVisitor.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/simulation/common/Simulation.h>


namespace sofa
{

namespace simulation
{



void Visitor::execute(sofa::core::objectmodel::BaseContext* c, bool doPrefetch)
{
    if (doPrefetch && getSimulation()->isPrefetchingUsed())
    {
#ifdef SOFA_DUMP_VISITOR_INFO
        const std::string prefetchName=std::string("Prefetch--") + std::string(getClassName());
        printNode(prefetchName);
#endif
        prefetching = true;
        sofa::core::objectmodel::BaseObject::setPrefetching(true);
        c->executeVisitor(this);
        prefetching = false;
        sofa::core::objectmodel::BaseObject::setPrefetching(false);
#ifdef SOFA_DUMP_VISITOR_INFO
        printCloseNode(prefetchName);
#endif
    }
    c->executeVisitor(this);
}
#ifdef SOFA_DUMP_VISITOR_INFO
Visitor::ctime_t Visitor::initDumpTime;
std::vector< Visitor::ctime_t  > Visitor::initNodeTime=std::vector< Visitor::ctime_t >();
bool Visitor::printActivated=false;
std::ostream *Visitor::outputVisitor=NULL;

void Visitor::setNode(core::objectmodel::Base* c)
{
    if (!enteringBase) enteringBase=c;
}
void Visitor::printInfo(const core::objectmodel::BaseContext* context, bool dirDown)
{
    if (!Visitor::printActivated) return;
    //Traversing the Graph: print the name of the context
    if (context != enteringBase)
    {
        std::string info;
        if (dirDown)
        {
            printNode("Node",context->getName());
        }
        else
        {
            printCloseNode("Node");
        }
        return;
    }
    else if (!this->infoPrinted)
    {
        //Beginning processing: Visitor entered its first node
        this->infoPrinted=true;

        std::string infos(this->getInfos());
        std::string NodeName;
        if (enteringBase) NodeName=enteringBase->getName();

        TRACE_ARGUMENT arg;
        arg.push_back(std::make_pair("infos",infos));
        printNode(this->getClassName(), std::string(), arg);


        arg.clear();
        printNode("Node",NodeName,arg);
    }
    else
    {
        //Ending the traversal: The visitor has finished its work
        if (this->infoPrinted)
        {
            if (enteringBase)
            {
                printCloseNode("Node");
            }
            printCloseNode(this->getClassName());
        }
        //Reinit the Visitor debug variables
        enteringBase=NULL;
        infoPrinted=false;
    }

}

void Visitor::printComment(const std::string &s)
{
    if (Visitor::printActivated)
    {
        std::string info;
        info+= "<!--";
        info+=  s + " -->\n";
        dumpInfo(info);
    }
}

void Visitor::dumpInfo( const std::string &info)
{
    if (printActivated) {(*outputVisitor) << info; /*outputVisitor->flush();*/}
}

void Visitor::startDumpVisitor(std::ostream *s, double time)
{
    initDumpTime = sofa::helper::system::thread::CTime::getRefTime();
    printActivated=true; outputVisitor=s;
    std::string initDump;
    std::ostringstream ff; ff << "<TraceVisitor time=\"" << time << "\">\n";
    dumpInfo(ff.str());
};
void Visitor::stopDumpVisitor()
{
    std::ostringstream s;
    s << "<TotalTime value=\"" << getTimeSpent(initDumpTime,  sofa::helper::system::thread::CTime::getRefTime() ) << "\" />\n";
    s << "</TraceVisitor>\n";
    dumpInfo(s.str());
    printActivated=false;
};

double Visitor::getTimeSpent(ctime_t initTime, ctime_t endTime)
{
    return (double)(endTime-initTime);
}

void Visitor::printNode(const std::string &type, const std::string &name, const TRACE_ARGUMENT &arguments)
{
    if (Visitor::printActivated)
    {
        std::ostringstream s;
        s << "<" << type;
        if (!name.empty()) s << " name=\"" << name << "\"";
        for (unsigned int i=0; i<arguments.size(); ++i)
        {
            if (!arguments[i].second.empty())
                s << " " <<arguments[i].first << "=\"" << arguments[i].second << "\"";
        }
        s << ">\n";

        initNodeTime.push_back(CTime::getRefTime());
        dumpInfo(s.str());
    }
}
void Visitor::printCloseNode(const std::string &type)
{
    if (Visitor::printActivated)
    {
        std::ostringstream s;
        ctime_t tSpent = initNodeTime.back(); initNodeTime.pop_back();
        s << "<Time value=\"" << getTimeSpent(tSpent,CTime::getRefTime()) << "\" />\n";
        s << "</" << type << ">\n";
        dumpInfo(s.str());
    }
}

#endif
/// Optional helper method to call before handling an object if not using the for_each method.
/// It currently takes care of time logging, but could be extended (step-by-step execution for instance)
simulation::Node::ctime_t Visitor::begin(simulation::Node* node, core::objectmodel::BaseObject*
#ifdef SOFA_DUMP_VISITOR_INFO
        obj
#endif
                                        )
{
#ifdef SOFA_DUMP_VISITOR_INFO
    if (printActivated)
    {
        TRACE_ARGUMENT arg;
        arg.push_back(std::make_pair("type",std::string(obj->getClassName())));
        std::ostringstream s; s << obj;
        arg.push_back(std::make_pair("ptr",s.str()));
        printNode("Component", obj->getName(), arg);
    }
#endif
    return node->startTime();
}

/// Optional helper method to call after handling an object if not using the for_each method.
/// It currently takes care of time logging, but could be extended (step-by-step execution for instance)
void Visitor::end(simulation::Node* node, core::objectmodel::BaseObject* obj, ctime_t t0)
{
    node->endTime(t0, getCategoryName(), obj);
#ifdef SOFA_DUMP_VISITOR_INFO
    if (printActivated)
    {
        printCloseNode("Component");
    }
#endif
}

#ifdef SOFA_VERBOSE_TRAVERSAL
void Visitor::debug_write_state_before( core::objectmodel::BaseObject* obj )
{
    using std::cerr;
    using std::endl;
    if( dynamic_cast<VisualVisitor*>(this) ) return;
    cerr<<"Visitor "<<getClassName()<<" enter component "<<obj->getName();
    using core::componentmodel::behavior::BaseMechanicalState;
    if( BaseMechanicalState* dof = dynamic_cast<BaseMechanicalState*> ( obj->getContext()->getMechanicalState() ) )
    {
        cerr<<", state:\nx= "; dof->writeX(cerr);
        cerr<<"\nv= ";        dof->writeV(cerr);
        cerr<<"\ndx= ";       dof->writeDx(cerr);
        cerr<<"\nf= ";        dof->writeF(cerr);
    }
    cerr<<endl;
}

void Visitor::debug_write_state_after( core::objectmodel::BaseObject* obj )
{
    using std::cerr;
    using std::endl;
    if( dynamic_cast<VisualVisitor*>(this) ) return;
    cerr<<"Visitor "<<getClassName()<<" leave component "<<obj->getName();
    using core::componentmodel::behavior::BaseMechanicalState;
    if( BaseMechanicalState* dof = dynamic_cast<BaseMechanicalState*> ( obj->getContext()->getMechanicalState() ) )
    {
        cerr<<", state:\nx= "; dof->writeX(cerr);
        cerr<<"\nv= ";        dof->writeV(cerr);
        cerr<<"\ndx= ";       dof->writeDx(cerr);
        cerr<<"\nf= ";        dof->writeF(cerr);
    }
    cerr<<endl;
}
#endif

} // namespace simulation

} // namespace sofa

