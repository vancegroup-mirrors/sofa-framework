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
// Author: François Faure, INRIA-UJF, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
#include <sofa/component/linearsolver/MultiCGLinearSolver.h>
#include <sofa/component/linearsolver/NewMatMatrix.h>
#include <sofa/component/linearsolver/FullMatrix.h>
#include <sofa/component/linearsolver/SparseMatrix.h>
#include <sofa/component/linearsolver/CompressedRowSparseMatrix.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/helper/system/thread/CTime.h>
#include <sofa/helper/AdvancedTimer.h>

#include <sofa/core/ObjectFactory.h>
#include <iostream>
#include <algorithm>

namespace sofa
{

namespace component
{

namespace linearsolver
{

using namespace sofa::defaulttype;
using namespace sofa::core::behavior;
using namespace sofa::simulation;
#ifdef DISPLAY_TIME
using sofa::helper::system::thread::CTime;
#endif


/// Linear system solver using the conjugate gradient iterative algorithm
template<class TMatrix, class TVector>
MultiCGLinearSolver<TMatrix,TVector>::MultiCGLinearSolver()
: f_maxIter( initData(&f_maxIter,(unsigned)25,"iterations","maximum number of iterations of the Conjugate Gradient solution") )
, f_tolerance( initData(&f_tolerance,1e-5,"tolerance","desired precision of the Conjugate Gradient Solution (ratio of current residual norm over initial residual norm)") )
, f_smallDenominatorThreshold( initData(&f_smallDenominatorThreshold,1e-5,"threshold","minimum value of the denominator in the conjugate Gradient solution") )
, f_verbose( initData(&f_verbose,false,"verbose","Dump system state at each iteration") )
, f_graph( initData(&f_graph,"graph","Graph of residuals at each iteration") )
{
    this->multiGroup.setValue(true);
    this->multiGroup.setReadOnly(true);
    f_graph.setWidget("graph");
//    f_graph.setReadOnly(true);
#ifdef DISPLAY_TIME
    timeStamp = 1.0 / (double)CTime::getRefTicksPerSec();
#endif
}

template<class TMatrix, class TVector>
void MultiCGLinearSolver<TMatrix,TVector>::resetSystem()
{
    f_graph.beginEdit()->clear();
    f_graph.endEdit();

    Inherit::resetSystem();
}

template<class TMatrix, class TVector>
void MultiCGLinearSolver<TMatrix,TVector>::setSystemMBKMatrix(double mFact, double bFact, double kFact)
{
#ifdef DISPLAY_TIME
    CTime * timer;
    time2 = (double) timer->getTime();
#endif
    
    Inherit::setSystemMBKMatrix(mFact,bFact,kFact);
    
#ifdef DISPLAY_TIME
    time2 = ((double) timer->getTime() - time2)  * timeStamp;
#endif
}

/// Solve Mx=b
template<class TMatrix, class TVector>
void MultiCGLinearSolver<TMatrix,TVector>::solve(Matrix& M, Vector& x, Vector& b)
{
#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printComment("MultiConjugateGradient");
#endif
    const bool printLog = this->f_printLog.getValue();
    const bool verbose  = f_verbose.getValue();
    const int nbg = this->getNbGroups();
    simulation::MultiNodeDataMap& nodeMap = this->nodeMap;
    simulation::MultiNodeDataMap& writeNodeMap = this->writeNodeMap;
    const helper::vector<simulation::Node*> groups = this->groups;

    if( verbose )
    {
        serr << nbg << " groups:";
        for (int g=0;g<nbg;++g)
            serr << " " << groups[g]->getName() << "(" << this->gData[groups[g]].systemSize << ")";
        serr << sendl;
        serr << "nodeMap:";
        for (simulation::MultiNodeDataMap::const_iterator it = nodeMap.begin(), itend = nodeMap.end();
             it != itend; ++it)
            serr << " " << it->first->getName() << "=" << it->second;
        serr << sendl;
    }

#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printNode("VectorAllocation");
#endif

    Vector& p = *this->createVector();
    Vector& q = *this->createVector();
    Vector& r = *this->createVector();

    // -- solve the system using a conjugate gradient solution
    helper::vector<double> rho(nbg), rho_1(nbg,0.0), alpha(nbg), beta(nbg), den(nbg);
    helper::vector<int> gEndIt(nbg, 0);
    helper::vector<const char*> gEndCond(nbg, NULL);

    if( verbose )
        serr<<"MultiCGLinearSolver, b = "<< b <<sendl;

    x.clear();
    r = b; // initial residual

    //double normb2 = b.dot(b);
    helper::vector<double> normb2(nbg);
    
    multiv_dot(b,b,normb2);
    
    //double normb = sqrt(normb2);
    helper::vector<double> normb(nbg);
    for (int g=0;g<nbg;++g) normb[g] = sqrt(normb2[g]);

	std::map < std::string, sofa::helper::vector<double> >& graph = *f_graph.beginEdit();
	sofa::helper::vector< sofa::helper::vector<double>* > graph_error(nbg, NULL);
	sofa::helper::vector< sofa::helper::vector<double>* > graph_den(nbg, NULL);
    for (int g=0;g<nbg;++g)
    {
        graph_error[g] = &(graph[groups[g]->getName()+std::string("-Error")]);
        graph_error[g]->clear();
        graph_den[g] = &(graph[groups[g]->getName()+std::string("-Denominator")]);
        graph_den[g]->clear();
        graph_error[g]->push_back(1);
    }
    unsigned nb_iter;
    //const char* endcond = "iterations";
    int nbg_active = nbg;
#ifdef DISPLAY_TIME
    CTime * timer;
    time1 = (double) timer->getTime();
#endif

#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printCloseNode("VectorAllocation");
    std::string dumpNode;
#endif
    for( nb_iter=1; nb_iter<=f_maxIter.getValue() && nbg_active > 0; nb_iter++ )
    {
#ifdef SOFA_DUMP_VISITOR_INFO
        std::ostringstream comment;
        comment << "Iteration_" << nb_iter;
        dumpNode = comment.str();
        simulation::Visitor::printNode(dumpNode);
#endif
        // 		printWithElapsedTime( x, helper::system::thread::CTime::getTime()-time0,sout );

        //z = r; // no precond
        //rho = r.dot(z);
        //rho = (nb_iter==1) ? normb2 : r.dot(r);
        if (nb_iter == 1)
            rho = normb2;
        else
        {
            multiv_dot(r,r,rho);
        }

        for (int g=0;g<nbg;++g)
        {
            if (gEndIt[g]) continue;
            if (nb_iter>1)
            {
                double normr = sqrt(rho[g]); //sqrt(r.dot(r));
                double err = normr/normb[g];
                graph_error[g]->push_back(err);
                if (err <= f_tolerance.getValue())
                {
                    gEndIt[g] = nb_iter;
                    gEndCond[g] = "tolerance";
                    nodeMap.erase(groups[g]);
                    writeNodeMap.erase(groups[g]);
                    --nbg_active;
                    continue;
                }
            }
        }
        if (nbg_active == 0) break;

        if( nb_iter==1 )
            p = r; //z;
        else
        {
            //beta = rho / rho_1;
            for (int g=0;g<nbg;++g)
                if (!gEndIt[g]) beta[g] = rho[g] / rho_1[g]; else beta[g] = 0.0;
            //p = p*beta + r; //z;
            //cgstep_beta(p,r,beta);
            for (int g=0;g<nbg;++g)
                if (!gEndIt[g]) nodeMap[groups[g]] = beta[g];
            cgstep_beta(p,r,1.0);
            for (int g=0;g<nbg;++g)
                if (!gEndIt[g]) nodeMap[groups[g]] = 1.0;
        }
        
        if( verbose )
        {
            serr<<"p : "<<p<<sendl;
        }
        
        // matrix-vector product
        q = M*p;
        
        if( verbose )
        {
            serr<<"q = M p : "<<q<<sendl;
        }
        
        //double den = p.dot(q);
        multiv_dot(p,q,den);
        for (int g=0;g<nbg;++g)
        {
            if (gEndIt[g]) continue;
            graph_den[g]->push_back(den[g]);
            
            if( fabs(den[g])<f_smallDenominatorThreshold.getValue() )
            {
                gEndIt[g] = nb_iter;
                gEndCond[g] = "threshold";
                nodeMap.erase(groups[g]);
                writeNodeMap.erase(groups[g]);
                --nbg_active;
                continue;
            }
        }
        if (nbg_active == 0)
            break;

        //alpha = rho/den;
        for (int g=0;g<nbg;++g)
                if (!gEndIt[g]) alpha[g] = rho[g]/den[g]; else alpha[g] = 0.0;
        //x.peq(p,alpha);                 // x = x + alpha p
        //r.peq(q,-alpha);                // r = r - alpha q
        //cgstep_alpha(x,r,p,q,alpha);
        for (int g=0;g<nbg;++g)
            if (!gEndIt[g]) nodeMap[groups[g]] = alpha[g];
        cgstep_alpha(x,r,p,q,1.0);
        for (int g=0;g<nbg;++g)
            if (!gEndIt[g]) nodeMap[groups[g]] = 1.0;
        if( verbose )
        {
            serr<<"den = "<<den<<", alpha = "<<alpha<<sendl;
            serr<<"x : "<<x<<sendl;
            serr<<"r : "<<r<<sendl;
        }
        
        rho_1 = rho;
#ifdef SOFA_DUMP_VISITOR_INFO
        simulation::Visitor::printCloseNode(dumpNode);
        dumpNode.clear();
#endif
    }
#ifdef SOFA_DUMP_VISITOR_INFO
    if (!dumpNode.empty())
        simulation::Visitor::printCloseNode(dumpNode);
#endif
    
#ifdef DISPLAY_TIME
	time1 = (double)(((double) timer->getTime() - time1) * timeStamp / (nb_iter-1));
#endif
    
	f_graph.endEdit();
    
    sofa::helper::AdvancedTimer::valSet("CG iterations", nb_iter);
    
    // x is the solution of the system
    if( printLog )
    {
#ifdef DISPLAY_TIME
        std::cerr<<"MultiCGLinearSolver::solve, CG = "<<time1<<" build = "<<time2<<std::endl;
#endif
        for (int g=0;g<nbg;++g)
            serr<<"MultiCGLinearSolver::solve, nbiter = "<< (gEndIt[g] ? gEndIt[g] : nb_iter) <<" stop because of "<< (gEndCond[g] ? gEndCond[g] : "iterations") <<sendl;
    }
    if( verbose )
    {
        serr<<"MultiCGLinearSolver::solve, solution = "<<x<<sendl;
    }
    this->deleteVector(&p);
    this->deleteVector(&q);
    this->deleteVector(&r);

}

template<>
inline void MultiCGLinearSolver<component::linearsolver::GraphScatteredMatrix,component::linearsolver::GraphScatteredVector>::cgstep_beta(Vector& p, Vector& r, double beta)
{
    this->v_op(p,r,p,beta); // p = p*beta + r
}

template<>
inline void MultiCGLinearSolver<component::linearsolver::GraphScatteredMatrix,component::linearsolver::GraphScatteredVector>::cgstep_alpha(Vector& x, Vector& r, Vector& p, Vector& q, double alpha)
{
#ifdef SOFA_NO_VMULTIOP // unoptimized version
    x.peq(p,alpha);                 // x = x + alpha p
    r.peq(q,-alpha);                // r = r - alpha q
#else // single-operation optimization
    typedef core::behavior::BaseMechanicalState::VMultiOp VMultiOp;
    VMultiOp ops;
    ops.resize(2);
    ops[0].first = (VecId)x;
    ops[0].second.push_back(std::make_pair((VecId)x,1.0));
    ops[0].second.push_back(std::make_pair((VecId)p,alpha));
    ops[1].first = (VecId)r;
    ops[1].second.push_back(std::make_pair((VecId)r,1.0));
    ops[1].second.push_back(std::make_pair((VecId)q,-alpha));
    this->executeVisitor(simulation::MechanicalVMultiOpVisitor(ops));
#endif
}

SOFA_DECL_CLASS(MultiCGLinearSolver)

int MultiCGLinearSolverClass = core::RegisterObject("Linear system solver using the conjugate gradient iterative algorithm, handling multiple integration groups simultaneously")
.add< MultiCGLinearSolver<GraphScatteredMatrix,GraphScatteredVector> >(true)
.addAlias("MultiCGSolver")
.addAlias("MultiConjugateGradient")
;

} // namespace linearsolver

} // namespace component

} // namespace sofa

