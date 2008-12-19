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
// Author: François Faure, INRIA-UJF, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
#include <sofa/component/linearsolver/PCGLinearSolver.h>
#include <sofa/component/linearsolver/NewMatMatrix.h>
#include <sofa/component/linearsolver/FullMatrix.h>
#include <sofa/component/linearsolver/SparseMatrix.h>
#include <sofa/core/ObjectFactory.h>
#include <iostream>
#include "sofa/helper/system/thread/CTime.h"
#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/core/componentmodel/behavior/LinearSolver.h>

namespace sofa
{

namespace component
{

namespace linearsolver
{

using namespace sofa::defaulttype;
using namespace sofa::core::componentmodel::behavior;
using namespace sofa::simulation;
using namespace sofa::core::objectmodel;
using std::cerr;
using std::endl;

template<class TMatrix, class TVector>
void PCGLinearSolver<TMatrix,TVector>::init()
{
    std::vector<sofa::core::componentmodel::behavior::LinearSolver*> solvers;
    BaseContext * c = this->getContext();
    c->get<LinearSolver>(&solvers,BaseContext::SearchDown);

    for (unsigned int i=0; i<solvers.size(); ++i)
    {
        if (solvers[i] != this)
        {
            this->preconditioners.push_back(solvers[i]);
        }
    }
}

template<class TMatrix, class TVector>
void PCGLinearSolver<TMatrix,TVector>::setSystemMBKMatrix(double mFact, double bFact, double kFact)
{
    Inherit::setSystemMBKMatrix(mFact,bFact,kFact);

    for (unsigned int i=0; i<this->preconditioners.size(); ++i)
    {
        preconditioners[i]->setSystemMBKMatrix(mFact,bFact,kFact);
    }
}

/*
template<class TMatrix, class TVector>
void PCGLinearSolver<TMatrix,TVector>::setSystemRHVector(VecId v) {
	Inherit::setSystemRHVector(v);

	for (unsigned int i=0;i<this->preconditioners.size();++i) {
		preconditioners[i]->setSystemRHVector(v);
	}
}

template<class TMatrix, class TVector>
void PCGLinearSolver<TMatrix,TVector>::setSystemLHVector(VecId v) {
	Inherit::setSystemLHVector(v);

	for (unsigned int i=0;i<this->preconditioners.size();++i) {
		preconditioners[i]->setSystemLHVector(v);
	}
}
*/

template<class TMatrix, class TVector>
void PCGLinearSolver<TMatrix,TVector>::solve (Matrix& M, Vector& x, Vector& b)
{
    using std::cerr;
    using std::endl;

    Vector& p = *this->createVector();
    Vector& q = *this->createVector();
    Vector& r = *this->createVector();
    Vector& z = *this->createVector();

    const bool printLog = f_printLog.getValue();
    const bool verbose  = f_verbose.getValue();

    // -- solve the system using a conjugate gradient solution
    double rho, rho_1=0, alpha, beta;

    if( verbose )
        cerr<<"PCGLinearSolver, b = "<< b <<endl;

    x.clear();
    r = b; // initial residual

    double normb2 = b.dot(b);
    double normb = sqrt(normb2);
    std::map < std::string, sofa::helper::vector<double> >& graph = *f_graph.beginEdit();
    sofa::helper::vector<double>& graph_error = graph["Error"];
    graph_error.clear();
    sofa::helper::vector<double>& graph_den = graph["Denominator"];
    graph_den.clear();
    graph_error.push_back(1);
    unsigned nb_iter;
    const char* endcond = "iterations";
    for( nb_iter=1; nb_iter<=f_maxIter.getValue(); nb_iter++ )
    {

#ifdef DUMP_VISITOR_INFO
        std::ostringstream comment;
        comment << "Iteration : " << nb_iter;
        simulation::Visitor::printComment(comment.str());
#endif
        // 		printWithElapsedTime( x, helper::system::thread::CTime::getTime()-time0,std::cout );

        if (this->preconditioners.size()==0)
        {
            z = r;
        }
        else
        {
            for (unsigned int i=0; i<this->preconditioners.size(); ++i)
            {
                //r = M * xi
                //Solve P * z = r

                preconditioners[i]->setSystemLHVector(z);
                preconditioners[i]->setSystemRHVector(r);
                preconditioners[i]->solveSystem();

                //z = M * A * xi
            }
        }

        rho = r.dot(z);

        if (nb_iter>1)
        {
            double normr = sqrt(r.dot(r));
            double err = normr/normb;
            graph_error.push_back(err);
            if (err <= f_tolerance.getValue())
            {
                endcond = "tolerance";
                break;
            }
        }

        if( nb_iter==1 ) p = z;
        else
        {
            beta = rho / rho_1;
            //p = p*beta + z;
            cgstep_beta(p,z,beta);
        }

        if( verbose )
        {
            cerr<<"p : "<<p<<endl;
        }

        // matrix-vector product
        q = M*p;

        if( verbose )
        {
            cerr<<"q = M p : "<<q<<endl;
        }

        double den = p.dot(q);

        graph_den.push_back(den);

        if( fabs(den)<f_smallDenominatorThreshold.getValue() )
        {
            endcond = "threshold";
            if( verbose )
            {
                cerr<<"PCGLinearSolver, den = "<<den<<", smallDenominatorThreshold = "<<f_smallDenominatorThreshold.getValue()<<endl;
            }
            break;
        }

        alpha = rho/den;
        //x.peq(p,alpha);                 // x = x + alpha p
        //r.peq(q,-alpha);                // r = r - alpha q
        cgstep_alpha(x,r,p,q,alpha);

        if( verbose )
        {
            cerr<<"den = "<<den<<", alpha = "<<alpha<<endl;
            cerr<<"x : "<<x<<endl;
            cerr<<"r : "<<r<<endl;
        }

        rho_1 = rho;
    }
    f_graph.endEdit();
    // x is the solution of the system
    if( printLog )
    {
        cerr<<"PCGLinearSolver::solve, nbiter = "<<nb_iter<<" stop because of "<<endcond<<endl;
    }
    if( verbose )
    {
        cerr<<"PCGLinearSolver::solve, solution = "<<x<<endl;
    }
    this->deleteVector(&p);
    this->deleteVector(&q);
    this->deleteVector(&r);

}

SOFA_DECL_CLASS(PCGLinearSolver)

int PCGLinearSolverClass = core::RegisterObject("Linear system solver using the conjugate gradient iterative algorithm")
        .add< PCGLinearSolver<GraphScatteredMatrix,GraphScatteredVector> >(true)
//.add< PCGLinearSolver<NewMatMatrix,NewMatVector> >()
//.add< PCGLinearSolver<NewMatSymmetricMatrix,NewMatVector> >()
//.add< PCGLinearSolver<NewMatBandMatrix,NewMatVector> >()
//.add< PCGLinearSolver<NewMatSymmetricBandMatrix,NewMatVector> >()
//.add< PCGLinearSolver< FullMatrix<double>, FullVector<double> > >()
//.add< PCGLinearSolver< SparseMatrix<double>, FullVector<double> > >()
        .addAlias("PCGSolver")
        .addAlias("PConjugateGradient")
        ;

} // namespace linearsolver

} // namespace component

} // namespace sofa

