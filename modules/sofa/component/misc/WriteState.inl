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
#ifndef SOFA_COMPONENT_MISC_WRITESTATE_INL
#define SOFA_COMPONENT_MISC_WRITESTATE_INL

#include <sofa/component/misc/WriteState.h>
#include <sofa/simulation/tree/GNode.h>

#include <fstream>

namespace sofa
{

namespace component
{

namespace misc
{


WriteState::WriteState()
    : f_filename( initData(&f_filename, "filename", "output file name"))
    , f_writeX( initData(&f_writeX, true, "writeX", "flag enabling output of X vector"))
    , f_writeV( initData(&f_writeV, false, "writeV", "flag enabling output of V vector"))
    , f_interval( initData(&f_interval, 0.0, "interval", "time duration between outputs"))
    , f_time( initData(&f_time, helper::vector<double>(0), "time", "set time to write outputs"))
    , f_period( initData(&f_period, 0.0, "period", "period between outputs"))
    , f_DOFsX( initData(&f_DOFsX, helper::vector<unsigned int>(0), "DOFsX", "set the position DOFs to write"))
    , f_DOFsV( initData(&f_DOFsV, helper::vector<unsigned int>(0), "DOFsV", "set the velocity DOFs to write"))
    , f_stopAt( initData(&f_stopAt, 0.0, "stopAt", "stop the simulation when the given threshold is reached"))
    , f_keperiod( initData(&f_keperiod, 0.0, "keperiod", "set the period to measure the kinetic energy increase"))
    , mmodel(NULL)
    , outfile(NULL)
    , nextTime(0)
    , lastTime(0)
    , kineticEnergyThresholdReached(false)
    , timeToTestEnergyIncrease(0)
    , savedKineticEnergy(0)
{
    this->f_listening.setValue(true);
}


WriteState::~WriteState()
{
    if (outfile)
        delete outfile;
}


void WriteState::init()
{
    mmodel = dynamic_cast<core::componentmodel::behavior::BaseMechanicalState*>(this->getContext()->getMechanicalState());

    // test the size and range of the DOFs to write in the file output
    if (mmodel)
    {
        timeToTestEnergyIncrease = f_keperiod.getValue();
    }
    ///////////// end of the tests.

    const std::string& filename = f_filename.getValue();
    if (!filename.empty())
    {
// 	    std::ifstream infile(filename.c_str());
// 	    if( infile.is_open() )
// 	      {
// 		std::cerr << "ERROR: file "<<filename<<" already exists. Remove it to record new motion."<<std::endl;
// 	      }
// 	    else
        {
            outfile = new std::ofstream(filename.c_str());
            if( !outfile->is_open() )
            {
                std::cerr << "Error creating file "<<filename<<std::endl;
                delete outfile;
                outfile = NULL;
            }
        }
    }
}


void WriteState::reset()
{
    nextTime = 0;
    lastTime = 0;
    kineticEnergyThresholdReached = false;
    timeToTestEnergyIncrease = f_keperiod.getValue();
    savedKineticEnergy = 0;
}


void WriteState::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (/* simulation::AnimateBeginEvent* ev = */ dynamic_cast<simulation::AnimateBeginEvent*>(event))
    {

        if (outfile && mmodel)
        {
            if (!kineticEnergyThresholdReached)
            {
                double time = getContext()->getTime();
                // the time to measure the increase of energy is reached
                if (f_stopAt.getValue())
                {
                    if (time > timeToTestEnergyIncrease)
                    {
                        simulation::tree::GNode *gnode = dynamic_cast<simulation::tree::GNode *>(this->getContext());
                        if (!gnode->mass)
                        {
                            // Error: the mechanical model has no mass
                            std::cerr << "Error: Kinetic energy can not be computed. The mass for " << mmodel->getName() << " has no been defined" << std::endl;
                            exit(-1);
                        }
                        else
                        {
                            // computes the energy increase
                            if (fabs(gnode->mass->getKineticEnergy() - savedKineticEnergy) < f_stopAt.getValue())
                            {
                                std::cout << "WriteState has been stopped. Kinetic energy threshold has been reached" << std::endl;
                                kineticEnergyThresholdReached = true;
                            }
                            else
                            {
                                // save the last energy measured
                                savedKineticEnergy = gnode->mass->getKineticEnergy();
                                // increase the period to measure the energy
                                timeToTestEnergyIncrease+=f_keperiod.getValue();
                            }
                        }
                    }
                }

                if (nextTime<f_time.getValue().size())
                {
                    // store the actual time instant
                    lastTime = f_time.getValue()[nextTime];
                    if (time >= lastTime) // if the time simulation is >= that the actual time instant
                    {
                        // write the X state
                        (*outfile) << "T= "<< time << "\n";
                        if (f_writeX.getValue())
                        {
                            (*outfile) << "  X= ";
                            mmodel->writeX(*outfile);
                            (*outfile) << "\n";
                        }
                        //write the V state
                        if (f_writeV.getValue())
                        {
                            (*outfile) << "  V= ";
                            mmodel->writeV(*outfile);
                            (*outfile) << "\n";
                        }
                        outfile->flush();
                        nextTime++;
                    }
                }
                else
                {
                    // write the state using a period
                    if (time >= (lastTime + f_period.getValue()))
                    {
                        (*outfile) << "T= "<< time << "\n";
                        if (f_writeX.getValue())
                        {
                            (*outfile) << "  X= ";
                            mmodel->writeX(*outfile);
                            (*outfile) << "\n";
                        }
                        if (f_writeV.getValue())
                        {
                            (*outfile) << "  V= ";
                            mmodel->writeV(*outfile);
                            (*outfile) << "\n";
                        }
                        outfile->flush();
                        lastTime += f_period.getValue();
                    }
                }
            }
        }
    }
}

} // namespace misc

} // namespace component

} // namespace sofa

#endif
