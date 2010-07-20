/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program; if not, write to the Free Software Foundation, Inc., 51  *
* Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.                   *
*******************************************************************************
*                            SOFA :: Applications                             *
*                                                                             *
* Authors: M. Adam, J. Allard, B. Andre, P-J. Bensoussan, S. Cotin, C. Duriez,*
* H. Delingette, F. Falipou, F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza,  *
* M. Nesme, P. Neumann, J-P. de la Plata Alcade, F. Poyer and F. Roy          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_CONTROLLER_ARTRACKDRIVER_H
#define SOFA_COMPONENT_CONTROLLER_ARTRACKDRIVER_H

#include <sofa/core/behavior/BaseController.h>
#include <dtracklib.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/helper/system/config.h>

#ifndef WIN32
    #include <pthread.h>
#else
    #include <process.h>
#endif

namespace sofa
{

namespace component
{

namespace controller
{

using namespace sofa::defaulttype;

#define UDPBUFSIZE  10000
#define UDPTIMEOUT  1000000
#define MAX_NMEATOOL 2
#define MAX_NMARKER 100
#define MAX_NGLOVE 2

class ARTrackDriver : public core::behavior::BaseController
{

public:
    ARTrackDriver();
    virtual ~ARTrackDriver(){}

    virtual void init();

	virtual void reinit();


    Data<double> aRTrackScale;
    Data<Vector3> localTrackerPos;
	Data<double> scaleAngleFinger;

private:

    class dataARTrackClass
    {
    public:
        dataARTrackClass(){}
        ~dataARTrackClass(){}
        dtracklib_type* handle;
        unsigned long framenr;
        double timestamp;
        int nmarker;
        dtracklib_marker_type marker[MAX_NMARKER];
        int nglove;
        dtracklib_glove_type glove[MAX_NGLOVE];
        Vector3 wristInitPos, wirstTranslation;
        sofa::helper::fixed_array<double,3> angle_finger, rest_angle_finger;
        Quat wristRotation;
		sofa::helper::fixed_array<Vector3,3> fingersGlobalPos;
		double aRTrackScale;
		Vector3 localTrackerPos;
#ifndef WIN32
        pthread_mutex_t mutex;
#else
        HANDLE mutex;
#endif
    };

    dataARTrackClass dataARTrack;

#ifndef WIN32
    pthread_t threadID;
#else
    uintptr_t threadID;
#endif

    void initARTrack();

#ifdef WIN32
    static void computeTracking(void *sarg);
#else 
    static void* computeTracking (void *sarg);
#endif

    void handleEvent(core::objectmodel::Event *);
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif
