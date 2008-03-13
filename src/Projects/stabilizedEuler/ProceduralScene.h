//
// C++ Interface: ProceduralScene
//
// Description: 
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef PROCEDURALSCENE_H
#define PROCEDURALSCENE_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

//----------------------------------------
// Stuff used in procedural scene building
#include <Sofa-old/Components/Common/Vec3Types.h>
#include "Sofa-old/Components/CoordinateSystem.h"
#include <Sofa-old/Components/Gravity.h>

using namespace Sofa::Components;
using namespace Sofa::Components::Graph;
using namespace Sofa::Core;
using namespace Sofa::Abstract;

namespace Projects
{

namespace procedural
{
    typedef Sofa::Components::Common::Vec3Types MyTypes;
    typedef MyTypes::Deriv Vec3;
    typedef CoordinateSystem::SpatialVector SpatialVector;

    GNode* buildPendulum(unsigned nbParticles );
    
    GNode* attachBranch( int nbParticles, double theta );
    
    GNode* attachSolidBranch();


// Stuff used for various tests
//----------------------------------------

    struct ProceduralScene
    {
        typedef CoordinateSystem::Frame Frame;
        typedef CoordinateSystem::SpatialVector SpatialVector; 
        typedef CoordinateSystem::Vec Vec;
        typedef CoordinateSystem::Rot Rot;


        ProceduralScene();

        GNode* getRoot();

        protected:
            GNode* m_root;

    };

    struct OneTetrahedron : public ProceduralScene 
    {
        OneTetrahedron();
    };


    struct ParticleInertia: public ProceduralScene
    {
        ParticleInertia( int n );
    };


    struct FreeSolid: public ProceduralScene
    {
        FreeSolid();
    };


    struct ArticulatedSolid: public ProceduralScene
    {
        enum { FREE, REVOLUTE };

        ArticulatedSolid( int n, int attachType, const Vec3& gravity );
    };


    struct ParticleString: public ProceduralScene
    {
        ParticleString( unsigned nbParticles );
    };


} // namespace procedural

} // namespace Projects

#endif
