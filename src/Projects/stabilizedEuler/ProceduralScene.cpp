//
// C++ Implementation: ProceduralScene
//
// Description:
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "ProceduralScene.h"
#include "Sofa-old/Components/MeshTopology.h" 
#include "Sofa-old/Components/Gravity.h" 
#include "Sofa-old/Components/EulerSolver.h"
#include "Sofa-old/Components/RungeKutta4Solver.h"
#include "Sofa-old/Components/CGImplicitSolver.h"
#include "Sofa-old/Components/StaticSolver.h"
#include "Sofa-old/Components/UniformMass.h"
#include "Sofa-old/Components/FixedConstraint.h"
#include "Sofa-old/Components/OscillatorConstraint.h"
#include "Sofa-old/Components/StiffSpringForceField.h"
#include "Sofa-old/Components/TetrahedronFEMForceField.h"
#include "Sofa-old/Components/ArticulatedBody.h"
#include "Sofa-old/Components/RevoluteJoint.h"
#include "Sofa-old/Components/FreeJoint.h"
#include "Sofa-old/Components/GuidedCoordinateSystem.h"
#include "Sofa-old/Components/Graph/MechanicalAction.h"
#include "Sofa-old/Core/MechanicalObject.h"
#include "Sofa-old/Core/Context.h"

namespace Projects
{

namespace procedural
{
    
    GNode* buildPendulum(unsigned nbParticles )
    {
        GNode* node= new GNode("pendulum");

        MechanicalObject<MyTypes>* particles = new MechanicalObject<MyTypes>;
        node->addObject(particles);
        particles->resize(nbParticles);
        particles->setName("pendulum1");

        UniformMass<MyTypes,double>* mass = new UniformMass<MyTypes,double>(particles);
        node->addObject(mass);
        mass->setMass( 1 );

        FixedConstraint<MyTypes>* constraint = new FixedConstraint<MyTypes>(particles);
        node->addObject(constraint);
        constraint->addConstraint(0);
        //         OscillatorConstraint<MyTypes>* constraint = new OscillatorConstraint<MyTypes>(particles);
        //         node->addObject(constraint);
        //         constraint->addConstraint(0,Vec3(0,0,0),Vec3(0,1,0),1,0);

        StiffSpringForceField<MyTypes>* spring = new StiffSpringForceField<MyTypes>(particles);
        node->addObject(spring);

        for( unsigned i=0; i<nbParticles; i++ )
            (*particles->getX())[i] = Vec3(i,0,0);
        for( unsigned i=1; i<nbParticles; i++ )
            spring->addSpring(i-1,i,10.f,1.1f,1);

        return node;
    }

    GNode* attachBranch( int nbParticles, double theta )
    {
        typedef CoordinateSystem::Frame Frame;
        typedef CoordinateSystem::Vec Vec;
        typedef CoordinateSystem::Rot Rot;

        GNode* n11 = new GNode;
        n11->setName("n11");


        GuidedCoordinateSystem* rc1 = new GuidedCoordinateSystem;
        rc1->setName("rc1");
        rc1->setTransform( Frame(Vec(0,0,0), Rot::set
                ( 0.0, theta, 0.0)) );
        rc1->setVelocity( SpatialVector(Vec(0,1,0),Vec(0,0,0))  );
        n11->addObject(rc1);

        n11->addChild( buildPendulum(nbParticles) );

        return n11;
    }

    GNode* attachSolidBranch()
    {
        typedef CoordinateSystem::Frame Frame;
        typedef CoordinateSystem::Vec Vec;
        typedef CoordinateSystem::Rot Rot;

        GNode* n11 = new GNode;
        n11->setName("n11");

        ArticulatedBody* ab1 = new ArticulatedBody;
        ab1->setName("ab1");
        ab1->setInertia( 1, Vec(0,1,0), 1,1,1, 0,0,0 );
        n11->addObject(ab1);

        RevoluteJoint* j1 = new RevoluteJoint( NULL, ab1);
        j1->setName("j1");
        n11->addObject(j1);


        GNode* n12 = new GNode;
        n12->setName("n12");
        n11->addChild(n12);

        ArticulatedBody* ab2 = new ArticulatedBody;
        ab2->setName("ab2");
        ab2->setTransform( Frame(Vec(0,2,0), Rot::identity()) );
        ab2->setInertia( 1, Vec(0,1,0), 1,1,1, 0,0,0 );
        n12->addObject(ab2);

        return n11;
    }



ProceduralScene::ProceduralScene()
{
    m_root = new GNode;
    m_root->setName( "root" );
}

GNode* ProceduralScene::getRoot()
{
    return m_root;
}


OneTetrahedron::OneTetrahedron()
{
    getRoot()->addObject( (new EulerSolver)->setPrintLog(false) );
    //getRoot()->addObject( (new Gravity) ->setGravity( Vec3(-10,0,0) ) );
    //getRoot()->addObject( (new Gravity) ->setGravity( Vec3(0,-10,0) ) );
    getRoot()->addObject( (new Gravity) ->setGravity( Vec3(0,0,-10) ) );

    GNode* node = new GNode;
    getRoot()->addChild( node );

    MechanicalObject<MyTypes>* particles = new MechanicalObject<MyTypes>;
    node->addObject(particles);
    particles->resize(4);
    particles->setName("tetra_dof");
    MyTypes::VecCoord& x = *particles->getX();
    x[0] = Vec(0,0,0);
    x[1] = Vec(1,0,0);
    x[2] = Vec(0,1,0);
    x[3] = Vec(0,0,1);

    UniformMass<MyTypes,double>* mass = new UniformMass<MyTypes,double>(particles);
    node->addObject(mass);
    mass->setMass( 1 );

    MeshTopology* topology = new MeshTopology;
    node->addObject( topology );
    topology->addTetrahedron( 0,1,2,3 );

    FixedConstraint<MyTypes>* constraint = new FixedConstraint<MyTypes>(particles);
    node->addObject(constraint);
    constraint->addConstraint(0);
    constraint->addConstraint(2);
    constraint->addConstraint(3);

    TetrahedronFEMForceField<MyTypes>* spring = new TetrahedronFEMForceField<MyTypes>(particles);
    node->addObject(spring);
    spring->setUpdateStiffnessMatrix(false);


}



ParticleInertia::ParticleInertia( int n )
{
    // Rotating frame
    getRoot()->addObject( (new EulerSolver)->setPrintLog(true) );
    getRoot()->addObject( (new Gravity) ->setGravity( Vec3(0,0,0) ) );

    ArticulatedBody* rc1 = new ArticulatedBody;
    rc1->setName("rc1");
    rc1->setTransform( Frame(Vec(1,0,0), Rot::set
                                 ( 1.5708, 0., 0.)) );
    rc1->setLinearVelocity( Vec(0,0,0)  );
    rc1->setAngularVelocity( Vec(0,1,0)  );
    getRoot()->addObject(rc1);

    // Intermediate frame
    GNode* gn1 = new GNode;
    getRoot()->addChild( gn1 );

    CoordinateSystem* cs1;
    gn1->addObject( cs1= new CoordinateSystem );
    cs1->setTransform( Frame( Vec(1,0,0), Rot::identity() ) );

    // particles
    GNode* pnode = new GNode;
    gn1->addChild( pnode );

    MechanicalObject<MyTypes>* particles = new MechanicalObject<MyTypes>;
    particles->setName("particles");
    pnode->addObject(particles);
    particles->resize(n);

    UniformMass<MyTypes,double>* mass = new UniformMass<MyTypes,double>(particles);
    mass->setName("mass");
    pnode->addObject(mass);
    mass->setMass( 1 );

    for( int i=0; i<n; i++ )
    {
        (*particles->getX())[i] = Vec3(i,0,0);
        (*particles->getV())[i] = Vec3(0,0,i+1);
    }

}



FreeSolid::FreeSolid()
{
    getRoot()->addObject( (new EulerSolver)->setPrintLog(false) );
    getRoot()->addObject( (new Gravity) ->setGravity( Vec3(0,-0,0) ) );

    ArticulatedBody* b1;
    getRoot()->addObject( b1=new ArticulatedBody );
    b1->setInertia( 1, Vec3(1,0,0), 1,1,1,0,0,0 );
    b1->setAngularVelocity( Vec3(0,1,0) );

}


ArticulatedSolid::ArticulatedSolid( int n, int attachType, const Vec3& gravity )
{
    getRoot()->addObject( (new EulerSolver)->setPrintLog(false) );
    getRoot()->addObject( (new Gravity) ->setGravity( gravity ) );

    ArticulatedBody* b1;
    getRoot()->addObject( b1=new ArticulatedBody );
    b1->setInertia( 1, Vec3(0,1,0), 1,1,1,0,0,0 );
    b1->setName("b0");
    b1->setListening(true);

    Joint* j1;
    if( attachType == REVOLUTE )
    {
        RevoluteJoint* rj1;
        getRoot()->addObject( j1=rj1=new RevoluteJoint(0,b1) );
        rj1->setIntraLinkFrame( Frame( Vec(0,0,0), Rot::set
                                           (0,0,0) ) );
        /*           rj1->setAngle( -1.0 );
                       rj1->setVelocity( 1 );*/
    }
    else if( attachType == FREE )
    {
        FreeJoint* fj1;
        getRoot()->addObject( j1=fj1=new FreeJoint(0,b1) );
        fj1->setInitialPosition( Frame( Vec(0,0,0), Rot::identity() ) );
        fj1->setInitialVelocity( SpatialVector( Vec3(1.5708,0,0),Vec3(0,0,1) ) );
    }
    else
    {
        cerr<<"ArticulatedSolid: unknow joint type"<<endl;
        exit(1);
    }

    GNode* parentNode = getRoot();
    ArticulatedBody* parentBody = b1;
    for( int i=1; i<n; i++ )
    {
        GNode* node = new GNode;
        parentNode->addChild( node );

        ArticulatedBody* b1;
        node->addObject( b1=new ArticulatedBody );
        b1->setName("b1");
        b1->setInertia( 1, Vec3(0,1,0), 1,1,1,0,0,0 );

        RevoluteJoint* j1;
        node->addObject( j1=new RevoluteJoint(parentBody,b1) );
        j1->setIntraLinkFrame( Frame( Vec(0,2,0), Rot::set
                                          (0,0,0) ) );

        parentNode = node;
        parentBody = b1;
    }

}


ParticleString::ParticleString( unsigned nbParticles )
{
    GNode* node= new GNode("pendulum");
    getRoot()->addChild(node);

    MechanicalObject<MyTypes>* particles = new MechanicalObject<MyTypes>;
    node->addObject(particles);
    particles->resize(nbParticles);
    //particles->setName("pendulum1");

    UniformMass<MyTypes,double>* mass = new UniformMass<MyTypes,double>(particles);
    node->addObject(mass);
    mass->setMass( 1 );

    FixedConstraint<MyTypes>* constraint = new FixedConstraint<MyTypes>(particles);
    node->addObject(constraint);
    constraint->addConstraint(0);
    //         OscillatorConstraint<MyTypes>* constraint = new OscillatorConstraint<MyTypes>(particles);
    //         node->addObject(constraint);
    //         constraint->addConstraint(0,Vec3(0,0,0),Vec3(0,1,0),1,0);

    StiffSpringForceField<MyTypes>* spring = new StiffSpringForceField<MyTypes>(particles);
    node->addObject(spring);

    for( unsigned i=0; i<nbParticles; i++ ){
        (*particles->getX())[i] = Vec3(i,0,0);
    }
    for( unsigned i=1; i<nbParticles; i++ ){
        (*particles->getV())[i] = Vec3(1,0,0);
    }
    for( unsigned i=1; i<nbParticles; i++ )
        spring->addSpring(i-1,i,10.f,0.f,1);

}



}

}
