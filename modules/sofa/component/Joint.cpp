// Author: François Faure, INRIA-UJF, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
#include <sofa/component/Joint.h>
#include <sofa/core/Mapping.inl>
#include <sofa/core/componentmodel/behavior/MechanicalMapping.inl>
#include <sofa/core/componentmodel/behavior/Constraint.inl>
#include <iostream>
using std::cerr;
using std::endl;

namespace sofa
{

namespace component
{

Joint::Joint( ArticulatedBody* parent, ArticulatedBody* child )
                : Mapping( parent, child )
                , massNode_(NULL)
        , m_jointOfParent( NULL )
{
        m_intraLinkTransform.clear();
        jointFrame_.clear();
        child->setJoint(this);
        //assert(child!=NULL);
}

void Joint::init()
{ 
    //reset();
    propagateX(); 
    ArticulatedBody* parentSolid = dynamic_cast<ArticulatedBody*>( getMechFrom() );
    //assert( parentSolid );
    m_jointOfParent = parentSolid!=NULL ? parentSolid->getJoint() : NULL;
}


ArticulatedBody* Joint::getParent()
{
        return this->fromModel;
}

ArticulatedBody* Joint::getChild()
{
        return this->toModel;
}

core::componentmodel::behavior::BaseMechanicalState* Joint::getDOFs() { return getChild() ; }

Joint* Joint::setMassNode( ArticulatedMass* mn )
{
        massNode_ = mn;
        return this;
}

Joint* Joint::setGeometry( const Vec& center, const Rot& ori )
{
        m_intraLinkTransform= Frame( center, ori );
	return this;
}

Joint::Frame Joint::getPosition()
{
        return m_intraLinkTransform * getJointPosition();
}
Joint::SpatialVector Joint::getVelocity()
{
        return m_intraLinkTransform * getJointVelocity();
}


void Joint::setIntraLinkFrame( const Joint::Frame& ilf )
{
        m_intraLinkTransform = ilf;
}

Joint::Frame& Joint::getIntraLinkFrame()
{
    return m_intraLinkTransform;
}

const Joint::Frame& Joint::getIntraLinkFrame() const
{
        return m_intraLinkTransform;
}

Joint::Frame Joint::getCoord(unsigned i)
{
        return m_intraLinkTransform * getFrame(i);
}

Joint::SpatialVector Joint::getDeriv(unsigned i)
{
        return m_intraLinkTransform * getSpatialVector(i);
}


// void Joint::propagateX()
// {
// }
// 
// void Joint::applyJ(SpatialVector& /*childVelocity*/, const SpatialVector& /*parentVelocity*/)
// {}
// 
// void Joint::applyJT(SpatialVector& /*parentForce*/, const SpatialVector& /*childForce*/)
// {}

void Joint::addToIa( const ArticulatedInertia& ia )
{
    sp_Ia += ia;
}
void Joint::addToBias( const SpatialVector& b )
{
    sp_Pi += b;
}

Joint* Joint::getJointOfParent()
{
    return m_jointOfParent;
}


} // namespace component

namespace core
{

namespace componentmodel
{

namespace behavior
{

template class MechanicalMapping<component::ArticulatedBody,component::ArticulatedBody>;
template class Constraint<component::ArticulatedBody>;

} // namespace behavior

} // namespace componentmodel

} // namespace core

} // namespace sofa
