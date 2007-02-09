#ifndef SOFA_COMPONENT_MAPPING_LAPAROSCOPICRIGIDMAPPING_H
#define SOFA_COMPONENT_MAPPING_LAPAROSCOPICRIGIDMAPPING_H

#include <sofa/core/componentmodel/behavior/MechanicalMapping.h>
#include <sofa/core/componentmodel/behavior/MechanicalState.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/LaparoscopicRigidTypes.h>
#include <sofa/core/VisualModel.h>
#include <vector>

namespace sofa
{

namespace component
{

namespace mapping
{

template <class BasicMapping>
class LaparoscopicRigidMapping : public BasicMapping, public core::VisualModel
{
public:
    typedef BasicMapping Inherit;
    typedef typename Inherit::In In;
    typedef typename Inherit::Out Out;
    typedef typename Out::VecCoord VecCoord;
    typedef typename Out::VecDeriv VecDeriv;
    typedef typename Out::Coord Coord;
    typedef typename Out::Deriv Deriv;
    //typedef typename Coord::value_type Real;

public:
    DataField<defaulttype::Vector3> pivot;
    DataField<defaulttype::Quat> rotation;

    LaparoscopicRigidMapping(In* from, Out* to)
        : Inherit(from, to)
        , pivot(dataField(&pivot, defaulttype::Vector3(0,0,0), "pivot","TODO-pivot"))
        , rotation(dataField(&rotation, defaulttype::Quat(0,0,0,1), "rotation", "TODO-rotation"))
    {
    }

    virtual ~LaparoscopicRigidMapping()
    {
    }

    //void setPivot(const defaulttype::Vector3& val) { this->pivot = val; }
    //void setRotation(const defaulttype::Quat& val) { this->rotation = val; this->rotation.normalize(); }

    void init();

    void apply( typename Out::VecCoord& out, const typename In::VecCoord& in );

    void applyJ( typename Out::VecDeriv& out, const typename In::VecDeriv& in );

    void applyJT( typename In::VecDeriv& out, const typename Out::VecDeriv& in );

    // -- VisualModel interface
    void draw();
    void initTextures() { }
    void update() { }

protected:

    bool getShow(const core::objectmodel::BaseObject* m) const { return m->getContext()->getShowMappings(); }

    bool getShow(const core::componentmodel::behavior::BaseMechanicalMapping* m) const { return m->getContext()->getShowMechanicalMappings(); }
};

} // namespace mapping

} // namespace component

} // namespace sofa

#endif
