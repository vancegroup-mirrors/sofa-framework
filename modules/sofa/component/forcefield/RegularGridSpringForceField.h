#ifndef SOFA_COMPONENT_FORCEFIELD_REGULARGRIDSPRINGFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_REGULARGRIDSPRINGFORCEFIELD_H

#include <sofa/component/forcefield/StiffSpringForceField.h>
#include <sofa/component/topology/FittedRegularGridTopology.h>

namespace sofa
{

namespace component
{

namespace forcefield
{

template<class DataTypes>
class RegularGridSpringForceField : public StiffSpringForceField<DataTypes>
{
    double m_potentialEnergy;
public:
    typedef StiffSpringForceField<DataTypes> Inherit;
    typedef typename Inherit::Spring Spring;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename Coord::value_type Real;

    //virtual const char* getTypeName() const { return "RegularGridSpringForceField"; }

protected:
    Real linesStiffness;
    Real linesDamping;
    Real quadsStiffness;
    Real quadsDamping;
    Real cubesStiffness;
    Real cubesDamping;

public:
    RegularGridSpringForceField(core::componentmodel::behavior::MechanicalState<DataTypes>* object1, core::componentmodel::behavior::MechanicalState<DataTypes>* object2)
        : StiffSpringForceField<DataTypes>(object1, object2),
          linesStiffness(0), linesDamping(0),
          quadsStiffness(0), quadsDamping(0),
          cubesStiffness(0), cubesDamping(0),
          topology(NULL), trimmedTopology(NULL)
    {
    }

    RegularGridSpringForceField()
        : linesStiffness(0), linesDamping(0),
          quadsStiffness(0), quadsDamping(0),
          cubesStiffness(0), cubesDamping(0),
          topology(NULL), trimmedTopology(NULL)
    {
    }

    Real getStiffness() const { return linesStiffness; }
    Real getLinesStiffness() const { return linesStiffness; }
    Real getQuadsStiffness() const { return quadsStiffness; }
    Real getCubesStiffness() const { return cubesStiffness; }
    void setStiffness(Real val)
    {
        linesStiffness = val;
        quadsStiffness = val;
        cubesStiffness = val;
    }
    void setLinesStiffness(Real val)
    {
        linesStiffness = val;
    }
    void setQuadsStiffness(Real val)
    {
        quadsStiffness = val;
    }
    void setCubesStiffness(Real val)
    {
        cubesStiffness = val;
    }

    Real getDamping() const { return linesDamping; }
    Real getLinesDamping() const { return linesDamping; }
    Real getQuadsDamping() const { return quadsDamping; }
    Real getCubesDamping() const { return cubesDamping; }
    void setDamping(Real val)
    {
        linesDamping = val;
        quadsDamping = val;
        cubesDamping = val;
    }
    void setLinesDamping(Real val)
    {
        linesDamping = val;
    }
    void setQuadsDamping(Real val)
    {
        quadsDamping = val;
    }
    void setCubesDamping(Real val)
    {
        cubesDamping = val;
    }

    virtual void parse(core::objectmodel::BaseObjectDescription* arg);

    virtual void init();

    virtual void addForce(VecDeriv& f1, VecDeriv& f2, const VecCoord& x1, const VecCoord& x2, const VecDeriv& v1, const VecDeriv& v2);

    virtual void addDForce(VecDeriv& df1, VecDeriv& df2, const VecDeriv& dx1, const VecDeriv& dx2);

    virtual void draw();

protected:
    topology::RegularGridTopology* topology;
    topology::FittedRegularGridTopology* trimmedTopology;
};

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif
