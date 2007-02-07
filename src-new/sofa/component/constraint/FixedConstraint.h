#ifndef SOFA_COMPONENTS_FIXEDCONSTRAINT_H
#define SOFA_COMPONENTS_FIXEDCONSTRAINT_H

#include "Sofa-old/Core/Constraint.h"
#include "Sofa-old/Core/MechanicalModel.h"
#include "Sofa-old/Abstract/VisualModel.h"
#include "Sofa-old/Abstract/Event.h"
#include "Sofa-old/Components/Common/SofaBaseMatrix.h"
#include "Sofa-old/Components/Common/SofaBaseVector.h"
#include "Sofa-old/Components/Common/vector.h"
#include "Topology/PointSubset.h"
#include <set>

namespace Sofa
{

namespace Components
{

using Common::vector;
using Common::DataField;
using namespace Sofa::Abstract;

/// This class can be overridden if needed for additionnal storage within template specializations.
template <class DataTypes>
class FixedConstraintInternalData
{
};

template <class DataTypes>
class FixedConstraint : public Core::Constraint<DataTypes>, public Abstract::VisualModel
{
public:
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef PointSubset SetIndex;
    typedef Common::vector<unsigned int> SetIndexArray;
    //typedef std::set<int> SetIndex;

protected:
    //SetIndex indices;

    FixedConstraintInternalData<DataTypes> data;

public:
    FixedConstraint();
    virtual const char* getTypeName() const { return "FixedConstraint"; }
    DataField<SetIndex> f_indices;


    FixedConstraint(Core::MechanicalModel<DataTypes>* mmodel);

    virtual ~FixedConstraint();

    FixedConstraint<DataTypes>* addConstraint(unsigned int index);
    FixedConstraint<DataTypes>* removeConstraint(unsigned int index);

    // -- Constraint interface
    void init();
    void projectResponse(VecDeriv& dx);
    virtual void projectVelocity(VecDeriv& /*dx*/) {} ///< project dx to constrained space (dx models a velocity)
    virtual void projectPosition(VecCoord& /*x*/) {} ///< project x to constrained space (x models a position)

    void applyConstraint(Components::Common::SofaBaseMatrix *mat, unsigned int &offset);
    void applyConstraint(Components::Common::SofaBaseVector *vect, unsigned int &offset);

    // handle topological changes
    virtual void handleEvent( Event* );

    // -- VisualModel interface

    virtual void draw();

    void initTextures() { }

    void update() { }
};

} // namespace Components

} // namespace Sofa

#endif
