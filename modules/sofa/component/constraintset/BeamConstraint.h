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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_BEAMCONSTRAINT_H
#define SOFA_COMPONENT_CONSTRAINTSET_BEAMCONSTRAINT_H

#include <sofa/core/behavior/InteractionConstraint.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/VisualModel.h>
#include <iostream>

namespace sofa
{

namespace component
{

namespace constraintset
{

template<class DataTypes>
class BeamConstraint : public core::behavior::InteractionConstraint
{
public:
	SOFA_CLASS(SOFA_TEMPLATE(BeamConstraint,DataTypes),sofa::core::behavior::InteractionConstraint);

	typedef typename DataTypes::VecCoord VecCoord;
	typedef typename DataTypes::VecDeriv VecDeriv;
	typedef typename DataTypes::VecConst VecConst;
	typedef typename DataTypes::SparseVecDeriv SparseVecDeriv;
	typedef typename DataTypes::Coord Coord;
	typedef typename DataTypes::Deriv Deriv;
	typedef typename Coord::value_type Real;
	typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;

protected:
	MechanicalState* object1;
	MechanicalState* object2;
	bool yetIntegrated;
	unsigned int cid;
		
	std::vector<Real> dists;	// constraints violations
	int nbConstraints; // number of constraints created
	std::vector<int> previousSegment; // the segment on which each point was projected
		
	Data<double> autoAdvance;
	
// for draw :
	std::vector<bool> projected;
	
	void internalInit();
	void getOrthogonalVectors(const Coord& dir, Coord& vec1, Coord& vec2);
	Real checkProjection(int point, int segment);
	void createConstraints(int point, int segment, int& nbConstraints, bool firstPoint);

	/// this constraint is holonomic
	bool isHolonomic() {return true;}

public:
	
	BeamConstraint(MechanicalState* object1, MechanicalState* object2)
	: object1(object1), object2(object2), yetIntegrated(false)
	, autoAdvance(initData(&autoAdvance, (double)0.0002, "auto_advance","speed of the sliding object (if 0, free movement)") )
	{
	}
	
	BeamConstraint(MechanicalState* object)
	: object1(object), object2(object), yetIntegrated(false)
	, autoAdvance(initData(&autoAdvance, (double)0.0002, "auto_advance","speed of the sliding object (if 0, free movement)") )
	{
	}

	BeamConstraint()
	: object1(NULL), object2(NULL), yetIntegrated(false)
	, autoAdvance(initData(&autoAdvance, (double)0.0002, "auto_advance","speed of the sliding object (if 0, free movement)") )
	{
	}
	
	virtual ~BeamConstraint()
	{
	}
	
	MechanicalState* getObject1() { return object1; }
	MechanicalState* getObject2() { return object2; }
	core::behavior::BaseMechanicalState* getMechModel1() { return object1; }
	core::behavior::BaseMechanicalState* getMechModel2() { return object2; }
	
	virtual void reset();
	
	virtual void init();
	
	virtual void bwdInit();

    virtual void buildConstraintMatrix(unsigned int & /*constraintId*/, core::VecId);

	virtual void getConstraintValue(defaulttype::BaseVector *, bool /* freeMotion */ = true );

	virtual void getConstraintId(long* id, unsigned int &offset);
#ifdef SOFA_DEV
	virtual void getConstraintResolution(std::vector<core::behavior::ConstraintResolution*>& resTab, unsigned int& offset);
#endif
    /// Pre-construction check method called by ObjectFactory.
    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        if (arg->getAttribute("object1") || arg->getAttribute("object2"))
        {
            if (dynamic_cast<MechanicalState*>(arg->findObject(arg->getAttribute("object1",".."))) == NULL)
                return false;
            if (dynamic_cast<MechanicalState*>(arg->findObject(arg->getAttribute("object2",".."))) == NULL)
                return false;
        }
        else
        {
            if (dynamic_cast<MechanicalState*>(context->getMechanicalState()) == NULL)
                return false;
        }
        return core::behavior::InteractionConstraint::canCreate(obj, context, arg);
    }

    /// Construction method called by ObjectFactory.
    template<class T>
    static void create(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        core::behavior::InteractionConstraint::create(obj, context, arg);
        if (arg && (arg->getAttribute("object1") || arg->getAttribute("object2")))
        {
            obj->object1 = dynamic_cast<MechanicalState*>(arg->findObject(arg->getAttribute("object1","..")));
            obj->object2 = dynamic_cast<MechanicalState*>(arg->findObject(arg->getAttribute("object2","..")));
        }
        else if (context)
        {
            obj->object1 =
            obj->object2 =
                dynamic_cast<MechanicalState*>(context->getMechanicalState());
        }
    }

    virtual std::string getTemplateName() const
    {
      return templateName(this);
    }

    static std::string templateName(const BeamConstraint<DataTypes>* = NULL)
    {
      return DataTypes::Name();
    }
	void draw();
};
} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_BILATERALINTERACTIONCONSTRAINT_H
