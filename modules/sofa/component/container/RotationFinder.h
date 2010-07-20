/*
 * RotationFinder.h
 *
 *  Created on: 14 avr. 2009
 *      Author: froy
 */

#ifndef ROTATIONFINDER_H_
#define ROTATIONFINDER_H_

#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/component/topology/TriangleSetTopologyContainer.h>
#include <sofa/component/topology/TriangleSetGeometryAlgorithms.h>
#include <sofa/component/topology/TriangleSetGeometryAlgorithms.inl>
#include <sofa/component/component.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/component/misc/BaseRotationFinder.h>

/*
 *	This class find Rotation Matrix from two position states (rest and current state)
 *	got from an associated Mechanical State.
 *	Formula & method taken from :
 *	"Meshless Deformations Based on Shape Matching" (Muller, Heidelberger, Teschner and Gross)
 *
 */

namespace sofa
{

namespace component
{

namespace container
{

template <class DataTypes>
class RotationFinder : public virtual sofa::component::misc::BaseRotationFinder
{
public:
	SOFA_CLASS(RotationFinder,sofa::core::objectmodel::BaseObject);

	typedef typename DataTypes::VecCoord VecCoord;
	typedef typename DataTypes::VecDeriv VecDeriv;
	typedef typename DataTypes::Coord Coord;
	typedef typename DataTypes::Deriv Deriv;
	typedef typename Coord::value_type Real;
	typedef defaulttype::Mat<3,3,Real> Mat3x3;
	typedef helper::fixed_array<Mat3x3,9> DMat3x3;

	typedef core::topology::BaseMeshTopology::PointID Point;

	typedef std::set<Point> Neighborhood;
	typedef helper::vector<Neighborhood> VecNeighborhood;

private:
    core::behavior::MechanicalState<DataTypes>* mechanicalState;
    core::topology::BaseMeshTopology* topo;

	//rest data
	Coord x0_cm;
	unsigned int oldRestPositionSize;
	VecNeighborhood pointNeighborhood, lastPointNeighborhood;
	VecCoord Xcm, Xcm0;
	helper::vector<Mat3x3> rotations, vecA;
	helper::vector<DMat3x3> dRotations;
	helper::vector<Mat3x3> dRotations_dx;

        template<class T>
        T min(const T a, const T b) const
	{
		return a < b ? a : b;
	}
        void ComputeNeighborhoodFromNeighborhood();

public:
	enum Axis { X, Y, Z };

	Data<int> axisToFlip;
	Data<bool> showRotations;
        Data<int> neighborhoodLevel;
	Data<int> numOfClusters;
        Data<unsigned int> maxIter;
        Data<Real> epsilon;
        Data<Real> radius;
        
	RotationFinder();
	virtual ~RotationFinder();

	void init();

	const helper::vector<Mat3x3>& getRotations();
	
	void getRotations(defaulttype::BaseMatrix * m,int offset = 0) ;

	const helper::vector<DMat3x3>& getDRotations();

	const helper::vector<Mat3x3>& getDRotations(const VecDeriv& dx);

	void computeQT();

	const VecCoord& getCM() { return Xcm; }

	const VecCoord& getCM0() { return Xcm0; }

	void computeNeighborhood();

	const VecNeighborhood& getNeighborhood();

	void handleEvent(sofa::core::objectmodel::Event* event);

	void draw();

	void flipAxis(Mat3x3 & rotation);
        
public:
    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MechanicalState.
    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        if (dynamic_cast<core::behavior::MechanicalState<DataTypes>*>(context->getMechanicalState()) == NULL)
            return false;
        return BaseObject::canCreate(obj, context, arg);
    }

    virtual std::string getTemplateName() const
      {
        return templateName(this);
      }

    static std::string templateName(const RotationFinder<DataTypes>* = NULL)
    {
      return DataTypes::Name();
    }

};

#if defined(WIN32) && !defined(SOFA_COMPONENT_CONTAINER_ROTATIONFINDER_CPP)
#pragma warning(disable : 4231)
#ifndef SOFA_FLOAT
extern template class SOFA_COMPONENT_CONTAINER_API RotationFinder<defaulttype::Vec3dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_COMPONENT_CONTAINER_API RotationFinder<defaulttype::Vec3fTypes>;
#endif
#endif

} // namespace constraint

} // namespace component

} // namespace sofa

#endif /* ROTATIONFINDER_H_ */
