#ifndef SOFA_COMPONENTS_TETRAHEDRALFEMFORCEFIELD_H
#define SOFA_COMPONENTS_TETRAHEDRALFEMFORCEFIELD_H

#include "Sofa-old/Core/ForceField.h"
#include "Sofa-old/Core/MechanicalObject.h"
#include "Sofa-old/Abstract/VisualModel.h"
#include "Sofa-old/Components/MeshTopology.h"
#include "Sofa-old/Components/Common/Vec.h"
#include "Sofa-old/Components/Common/Mat.h"
//---- Added by Xunlei Wu ----
#include "Sofa-old/Components/Common/Quat.h"
#include "NewMAT/include.h"              // include.h will get math fns
#include "NewMAT/newmatap.h"             // need matrix applications
#include "NewMAT/newmatio.h"            // need matrix output routines
#ifdef use_namespace
using namespace NewMAT;              // access NewMAT namespace
#endif
//---- Added by Xunlei Wu ----

namespace Sofa
{

namespace Components
{

using namespace Common;

template<class DataTypes>
class TetrahedralFEMForceField : public Core::ForceField<DataTypes>, public Abstract::VisualModel
{
public:
	typedef typename DataTypes::VecCoord VecCoord;
	typedef typename DataTypes::VecDeriv VecDeriv;
	typedef VecCoord Vector;
	typedef typename DataTypes::Coord Coord;
	typedef typename DataTypes::Deriv Deriv;
	typedef typename Coord::value_type Real;

	typedef MeshTopology::index_type Index;
	typedef MeshTopology::Tetra Element;
	typedef MeshTopology::SeqTetras VecElement;

	static const int SMALL = 0;   ///< Symbol of small displacements tetrahedron solver
	static const int LARGE = 1;   ///< Symbol of large displacements tetrahedron solver
	static const int POLAR = 2;   ///< Symbol of polar displacements tetrahedron solver

	typedef Mat<12, 12, Real> ComplianceMatrix;
 	typedef Mat<3, 3, Real> Rotation; ///< matrix for rigid Rotations like rotations

protected:
	typedef Vec<12, Real> Displacement;		///< the displacement vector
	
	typedef Mat<6, 6, Real> MaterialStiffness;	///< the matrix of material stiffness
	typedef std::vector<MaterialStiffness> VecMaterialStiffness;         ///< a vector of material stiffness matrices
	VecMaterialStiffness _materialsStiffnesses;					///< the material stiffness matrices vector
	
	typedef Mat<12, 6, Real> StrainDisplacement;	///< the strain-displacement matrix
	typedef std::vector<StrainDisplacement> VecStrainDisplacement;		///< a vector of strain-displacement matrices
	VecStrainDisplacement _strainDisplacements;					   ///< the strain-displacement matrices vector
	
	

	typedef Mat<12, 12, Real> StiffnessMatrix;
	//typedef typename matrix<Real,rectangle<>,compressed<>,row_major >::type CompressedMatrix;
	//CompressedMatrix *_stiffnesses;
	
	
	typedef std::pair<int,Real> Col_Value;
	typedef std::vector< Col_Value > CompressedValue;
	typedef std::vector< CompressedValue > CompressedMatrix;
	CompressedMatrix _stiffnesses;
	//---- Added by Xunlei Wu ----
	NewMAT::SymmetricMatrix _globalInitialStiffnessMatrix;
	NewMAT::SymmetricMatrix _globalInitialComplianceMatrix;
	typedef std::vector<ComplianceMatrix> VecComplianceMatrix;
	VecComplianceMatrix _complianceMatrices;
	//---- Added by Xunlei Wu ----
	
	//just for draw forces
	VecDeriv _forces;
	
	MeshTopology* _mesh;
	const VecElement *_indexedElements;
	VecCoord _initialPoints; ///< the intial positions of the points
	int _method; ///< the computation method of the displacements
	Real _poissonRatio;
	Real _youngModulus;
	Real _dampingRatio;
	bool _updateStiffnessMatrix;
	bool _assembling;

public:
	TetrahedralFEMForceField(Core::MechanicalObject<DataTypes>* /*object*/ = NULL)
	: _mesh(NULL)
	, _indexedElements(NULL)
	, _method(0)
	, _poissonRatio(0)
	, _youngModulus(0)
	, _dampingRatio(0)
	, _updateStiffnessMatrix(true)
	, _assembling(false)
	{
	}

	void setPoissonRatio(Real val) { this->_poissonRatio = val; }

	void setYoungModulus(Real val) { this->_youngModulus = val; }

	void setMethod(int val) { this->_method = val; }

	void setUpdateStiffnessMatrix(bool val) { this->_updateStiffnessMatrix = val; }

	void setComputeGlobalMatrix(bool val) { this->_assembling= val; }
	
	//Core::MechanicalObject<DataTypes>* getObject() { return object; }

	virtual void init();

	virtual void addForce (VecDeriv& f, const VecCoord& x, const VecDeriv& v);
	
	virtual void addDForce (VecDeriv& df, const VecDeriv& dx);
	
        virtual double getPotentialEnergy(const VecCoord& x);
	
        // -- VisualModel interface
	void draw();
	void initTextures() { }
	void update() { }

	// -- Access for Collision response force fields

	const ComplianceMatrix& getElementComplianceMatrix(int elementIndex)
	{
		return _complianceMatrices[elementIndex];
	}

	const Rotation& getElementRotation(int elementIndex)
	{
		return _elementRotations[elementIndex];
	}
	//---- Added by Xunlei Wu ----
	int getMethod() { return _method; };
	VecDeriv& getF() { return _forces; }
	const VecCoord& getInitialPoints() { return _initialPoints; }
	const Quaternion& getNodalQuaternion(Index nodalIndex) { return _nodalQuaternions[nodalIndex]; };
	const std::vector<Quaternion>& getNodalQuaternions() { return _nodalQuaternions; }
	const NewMAT::SymmetricMatrix& getGlobalInitialComplianceMatrix() { return _globalInitialComplianceMatrix; }
	const NewMAT::SymmetricMatrix& getGlobalInitialStiffnessMatrix() { return _globalInitialStiffnessMatrix; }
	//---- Added by Xunlei Wu ----

protected:
	
	void computeGlobalInitialStiffnessMatrix();

	void computeStrainDisplacement( StrainDisplacement &J, Coord a, Coord b, Coord c, Coord d );
	Real peudo_determinant_for_coef ( const Mat<2, 3, Real>&  M );
	
	void computeStiffnessMatrix( StiffnessMatrix& S,StiffnessMatrix& SR,const MaterialStiffness &K, const StrainDisplacement &J, const Rotation& Rot );

	void computeMaterialStiffness(int i, Index&a, Index&b, Index&c, Index&d);

	void computeForce( Displacement &F, const Displacement &Depl, const MaterialStiffness &K, const StrainDisplacement &J );

////////////// small displacements method
	void initSmall(int i, Index&a, Index&b, Index&c, Index&d);
	void accumulateForceSmall( Vector& f, const Vector & p, typename VecElement::const_iterator elementIt, Index elementIndex );
	void accumulateDampingSmall( Vector& f, Index elementIndex );
	void applyStiffnessSmall( Vector& f, Real h, const Vector& x, int i=0, Index a=0,Index b=1,Index c=2,Index d=3  );
	//---- Added by Xunlei Wu ----
	void assignLocalComplianceMatrix( typename VecElement::const_iterator elementIt, Index elementIndex );
	//---- Added by Xunlei Wu ----
	
	std::vector<Rotation> _initialElementInverseRotations;
////////////// large displacements method
	std::vector<fixed_array<Coord,4> > _rotatedInitialElements;   ///< The initials positions in its frame
	std::vector<Rotation> _elementRotations;
	std::vector<Quaternion> _nodalQuaternions;
	void computeNodalQuaternions();
	void initLarge(int i, Index&a, Index&b, Index&c, Index&d);
	void computeRotationLarge( Rotation &r, const Vector &p, const Index &a, const Index &b, const Index &c);
	void accumulateForceLarge( Vector& f, const Vector & p, typename VecElement::const_iterator elementIt, Index elementIndex );
	void accumulateDampingLarge( Vector& f, Index elementIndex );
	void applyStiffnessLarge( Vector& f, Real h, const Vector& x, int i=0, Index a=0,Index b=1,Index c=2,Index d=3 );
	
////////////// polar decomposition method
	void initPolar(int i, Index&a, Index&b, Index&c, Index&d);
	void accumulateForcePolar( Vector& f, const Vector & p, typename VecElement::const_iterator elementIt, Index elementIndex );
	void applyStiffnessPolar( Vector& f, Real h, const Vector& x, int i=0, Index a=0,Index b=1,Index c=2,Index d=3  );
};

} // namespace Components

} // namespace Sofa

#endif
