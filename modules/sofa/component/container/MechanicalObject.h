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
#ifndef SOFA_COMPONENT_MECHANICALOBJECT_H
#define SOFA_COMPONENT_MECHANICALOBJECT_H

#include <sofa/component/component.h>

#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/core/topology/BaseMeshTopology.h>

#include <sofa/defaulttype/BaseVector.h>
#include <sofa/defaulttype/LaparoscopicRigidTypes.h>
#include <sofa/defaulttype/MapMapSparseMatrix.h>
#include <sofa/defaulttype/Quat.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/VecTypes.h>

#include <vector>
#include <fstream>

namespace sofa
{

namespace component
{

namespace container
{

using namespace core::behavior;
using namespace core::objectmodel;
using sofa::defaulttype::Vector3;

/// This class can be overridden if needed for additionnal storage within template specializations.
template <class DataTypes>
class MechanicalObject;

template<class DataTypes>
class MechanicalObjectInternalData
{
public:
    MechanicalObjectInternalData(MechanicalObject<DataTypes>* = NULL) {};
};

/**
 * @brief MechanicalObject class
 */
template <class DataTypes>
class MechanicalObject : public MechanicalState<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(MechanicalObject, DataTypes),SOFA_TEMPLATE(MechanicalState, DataTypes));

    typedef MechanicalState<DataTypes>      Inherited;
    typedef typename Inherited::VMultiOp    VMultiOp;
    typedef typename DataTypes::Real        Real;
    typedef typename DataTypes::Coord       Coord;
    typedef typename DataTypes::Deriv       Deriv;
    typedef typename DataTypes::VecCoord    VecCoord;
    typedef typename DataTypes::VecDeriv    VecDeriv;
    typedef typename DataTypes::MatrixDeriv						MatrixDeriv;
    typedef typename DataTypes::MatrixDeriv::RowConstIterator	MatrixDerivRowConstIterator;
    typedef typename DataTypes::MatrixDeriv::ColConstIterator	MatrixDerivColConstIterator;
    typedef typename DataTypes::MatrixDeriv::RowIterator		MatrixDerivRowIterator;
    typedef typename DataTypes::MatrixDeriv::ColIterator		MatrixDerivColIterator;

    typedef typename core::behavior::BaseMechanicalState::ConstraintBlock ConstraintBlock;

    using Inherited::getX;
    using Inherited::getV;
    using Inherited::getF;
    using Inherited::getX0;
    using Inherited::getN;
    using Inherited::getDx;

    MechanicalObject();

    MechanicalObject& operator = ( const MechanicalObject& );

    virtual ~MechanicalObject();

    virtual bool load(const char* filename);

    virtual void parse ( BaseObjectDescription* arg );

    Data< VecCoord > x;
    Data< VecDeriv > v;
    Data< VecDeriv > f;
    Data< VecDeriv > externalForces;
    Data< VecDeriv > dx;
    Data< VecCoord > xfree;
    Data< VecDeriv > vfree;
    Data< VecCoord > x0;
    Data< MatrixDeriv > c;
    Data< VecCoord > reset_position;
    Data< VecDeriv > reset_velocity;

    defaulttype::MapMapSparseMatrix< Deriv > c2;

    Data< SReal > restScale;

    Data< bool >  showObject;
    Data< float > showObjectScale;
    Data< bool >  showIndices;
    Data< float > showIndicesScale;

    virtual void init();
    virtual void reinit();

    virtual void storeResetState();

    virtual void reset();

    virtual void writeX(std::ostream& out);
    virtual void readX(std::istream& in);
    virtual double compareX(std::istream& in);
    virtual void writeV(std::ostream& out);
    virtual void readV(std::istream& in);
    virtual double compareV(std::istream& in);
    virtual void writeF(std::ostream& out);
    virtual void writeDx(std::ostream& out);

    virtual void writeVec(core::ConstVecId v, std::ostream &out);
    virtual void readVec(core::VecId v, std::istream &in);
    virtual double compareVec(core::ConstVecId v, std::istream &in);

    virtual void writeState( std::ostream& out );

    /// @name New vectors access API based on VecId
    /// @{

    virtual Data< VecCoord >* write(core::VecCoordId v);
    virtual const Data< VecCoord >* read(core::ConstVecCoordId v) const;

    virtual Data< VecDeriv >* write(core::VecDerivId v);
    virtual const Data< VecDeriv >* read(core::ConstVecDerivId v) const;

    virtual Data< MatrixDeriv >* write(core::MatrixDerivId v);
    virtual const Data< MatrixDeriv >* read(core::ConstMatrixDerivId v) const;

    /// @}

    virtual void initGnuplot(const std::string path);
    virtual void exportGnuplot(Real time);

    virtual void resize( int vsize);
    virtual void reserve(int vsize);

    virtual bool addBBox(double* minBBox, double* maxBBox);

    int getSize() const { return vsize; }

    double getPX(int i) const { Real x=0.0,y=0.0,z=0.0; DataTypes::get(x,y,z,(*getX())[i]); return (SReal)x; }
    double getPY(int i) const { Real x=0.0,y=0.0,z=0.0; DataTypes::get(x,y,z,(*getX())[i]); return (SReal)y; }
    double getPZ(int i) const { Real x=0.0,y=0.0,z=0.0; DataTypes::get(x,y,z,(*getX())[i]); return (SReal)z; }

    /** \brief Overwrite values at index outputIndex by the ones at inputIndex.
     *
     */
    void replaceValue (const int inputIndex, const int outputIndex);

    /** \brief Exchange values at indices idx1 and idx2.
     *
     */
    void swapValues (const int idx1, const int idx2);

    /** \brief Reorder values according to parameter.
     *
     * Result of this method is :
     * newValue[ i ] = oldValue[ index[i] ];
     */
    void renumberValues( const sofa::helper::vector<unsigned int> &index );

    /** \brief Replace the value at index by the sum of the ancestors values weithed by the coefs.
     *
     * Sum of the coefs should usually equal to 1.0
     */
    void computeWeightedValue( const unsigned int i, const sofa::helper::vector< unsigned int >& ancestors, const sofa::helper::vector< double >& coefs);

    /// Force the position of a point (and force its velocity to zero value)
    void forcePointPosition( const unsigned int i, const sofa::helper::vector< double >& m_x);

    /// @name Initial transformations application methods.
    /// @{

    /// Apply translation vector to the position.
    virtual void applyTranslation (const double dx, const double dy, const double dz);

    /// Rotation using Euler Angles in degree.
    virtual void applyRotation (const double rx, const double ry, const double rz);

    virtual void applyRotation (const defaulttype::Quat q);

    virtual void applyScale (const double sx, const double sy, const double sz);

    /// @}

    /// Get the indices of the particles located in the given bounding box
    void getIndicesInSpace(sofa::helper::vector<unsigned>& indices, Real xmin, Real xmax, Real ymin, Real ymax, Real zmin, Real zmax) const;

    /// @name Base Matrices and Vectors Interface
    /// @{

    /// Copy data to a global BaseVector the state stored in a local vector
    /// @param offset the offset in the BaseVector where the scalar values will be used. It will be updated to the first scalar value after the ones used by this operation when this method returns
    virtual void copyToBaseVector(defaulttype::BaseVector* dest, core::ConstVecId src, unsigned int &offset);

    /// Copy data to a local vector the state stored in a global BaseVector
    /// @param offset the offset in the BaseVector where the scalar values will be used. It will be updated to the first scalar value after the ones used by this operation when this method returns
    virtual void copyFromBaseVector(core::VecId dest, const defaulttype::BaseVector* src, unsigned int &offset);

    /// Add data to a global BaseVector from the state stored in a local vector
    /// @param offset the offset in the BaseVector where the scalar values will be used. It will be updated to the first scalar value after the ones used by this operation when this method returns
    virtual void addToBaseVector(defaulttype::BaseVector* dest, core::ConstVecId src, unsigned int &offset);

    /// src and dest must have the same size.
    /// Performs: dest[i][j] += src[offset + i][j] 0<= i < src_entries  0<= j < 3 (for 3D objects) 0 <= j < 2 (for 2D objects)
    /// @param offset the offset in the BaseVector where the scalar values will be used. It will be updated to the first scalar value after the ones used by this operation when this method returns
    virtual void addFromBaseVectorSameSize(core::VecId dest, const defaulttype::BaseVector* src, unsigned int &offset);

    /// src size can be smaller or equal to dest size.
    /// Performs: dest[ offset + i ][j] += src[i][j]  0<= i < src_entries  0<= j < 3 (for 3D objects) 0 <= j < 2 (for 2D objects)
    /// @param offset the offset in the MechanicalObject local vector specified by VecId dest. It will be updated to the first scalar value after the ones used by this operation when this method returns.
    virtual void addFromBaseVectorDifferentSize(core::VecId dest, const defaulttype::BaseVector* src, unsigned int &offset );


    /// @}

    /// Express the matrix L in term of block of matrices, using the indices of the lines in the MatrixDeriv container
    virtual std::list<ConstraintBlock> constraintBlocks( const std::list<unsigned int> &indices) const;
    virtual SReal getConstraintJacobianTimesVecDeriv( unsigned int line, core::ConstVecId id);

    void setFilename(std::string s) {filename.setValue(s);};

    /// @name Initial transformations accessors.
    /// @{

    void setTranslation(double dx, double dy, double dz) {translation.setValue(Vector3(dx,dy,dz));};
    void setRotation(double rx, double ry, double rz) {rotation.setValue(Vector3(rx,ry,rz));};
    void setScale(double sx, double sy, double sz) {scale.setValue(Vector3(sx,sy,sz));};

    virtual Vector3 getTranslation() const {return translation.getValue();};
    virtual Vector3 getRotation() const {return rotation.getValue();};
    virtual Vector3 getScale() const {return scale.getValue();};

    /// @}

    void setIgnoreLoader(bool b) {ignoreLoader.setValue(b);}

    std::string getFilename() {return filename.getValue();};

    virtual void addDxToCollisionModel(void);

    /// Renumber the constraint ids with the given permutation vector
    void renumberConstraintId(const sofa::helper::vector< unsigned >& renumbering);


    /// @name Integration related methods
    /// @{

    virtual void beginIntegration(Real dt);

    virtual void endIntegration(Real dt, const core::ExecParams* params);

    virtual void accumulateForce(const core::ExecParams* params);

    /// Increment the index of the given VecCoordId, so that all 'allocated' vectors in this state have a lower index
    virtual void vAvail(core::VecCoordId& v, const core::ExecParams* params);
    /// Increment the index of the given VecDerivId, so that all 'allocated' vectors in this state have a lower index
    virtual void vAvail(core::VecDerivId& v, const core::ExecParams* params);
    /// Increment the index of the given MatrixDerivId, so that all 'allocated' vectors in this state have a lower index
    //virtual void vAvail(core::MatrixDerivId& v);

    /// Allocate a new temporary vector
    virtual void vAlloc(core::VecCoordId v, const core::ExecParams* params);
    /// Allocate a new temporary vector
    virtual void vAlloc(core::VecDerivId v, const core::ExecParams* params);
    /// Allocate a new temporary vector
    //virtual void vAlloc(core::MatrixDerivId v);

    /// Free a temporary vector
    virtual void vFree(core::VecCoordId v, const core::ExecParams* params);
    /// Free a temporary vector
    virtual void vFree(core::VecDerivId v, const core::ExecParams* params);
    /// Free a temporary vector
    //virtual void vFree(core::MatrixDerivId v);

    /// Initialize an unset vector
    virtual void vInit(core::VecCoordId v, core::ConstVecCoordId vSrc, const core::ExecParams* params);
    /// Initialize an unset vector
    virtual void vInit(core::VecDerivId v, core::ConstVecDerivId vSrc, const core::ExecParams* params);
    /// Initialize an unset vector
    //virtual void vInit(core::MatrixDerivId v, core::ConstMatrixDerivId vSrc, const core::ExecParams* params);

    virtual void vOp(core::VecId v, core::ConstVecId a = core::ConstVecId::null(), core::ConstVecId b = core::ConstVecId::null(), double f=1.0, const core::ExecParams* params= core::ExecParams::defaultInstance());

#ifdef SOFA_SMP
    virtual void vOp(core::VecId, core::ConstVecId, core::ConstVecId, double f, a1::Shared<double> *fSh, const core::ExecParams* params);
    virtual void vOpMEq(core::VecId, core::ConstVecId  = core::ConstVecId::null(), a1::Shared<double> * =NULL, const core::ExecParams* params = core::ExecParams::defaultInstance());
    virtual void vDot(a1::Shared<double> *, core::ConstVecId , core::ConstVecId, const core::ExecParams* params);
#endif

    virtual void vMultiOp(const VMultiOp& ops, const core::ExecParams* params);

    virtual void vThreshold(core::VecId a, double threshold );

    virtual double vDot(core::ConstVecId a, core::ConstVecId b, const core::ExecParams* params);

    virtual void resetForce(const core::ExecParams* params);

    virtual void resetAcc(const core::ExecParams* params);

    virtual void resetConstraint(const core::ExecParams* params);

    /// @}

    /// @name Debug
    /// @{

    virtual void printDOF(core::ConstVecId, std::ostream& =std::cerr, int firstIndex=0, int range=-1 ) const ;
    virtual unsigned printDOFWithElapsedTime(core::VecId, unsigned =0, unsigned =0, std::ostream& =std::cerr );

    void draw();

    /// @}

    // handle state changes
    virtual void handleStateChange();

    /// Find mechanical particles hit by the given ray.
    /// A mechanical particle is defined as a 2D or 3D, position or rigid DOF
    /// Returns false if this object does not support picking
    virtual bool pickParticles(double rayOx, double rayOy, double rayOz, double rayDx, double rayDy, double rayDz, double radius0, double dRadius,
            std::multimap< double, std::pair<sofa::core::behavior::BaseMechanicalState*, int> >& particles, const core::ExecParams* params);

protected :

    /// @name Initial geometric transformations
    /// @{

    Data< Vector3 > translation;
    Data< Vector3 > rotation;
    Data< Vector3 > scale;
    Data< Vector3 > translation2;
    Data< Vector3 > rotation2;

    /// @}

    sofa::core::objectmodel::DataFileName filename;
    Data< bool> ignoreLoader;
    Data< int > f_reserve;

    bool m_initialized;

    /// @name Integration-related data
    /// @{

    sofa::helper::vector< Data< VecCoord >		* > vectorsCoord;		///< Coordinates DOFs vectors table (static and dynamic allocated)
    sofa::helper::vector< Data< VecDeriv >		* > vectorsDeriv;		///< Derivates DOFs vectors table (static and dynamic allocated)
    sofa::helper::vector< Data< MatrixDeriv >	* > vectorsMatrixDeriv; ///< Constraint vectors table

    int vsize; ///< Number of elements to allocate in vectors

    /**
     * @brief Inserts VecCoord DOF coordinates vector at index in the vectorsCoord container.
     */
    void setVecCoord(unsigned int /*index*/, Data< VecCoord >* /*vCoord*/);

    /**
     * @brief Inserts VecDeriv DOF derivates vector at index in the vectorsDeriv container.
     */
    void setVecDeriv(unsigned int /*index*/, Data< VecDeriv >* /*vDeriv*/);

    /**
     * @brief Inserts MatrixDeriv DOF  at index in the MatrixDeriv container.
     */
    void setVecMatrixDeriv(unsigned int /*index*/, Data< MatrixDeriv> * /*mDeriv*/);

    /// @}

    /// Given the number of a constraint Equation, find the index in the MatrixDeriv C, where the constraint is actually stored
    // unsigned int getIdxConstraintFromId(unsigned int id) const;

    MechanicalObjectInternalData<DataTypes> data;

    friend class MechanicalObjectInternalData<DataTypes>;

    std::ofstream* m_gnuplotFileX;
    std::ofstream* m_gnuplotFileV;

    class Loader;

    sofa::core::topology::BaseMeshTopology* m_topology;
};

#if defined(WIN32) && !defined(SOFA_COMPONENT_CONTAINER_MECHANICALOBJECT_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_COMPONENT_CONTAINER_API MechanicalObject<defaulttype::Vec3dTypes>;
extern template class SOFA_COMPONENT_CONTAINER_API MechanicalObject<defaulttype::Vec2dTypes>;
extern template class SOFA_COMPONENT_CONTAINER_API MechanicalObject<defaulttype::Vec1dTypes>;
extern template class SOFA_COMPONENT_CONTAINER_API MechanicalObject<defaulttype::Vec6dTypes>;
extern template class SOFA_COMPONENT_CONTAINER_API MechanicalObject<defaulttype::Rigid3dTypes>;
extern template class SOFA_COMPONENT_CONTAINER_API MechanicalObject<defaulttype::Rigid2dTypes>;
#ifdef SOFA_DEV
#endif
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_COMPONENT_CONTAINER_API MechanicalObject<defaulttype::Vec3fTypes>;
extern template class SOFA_COMPONENT_CONTAINER_API MechanicalObject<defaulttype::Vec2fTypes>;
extern template class SOFA_COMPONENT_CONTAINER_API MechanicalObject<defaulttype::Vec1fTypes>;
extern template class SOFA_COMPONENT_CONTAINER_API MechanicalObject<defaulttype::Vec6fTypes>;
extern template class SOFA_COMPONENT_CONTAINER_API MechanicalObject<defaulttype::Rigid3fTypes>;
extern template class SOFA_COMPONENT_CONTAINER_API MechanicalObject<defaulttype::Rigid2fTypes>;
#endif
extern template class SOFA_COMPONENT_CONTAINER_API MechanicalObject<defaulttype::LaparoscopicRigid3Types>;
#endif

} // namespace container

} // namespace component

} // namespace sofa

#endif
