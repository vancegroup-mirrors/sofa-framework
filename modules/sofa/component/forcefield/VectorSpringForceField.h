#ifndef SOFA_COMPONENT_FORCEFIELD_VECTORSPRINGFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_VECTORSPRINGFORCEFIELD_H

#include <sofa/component/forcefield/SpringForceField.h>
#include <sofa/core/componentmodel/behavior/ForceField.h>
#include <sofa/core/componentmodel/behavior/MechanicalState.h>
#include <sofa/core/VisualModel.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/component/topology/EdgeData.h>
#include <sofa/component/topology/TopologyChangedEvent.h>

namespace sofa
{

namespace component
{

namespace forcefield
{

using namespace sofa::defaulttype;

template<class DataTypes>
class VectorSpringForceField
: public core::componentmodel::behavior::PairInteractionForceField<DataTypes>, public core::VisualModel
//: public core::componentmodel::behavior::ForceField<DataTypes>, public core::VisualModel
{
public:
    typedef typename core::componentmodel::behavior::PairInteractionForceField<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename Coord::value_type Real;
    typedef core::componentmodel::behavior::MechanicalState<DataTypes> MechanicalState;
    enum { N=Coord::static_size };

    struct Spring
    {
        Real  ks;          ///< spring stiffness
        Real  kd;          ///< damping factor 
        Deriv restVector;  ///< rest vector of the spring 

        Spring(Real _ks, Real _kd, Deriv _rl) : ks(_ks), kd(_kd), restVector(_rl)
        {
        }
        Spring() : ks(1.0), kd(1.0)
        {
        }
    };
protected:

    double m_potentialEnergy;
    /// true if the springs are initialized from the topology
    bool useTopology;
    /// the EdgeSet topology used to get the list of edges
    sofa::component::topology::EdgeSetTopology<DataTypes> *topology;
    /// indices in case we don't use the topology
    sofa::helper::vector<topology::Edge> edgeArray;
    /// where the springs information are stored
    sofa::component::topology::EdgeData<Spring> springArray;

    /// the filename where to load the spring information
    Data<std::string> m_filename;
    /// By default, assume that all edges have the same stiffness
    Data<double> m_stiffness;
    /// By default, assume that all edges have the same viscosity
    Data<double> m_viscosity;

    void resizeArray(unsigned int n);

    static void springCreationFunction(int /*index*/,
        void* param, Spring& t,
        const topology::Edge& e,
        const sofa::helper::vector< unsigned int > &ancestors,
        const sofa::helper::vector< double >& coefs);

public:

    VectorSpringForceField(MechanicalState* _object=NULL);

    VectorSpringForceField(MechanicalState* _object1, MechanicalState* _object2);

    bool load(const char *filename);

    core::componentmodel::behavior::MechanicalState<DataTypes>* getObject1() { return this->mstate1; }
    core::componentmodel::behavior::MechanicalState<DataTypes>* getObject2() { return this->mstate2; }

    virtual void init();

    void createDefaultSprings();

    virtual void handleEvent( Event* e );

    virtual void addForce(VecDeriv& f1, VecDeriv& f2, const VecCoord& x1, const VecCoord& x2, const VecDeriv& v1, const VecDeriv& v2);
    //virtual void addForce (VecDeriv& f, const VecCoord& x, const VecDeriv& v);

    virtual void addDForce(VecDeriv& df1, VecDeriv& df2, const VecDeriv& dx1, const VecDeriv& dx2);
    //virtual void addDForce (VecDeriv& df, const VecDeriv& dx);

    //virtual double getPotentialEnergy(const VecCoord& )
    virtual double getPotentialEnergy(const VecCoord&, const VecCoord&)
    { return m_potentialEnergy; }

    Real getStiffness() const
    {
        return (Real)(m_stiffness.getValue());
    }
    const Real getViscosity() const
    {
        return (Real)(m_viscosity.getValue());
    }
    const topology::EdgeData<Spring>& getSpringArray() const
    {
        return springArray;
    }
    // -- VisualModel interface
    void draw();
    void initTextures() {}
    void update() {}

    // -- Modifiers

    void clear(int reserve=0)
    {
        springArray.clear();
        if (reserve) springArray.reserve(reserve);
    }

    void addSpring(int m1, int m2, double ks, double kd, Coord restVector);

    /// forward declaration of the loader class used to read spring information from file
    class Loader;
    friend class Loader;

};

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif
