#ifndef SOFA_COMPONENT_FORCEFIELD_NONUNIFORMHEXAHEDRALFEMFORCEFIELDANDMASSCORRECTED_H
#define SOFA_COMPONENT_FORCEFIELD_NONUNIFORMHEXAHEDRALFEMFORCEFIELDANDMASSCORRECTED_H


#include <sofa/component/forcefield/NonUniformHexahedralFEMForceFieldAndMass.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include<Eigen/StdVector>


namespace sofa{
    namespace component{
        namespace forcefield{


            /** \brief NonUniformHexahedralFEMForceFieldAndMass variant with matrix correction to limit ill-conditioning problems.
              Matrix \f$ \alpha M + \beta B + \gamma C \f$ used in implicit solvers is explicitly computed and stored (note that this requires that attribute useMBK in parent class is true).
              At each time step, an SVD factorization of this matrix is computed, the singular values are tested and the smallest ones are increased if necessary.
              */
            template< class DataTypes >
            class NonUniformHexahedralFEMForceFieldAndMassCorrected : public NonUniformHexahedralFEMForceFieldAndMass<DataTypes>
            {
            public:
                SOFA_CLASS( SOFA_TEMPLATE(NonUniformHexahedralFEMForceFieldAndMassCorrected,DataTypes),SOFA_TEMPLATE(NonUniformHexahedralFEMForceFieldAndMass,DataTypes) );

                typedef NonUniformHexahedralFEMForceFieldAndMass<DataTypes> Inherited;
                typedef typename DataTypes::VecCoord VecCoord;
                typedef typename DataTypes::VecDeriv VecDeriv;
                typedef typename Inherited::Element Element;
                typedef typename Inherited::Deriv Deriv;
                typedef typename Inherited::Mat33 Mat33;
                typedef typename Inherited::ElementMass ElementMass;
                typedef typename Inherited::ElementStiffness ElementStiffness;
                typedef typename Inherited::VecElement VecElement;
                typedef typename Inherited::HexahedronInformation HexahedronInformation;
                typedef typename Inherited::Displacement Displacement;
                typedef typename ElementMass::Real Real;
                typedef Eigen::Matrix<Real, ElementMass::Line::spatial_dimensions, ElementMass::Col::spatial_dimensions> EigenElementMass;
                typedef typename Eigen::SVD< EigenElementMass>::SingularValuesType SingularValuesType;

                NonUniformHexahedralFEMForceFieldAndMassCorrected();

                class MatrixConditionModify : public Eigen::SVD< EigenElementMass >
                {
                public:
                    explicit MatrixConditionModify( const EigenElementMass& m):Eigen::SVD<EigenElementMass>( m ){}

                    void lower_condition( Eigen::Map<EigenElementMass>& m , Real threshold, SingularValuesType& sigma );
                };

                Data< Real > maxConditioning;  ///< Limit conditioning number of each mbkMatrix


            protected:
                virtual void computeCorrection( ElementMass& M);  ///< Limit the conditioning number of each mbkMatrix as defined by maxConditioning

            };



        }
    }
}

#endif //SOFA_COMPONENT_FORCEFIELD_NONUNIFORMFEMMASSCORRECTED_H
