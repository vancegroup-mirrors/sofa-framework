#include "NonUniformHexahedralFEMForceFieldAndMassCorrected.h"
#include <iostream>
using std::cerr;
using std::endl;

namespace sofa{
    namespace component{
        namespace forcefield{


            template <class DataTypes>
                    NonUniformHexahedralFEMForceFieldAndMassCorrected<DataTypes>::NonUniformHexahedralFEMForceFieldAndMassCorrected()
                        :Inherited()
                        ,maxConditioning( initData(&maxConditioning, (Real) 100, "maxConditioning", "limit the ratio maximal/minimal singular value of the MBK matrices to the given threshold") )
            {
            }

            template < class DataTypes >
                    void NonUniformHexahedralFEMForceFieldAndMassCorrected<DataTypes>::computeCorrection(ElementMass &M)
            {

                Eigen::Map<EigenElementMass> matrixMapped = EigenElementMass::Map(M.ptr() );
                MatrixConditionModify svd( matrixMapped );
                SingularValuesType sigma;

                svd.lower_condition( matrixMapped , maxConditioning.getValue(),sigma );
                if( this->f_printLog.getValue() == true ){
                    svd.compute( matrixMapped ); //update the svd with the new matrixMapped value.
                    this->sout << "Corrected matrix:"<< M << this->sendl;
                    this->sout << "Corrected matrix singular values:"<< svd.singularValues() << this->sendl;
                }

            }

            template < class DataTypes >
                    void NonUniformHexahedralFEMForceFieldAndMassCorrected<DataTypes>::MatrixConditionModify::lower_condition
                    ( Eigen::Map<EigenElementMass>& m ,
                    Real threshold, SingularValuesType& sigma )
            {
                assert ( threshold > 0 );
                sigma = this->m_sigma;
                Real maxsigma = sigma[0];
                Real minsigma = sigma[0];
                for ( int i=1; i <sigma.rows() ; ++i )
                {
                    assert( sigma[i]>=0 );
                    if ( sigma[i] > maxsigma ) maxsigma = sigma[i];
                    if ( sigma[i] < minsigma ) minsigma = sigma[i];
                }
//                cerr<<"NonUniformHexahedralFEMForceFieldAndMassCorrected<DataTypes>::MatrixConditionModify::lower_condition, sigma = "<<sigma<<endl;
//                cerr<<"NonUniformHexahedralFEMForceFieldAndMassCorrected<DataTypes>::MatrixConditionModify::lower_condition, maxsigma = "<<maxsigma<<endl;
//                cerr<<"NonUniformHexahedralFEMForceFieldAndMassCorrected<DataTypes>::MatrixConditionModify::lower_condition, minsigma = "<<minsigma<<endl;
//                cerr<<"NonUniformHexahedralFEMForceFieldAndMassCorrected<DataTypes>::MatrixConditionModify::lower_condition, maxsigma/minsigma = "<<maxsigma/minsigma<<endl;
//                cerr<<"NonUniformHexahedralFEMForceFieldAndMassCorrected<DataTypes>::MatrixConditionModify::lower_condition, threshold = "<<threshold<<endl;
                Real minsigmathreshold = maxsigma/threshold;
                for ( int i=0; i <sigma.rows() ; ++i )
                {
                    if ( sigma[i] < minsigmathreshold ) sigma[i] = minsigmathreshold;
                }
                m = this->m_matU * sigma.asDiagonal() * this->m_matV.transpose();
            }



        }
    }
}
