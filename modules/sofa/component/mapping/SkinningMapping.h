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
#ifndef SOFA_COMPONENT_MAPPING_SKINNINGMAPPING_H
#define SOFA_COMPONENT_MAPPING_SKINNINGMAPPING_H

#include <sofa/core/behavior/MechanicalMapping.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/behavior/MappedModel.h>

#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/VecTypes.h>

#include <sofa/helper/SVector.h>

#include <vector>

#include <sofa/component/component.h>
#include <sofa/helper/OptionsGroup.h>


namespace sofa
{

namespace component
{

namespace mapping
{

using sofa::helper::vector;
using sofa::helper::Quater;
using sofa::helper::SVector;

#define DISTANCE_EUCLIDIAN 0
#define DISTANCE_GEODESIC 1
#define DISTANCE_HARMONIC 2

#define WEIGHT_NONE 0
#define WEIGHT_INVDIST_SQUARE 1
#define WEIGHT_LINEAR 2
#define WEIGHT_HERMITE 3
#define WEIGHT_SPLINE 4





template <class BasicMapping>
class SkinningMapping : public BasicMapping
{
public:
    SOFA_CLASS ( SOFA_TEMPLATE ( SkinningMapping,BasicMapping ), BasicMapping );
          typedef BasicMapping Inherit;
          typedef typename Inherit::In In;
          typedef typename Inherit::Out Out;
          typedef typename Out::DataTypes DataTypes;
          typedef typename Out::VecCoord VecCoord;
          typedef typename Out::VecDeriv VecDeriv;
          typedef typename Out::Coord Coord;
          typedef typename Out::Deriv Deriv;
          typedef typename defaulttype::SparseConstraint<Deriv> OutSparseConstraint;
          typedef typename OutSparseConstraint::const_data_iterator OutConstraintIterator;

          typedef typename In::Coord InCoord;
          typedef typename In::Deriv InDeriv;
          typedef typename In::VecCoord VecInCoord;
          typedef typename In::Real InReal;
          typedef typename Out::Real Real;
          enum { N=DataTypes::spatial_dimensions };
          enum { InDerivDim=In::DataTypes::deriv_total_size };
          typedef defaulttype::Mat<N,N,Real> Mat;
          typedef defaulttype::Mat<3,3,Real> Mat33;
          typedef defaulttype::Mat<3,InDerivDim,Real> Mat3xIn;
          typedef vector<Mat3xIn> VMat3xIn;
          typedef vector<VMat3xIn> VVMat3xIn;
          typedef defaulttype::Mat<3,6,Real> Mat36;
          typedef vector<Mat36> VMat36;
          typedef vector<VMat36> VVMat36;
          typedef defaulttype::Mat<3,7,Real> Mat37;
          typedef defaulttype::Mat<3,8,Real> Mat38;
          typedef defaulttype::Mat<4,3,Real> Mat43;
          typedef vector<Mat43> VMat43;
          typedef defaulttype::Mat<4,4,Real> Mat44;
          typedef defaulttype::Mat<6,3,Real> Mat63;
          typedef defaulttype::Mat<6,6,Real> Mat66;
          typedef vector<Mat66> VMat66;
          typedef vector<VMat66> VVMat66;
          typedef defaulttype::Mat<6,7,Real> Mat67;
          typedef defaulttype::Mat<6,InDerivDim,Real> Mat6xIn;
          typedef defaulttype::Mat<7,6,Real> Mat76;
          typedef vector<Mat76> VMat76;
          typedef defaulttype::Mat<8,3,Real> Mat83;
          typedef defaulttype::Mat<8,6,Real> Mat86;
          typedef vector<Mat86> VMat86;
          typedef defaulttype::Mat<8,8,Real> Mat88;
          typedef vector<Mat88> VMat88;
          typedef defaulttype::Mat<12,3,Real> Mat12x3;

          typedef defaulttype::Vec<3,Real> Vec3;
          typedef vector<Vec3> VVec3;
          typedef vector<VVec3> VVVec3;
          typedef defaulttype::Vec<4,Real> Vec4;
          typedef defaulttype::Vec<6,Real> Vec6;
          typedef vector<Vec6> VVec6;
          typedef vector<VVec6> VVVec6;
          typedef defaulttype::Vec<8,Real> Vec8;
          typedef defaulttype::Vec<12,Real> Vec12;
          typedef Quater<InReal> Quat;
          typedef sofa::helper::vector< VecCoord > VecVecCoord;
          typedef SVector<double> VD;
          typedef SVector<SVector<double> > VVD;

          typedef Coord GeoCoord;
          typedef VecCoord GeoVecCoord;
        protected:
          vector<Coord> initPos; // pos: point coord in the local reference frame of In[i].
          vector<Coord> rotatedPoints;

          helper::ParticleMask* maskFrom;
          helper::ParticleMask* maskTo;

          Data<vector<int> > repartition;
          Data<VVD > coefs;
          Data<SVector<SVector<GeoCoord> > > weightGradients;
          Data<unsigned int> nbRefs;
        public:
          Data<bool> showBlendedFrame;
          Data<bool> showDefTensors;
          Data<bool> showDefTensorsValues;
          Data<double> showDefTensorScale;
          Data<unsigned int> showFromIndex;
          Data<bool> showDistancesValues;
          Data<bool> showCoefs;
          Data<double> showGammaCorrection;
          Data<bool> showCoefsValues;
          Data<bool> showReps;
          Data<int> showValuesNbDecimals;
          Data<double> showTextScaleFactor;
          Data<bool> showGradients;
          Data<bool> showGradientsValues;
          Data<double> showGradientsScaleFactor;

        protected:
          Data<sofa::helper::OptionsGroup> wheightingType;
          Data<sofa::helper::OptionsGroup> distanceType;
          bool computeWeights;
          VVD distances;
          vector<vector<GeoCoord> > distGradients;

          inline void computeInitPos();
          inline void computeDistances();
          inline void sortReferences( vector<int>& references);
          inline void normalizeWeights();

        public:
          SkinningMapping ( In* from, Out* to );
          virtual ~SkinningMapping();

          void init();

          void apply ( typename Out::VecCoord& out, const typename In::VecCoord& in );
          void applyJ ( typename Out::VecDeriv& out, const typename In::VecDeriv& in );
          void applyJT ( typename In::VecDeriv& out, const typename Out::VecDeriv& in );
          void applyJT ( typename In::MatrixDeriv& out, const typename Out::MatrixDeriv& in );

          void draw();
          void clear();

          // Weights
          void setWeightsToHermite();
          void setWeightsToInvDist();
          void setWeightsToLinear();
          inline void updateWeights();
          inline void getDistances( int xfromBegin);

          // Accessors
          void setNbRefs ( unsigned int nb )
          {
            nbRefs.setValue ( nb );
          }
          void setWeightCoefs ( VVD& weights );
          void setRepartition ( vector<int> &rep );
          void setComputeWeights ( bool val )
          {
            computeWeights=val;
          }
          unsigned int getNbRefs()
          {
            return nbRefs.getValue();
          }
          const VVD& getWeightCoefs()
          {
            return coefs.getValue();
          }
          const vector<int>& getRepartition()
          {
            return repartition.getValue();
          }
          bool getComputeWeights()
          {
            return computeWeights;
          }

};

      using core::Mapping;
      using core::behavior::MechanicalMapping;
      using core::behavior::MappedModel;
      using core::behavior::State;
      using core::behavior::MechanicalState;

      using sofa::defaulttype::Vec2dTypes;
      using sofa::defaulttype::Vec3dTypes;
      using sofa::defaulttype::Vec2fTypes;
      using sofa::defaulttype::Vec3fTypes;
      using sofa::defaulttype::ExtVec2fTypes;
      using sofa::defaulttype::ExtVec3fTypes;
      using sofa::defaulttype::Rigid2dTypes;
      using sofa::defaulttype::Rigid3dTypes;
      using sofa::defaulttype::Rigid2fTypes;
      using sofa::defaulttype::Rigid3fTypes;


#if defined(WIN32) && !defined(SOFA_COMPONENT_MAPPING_SKINNINGMAPPING_CPP)
#pragma warning(disable : 4231)
#ifndef SOFA_FLOAT
      extern template class SOFA_COMPONENT_MAPPING_API SkinningMapping< MechanicalMapping< MechanicalState<Rigid3dTypes>, MechanicalState<Vec3dTypes> > >;
      extern template class SOFA_COMPONENT_MAPPING_API SkinningMapping< Mapping< State<Rigid3dTypes>, MappedModel<Vec3dTypes> > >;
// template class SOFA_COMPONENT_MAPPING_API SkinningMapping< Mapping< State<Rigid3dTypes>, MappedModel<ExtVec3dTypes> > >;
      extern template class SOFA_COMPONENT_MAPPING_API SkinningMapping< Mapping< State<Rigid3dTypes>, MappedModel<ExtVec3fTypes> > >;
#endif
#ifndef SOFA_DOUBLE
      extern template class SOFA_COMPONENT_MAPPING_API SkinningMapping< MechanicalMapping< MechanicalState<Rigid3fTypes>, MechanicalState<Vec3fTypes> > >;
      extern template class SOFA_COMPONENT_MAPPING_API SkinningMapping< Mapping< State<Rigid3fTypes>, MappedModel<Vec3fTypes> > >;
// template class SOFA_COMPONENT_MAPPING_API SkinningMapping< Mapping< State<Rigid3fTypes>, MappedModel<ExtVec3dTypes> > >;
      extern template class SOFA_COMPONENT_MAPPING_API SkinningMapping< Mapping< State<Rigid3fTypes>, MappedModel<ExtVec3fTypes> > >;
#endif
#ifndef SOFA_FLOAT
#ifndef SOFA_DOUBLE
      extern template class SOFA_COMPONENT_MAPPING_API SkinningMapping< MechanicalMapping< MechanicalState<Rigid3dTypes>, MechanicalState<Vec3fTypes> > >;
      extern template class SOFA_COMPONENT_MAPPING_API SkinningMapping< MechanicalMapping< MechanicalState<Rigid3fTypes>, MechanicalState<Vec3dTypes> > >;
      extern template class SOFA_COMPONENT_MAPPING_API SkinningMapping< Mapping< State<Rigid3dTypes>, MappedModel<Vec3fTypes> > >;
      extern template class SOFA_COMPONENT_MAPPING_API SkinningMapping< Mapping< State<Rigid3fTypes>, MappedModel<Vec3dTypes> > >;
#endif
#endif
#endif




    } // namespace mapping

  } // namespace component

} // namespace sofa

#endif
