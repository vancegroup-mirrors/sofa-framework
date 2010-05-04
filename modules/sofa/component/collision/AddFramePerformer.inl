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

#include <sofa/component/collision/AddFramePerformer.h>
#include <sofa/component/collision/MouseInteractor.h>
#include <sofa/component/mapping/SkinningMapping.inl>
#include <sofa/helper/Quater.h>

namespace sofa
{

  namespace component
  {

    namespace collision
    {

      template <class DataTypes>
      void AddFramePerformer<DataTypes>::start()
      {
        BodyPicked picked=this->interactor->getBodyPicked();
        if (!picked.body && !picked.mstate) return;

        sofa::component::mapping::SkinningMapping<sofa::component::mapping::MechanicalMapping< core::componentmodel::behavior::MechanicalState<StdRigidTypes<3, typename DataTypes::Real> >, core::componentmodel::behavior::MechanicalState<DataTypes> > >* sMapping=NULL;

        typename DataTypes::Coord point;

        if (picked.body)
          {
            point = picked.point;
            picked.body->getContext()->get(sMapping);
          }
        else
          {         
            core::componentmodel::behavior::MechanicalState<DataTypes>* mstateCollision=dynamic_cast< core::componentmodel::behavior::MechanicalState<DataTypes>*  >(picked.mstate);            
            if (!mstateCollision)
              {
                this->interactor->serr << "uncompatible MState during Mouse Interaction " << this->interactor->sendl;
                return;
              }            
            mstateCollision->getContext()->get( sMapping);
            int index = picked.indexCollisionElement;
            point=(*(mstateCollision->getX()))[index];
          }

        if( !(sMapping && sMapping->computeAllMatrices.getValue()) ) return;   
        sMapping->insertFrame( point, helper::Quater<double>(0, 0, 0, 1) );
      }

      template <class DataTypes>
      void AddFramePerformer<DataTypes>::execute()
      {
      };

      template <class DataTypes>
      AddFramePerformer<DataTypes>::AddFramePerformer(BaseMouseInteractor *i):TInteractionPerformer<DataTypes>(i)
      {
      }


      template <class DataTypes>
      AddFramePerformer<DataTypes>::~AddFramePerformer()
      {
          //Should remove the frames added
      };


    }
  }
}
