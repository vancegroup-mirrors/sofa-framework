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
#include <sofa/component/collision/PotentialInjectionPerformer.h>
#include <sofa/component/container/MechanicalObject.h>
#include <sofa/component/topology/TriangleSetTopologyContainer.h>
#include <sofa/component/collision/TriangleModel.h>
#include <sofa/defaulttype/Vec3Types.h>

#include <sofa/helper/Factory.inl>

namespace sofa
{

  namespace component
  {

    namespace collision
    {
      helper::Creator<InteractionPerformer::InteractionPerformerFactory, PotentialInjectionPerformer>  PotentialInjectionPerformerClass("SetActionPotential"); 
      
      void PotentialInjectionPerformer::start()
      {
         BodyPicked picked=this->interactor->getBodyPicked();
         TriangleModel* CollisionModel = dynamic_cast< TriangleModel* >(picked.body);
	
         if (picked.body == NULL || CollisionModel == NULL)
         {
            this->interactor->serr << "Error: PotentialInjectionPerformer no picked body;" << this->interactor->sendl;
            return;
         }

         // Get mechanical Object (3D positions)
         sofa::component::container::MechanicalObject<defaulttype::Vec3dTypes>* MechanicalObject=NULL;
         CollisionModel->getContext()->get (MechanicalObject,  sofa::core::objectmodel::BaseContext::SearchRoot);

         if (MechanicalObject == NULL)
         {
            this->interactor->serr << "Error, can't find mechanicalObject" << this->interactor->sendl;
            return;
         }

         // Get potential container
         sofa::core::objectmodel::Tag mytag (stateTag);
         sofa::component::container::MechanicalObject<defaulttype::Vec1dTypes>* Potential1D;
         sofa::component::container::MechanicalObject<defaulttype::Vec2dTypes>* Potential2D;

         CollisionModel->getContext()->get (Potential1D, mytag, sofa::core::objectmodel::BaseContext::SearchRoot);
         CollisionModel->getContext()->get (Potential2D, mytag, sofa::core::objectmodel::BaseContext::SearchRoot);

         // Hack, potential is either 1D or 2D, no possibility to guess
         if (Potential1D == NULL && Potential2D == NULL)
         {
            this->interactor->serr << "Error, can't find potentialObject" << this->interactor->sendl;
            return;
         }

         if (Potential2D)
            isScalar = false;


         // Get triangle topology - TODO: allow others topologies
         sofa::component::topology::TriangleSetTopologyContainer* triangleContainer;
         CollisionModel->getContext()->get (triangleContainer);

         const component::topology::Triangle pickedTriangle = triangleContainer->getTriangle(picked.indexCollisionElement);

         sofa::helper::vector<sofa::defaulttype::Vector3 > listCoords;
         sofa::defaulttype::Vector3 the_point = picked.point;

         for (unsigned int i=0; i<3; i++)
         {
            sofa::defaulttype::Vector3& tmp = (*MechanicalObject->getX())[ pickedTriangle[i] ];
            listCoords.push_back (tmp);
         }

         // Find the closest dof to pickedPoint:
         double distance1 = 0.0;
         double distance2 = 0.0;
         double sum = 0.0;

         //case 1;
         for (unsigned int i =0; i<3; i++)
            sum += (listCoords[0][i] - the_point[i])*(listCoords[0][i] - the_point[i]);

         distance1 = sqrt (sum);
         unsigned int cpt = 0;

         for (unsigned int i =1; i<3; i++)
         {
            sum = 0.0;
            for (unsigned int j=0; j<3; j++)
               sum += (listCoords[i][j] - the_point[j])*(listCoords[i][j] - the_point[j]);

            distance2 = sqrt (sum);

            if (distance2 < distance1) // this point is closer
            {
               cpt = i;
               distance1 = distance2;
            }
         }

         indexToChange.push_back (pickedTriangle[cpt]);
      }


      void PotentialInjectionPerformer::execute()
      {
         if (!indexToChange.empty())
         {
            sofa::core::objectmodel::Tag mytag (stateTag);
            BodyPicked picked=this->interactor->getBodyPicked();
            TriangleModel* CollisionModel = dynamic_cast< TriangleModel* >(picked.body);

            // hack for template guess
            if (isScalar)
            {
               sofa::component::container::MechanicalObject<defaulttype::Vec1dTypes>* Potential1D;
               CollisionModel->getContext()->get (Potential1D, mytag, sofa::core::objectmodel::BaseContext::SearchRoot);

               if (Potential1D == NULL)
                  return;

               for (unsigned int i=0; i<indexToChange.size(); i++)
                 (*Potential1D->getX())[ indexToChange[i] ][0] = potentialValue;
            }
            else
            {
               sofa::component::container::MechanicalObject<defaulttype::Vec2dTypes>* Potential2D;
               CollisionModel->getContext()->get (Potential2D, mytag, sofa::core::objectmodel::BaseContext::SearchRoot);

               if (Potential2D == NULL)
                  return;

               for (unsigned int i=0; i<indexToChange.size(); i++)
                 (*Potential2D->getX())[ indexToChange[i] ][0] = potentialValue;
            }

            indexToChange.clear();
         }
      }
      
    }
  }
}

