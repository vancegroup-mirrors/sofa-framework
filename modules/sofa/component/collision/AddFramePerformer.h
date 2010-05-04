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
#ifndef SOFA_COMPONENT_COLLISION_ADDFRAMEPERFORMER_H
#define SOFA_COMPONENT_COLLISION_ADDFRAMEPERFORMER_H

#include <sofa/component/collision/InteractionPerformer.h>
#include <sofa/defaulttype/Vec3Types.h>


namespace sofa
{

  namespace component
  {

    namespace collision
    {
     
      template <class DataTypes>
        class AddFramePerformer: public TInteractionPerformer<DataTypes>
      {          
      public:
        AddFramePerformer(BaseMouseInteractor *i);
        ~AddFramePerformer();

        void start();
        void execute();

      };
   


#if defined(WIN32) && !defined(SOFA_COMPONENT_COLLISION_ADDFRAMEPERFORMER_CPP)
extern template class SOFA_COMPONENT_COLLISION_API AddFramePerformer<defaulttype::Vec3Types>;
#endif


    }
  }
}

#endif
