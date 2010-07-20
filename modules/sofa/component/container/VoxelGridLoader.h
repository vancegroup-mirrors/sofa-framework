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
#ifndef SOFA_COMPONENT_VOXELGRIDLOADER_H
#define SOFA_COMPONENT_VOXELGRIDLOADER_H

#include <sofa/component/container/MeshLoader.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/component/component.h>
#include <sofa/core/objectmodel/DataFileName.h>

namespace sofa
{
    namespace helper{ namespace io { class Image; }}

    namespace component
    {

        namespace container
        {


            class SOFA_COMPONENT_CONTAINER_API VoxelGridLoader : public MeshLoader
            {
            public:
                SOFA_CLASS(VoxelGridLoader,MeshLoader);

                typedef defaulttype::Vec<3, int> Vec3i;
                typedef defaulttype::Vec<6, int> Vec6i;

                VoxelGridLoader();
                virtual ~VoxelGridLoader();

                virtual void init();

		virtual void reinit();

                virtual void clear();

                virtual bool load ( const char* filename );

                void setVoxelSize ( const defaulttype::Vector3 vSize );
                void getVoxelSize ( defaulttype::Vector3& vSize ) const;

                void addBackgroundValue ( const int value );
                int getBackgroundValue( const unsigned int idx = 0) const;

		void addActiveDataValue(const int value);
		int getActiveDataValue(const unsigned int idx = 0) const;

                void getResolution ( Vec3i& res ) const;

		int getDataSize() const;

                unsigned char * getData();
                const unsigned char * getData() const;
		
                unsigned char * getSegmentID();
                const unsigned char * getSegmentID() const;

                Vec6i getROI() const;

		// fill the texture by 'image' only where there is the 'segmentation' of 'activeValue' and give the 3D texture sizes
		void createSegmentation3DTexture( unsigned char **textureData, int& width, int& height, int& depth); 

		void getIndicesInRegularGrid(helper::vector<unsigned int>& idxInRegularGrid) const;

            private:
                void setResolution ( const Vec3i res );

		bool isActive(const unsigned int idx) const;

                helper::io::Image* loadImage ( const std::string& filename, const Vec3i& res, const int hsize) const;

            protected:
                Data< defaulttype::Vector3 > voxelSize;
                Data< Vec3i > dataResolution;
                Data< Vec6i > roi;
                Data< int > headerSize;
                sofa::core::objectmodel::DataFileName segmentationFile;
                Data< int > segmentationHeaderSize;

                Data< helper::vector<int> > backgroundValue;
                Data< helper::vector<int> > activeValue;

                Data<bool> generateHexa;

                helper::vector<unsigned int> _idxInRegularGrid;

                helper::io::Image* image;
                helper::io::Image* segmentation;

                int bpp;
            };

        }

    } // namespace component

} // namespace sofa

#endif
