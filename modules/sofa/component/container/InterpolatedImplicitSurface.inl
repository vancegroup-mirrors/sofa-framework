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
#ifndef SOFA_COMPONENT_CONTAINER_INTERPOLATEDIMPLICITSURFACE_INL
#define SOFA_COMPONENT_CONTAINER_INTERPOLATEDIMPLICITSURFACE_INL


#include "InterpolatedImplicitSurface.h"


namespace sofa
{
namespace component
{
namespace container
{
using namespace sofa::defaulttype;



void InterpolatedImplicitSurface::updateCache( DomainCache *cache, defaulttype::Vec3d& pos )
{
    cache->insideImg = true;
    for (int d=0; d<3; d++) if (pos[d]<imgMin[d] || pos[d]>=imgMax[d])
        {
            cache->insideImg = false;
            break;
        }
    if (cache->insideImg)
    {
        int voxMinPos[3];
        for (int d=0; d<3; d++)
        {
            voxMinPos[d] = (int)(scale[d] * (pos[d]-imgMin[d]));
            cache->bbMin[d] = spacing[d]*(double)voxMinPos[d] + imgMin[d];
            cache->bbMax[d] = cache->bbMin[d] + spacing[d];
        }
        unsigned int ofs = voxMinPos[0] + imgSize[0]*(voxMinPos[1] + imgSize[1]*voxMinPos[2]);
        cache->val[0] = imgData[ofs];
        for (int i=1; i<8; i++) cache->val[i] = imgData[ofs+deltaOfs[i]];
    }
    else
    {
        // init bounding box to be as large as possible to prevent unnecessary cache updates while outside image
        const double MIN=-10e6, MAX=10e6;
        int voxMappedPos[3];
        for (int d=0; d<3; d++)
        {
            if (pos[d] < imgMin[d])
            {
                cache->bbMin[d] = MIN;
                cache->bbMax[d] = imgMin[d];
                voxMappedPos[d] = 0;
            }
            else if (pos[d] >= imgMax[d])
            {
                cache->bbMin[d] = imgMax[d];
                cache->bbMax[d] = MAX;
                voxMappedPos[d] = imgSize[d]-1;
            }
            else
            {
                cache->bbMin[d] = MIN;
                cache->bbMax[d] = MAX;
                voxMappedPos[d] = (int)(scale[d] * (pos[d]-imgMin[d]));
            }
        }
        unsigned int ofs = voxMappedPos[0] + imgSize[0]*(voxMappedPos[1] + imgSize[1]*voxMappedPos[2]);
        // if cache lies outside image, the returned distance is not updated anymore, instead this boundary value is returned
        cache->val[0] = imgData[ofs] + spacing[0]+spacing[1]+spacing[2];
    }
}


int InterpolatedImplicitSurface::getNextDomain()
{
    // while we have free domains always return the next one, afterwards always use the last one
    if (usedDomains < domainCache.size()) usedDomains++;
    return usedDomains-1;
}


double InterpolatedImplicitSurface::getValue( defaulttype::Vec3d& pos, int& domain )
{
    // find cache domain and check if it needs an update
    DomainCache *cache;
    if (domain < 0)
    {
        domain = getNextDomain();
        cache = &(domainCache[domain]);
        updateCache( cache, pos );
    }
    else
    {
        cache = &(domainCache[domain]);
        for (int d=0; d<3; d++)
        {
            if (pos[d]<cache->bbMin[d] || pos[d]>cache->bbMax[d])
            {
                updateCache( cache, pos );
                break;
            }
        }
    }

    // if cache lies outside image, the returned distance is not updated anymore, instead this boundary value is returned
    if (!cache->insideImg) return cache->val[0];

    // use trilinear interpolation on cached cube
    double weight[3];
    for (int d=0; d<3; d++)
    {
        weight[d] = scale[d] * (pos[d]-cache->bbMin[d]);
    }
    double d = weight[0]*weight[1];
    double c = weight[1] - d;
    double b = weight[0] - d;
    double a = (1.0-weight[1]) - b;
    double res = ( cache->val[0]*a + cache->val[1]*b + cache->val[2]*c + cache->val[3]*d ) * (1.0-weight[2])
            + ( cache->val[4]*a + cache->val[5]*b + cache->val[6]*c + cache->val[7]*d ) * weight[2];

    return res;
}


double InterpolatedImplicitSurface::getValue( defaulttype::Vec3d& pos )
{
    static int domain=-1;
    return getValue( pos, domain );
}


} // namespace container
} // namespace component
} // namespace sofa

#endif

/*
  double a = wMin[0]*wMin[1];
  double b = wMax[0]*wMin[1];
  double c = wMin[0]*wMax[1];
  double d = wMax[0]*wMax[1];
  double res = ( val[0]*a + val[1]*b + val[2]*c + val[3]*d ) * wMin[2]
             + ( val[4]*a + val[5]*b + val[6]*c + val[7]*d ) * wMax[2];

*/
