/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#include <sofa/component/topology/FittedRegularGridTopology.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/io/Mesh.h>

namespace sofa
{

namespace component
{

namespace topology
{

using namespace sofa::defaulttype;

SOFA_DECL_CLASS(FittedRegularGridTopology)

int FittedRegularGridTopologyClass = core::RegisterObject("Regular grid in 3D where the empty cells are identified and not used in some computations")
.addAlias("TrimmedRegularGrid")
.addAlias("FittedRegularGrid")
.add< FittedRegularGridTopology >()
;

FittedRegularGridTopology::FittedRegularGridTopology(int nx, int ny, int nz)
: RegularGridTopology(nx, ny, nz), validActiveCubeFlags(false)
{
}

void FittedRegularGridTopology::setSize()
{
    this->RegularGridTopology::setSize();
    int nc = getNbCubes();
    activeCubeFlag.resize(nc);
    //for (int c=0;c<nc;c++)
    //    activeCubeFlag[c]= false;
}

FittedRegularGridTopology::FittedRegularGridTopology()
: validActiveCubeFlags(false)
{
}

bool FittedRegularGridTopology::load(const char* filename)
{
    this->filename = filename;
    return true;
}

void FittedRegularGridTopology::init()
{
    this->RegularGridTopology::init();
    if (!filename.empty())
    {
        std::cout << "FittedRegularGridTopology: using mesh "<<filename<<std::endl;
        helper::io::Mesh* mesh = helper::io::Mesh::Create(filename.c_str());
        if (mesh)
        {
            const helper::vector<Vec3>& vertices = mesh->getVertices();
            const helper::vector< helper::vector < helper::vector <int> > >& facets = mesh->getFacets();
            for (unsigned int i=0;i<vertices.size();i++)
                findNearestCube(vertices[i]); // this will activate the underlying cube
            // recursively split large triangles
            for (unsigned int i=0;i<facets.size();i++)
            {
                const helper::vector<int>& facet = facets[i][0];
                // Triangularize
                for (unsigned int j=2; j<facet.size(); j++)
                    addTriangle(vertices[facet[0]],vertices[facet[j-1]],vertices[facet[j]]);
            }
            delete mesh;
            updateActiveCubeFlags();
        }
        else
            std::cerr << "FittedRegularGridTopology: loading mesh "<<filename<<" failed."<<std::endl;
    }
}

void FittedRegularGridTopology::addTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3)
{
    addTriangle(p1,findNearestCube(p1),p2,findNearestCube(p2),p3,findNearestCube(p3));
}

void FittedRegularGridTopology::addTriangle(const Vec3& p1, int c1, const Vec3& p2, int c2, const Vec3& p3, int c3, int depth)
{
    if (++depth > 10) return;
    const int cnx = getNx()-1;
    const int cnxcny = cnx*(getNy()-1);
    if (c1==c2 && c1==c3) return; // all in the same cube
    if (  (c2==c1 || c2==c1+1 || c2 == c1-1 || c2 == c1+cnx || c2 == c1-cnx || c2 == c1+cnxcny || c2 == c1-cnxcny)
       && (c3==c1 || c3==c1+1 || c3 == c1-1 || c3 == c1+cnx || c3 == c1-cnx || c3 == c1+cnxcny || c3 == c1-cnxcny)
       && (c2==c3 || c2==c3+1 || c2 == c3-1 || c2 == c3+cnx || c2 == c3-cnx || c2 == c3+cnxcny || c2 == c3-cnxcny))
       return; // all edges are between neightbors
    // else subdivide
    Vec3 p12 = (p1+p2)*0.5; int c12 = findNearestCube(p12);
    Vec3 p13 = (p1+p3)*0.5; int c13 = findNearestCube(p13);
    Vec3 p23 = (p2+p3)*0.5; int c23 = findNearestCube(p23);
    addTriangle(p1,c1,p12,c12,p13,c13,depth);
    addTriangle(p2,c2,p12,c12,p23,c23,depth);
    addTriangle(p3,c3,p13,c13,p23,c23,depth);
    addTriangle(p12,c12,p13,c13,p23,c23,depth);
}

/// return the cube containing the given point (or -1 if not found).
int FittedRegularGridTopology::findCube(const Vec3& pos)
{
	int c = this->RegularGridTopology::findCube(pos);
	setCubeActive(c);
	return c;
}

/// return the nearest cube (or -1 if not found).
int FittedRegularGridTopology::findNearestCube(const Vec3& pos)
{
	int c = this->RegularGridTopology::findNearestCube(pos);
	setCubeActive(c);
	return c;
}

/// return the cube containing the given point (or -1 if not found),
/// as well as deplacements from its first corner in terms of dx, dy, dz (i.e. barycentric coordinates).
int FittedRegularGridTopology::findCube(const Vec3& pos, double& fx, double &fy, double &fz)
{
	int c = this->RegularGridTopology::findCube(pos, fx, fy, fz);
	setCubeActive(c);
	return c;
}

/// return the cube containing the given point (or -1 if not found),
/// as well as deplacements from its first corner in terms of dx, dy, dz (i.e. barycentric coordinates).
int FittedRegularGridTopology::findNearestCube(const Vec3& pos, double& fx, double &fy, double &fz)
{
	int c = this->RegularGridTopology::findNearestCube(pos, fx, fy, fz);
	setCubeActive(c);
	return c;
}

/// set a given cube as active
void FittedRegularGridTopology::setCubeActive(int c)
{
	if (c>=0)
	{
		if (!activeCubeFlag[c])
		{
			activeCubeFlag[c] = true;
			validActiveCubeFlags = false; // Note that we could detect where an update is really needed
		}
	}
}

/// set a given cube as active
void FittedRegularGridTopology::setCubeActive(int x, int y, int z)
{
  if ((unsigned)x>=(unsigned)(n.getValue()[0]-1) || (unsigned)y>=(unsigned)(n.getValue()[1]-1) || (unsigned)z>=(unsigned)(n.getValue()[2]-1))
		return;
	setCubeActive(cube(x,y,z));
}


/// return true if the given cube is active, i.e. it contains or is surrounded by mapped points.
bool FittedRegularGridTopology::isCubeActive(int index)
{
	if (!validActiveCubeFlags)
		updateActiveCubeFlags();
	if (index < 0 || index >= getNbCubes()) return false;
	return activeCubeFlag[index];
}

/// return true if the given cube is active, i.e. it contains or is surrounded by mapped points.
bool FittedRegularGridTopology::isCubeActive(int x, int y, int z)
{
	if ((unsigned)x>=(unsigned)(n.getValue()[0]-1) || (unsigned)y>=(unsigned)(n.getValue()[1]-1) || (unsigned)z>=(unsigned)(n.getValue()[2]-1))
		return false;
	else
		return isCubeActive(cube(x,y,z));
}

/// Update activeCubeFlags.
void FittedRegularGridTopology::updateActiveCubeFlags()
{
	const int nc = getNbCubes();
	const int cnx = n.getValue()[0]-1;
	const int cny = n.getValue()[1]-1;
	const int cnz = n.getValue()[2]-1;
	sofa::helper::vector<int> visited;
	visited.reserve(nc);
	// First add all inactive border cubes
	int x,y,z,c;

	z = 0;
	{
		for (y = 0; y < cny; ++y)
		{
			for (x = 0; x < cnx; ++x)
			{
				c = cube(x,y,z);
				if (!activeCubeFlag[c])
				{
					visited.push_back(c);
					activeCubeFlag[c] = true;
				}
			}
		}
	}
	for (z = 1; z < cnz-1; ++z)
	{
		y = 0;
		{
			for (x = 0; x < cnx; ++x)
			{
				c = cube(x,y,z);
				if (!activeCubeFlag[c])
				{
					visited.push_back(c);
					activeCubeFlag[c] = true;
				}
			}
		}
		for (y = 1; y < cny-1; ++y)
		{
			x = 0;
			{
				c = cube(x,y,z);
				if (!activeCubeFlag[c])
				{
					visited.push_back(c);
					activeCubeFlag[c] = true;
				}
			}
			x = cnx-1;
			{
				c = cube(x,y,z);
				if (!activeCubeFlag[c])
				{
					visited.push_back(c);
					activeCubeFlag[c] = true;
				}
			}
		}
		y = cny-1;
		{
			for (x = 0; x < cnx; ++x)
			{
				c = cube(x,y,z);
				if (!activeCubeFlag[c])
				{
					visited.push_back(c);
					activeCubeFlag[c] = true;
				}
			}
		}
	}
	z = cnz-1;
	{
		for (y = 0; y < cny; ++y)
		{
			for (x = 0; x < cnx; ++x)
			{
				c = cube(x,y,z);
				if (!activeCubeFlag[c])
				{
					visited.push_back(c);
					activeCubeFlag[c] = true;
				}
			}
		}
	}
	
	// Then add all not-active neighbors or visited cells
	
	unsigned int current = 0;
	int neighbors[6] = { -1, 1, -cnx, cnx, -cnx*cny, cnx*cny };
	while (current < visited.size())
	{
		int c = visited[current];
		++current;
		for (int b = 0; b<6; ++b)
		{
			int c2 = c+neighbors[b];
			if (c2 >= 0 && c2 < nc)
			{
				if (!activeCubeFlag[c2])
				{
					visited.push_back(c2);
					activeCubeFlag[c2] = true;
				}
			}
		}
	}

	// Finally set all non-visited cells to active, and visited cells to inactive
	for (int c=0; c<nc; ++c)
		activeCubeFlag[c] = true;
	for (current = 0; current < visited.size(); ++current)
		activeCubeFlag[visited[current]] = false;
	
	validActiveCubeFlags = true;
        std::cout << "FittedRegularGridTopology: "<<(nc-visited.size())<<"/"<<nc<<" active cells, "<<(((nc-visited.size())*1000)/nc)*0.1<<"% occupancy."<<std::endl;
}

} // namespace topology

} // namespace component

} // namespace sofa

