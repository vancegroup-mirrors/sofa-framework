#include <sofa/component/collision/DistanceOctreeCollisionModel.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/component/collision/CubeModel.h>
#include <sofa/helper/system/gl.h>

namespace sofa
{

namespace component
{

namespace collision
{

SOFA_DECL_CLASS(DistanceOctreeCollisionModel)

int DistanceOctreeCollisionModelClass = core::RegisterObject("Octree-based distance field, based on SLC code ( http://sourceforge.net/projects/bfast/ )")
.add< DistanceOctreeCollisionModel >()
.addAlias("DistanceOctree")
;

using namespace defaulttype;

void DistanceOctreeCollisionModel::parse(core::objectmodel::BaseObjectDescription* arg)
{
    this->CollisionModel::parse(arg);
    setFilename(arg->getAttribute("filename",""));
    dumpfilename = arg->getAttribute("dumpfilename","");
    setTranslation(Vec3d(atof(arg->getAttribute("dx","0")),atof(arg->getAttribute("dy","0")),atof(arg->getAttribute("dz","0"))));
    setScale(atof(arg->getAttribute("scale","1")));
    setBorder(atof(arg->getAttribute("border","0.5")));
    setDepth(atoi(arg->getAttribute("depth","4")));
    marchingcube = (atoi(arg->getAttribute("marchingcube","0"))!=0);
}

void DistanceOctreeCollisionModel::draw(int index)
{
    SlcSurface* surface = getSurface(index);
	glEnable(GL_BLEND);
	glDepthMask(0);
	glBegin(GL_LINES);
	{
		const DtCellList& cells = surface->tree->cells;
		const BfastVecList& pts = surface->tree->pts;
		for (unsigned int i=0; i<cells.numCells(); i++)
		{
			const DtCell& c = cells[i];
			const BfastVector3 p0 = c.lc(pts);
			const BfastVector3 p1 = c.uc(pts);
			if (!isSimulated())
				glColor4f(0.25f, 0.25f, 0.25f, 0.1f);
			else
				glColor4f(0.5f, 0.5f, 0.5f, 0.1f);
			glVertex3dv(pts[c.vertices[0]].data); glVertex3dv(pts[c.vertices[4]].data);
			glVertex3dv(pts[c.vertices[1]].data); glVertex3dv(pts[c.vertices[5]].data);
			glVertex3dv(pts[c.vertices[2]].data); glVertex3dv(pts[c.vertices[6]].data);
			glVertex3dv(pts[c.vertices[3]].data); glVertex3dv(pts[c.vertices[7]].data);
			glVertex3dv(pts[c.vertices[0]].data); glVertex3dv(pts[c.vertices[2]].data);
			glVertex3dv(pts[c.vertices[1]].data); glVertex3dv(pts[c.vertices[3]].data);
			glVertex3dv(pts[c.vertices[4]].data); glVertex3dv(pts[c.vertices[6]].data);
			glVertex3dv(pts[c.vertices[5]].data); glVertex3dv(pts[c.vertices[7]].data);
			glVertex3dv(pts[c.vertices[0]].data); glVertex3dv(pts[c.vertices[1]].data);
			glVertex3dv(pts[c.vertices[2]].data); glVertex3dv(pts[c.vertices[3]].data);
			glVertex3dv(pts[c.vertices[4]].data); glVertex3dv(pts[c.vertices[5]].data);
			glVertex3dv(pts[c.vertices[6]].data); glVertex3dv(pts[c.vertices[7]].data);
			/*
			glVertex3d(p0[0],p0[1],p0[2]);
			glVertex3d(p1[0],p0[1],p0[2]);
			glVertex3d(p0[0],p1[1],p0[2]);
			glVertex3d(p1[0],p1[1],p0[2]);
			glVertex3d(p0[0],p0[1],p1[2]);
			glVertex3d(p1[0],p0[1],p1[2]);
			glVertex3d(p0[0],p1[1],p1[2]);
			glVertex3d(p1[0],p1[1],p1[2]);
			glVertex3d(p0[0],p0[1],p0[2]);
			glVertex3d(p0[0],p1[1],p0[2]);
			glVertex3d(p1[0],p0[1],p0[2]);
			glVertex3d(p1[0],p1[1],p0[2]);
			glVertex3d(p0[0],p0[1],p1[2]);
			glVertex3d(p0[0],p1[1],p1[2]);
			glVertex3d(p1[0],p0[1],p1[2]);
			glVertex3d(p1[0],p1[1],p1[2]);
			glVertex3d(p0[0],p0[1],p0[2]);
			glVertex3d(p0[0],p0[1],p1[2]);
			glVertex3d(p1[0],p0[1],p0[2]);
			glVertex3d(p1[0],p0[1],p1[2]);
			glVertex3d(p0[0],p1[1],p0[2]);
			glVertex3d(p0[0],p1[1],p1[2]);
			glVertex3d(p1[0],p1[1],p0[2]);
			glVertex3d(p1[0],p1[1],p1[2]);
			*/
			if (c.child[0] == -1)
			{
				if (!isSimulated())
					glColor3f(0.0, 0.0, 0.5);
				else
					glColor3f(0.0, 0.0, 1.0);
				const BfastVector3 center = (p0+p1)*0.5;
				const BfastVector3 gr = surface->grad(center);
				//const double d = surface->eval(center,true);
				const BfastVector3 p2 = center+gr*((p1[0]-p0[0])*0.5);
				glVertex3dv(center.data);
				glVertex3dv(p2.data);
			}
		}
	}
	glEnd();
	glDisable(GL_BLEND);
	glDepthMask(1);
	glBegin(GL_POINTS);
	{
		const BfastVecList& pts = surface->tree->pts;
		const BfastRealList& dists = surface->tree->phi;
		for (unsigned int i=0; i<pts.numVecs(); i++)
		{
			const BfastVector3& p = pts[i];
			const BfastReal d = dists[i];
			//const BfastVector3 gr = surface->grad(p);
			//const double d = surface->eval(p,true);
			if (d<0)
				glColor3d(1-d*0.25, 0, 1-d);
			else
				glColor3d(0, 1-d*0.25, 1-d);
			glVertex3dv(p.data);
		}
	}
	glEnd();
	glColor4fv(getColor4f());
    if (!filename.empty())
    {
	glBegin(GL_TRIANGLES);
	{
		const BfastVecList& pts = surface->meshPts;
		const BfastTriList& tris = surface->triangles;
		for (unsigned int i=0; i<tris.numTriangles(); i++)
		{
			glVertex3dv(pts[tris[i].a].data);
			glVertex3dv(pts[tris[i].b].data);
			glVertex3dv(pts[tris[i].c].data);
		}
	}
	glEnd();
    }
	if (getContext()->getShowNormals())
	{
		glBegin(GL_LINES);
		{
			const BfastVecList& pts = surface->meshPts;
			const BfastTriList& tris = surface->triangles;
			glColor3f(0,1,1);
			for (unsigned int i=0; i<tris.numTriangles(); i++)
			{
				BfastVector3 c = (pts[tris[i].a]+pts[tris[i].b]+pts[tris[i].c])/3.0;
				BfastVector3 cn = c + surface->faceNormals[i];
				glVertex3dv(c.data);
				glVertex3dv(cn.data);
			}
			glColor3f(1,1,0);
			for (unsigned int i=0; i<pts.numVecs(); i++)
			{
				BfastVector3 c = pts[i];
				BfastVector3 cn = c + surface->psuedoNormals[i];
				glVertex3dv(c.data);
				glVertex3dv(cn.data);
			}
		}
		glEnd();
	}
}

DistanceOctreeCollisionModel::DistanceOctreeCollisionModel()
{
	mstate = NULL;
	rigid = NULL;
	mesh = NULL;
	previous = NULL;
	next = NULL;
	static_ = false;
	translation = Vec3d(0,0,0);
	scale = 1.0;
	border = 0.5;
	depth = 4;
}

DistanceOctreeCollisionModel::~DistanceOctreeCollisionModel() 
{
    for (unsigned int i=0; i<elems.size(); i++)
        if (elems[i]!=NULL) delete elems[i];
}

void DistanceOctreeCollisionModel::init()
{
    std::cout << "> DistanceOctreeCollisionModel::init()"<<std::endl;
    this->core::CollisionModel::init();
	mstate = dynamic_cast< core::componentmodel::behavior::MechanicalState<Vec3Types>* > (getContext()->getMechanicalState());
	rigid = dynamic_cast< core::componentmodel::behavior::MechanicalState<RigidTypes>* > (getContext()->getMechanicalState());
	mesh = dynamic_cast< topology::MeshTopology* > (getContext()->getTopology());

	SlcSurface* octree = NULL;
	if (filename.empty())
	{
		if (mstate==NULL)
		{
			std::cerr << "ERROR: DistanceOctreeCollisionModel requires a Vec3 Mechanical Model or a filename.\n";
			return;
		}
		/*
		if (mstate->getX()->size()!=1)
		{
			std::cerr << "ERROR: DistanceOctreeCollisionModel requires a Vec3 Mechanical Model with 1 element.\n";
			return;
		}
		*/
		if (mesh==NULL)
		{
			std::cerr << "ERROR: DistanceOctreeCollisionModel requires a Mesh Topology or a filename.\n";
			return;
		}
	
		if (mesh->getNbTriangles()==0 || !mesh->hasPos())
		{
			std::cerr << "ERROR: DistanceOctreeCollisionModel requires a Mesh Topology with triangles and vertice positions or a filename.\n";
			return;
		}
		octree = new SlcSurface(new DtTree);
	}
	else
	{
                std::cout << "DistanceOctreeCollisionModel: creating SlcSurface"<<std::endl;
		octree = new SlcSurface(new DtTree);
                std::cout << "DistanceOctreeCollisionModel: loading "<<filename<<std::endl;
		if (filename.length()>4 && filename.substr(filename.length()-4) == ".bvt")
		{
			octree->bvtRead(filename.c_str());
		}
		else
		if (filename.length()>4 && filename.substr(filename.length()-4) == ".obj")
		{
			readObjFile(filename.c_str(), octree->meshPts, octree->triangles);
			octree->builtFromTriangles = true;
		}
		else
		if (filename.length()>4 && filename.substr(filename.length()-4) == ".bdt")
		{
			octree->bdtRead(filename.c_str());
		}
	}
	// Apply translations and rotations if any
	if (translation.norm2() != 0.0 || scale != 1.0)
	{
                std::cout << "DistanceOctreeCollisionModel: applying transforms"<<std::endl;
		const double s = scale;
		const BfastVector3 t ( translation.ptr() );
		unsigned int nbp = octree->meshPts.numVecs();
		for (unsigned int i=0;i<nbp;i++)
		{
			BfastVector3& p = octree->meshPts[i];
			p = p*s + t;
		}
		nbp = octree->tree->pts.numVecs();
		for (unsigned int i=0;i<nbp;i++)
		{
			BfastVector3& p = octree->tree->pts[i];
			p = p*s + t;
			octree->tree->phi[i] *= s;
		}
		octree->tree->lc = octree->tree->lc*s + t;
		octree->tree->uc = octree->tree->uc*s + t;
		//unsigned int nbc = octree->tree->cells.numCells();
		//for (unsigned int i=0;i<nbc;i++)
		//{
		//	DtCell& c = octree->tree->cells[i];
		//	c.lc = c.lc*s + t;
		//	c.uc = c.uc*s + t;
		//	c.dist *= s;
		//}
	}
        std::cout << "DistanceOctreeCollisionModel: resize(1)"<<std::endl;
	resize(1);
	elems[0] = octree;
	if (filename.empty() || octree->tree->cells.numCells()==0)
	{
                std::cout << "DistanceOctreeCollisionModel: updating octree"<<std::endl;
		updateOctree();
	}
	if (marchingcube)
	{
		octree->builtFromTriangles = false;
		octree->meshPts.clear();
		octree->triangles.setNumTriangles(0);
                std::cout << "DistanceOctreeCollisionModel: computing marching cube"<<std::endl;
		octree->contourTree(NULL,NULL);
		octree->computeNormals();
		octree->computePsuedoNormals();
	}
	if (!dumpfilename.empty())
	{
		std::cout << "DistanceOctreeCollisionModel: dump octree to "<<dumpfilename<<std::endl;
		//octree->bdtDump(dumpfilename.c_str());
		if (dumpfilename.length()>4 && dumpfilename.substr(dumpfilename.length()-4) == ".bvt")
		{
			octree->bvtDump(dumpfilename.c_str());
		}
		else
		if (dumpfilename.length()>4 && dumpfilename.substr(dumpfilename.length()-4) == ".obj")
		{
			octree->objDump(dumpfilename.c_str());
		}
		else
		if (dumpfilename.length()>4 && dumpfilename.substr(dumpfilename.length()-4) == ".bdt")
		{
			octree->bdtDump(dumpfilename.c_str());
		}
	}
    std::cout << "< DistanceOctreeCollisionModel::init()"<<std::endl;
}

void DistanceOctreeCollisionModel::resize(int s)
{
    this->core::CollisionModel::resize(s);
    elems.resize(s);
}

SlcSurface* DistanceOctreeCollisionModel::getSurface(int index)
{
    return elems[index];
}

void DistanceOctreeCollisionModel::setSurface(SlcSurface* surf, int index)
{
    elems[index] = surf;
}

void DistanceOctreeCollisionModel::updateOctree()
{
	if (elems.size()<1) return;
	SlcSurface* octree = elems[0];
	if (filename.empty())
	{
		octree->builtFromTriangles = true;
		if (mstate != NULL)
		{
			const Vec3Types::VecCoord& x = *mstate->getX();
			unsigned int nbp = x.size();
			octree->meshPts.setNumVecs(nbp);
			for (unsigned int i=0;i<nbp;i++)
			{
				Vec3d p = x[i];
				octree->meshPts[i][0] = p[0];
				octree->meshPts[i][1] = p[1];
				octree->meshPts[i][2] = p[2];
			}
		}
		else
		{
			unsigned int nbp = mesh->getNbPoints();
			octree->meshPts.setNumVecs(nbp);
			for (unsigned int i=0;i<nbp;i++)
			{
				octree->meshPts[i][0] = mesh->getPX(i);
				octree->meshPts[i][1] = mesh->getPY(i);
				octree->meshPts[i][2] = mesh->getPZ(i);
			}
		}
		const topology::MeshTopology::SeqTriangles& tris = mesh->getTriangles();
		octree->triangles.setNumTriangles(tris.size());
		for (unsigned int i=0;i<tris.size();i++)
		{
			octree->triangles[i].a = tris[i][0];
			octree->triangles[i].b = tris[i][1];
			octree->triangles[i].c = tris[i][2];
		}
	}
	// Compute mesh bounding box
	BfastVector3 bbmin;
	BfastVector3 bbmax;
	unsigned int nbp = octree->meshPts.numVecs();
	for (unsigned int i=0;i<nbp;i++)
	{
		const BfastVector3& p = octree->meshPts[i];
		if (!i || p[0] < bbmin[0]) bbmin[0] = p[0];
		if (!i || p[0] > bbmax[0]) bbmax[0] = p[0];
		if (!i || p[1] < bbmin[1]) bbmin[1] = p[1];
		if (!i || p[1] > bbmax[1]) bbmax[1] = p[1];
		if (!i || p[2] < bbmin[2]) bbmin[2] = p[2];
		if (!i || p[2] > bbmax[2]) bbmax[2] = p[2];
	}
	// Octree's bounding box should be square
	BfastReal width = bbmax[0] - bbmin[0];
	if (bbmax[1] - bbmin[1] > width) width = bbmax[1] - bbmin[1];
	if (bbmax[2] - bbmin[2] > width) width = bbmax[2] - bbmin[2];
	width *= (1+2*border); // Build a bigger octree
	BfastReal xcenter = (bbmin[0] + bbmax[0])/2;
	BfastReal ycenter = (bbmin[1] + bbmax[1])/2;
	BfastReal zcenter = (bbmin[2] + bbmax[2])/2;
	bbmin[0] = xcenter - width/2; bbmax[0] = xcenter + width/2;
	bbmin[1] = ycenter - width/2; bbmax[1] = ycenter + width/2;
	bbmin[2] = zcenter - width/2; bbmax[2] = zcenter + width/2;

	std::cout << "Building Distance Octree with " << octree->meshPts.numVecs() << " vertices and " << octree->triangles.numTriangles() << " triangles, bbox=<"<<bbmin[0]<<','<<bbmin[1]<<','<<bbmin[2]<<">-<"<<bbmax[0]<<','<<bbmax[1]<<','<<bbmax[2]<<">." << std::endl;
	octree->computeNormals();
	octree->computePsuedoNormals();
	delete octree->tree;
	octree->tree = new DtTree;
	buildTree(octree->tree, bbmin, bbmax, depth, &(octree->meshPts), &(octree->triangles), &(octree->faceNormals));
	octree->redistance();
	std::cout << "Octree built: " << octree->tree->cells.numCells() << " cells." << std::endl;
}

void DistanceOctreeCollisionModel::draw()
{
	return;
	if (!isActive() || !getContext()->getShowCollisionModels()) return;

	if (rigid!=NULL)
	{
		glPushMatrix();
		float m[16];
		(*rigid->getX())[0].writeOpenGlMatrix( m );
		glMultMatrixf(m);
	}

	if (getContext()->getShowWireFrame())
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDisable(GL_LIGHTING);
	glColor4fv(getColor4f());
	glPointSize(3);
	for (unsigned int i=0;i<elems.size();i++)
	{
		draw(i);
	}
	if (getContext()->getShowWireFrame())
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	if (getPrevious()!=NULL && dynamic_cast<core::VisualModel*>(getPrevious())!=NULL)
		dynamic_cast<core::VisualModel*>(getPrevious())->draw();

	if (rigid!=NULL)
	{
		glPopMatrix();
	}
}

/// Create or update the bounding volume hierarchy.
void DistanceOctreeCollisionModel::computeBoundingTree(int maxDepth)
{
    CubeModel* cubeModel = this->createPrevious<CubeModel>();
    
    if (!isMoving() && !cubeModel->empty()) return; // No need to recompute BBox if immobile
    
    if (filename.empty())
        updateOctree();
    
    Vector3 minBB, maxBB;
    for (unsigned int i=0; i<elems.size(); i++)
    {
        //static_cast<DistanceOctreeCollisionElement*>(elems[i])->recalcBBox();
        Vector3 emin, emax;
        for (int c = 0; c < 3; c++)
        {
            emin[c] = elems[i]->tree->lc[c];
            emax[c] = elems[i]->tree->uc[c];
        }
        cubeModel->setParentOf(i, emin, emax); // define the bounding box of the current triangle
    }
    cubeModel->computeBoundingTree(maxDepth);
}

} // namespace collision

} // namespace component

} // namespace sofa
