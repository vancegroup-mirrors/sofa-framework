#include <sofa/component/collision/SphereModel.h>
#include <sofa/helper/io/SphereLoader.h>
#include <sofa/component/collision/CubeModel.h>
#include <sofa/core/ObjectFactory.h>
#include <GL/glut.h>


namespace sofa
{

namespace component
{

namespace collision
{

SOFA_DECL_CLASS(Sphere)

int SphereModelClass = core::RegisterObject("TODO")
        .add< SphereModel >()
        .addAlias("Sphere")
        ;

SphereModel::SphereModel(double radius)
    : defaultRadius(dataField(&defaultRadius, radius, "radius","TODO"))
{
}

void SphereModel::resize(int size)
{
    this->component::MechanicalObject<Vec3Types>::resize(size);
    this->core::CollisionModel::resize(size);
    if ((int)radius.size() < size)
    {
        radius.reserve(size);
        while ((int)radius.size() < size)
            radius.push_back(defaultRadius.getValue());
    }
    else
    {
        radius.resize(size);
    }
}

int SphereModel::addSphere(const Vector3& pos, double radius)
{
    int i = size;
    resize(i+1);
    setSphere(i, pos, radius);
    return i;
}

void SphereModel:: setSphere(int i, const Vector3& pos, double r)
{
    if ((unsigned)i >= (unsigned) size) return;
    (*this->getX())[i] = pos;
    radius[i] = r;
}

class SphereModel::Loader : public helper::io::SphereLoader
{
public:
    SphereModel* dest;
    Loader(SphereModel* dest) : dest(dest) { }
    void addSphere(double x, double y, double z, double r)
    {
        dest->addSphere(Vector3(x,y,z),r);
    }
};

bool SphereModel::load(const char* filename)
{
    this->resize(0);
    Loader loader(this);
    return loader.load(filename);
}

void SphereModel::draw(int index)
{
    Vector3 p = (*getX())[index];
    glPushMatrix();
    glTranslated(p[0], p[1], p[2]);
    glutSolidSphere(radius[index], 8, 4);
    //glutSolidSphere(radius[index], 32,16);
    glPopMatrix();
}

void SphereModel::draw()
{
    if (isActive() && getContext()->getShowCollisionModels())
    {
        glEnable(GL_LIGHTING);
        glEnable(GL_COLOR_MATERIAL);
        //glDisable(GL_LIGHTING);
        if (isStatic())
            glColor3f(0.5, 0.5, 0.5);
        else
            glColor3f(1.0, 0.0, 0.0);
        for (int i=0; i<size; i++)
        {
            draw(i);
        }
        glDisable(GL_LIGHTING);
    }
    if (isActive() && getPrevious()!=NULL && getContext()->getShowBoundingCollisionModels() && dynamic_cast<core::VisualModel*>(getPrevious())!=NULL)
        dynamic_cast<core::VisualModel*>(getPrevious())->draw();
}

void SphereModel::computeBoundingTree(int maxDepth)
{
    CubeModel* cubeModel = createPrevious<CubeModel>();
    if (isStatic() && !cubeModel->empty()) return; // No need to recompute BBox if immobile

    Vector3 minElem, maxElem;

    cubeModel->resize(size);
    if (!empty())
    {
        VecCoord& x = *getX();
        for (int i=0; i<size; i++)
        {
            double r = radius[i];
            for (int c=0; c<3; c++)
            {
                minElem[c] = x[i][c] - r;
                maxElem[c] = x[i][c] + r;
            }
            cubeModel->setParentOf(i, minElem, maxElem);
        }
        cubeModel->computeBoundingTree(maxDepth);
    }
}

void SphereModel::computeContinuousBoundingTree(double dt, int maxDepth)
{
    CubeModel* cubeModel = createPrevious<CubeModel>();
    if (isStatic() && !cubeModel->empty()) return; // No need to recompute BBox if immobile

    Vector3 minElem, maxElem;

    cubeModel->resize(size);
    if (!empty())
    {
        VecCoord& x = *getX();
        VecDeriv& v = *getV();
        for (int i=0; i<size; i++)
        {
            double r = radius[i];
            for (int c=0; c<3; c++)
            {
                if (v[i][c] < 0)
                {
                    minElem[c] = x[i][c] + v[i][c]*dt - r;
                    maxElem[c] = x[i][c]           + r;
                }
                else
                {
                    minElem[c] = x[i][c]           - r;
                    maxElem[c] = x[i][c] + v[i][c]*dt + r;
                }
            }
            cubeModel->setParentOf(i, minElem, maxElem);
        }
        cubeModel->computeBoundingTree(maxDepth);
    }
}

} // namespace collision

} // namespace component

} // namespace sofa

