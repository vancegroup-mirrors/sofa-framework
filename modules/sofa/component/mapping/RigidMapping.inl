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
#ifndef SOFA_COMPONENT_MAPPING_RIGIDMAPPING_INL
#define SOFA_COMPONENT_MAPPING_RIGIDMAPPING_INL

#include <sofa/component/mapping/RigidMapping.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/helper/io/MassSpringLoader.h>
#include <sofa/helper/io/SphereLoader.h>
#include <sofa/helper/io/Mesh.h>
#include <sofa/component/container/MultiMeshLoader.h>
#include <sofa/helper/gl/template.h>
#include <sofa/core/behavior/MechanicalMapping.inl>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/Mapping.h>
#include <sofa/core/behavior/MappedModel.h>
#include <sofa/simulation/common/Simulation.h>
#include <string.h>
#include <iostream>

namespace sofa
{

namespace component
{

namespace mapping
{

using namespace sofa::defaulttype;

template<class BasicMapping>
class RigidMapping<BasicMapping>::Loader : public helper::io::MassSpringLoader,
    public helper::io::SphereLoader
{
public:

    RigidMapping<BasicMapping>* dest;
    Loader(RigidMapping<BasicMapping>* dest) :
        dest(dest)
    {
    }
    virtual void addMass(SReal px, SReal py, SReal pz, SReal, SReal, SReal,
                         SReal, SReal, bool, bool)
    {
        Coord c;
        Out::DataTypes::set(c, px, py, pz);
        dest->points.beginEdit()->push_back(c); //Coord((Real)px,(Real)py,(Real)pz));
    }
    virtual void addSphere(SReal px, SReal py, SReal pz, SReal)
    {
        Coord c;
        Out::DataTypes::set(c, px, py, pz);
        dest->points.beginEdit()->push_back(c); //Coord((Real)px,(Real)py,(Real)pz));
    }
};

template<class BasicMapping>
void RigidMapping<BasicMapping>::load(const char *filename)
{
    points.beginEdit()->resize(0);

    if (strlen(filename) > 4
        && !strcmp(filename + strlen(filename) - 4, ".xs3"))
    {
        Loader loader(this);
        loader.helper::io::MassSpringLoader::load(filename);
    }
    else if (strlen(filename) > 4
        && !strcmp(filename + strlen(filename) - 4, ".sph"))
    {
        Loader loader(this);
        loader.helper::io::SphereLoader::load(filename);
    }
    else if (strlen(filename) > 0)
    { // Default to mesh loader
        helper::io::Mesh* mesh = helper::io::Mesh::Create(filename);
        if (mesh != NULL)
        {
            points.beginEdit()->resize(mesh->getVertices().size());
            for (unsigned int i = 0; i < mesh->getVertices().size(); i++)
            {
                Out::DataTypes::set((*points.beginEdit())[i],
                                    mesh->getVertices()[i][0],
                                    mesh->getVertices()[i][1],
                                    mesh->getVertices()[i][2]);
            }
            delete mesh;
        }
    }
}

template<class BasicMapping>
int RigidMapping<BasicMapping>::addPoint(const Coord& c)
{
    int i = points.getValue().size();
    points.beginEdit()->push_back(c);
    return i;
}

template<class BasicMapping>
int RigidMapping<BasicMapping>::addPoint(const Coord& c, int indexFrom)
{
    int i = points.getValue().size();
    points.beginEdit()->push_back(c);
    if (!repartition.getValue().empty())
    {
        repartition.beginEdit()->push_back(indexFrom);
        repartition.endEdit();
    }
    else if (!i)
    {
        index.setValue(indexFrom);
    }
    else if ((int) index.getValue() != indexFrom)
    {
        sofa::helper::vector<unsigned int>& rep = *repartition.beginEdit();
        rep.clear();
        rep.reserve(i + 1);
        rep.insert(rep.end(), index.getValue(), i);
        rep.push_back(indexFrom);
        repartition.endEdit();
    }
    return i;
}

template <class BasicMapping>
void RigidMapping<BasicMapping>::beginAddContactPoint()
{

    this->clear(0);
    this->toModel->resize(0);

}

template <class BasicMapping>
int RigidMapping<BasicMapping>::addContactPointFromInputMapping(const sofa::defaulttype::Vector3& pos, std::vector< std::pair<int, double> > & /*baryCoords*/)
{


    typename In::VecCoord& xfrom = *this->fromModel->getX();

    Coord posContact;
    for (unsigned int i=0; i<3; i++)
        posContact[i] = pos[i];



    unsigned int inputIdx=_inputMapping->index.getValue();

    this->index.setValue(inputIdx);
    this->repartition.setValue(_inputMapping->repartition.getValue());
    Coord x_local = xfrom[inputIdx].inverseRotate(posContact - xfrom[inputIdx].getCenter());


    this->addPoint(x_local, inputIdx);

    int index= points.getValue().size() -1;
     this->toModel->resize(index+1);
    return index;

}

template <class BasicMapping>
void RigidMapping<BasicMapping>::init()
{
    if (!fileRigidMapping.getValue().empty())
        this->load(fileRigidMapping.getFullPath().c_str());

    if (this->points.getValue().empty() && this->toModel != NULL
        && !useX0.getValue())
    {
        VecCoord& x = *this->toModel->getX();
        points.beginEdit()->resize(x.size());
        unsigned int i = 0, cpt = 0;
        if (globalToLocalCoords.getValue() == true)
        {//test booleen fromWorldCoord
            InVecCoord& xfrom = *this->fromModel->getX();
            switch (repartition.getValue().size())
            {
            case 0:
                for (i = 0; i < x.size(); i++)
                {
                    (*points.beginEdit())[i] = xfrom[0].inverseRotate(x[i]
                        - xfrom[0].getCenter());
                }
                break;
            case 1:
                for (i = 0; i < xfrom.size(); i++)
                {
                    for (unsigned int j = 0; j < repartition.getValue()[0]; j++, cpt++)
                    {
                        (*points.beginEdit())[cpt]
                            = xfrom[i].inverseRotate(x[cpt]
                                - xfrom[i].getCenter());
                    }
                }
                break;
            default:
                for (i = 0; i < xfrom.size(); i++)
                {
                    for (unsigned int j = 0; j < repartition.getValue()[i]; j++, cpt++)
                    {
                        (*points.beginEdit())[cpt]
                            = xfrom[i].inverseRotate(x[cpt]
                                - xfrom[i].getCenter());
                    }
                }
                break;
            }
        }
        else
        {
            for (i = 0; i < x.size(); i++)
            {
                (*points.beginEdit())[i] = x[i];
            }
        }
    }

    this->BasicMapping::init();

    sofa::component::container::MultiMeshLoader * loader;
    this->getContext()->get(loader);
    if (loader)
    {
        sofa::helper::vector<unsigned int>& rep = *repartition.beginEdit();
        unsigned int cpt = 0;
        InVecCoord& xfrom = *this->fromModel->getX();
        VecCoord& xto = *this->toModel->getX();
        for (unsigned int i = 0; i < loader->getNbMeshs(); i++)
        {
            rep.push_back(loader->getNbPoints(i));
            if (globalToLocalCoords.getValue() == true)
            {
                for (unsigned int j = 0; j < loader->getNbPoints(i); j++, cpt++)
                {
                    (*points.beginEdit())[cpt]
                        = xfrom[i].inverseRotate(xto[cpt] - xfrom[i].getCenter());
                }
            }
        }
        repartition.endEdit();
    }
}

template <class BasicMapping>
void RigidMapping<BasicMapping>::bwdInit()
{
    if(contactDuplicate.getValue()==true)
    {
        const std::string path = nameOfInputMap.getValue();
        this->fromModel->getContext()->get(_inputMapping, sofa::core::objectmodel::BaseContext::SearchRoot);
        if(_inputMapping==NULL)
            serr<<"WARNING : can not found the input  Mapping"<<sendl;
        else
            sout<<"input Mapping named "<<_inputMapping->getName()<<" is found"<<sendl;
    }
}

/*
template <class BasicMapping>
void RigidMapping<BasicMapping>::disable()
{

	if (!this->points.getValue().empty() && this->toModel!=NULL)
	{
		VecCoord& x = *this->toModel->getX();
		x.resize(points.getValue().size());
		for (unsigned int i=0;i<points.getValue().size();i++)
			x[i] = points.getValue()[i];
	}
}
*/
template <class BasicMapping>
void RigidMapping<BasicMapping>::clear(int reserve)
{
    this->points.beginEdit()->clear();
    if (reserve)
        this->points.beginEdit()->reserve(reserve);
    this->repartition.beginEdit()->clear();
    this->repartition.endEdit();
}

template<class BasicMapping>
void RigidMapping<BasicMapping>::setRepartition(unsigned int value)
{
    vector<unsigned int>& rep = *this->repartition.beginEdit();
    rep.clear();
    rep.push_back(value);
    this->repartition.endEdit();
}

template<class BasicMapping>
void RigidMapping<BasicMapping>::setRepartition(sofa::helper::vector<
    unsigned int> values)
{
    vector<unsigned int>& rep = *this->repartition.beginEdit();
    rep.clear();
    rep.reserve(values.size());
    //repartition.setValue(values);
    sofa::helper::vector<unsigned int>::iterator it = values.begin();
    while (it != values.end())
    {
        rep.push_back(*it);
        it++;
    }
    this->repartition.endEdit();
}

template<class DataTypes>
const typename DataTypes::VecCoord* M_getX0(core::behavior::MechanicalState<DataTypes>* model)
{
    return model->getX0();
}

template<class DataTypes>
const typename DataTypes::VecCoord* M_getX0(core::behavior::MappedModel<DataTypes>* /*model*/)
{
    return NULL;
}

template<class BasicMapping>
const typename RigidMapping<BasicMapping>::VecCoord & RigidMapping<BasicMapping>::getPoints()
{
    if (useX0.getValue())
    {
        const VecCoord* v = M_getX0(this->toModel);
        if (v)
            return *v;
        else
            serr
                << "RigidMapping: ERROR useX0 can only be used in MechanicalMappings."
                << sendl;
    }
    return points.getValue();
}

template<class BasicMapping>
void RigidMapping<BasicMapping>::apply(VecCoord& out, const InVecCoord& in)
{
    const VecCoord& pts = this->getPoints();

    updateJ = true;

    rotatedPoints.resize(pts.size());
    out.resize(pts.size());

    unsigned repartitionCount = repartition.getValue().size();

    if (repartitionCount > 1 && repartitionCount != in.size())
    {
        serr << "Error : mapping dofs repartition is not correct" << sendl;
        return;
    }

    unsigned inIdxBegin;
    unsigned inIdxEnd;

    if (repartitionCount == 0)
    {
        inIdxBegin = index.getValue();
        if (indexFromEnd.getValue())
        {
            inIdxBegin = in.size() - 1 - inIdxBegin;
        }
        inIdxEnd = inIdxBegin + 1;
    }
    else
    {
        inIdxBegin = 0;
        inIdxEnd = in.size();
    }

    unsigned outputPerInput;
    if (repartitionCount == 0)
    {
        outputPerInput = pts.size();
    }
    else
    {
        outputPerInput = repartition.getValue()[0];
    }

    Coord translation;
    Mat rotation;

    for (unsigned inIdx = inIdxBegin, outIdx = 0; inIdx < inIdxEnd; ++inIdx)
    {
        if (repartitionCount > 1)
        {
            outputPerInput = repartition.getValue()[inIdx];
        }

        translation = in[inIdx].getCenter();
        in[inIdx].writeRotationMatrix(rotation);

        for (unsigned iOutput = 0;
            iOutput < outputPerInput;
             ++iOutput, ++outIdx)
        {
            rotatedPoints[outIdx] = rotation * pts[outIdx];
            out[outIdx] = rotatedPoints[outIdx];
            out[outIdx] += translation;
        }
    }
}

template<class BasicMapping>
void RigidMapping<BasicMapping>::applyJ(VecDeriv& out, const InVecDeriv& in)
{
    const VecCoord& pts = this->getPoints();
    out.resize(pts.size());

    bool isMaskInUse = maskTo->isInUse();
    unsigned repartitionCount = repartition.getValue().size();

    if (repartitionCount > 1 && repartitionCount != in.size())
    {
        serr << "Error : mapping dofs repartition is not correct" << sendl;
        return;
    }

    unsigned inIdxBegin;
    unsigned inIdxEnd;

    if (repartitionCount == 0)
    {
        inIdxBegin = index.getValue();
        if (indexFromEnd.getValue())
        {
            inIdxBegin = in.size() - 1 - inIdxBegin;
        }
        inIdxEnd = inIdxBegin + 1;
    }
    else
    {
        inIdxBegin = 0;
        inIdxEnd = in.size();
    }

    unsigned outputPerInput;
    if (repartitionCount == 0)
    {
        outputPerInput = pts.size();
    }
    else
    {
        outputPerInput = repartition.getValue()[0];
    }

    typedef core::behavior::BaseMechanicalState::ParticleMask ParticleMask;
    const ParticleMask::InternalStorage& indices = maskTo->getEntries();
    ParticleMask::InternalStorage::const_iterator it = indices.begin();

    for (unsigned inIdx = inIdxBegin, outIdx = 0; inIdx < inIdxEnd; ++inIdx)
    {
        if (repartitionCount > 1)
        {
            outputPerInput = repartition.getValue()[inIdx];
        }

        for (unsigned iOutput = 0;
            iOutput < outputPerInput && !(isMaskInUse && it == indices.end());
             ++iOutput, ++outIdx)
        {
            if (isMaskInUse)
            {
                if (outIdx != *it)
                {
                    continue;
                }
                ++it;
            }
            out[outIdx] = in[inIdx].velocityAtRotatedPoint(rotatedPoints[outIdx]);
        }
    }
}

template<class BasicMapping>
void RigidMapping<BasicMapping>::applyJT(InVecDeriv& out, const VecDeriv& in)
{
    const VecCoord& pts = this->getPoints();

    bool isMaskInUse = maskTo->isInUse();
    maskFrom->setInUse(isMaskInUse);

    unsigned repartitionCount = repartition.getValue().size();

    if (repartitionCount > 1 && repartitionCount != out.size())
    {
        serr << "Error : mapping dofs repartition is not correct" << sendl;
        return;
    }

    unsigned outIdxBegin;
    unsigned outIdxEnd;

    if (repartitionCount == 0)
    {
        outIdxBegin = index.getValue();
        if (indexFromEnd.getValue())
        {
            outIdxBegin = out.size() - 1 - outIdxBegin;
        }
        outIdxEnd = outIdxBegin + 1;
    }
    else
    {
        outIdxBegin = 0;
        outIdxEnd = out.size();
    }

    unsigned inputPerOutput;
    if (repartitionCount == 0)
    {
        inputPerOutput = pts.size();
    }
    else
    {
        inputPerOutput = repartition.getValue()[0];
    }

    typedef core::behavior::BaseMechanicalState::ParticleMask ParticleMask;
    const ParticleMask::InternalStorage& indices = maskTo->getEntries();
    ParticleMask::InternalStorage::const_iterator it = indices.begin();

    for (unsigned outIdx = outIdxBegin, inIdx = 0; outIdx < outIdxEnd; ++outIdx)
    {
        if (repartitionCount > 1)
        {
            inputPerOutput = repartition.getValue()[outIdx];
        }

        for (unsigned iInput = 0;
            iInput < inputPerOutput && !(isMaskInUse && it == indices.end());
             ++iInput, ++inIdx)
        {
            if (isMaskInUse)
            {
                if (inIdx != *it)
                {
                    continue;
                }
                ++it;
            }
            out[outIdx].getVCenter() += in[inIdx];
            out[outIdx].getVOrientation() += cross(rotatedPoints[inIdx], in[inIdx]);

        }
        if (isMaskInUse)
        {
            maskFrom->insertEntry(outIdx);
        }
    }
}

// RigidMapping::applyJT( InVecConst& out, const VecConst& in ) //
// this function propagate the constraint through the rigid mapping :
// if one constraint along (vector n) with a value (v) is applied on the childModel (like collision model)
// then this constraint is transformed by (Jt.n) with value (v) for the rigid model
// There is a specificity of this propagateConstraint: we have to find the application point on the childModel
// in order to compute the right constaint on the rigidModel.
template<class BaseMapping>
void RigidMapping<BaseMapping>::applyJT(InVecConst& out, const VecConst& in)
{
    int outSize = out.size();
    out.resize(in.size() + outSize); // we can accumulate in "out" constraints from several mappings

    switch (repartition.getValue().size())
    {
    case 0:
    {
        for (unsigned int i = 0; i < in.size(); i++)
        {
            Deriv v;
            DRot omega = DRot();
            OutConstraintIterator itOut;
            std::pair<OutConstraintIterator, OutConstraintIterator> iter =
                in[i].data();

            for (itOut = iter.first; itOut != iter.second; itOut++)
            {
                const unsigned int i = itOut->first;// index of the node
                // out = Jt in
                // Jt = [ I     ]
                //      [ -OM^t ]
                // -OM^t = OM^

                const Deriv f = (Deriv) itOut->second;
                v += f;
                omega += cross(rotatedPoints[i], f);
            }

            const InDeriv result(v, omega);
            if (!indexFromEnd.getValue())
            {
                out[outSize + i].add(index.getValue(), result);
            }
            else
            {
                out[outSize + i].add(out.size() - 1 - index.getValue(), result);
            }
        }
        break;
    }
    case 1:
    {
        const unsigned int numDofs = this->getFromModel()->getX()->size();

        const unsigned int val = repartition.getValue()[0];
        for (unsigned int i = 0; i < in.size(); i++)
        {
            unsigned int cpt = 0;

            std::pair<OutConstraintIterator, OutConstraintIterator> iter =
                in[i].data();

            OutConstraintIterator it = iter.first;
            for (unsigned int ito = 0; ito < numDofs; ito++)
            {
                Deriv v;
                DRot omega = DRot();
                bool needToInsert = false;

                for (unsigned int r = 0; r < val && it != iter.second; r++, cpt++)
                {
                    const unsigned int i = it->first;// index of the node
                    if (i != cpt)
                        continue;

                    needToInsert = true;
                    const Deriv f = (Deriv) it->second;
                    v += f;
                    omega += cross(rotatedPoints[cpt], f);
                    it++;
                }
                if (needToInsert)
                {
                    const InDeriv result(v, omega);
                    out[outSize + i].add(ito, result);
                }
            }
        }
        break;
    }
    default:
    {
        const unsigned int numDofs = this->getFromModel()->getX()->size();

        for (unsigned int i = 0; i < in.size(); i++)
        {
            unsigned int cpt = 0;

            std::pair<OutConstraintIterator, OutConstraintIterator> iter =
                in[i].data();

            OutConstraintIterator it = iter.first;
            for (unsigned int ito = 0; ito < numDofs; ito++)
            {
                Deriv v;
                DRot omega = DRot();
                bool needToInsert = false;

                for (unsigned int r = 0; r < repartition.getValue()[ito] && it
                    != iter.second; r++, cpt++)
                {
                    const unsigned int i = it->first;// index of the node
                    if (i != cpt)
                        continue;

                    needToInsert = true;
                    const Deriv f = (Deriv) it->second;
                    v += f;
                    omega += cross(rotatedPoints[cpt], f);
                    it++;
                }
                if (needToInsert)
                {
                    const InDeriv result(v, omega);
                    out[outSize + i].add(ito, result);
                }
            }
        }
        break;
    }
    }
}

template <class BaseMapping>
const sofa::defaulttype::BaseMatrix* RigidMapping<BaseMapping>::getJ()
{
    const VecCoord& out = *this->toModel->getX();
    const InVecCoord& in = *this->fromModel->getX();
    const VecCoord& pts = this->getPoints();
    assert(pts.size() == out.size());

    if (matrixJ.get() == 0 || updateJ)
    {
        updateJ = false;
        if (matrixJ.get() == 0 ||
            matrixJ->rowBSize() != out.size() ||
            matrixJ->colBSize() != in.size())
        {
            matrixJ.reset(new MatrixType(out.size() * NOut, in.size() * NIn));
        }
        else
        {
            matrixJ->clear();
        }

//        bool isMaskInUse = maskTo->isInUse();
        unsigned repartitionCount = repartition.getValue().size();

        if (repartitionCount > 1 && repartitionCount != in.size())
        {
            serr << "Error : mapping dofs repartition is not correct" << sendl;
            return 0;
        }

        unsigned inIdxBegin;
        unsigned inIdxEnd;

        if (repartitionCount == 0)
        {
            inIdxBegin = index.getValue();
            if (indexFromEnd.getValue())
            {
                inIdxBegin = in.size() - 1 - inIdxBegin;
            }
            inIdxEnd = inIdxBegin + 1;
        }
        else
        {
            inIdxBegin = 0;
            inIdxEnd = in.size();
        }

        unsigned outputPerInput;
        if (repartitionCount == 0)
        {
            outputPerInput = pts.size();
        }
        else
        {
            outputPerInput = repartition.getValue()[0];
        }

//        typedef core::behavior::BaseMechanicalState::ParticleMask ParticleMask;
//        const ParticleMask::InternalStorage& indices = maskTo->getEntries();
//        ParticleMask::InternalStorage::const_iterator it = indices.begin();

        for (unsigned inIdx = inIdxBegin, outIdx = 0; inIdx < inIdxEnd; ++inIdx)
        {
            if (repartitionCount > 1)
            {
                outputPerInput = repartition.getValue()[inIdx];
            }

            for (unsigned iOutput = 0;
                iOutput < outputPerInput; // iOutput < outputPerInput && !(isMaskInUse && it == indices.end());
                 ++iOutput, ++outIdx)
            {
//                if (isMaskInUse)
//                {
//                    if (outIdx != *it)
//                    {
//                        continue;
//                    }
//                    ++it;
//                }
                setJMatrixBlock(outIdx, inIdx);
            }
        }
    }
    return matrixJ.get();
}

template<class Real>
struct RigidMappingMatrixHelper<2, Real>
{
    template <class Matrix, class Vector>
    static void setMatrix(Matrix& mat,
                          const Vector& vec)
    {
        mat[0][0] = (Real) 1     ;    mat[1][0] = (Real) 0     ;
        mat[0][1] = (Real) 0     ;    mat[1][1] = (Real) 1     ;
        mat[0][2] = (Real)-vec[1];    mat[1][2] = (Real) vec[0];
    }
};

template<class Real>
struct RigidMappingMatrixHelper<3, Real>
{
    template <class Matrix, class Vector>
    static void setMatrix(Matrix& mat,
                          const Vector& vec)
    {
        // out = J in
        // J = [ I -OM^ ]
        mat[0][0] = (Real) 1     ;    mat[1][0] = (Real) 0     ;    mat[2][0] = (Real) 0     ;
        mat[0][1] = (Real) 0     ;    mat[1][1] = (Real) 1     ;    mat[2][1] = (Real) 0     ;
        mat[0][2] = (Real) 0     ;    mat[1][2] = (Real) 0     ;    mat[2][2] = (Real) 1     ;
        mat[0][3] = (Real) 0     ;    mat[1][3] = (Real)-vec[2];    mat[2][3] = (Real) vec[1];
        mat[0][4] = (Real) vec[2];    mat[1][4] = (Real) 0     ;    mat[2][4] = (Real)-vec[0];
        mat[0][5] = (Real)-vec[1];    mat[1][5] = (Real) vec[0];    mat[2][5] = (Real) 0     ;
    }
};

template<class BasicMapping>
void RigidMapping<BasicMapping>::setJMatrixBlock(unsigned outIdx, unsigned inIdx)
{
    MBloc& block = *matrixJ->wbloc(outIdx, inIdx, true);
    RigidMappingMatrixHelper<N, Real>::setMatrix(block, rotatedPoints[outIdx]);
}

template<class BasicMapping>
void RigidMapping<BasicMapping>::draw()
{
    if (!this->getShow())
        return;
    std::vector<Vector3> points;
    Vector3 point;

    const VecCoord& x = *this->toModel->getX();
    for (unsigned int i = 0; i < x.size(); i++)
    {
        point = OutDataTypes::getCPos(x[i]);
        points.push_back(point);
    }
    simulation::getSimulation()->DrawUtility.drawPoints(points, 7,
                                                        Vec<4, float>(1, 1, 0,1));
}

} // namespace mapping

} // namespace component

} // namespace sofa

#endif
