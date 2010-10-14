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
#ifndef SOFA_COMPONENT_FORCEFIELD_MAPPEDBEAMTOTETRAFORCEFIELD_INL
#define SOFA_COMPONENT_FORCEFIELD_MAPPEDBEAMTOTETRAFORCEFIELD_INL

#include <sofa/core/behavior/ForceField.inl>
#include <sofa/helper/system/config.h>
#include <sofa/helper/system/gl.h>
#include <assert.h>
#include <iostream>

#include <fstream>

#include <sofa/component/forcefield/MappedBeamToTetraForceField.h>


#include <time.h>
#include <sys/time.h>


namespace sofa
{

    namespace component
    {

        namespace forcefield
        {
            template<class DataTypes>
            void MappedBeamToTetraForceField<DataTypes>::init()
            {
                this->core::behavior::ForceField<DataTypes>::init();
                std::cout << "Initializing Mapped BeamToTetra Force Field" << std::endl;
                //mappedBeamForceField = this->getContext()->get< BeamFEMForceField<DataTypes> >();
                this->getContext()->get(mappedBeamForceField, "Beam/MappedBeamFEMFF");
                if (mappedBeamForceField == NULL) {                    
                    std::cerr << "No BeamForceField to mapped found " << std::endl;
                } else {
                    std::cout << "Beam force field found, name = " << mappedBeamForceField->getName() << std::endl;
                }

                this->getContext()->get(mappingBeamTetra, "Beam/MappingBeamTetra");
                if (mappingBeamTetra == NULL) {
                    std::cerr << "No Mapping found" << std::endl;
                } else {
                    std::cerr <<  "Barycentric mapping found, name = " << mappingBeamTetra->getName() << std::endl;
                }

                this->getContext()->get(beamMO, "Beam/BeamMO");

            }

            template<class DataTypes>
            void MappedBeamToTetraForceField<DataTypes>::addForce(VecDeriv& /*f*/, const VecCoord& /*x*/, const VecDeriv& /*v*/)
            {
                //std::cout << "AddForce in MappedFF" << std::endl;
            }

            template<class DataTypes>
            void MappedBeamToTetraForceField<DataTypes>::addDForce(VecDeriv& /*f1*/, const VecDeriv& /*dx1*/, double /*kFactor*/, double /*bFactor*/)
            {
                //std::cout << "AddDForce in MappedFF" << std::endl;
            }

            template<class DataTypes>
            void MappedBeamToTetraForceField<DataTypes>::addKToMatrix(sofa::defaulttype::BaseMatrix *mat, SReal k, unsigned int &offset)
            {

                unsigned int nbBeamDOFs = beamMO->getSize();              
                unsigned int nbTetDOFs =  mat->rowSize()/3;   //this is not OK

                sofa::component::linearsolver::CompressedRowSparseMatrix<BeamBlockType> beamMatrix;
                beamMatrix.resizeBloc(nbBeamDOFs, nbBeamDOFs);                                

                mappedBeamForceField->addKToMatrix(&beamMatrix, k, offset);

                const sofa::component::linearsolver::CompressedRowSparseMatrix<MappingBlockType> * mappingMatrix;
                mappingMatrix = dynamic_cast<const sofa::component::linearsolver::CompressedRowSparseMatrix<MappingBlockType> *>(mappingBeamTetra->getJ());

                MappingBlockType mappingBuffer;
                BeamBlockType beamBuffer;

                sofa::component::linearsolver::CompressedRowSparseMatrix<TempBlockType> tempMatrix;
                tempMatrix.resizeBloc(nbTetDOFs, nbBeamDOFs);

                for (int mappingRowIndex = 0; mappingRowIndex < mappingMatrix->nBlocRow; mappingRowIndex++) {   //through X, must take each row  (but test can be added)
                    for (MappingColBlockConstIterator mappingColIter = mappingMatrix->bRowBegin(mappingRowIndex); mappingColIter < mappingMatrix->bRowEnd(mappingRowIndex); mappingColIter++) {  //take non zero blocks in row, determines the row in K)
                        MappingBlockConstAccessor mappingBlock = mappingColIter.bloc();
                        const MappingBlockType& mappingBlockData = *(const MappingBlockType*)mappingBlock.elements(mappingBuffer.ptr());
                        int mappingColIndex = mappingBlock.getCol();
                        for (BeamColBlockConstIterator beamColIter = beamMatrix.bRowBegin(mappingRowIndex); beamColIter < beamMatrix.bRowEnd(mappingRowIndex); beamColIter++) {
                            BeamBlockConstAccessor beamBlock = beamColIter.bloc();
                            const BeamBlockType& beamBlockData = *(const BeamBlockType*)beamBlock.elements(beamBuffer.ptr());
                            int beamColIndex = beamBlock.getCol();                            
                            TempBlockType tempBlockData(0.0);
                            //multiply the block, could be done more efficiently
                            for (int i = 0; i < 3; i++)
                                for (int j = 0; j < 6; j++)
                                    for (int k = 0; k < 6; k++)
                                        tempBlockData(i,j) += mappingBlockData(k,i)*beamBlockData(k,j);                            
                            tempMatrix.blocAdd(mappingColIndex, beamColIndex, tempBlockData.ptr());
                        }
                    }
                }

                TempBlockType tempBuffer;
                for (int tempRowIndex = 0; tempRowIndex < tempMatrix.nBlocRow; tempRowIndex++) {
                    for (TempColBlockConstIterator tempColIter = tempMatrix.bRowBegin(tempRowIndex); tempColIter < tempMatrix.bRowEnd(tempRowIndex); tempColIter++) {
                        TempBlockConstAccessor tempBlock = tempColIter.bloc();
                        const TempBlockType& tempBlockData = *(const TempBlockType*) tempBlock.elements(tempBuffer.ptr());
                        int tempColIndex = tempBlock.getCol();
                        for (MappingColBlockConstIterator mappingColIter = mappingMatrix->bRowBegin(tempColIndex); mappingColIter < mappingMatrix->bRowEnd(tempColIndex); mappingColIter++) {
                            MappingBlockConstAccessor mappingBlock = mappingColIter.bloc();
                            const MappingBlockType &mappingBlockData = *(const MappingBlockType*) mappingBlock.elements(mappingBuffer.ptr());
                            int mappingColIndex = mappingBlock.getCol();
                            TetraBlockType tetraBlockData(0.0);
                            //multiply the block, could be done more efficiently
                            for (int i = 0; i < 3; i++)
                                for (int j = 0; j < 3; j++) {
                                    for (int k = 0; k < 6; k++)
                                        tetraBlockData(i,j) += tempBlockData(i,k) * mappingBlockData(k,j);
                                    //because input matrix is  BaseMatrix, we do it like this for now
                                    mat->add(3*tempRowIndex+i, 3*mappingColIndex+j, tetraBlockData(i,j));
                                }                            
                            //mat->blocAdd(tempRowIndex,mappingColIndex,tetraBlockData.ptr());   //if mat is block matrix
                        }
                    }
                }         

            }


            template <class DataTypes>
                    double MappedBeamToTetraForceField<DataTypes>::getPotentialEnergy(const VecCoord& /*x*/) const
            {
                return(0.0);
            }

            template<class DataTypes>
            void MappedBeamToTetraForceField<DataTypes>::draw()
            {
            }

        } // namespace forcefield

    } // namespace component

} // namespace sofa

#endif
