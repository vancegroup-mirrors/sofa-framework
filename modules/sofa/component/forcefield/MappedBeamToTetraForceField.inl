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

#include <sofa/component/forcefield/MappedBeamToTetraForceField.h>


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
            void MappedBeamToTetraForceField<DataTypes>::addForce(VecDeriv& f, const VecCoord& x, const VecDeriv& v)
            {
                //std::cout << "AddForce in MappedFF" << std::endl;
            }

            template<class DataTypes>
            void MappedBeamToTetraForceField<DataTypes>::addDForce(VecDeriv& f1, const VecDeriv& dx1, double kFactor, double bFactor)
            {
                //std::cout << "AddDForce in MappedFF" << std::endl;
            }

            template<class DataTypes>
            void MappedBeamToTetraForceField<DataTypes>::addKToMatrix(sofa::defaulttype::BaseMatrix *mat, SReal k, unsigned int &offset)
            {                
                std::cout << "AddKToMatrix in MappedFF" << std::endl;
                std::cout << "K = " << k << " offset = " << offset << std::endl;
                //get stiffness matrix from the beam FEM
                unsigned int nbBeamElements = mappedBeamForceField->getNbBeams();
                unsigned int nbBeamDOFs = 6;
                unsigned int nbBeamNodes = beamMO->getSize();

                std::cout << "Number of beam elements = " << nbBeamElements << std::endl;
                std::cout << "Number of beam nodes = " << nbBeamNodes << std::endl;

                sofa::component::linearsolver::SparseMatrix<Real> beamMatrix(nbBeamDOFs*nbBeamNodes,nbBeamDOFs*nbBeamNodes);                
                mappedBeamForceField->addKToMatrix(&beamMatrix, k, offset);
                //std::cout << "BeamMatrix = " << beamMatrix << std::endl;
                //std::cout << "size = " << beamMatrix.rowSize() << " X "  << beamMatrix.colSize() << std::endl;

                //get J from the mapping
            }

            template <class DataTypes>
                    double MappedBeamToTetraForceField<DataTypes>::getPotentialEnergy(const VecCoord&x) const
            {
            }

            template<class DataTypes>
            void MappedBeamToTetraForceField<DataTypes>::draw()
            {
            }

	} // namespace forcefield

    } // namespace component

} // namespace sofa

#endif
