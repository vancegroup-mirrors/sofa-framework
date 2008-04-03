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
#ifndef SOFA_CORE_OBJECTMODEL_XFIELD_H
#define SOFA_CORE_OBJECTMODEL_XFIELD_H

#include <sofa/core/objectmodel/DataPtr.h>

namespace sofa
{

namespace core
{

namespace objectmodel
{

/**
 *  \brief Access to the getX() vector of a MechanicalObject.
 *
 *  @author Francois Faure
 */
template<class DataTypes>
class XDataPtr : public DataPtr<typename DataTypes::VecCoord>
{
public:
    typedef typename DataTypes::VecCoord VecCoord;
    
    XDataPtr(VecCoord** ptr, const char* name)
    : DataPtr<VecCoord>(0,name)
            , m_coordPtr(ptr)
    {}

    ~XDataPtr()
    {}


    VecCoord* beginEdit()
    {
        this->ptr = *m_coordPtr;
        //std::cerr<<"XDataPtr<DataTypes>::beginEdit()"<<std::endl;
        return this->DataPtr<VecCoord>::beginEdit();
    }

protected:
    VecCoord** m_coordPtr;

};

} // namespace objectmodel

} // namespace core

} // namespace sofa

#endif

