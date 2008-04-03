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
#ifndef SOFA_CORE_OBJECTMODEL_DATAPTR_H
#define SOFA_CORE_OBJECTMODEL_DATAPTR_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/core/objectmodel/BaseData.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <map>


namespace sofa
{

namespace core
{

namespace objectmodel
{

/**
 *  \brief Pointer to data, readable and writable from/to a string.
 *
 */
template < class T = void* >
class DataPtr : public BaseData
{
public:
    /** Constructor
    \param t a pointer to the value
    \param h help on the field
    */
    DataPtr( T* t, const char* h )
            : BaseData(h)
            , ptr(t)
    {}

    virtual ~DataPtr()
    {}

    inline void printValue(std::ostream& out) const ;
    inline std::string getValueString() const ;
    inline std::string getValueTypeString() const ; //{ return std::string(typeid(*ptr).name()); }
    inline T* beginEdit()
    {
        m_isSet = true;
        return ptr;
    }
    inline void endEdit()
    {}
    inline void setValue(const T& value )
    {
        *beginEdit()=value;
        endEdit();
    }
    inline const T& getValue() const
    {
        return *ptr;
    }

protected:
    /// Pointer to the parameter
    T* ptr;


    /** Try to read argument value from an input stream.
        Return false if failed
    */
    inline bool read( std::string& s )
    {
        if (s.empty())
            return false;
        //std::cerr<<"DataPtr::read "<<s.c_str()<<std::endl;
        std::istringstream istr( s.c_str() );
        istr >> *ptr;
        if( istr.fail() )
        {
            //std::cerr<<"field "<<getName<<" could not read value: "<<s<<std::endl;
            return false;
        }
        else
        {
            m_isSet = true;
            return true;
        }
    }
};

/// Specialization for reading strings
template<>
inline
bool DataPtr<std::string>::read( std::string& str )
{
    *ptr = str;
    m_isSet = true;
    return true;
}

/// Specialization for reading booleans
template<>
inline
bool DataPtr<bool>::read( std::string& str )
{
    if (str.empty())
        return false;
	if (str[0] == 'T' || str[0] == 't')
	    *ptr = true;
	else if (str[0] == 'F' || str[0] == 'f')
	    *ptr = false;
	else if ((str[0] >= '0' && str[0] <= '9') || str[0] == '-')
		*ptr = (atoi(str.c_str()) != 0);
	else return false;
    m_isSet = true;
    return true;
}

/// General case for printing default value
template<class T>
inline
void DataPtr<T>::printValue( std::ostream& out=std::cout ) const
{
    out << *ptr << " ";
}

/// General case for printing default value
template<class T>
inline
std::string DataPtr<T>::getValueString() const
{
    std::ostringstream out;
    out << *ptr;
    return out.str();
}

template<class T>
inline
std::string DataPtr<T>::getValueTypeString() const
{
    return BaseData::typeName(ptr);
}

} // namespace objectmodel

} // namespace core

} // namespace sofa

#endif
