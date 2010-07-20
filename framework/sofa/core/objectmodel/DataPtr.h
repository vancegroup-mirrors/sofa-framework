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
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: M. Adam, J. Allard, B. Andre, P-J. Bensoussan, S. Cotin, C. Duriez,*
* H. Delingette, F. Falipou, F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza,  *
* M. Nesme, P. Neumann, J-P. de la Plata Alcade, F. Poyer and F. Roy          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_CORE_OBJECTMODEL_DATAPTR_H
#define SOFA_CORE_OBJECTMODEL_DATAPTR_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <sofa/core/objectmodel/Data.h>

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
class DataPtr : public TData<T>
{
public:
    /** Constructor
    \param t a pointer to the value
    \param h help on the field
    */
    DataPtr( T* t, const char* h, bool isDisplayed=true, bool isReadOnly=false, Base* owner=NULL, const char* name="")
    : TData<T>(h, isDisplayed, isReadOnly, owner, name)
    , ptr(t)
    {
    }

    virtual ~DataPtr()
    {}

    inline T* beginEdit()
    {
        this->updateIfDirty();
        ++this->m_counter;
        BaseData::setDirtyOutputs();
        return ptr;
    }
    inline void endEdit()
    {
    }
    inline void setPointer(T* p )
    {
        ptr = p;
    }
    inline void setValue(const T& value )
    {
        *beginEdit()=value;
        endEdit();
    }
    inline const T& getValue() const
    {
        this->updateIfDirty();
        return *ptr;
    }

    virtual const T& virtualGetValue() const { return getValue(); }
    virtual void virtualSetValue(const T& v) { setValue(v); }
    virtual T* virtualBeginEdit() { return beginEdit(); }
    virtual void virtualEndEdit() { endEdit(); }

    /// The value stored in counter can be false, as the pointer can be modified without using the BaseData API. The counter doesn't mean anything in that case
    inline bool isCounterValid() const {return false;}
protected:
    /// Pointer to the parameter
    T* ptr;
};

} // namespace objectmodel

} // namespace core

// Overload helper::ReadAccessor and helper::WriteAccessor

namespace helper
{

template<class T>
class ReadAccessor< core::objectmodel::DataPtr<T> > : public ReadAccessor<T>
{
public:
    typedef ReadAccessor<T> Inherit;
    typedef core::objectmodel::DataPtr<T> data_container_type;
    typedef T container_type;

protected:
    const data_container_type& data;
public:
    ReadAccessor(const data_container_type& d) : Inherit(d.getValue()), data(d) {}
    ~ReadAccessor() {}
};

template<class T>
class WriteAccessor< core::objectmodel::DataPtr<T> > : public WriteAccessor<T>
{
public:
    typedef WriteAccessor<T> Inherit;
    typedef core::objectmodel::DataPtr<T> data_container_type;
    typedef T container_type;
    typedef typename container_type::size_type size_type;
    typedef typename container_type::value_type value_type;
    typedef typename container_type::reference reference;
    typedef typename container_type::const_reference const_reference;
    typedef typename container_type::iterator iterator;
    typedef typename container_type::const_iterator const_iterator;

protected:
    data_container_type& data;

public:
    WriteAccessor(data_container_type& d) : Inherit(*d.beginEdit()), data(d) {}
    ~WriteAccessor() { data.endEdit(); }
};

} // namespace helper

} // namespace sofa

#endif
