/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
*                               SOFA :: Plugins                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/core/objectmodel/BaseContext.h>
using namespace sofa::core::objectmodel;

#include "Binding_BaseObject.h"
#include "Binding_Base.h"

extern "C" PyObject * BaseObject_setSrc(PyObject *self, PyObject * args)
{
    BaseObject* obj=dynamic_cast<BaseObject*>(((PySPtr<Base>*)self)->object.get());
    char *valueString;
    PyObject *pyLoader;
    if (!PyArg_ParseTuple(args, "sO",&valueString,&pyLoader))
    {
        PyErr_BadArgument();
        return 0;
    }
    BaseObject* loader=dynamic_cast<BaseObject*>(((PySPtr<Base>*)pyLoader)->object.get());
    obj->setSrc(valueString,loader);
    return Py_BuildValue("i",0);
}


SP_CLASS_METHODS_BEGIN(BaseObject)
SP_CLASS_METHOD(BaseObject,setSrc)
SP_CLASS_METHODS_END


SP_CLASS_TYPE_SPTR(BaseObject,BaseObject,Base)
