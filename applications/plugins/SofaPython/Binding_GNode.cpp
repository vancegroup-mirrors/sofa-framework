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
#include <sofa/simulation/tree/GNode.h>
using namespace sofa::core::objectmodel;
using namespace sofa::simulation::tree;

#include "Binding_GNode.h"
#include "Binding_Node.h"

extern "C" PyObject * GNode_createChild(PyObject *self, PyObject * args)
{
    GNode* obj=dynamic_cast<GNode*>(((PySPtr<Base>*)self)->object.get());
    char *nodeName;
    if (!PyArg_ParseTuple(args, "s",&nodeName))
        return 0;
    return SP_BUILD_PYSPTR(obj->createChild(nodeName));
}

extern "C" PyObject * GNode_addChild(PyObject *self, PyObject * args)
{
    GNode* obj=dynamic_cast<GNode*>(((PySPtr<Base>*)self)->object.get());
    PyObject* pyChild;
    if (!PyArg_ParseTuple(args, "O",&pyChild))
        return 0;
    BaseNode* child=dynamic_cast<BaseNode*>(((PySPtr<Base>*)pyChild)->object.get());
    if (!child)
    {
        PyErr_BadArgument();
        return 0;
    }
    obj->addChild(child);
    return Py_BuildValue("i",0);
}

extern "C" PyObject * GNode_removeChild(PyObject *self, PyObject * args)
{
    GNode* obj=dynamic_cast<GNode*>(((PySPtr<Base>*)self)->object.get());
    PyObject* pyChild;
    if (!PyArg_ParseTuple(args, "O",&pyChild))
        return 0;
    BaseNode* child=dynamic_cast<BaseNode*>(((PySPtr<Base>*)pyChild)->object.get());
    if (!child)
    {
        PyErr_BadArgument();
        return 0;
    }
    obj->removeChild(child);
    return Py_BuildValue("i",0);
}

extern "C" PyObject * GNode_moveChild(PyObject *self, PyObject * args)
{
    GNode* obj=dynamic_cast<GNode*>(((PySPtr<Base>*)self)->object.get());
    PyObject* pyChild;
    if (!PyArg_ParseTuple(args, "O",&pyChild))
        return 0;
    BaseNode* child=dynamic_cast<BaseNode*>(((PySPtr<Base>*)pyChild)->object.get());
    if (!child)
    {
        PyErr_BadArgument();
        return 0;
    }
    obj->moveChild(child);
    return Py_BuildValue("i",0);
}

extern "C" PyObject * GNode_addObject(PyObject *self, PyObject * args)
{
    GNode* node=dynamic_cast<GNode*>(((PySPtr<Base>*)self)->object.get());
    PyObject* pyChild;
    if (!PyArg_ParseTuple(args, "O",&pyChild))
        return 0;
    BaseObject* object=dynamic_cast<BaseObject*>(((PySPtr<Base>*)pyChild)->object.get());
    if (!object)
    {
        PyErr_BadArgument();
        return 0;
    }
    node->addObject(object);
    return Py_BuildValue("i",0);
}

extern "C" PyObject * GNode_removeObject(PyObject *self, PyObject * args)
{
    GNode* node=dynamic_cast<GNode*>(((PySPtr<Base>*)self)->object.get());
    PyObject* pyChild;
    if (!PyArg_ParseTuple(args, "O",&pyChild))
        return 0;
    BaseObject* object=dynamic_cast<BaseObject*>(((PySPtr<Base>*)pyChild)->object.get());
    if (!object)
    {
        PyErr_BadArgument();
        return 0;
    }
    node->removeObject(object);
    return Py_BuildValue("i",0);
}

extern "C" PyObject * GNode_detachFromGraph(PyObject *self, PyObject * /*args*/)
{
    GNode* node=dynamic_cast<GNode*>(((PySPtr<Base>*)self)->object.get());
    node->detachFromGraph();
    return Py_BuildValue("i",0);
}



SP_CLASS_METHODS_BEGIN(GNode)
SP_CLASS_METHOD(GNode,createChild)
SP_CLASS_METHOD(GNode,addChild)
SP_CLASS_METHOD(GNode,removeChild)
SP_CLASS_METHOD(GNode,moveChild)
SP_CLASS_METHOD(GNode,addObject)
SP_CLASS_METHOD(GNode,removeObject)
SP_CLASS_METHOD(GNode,detachFromGraph)
SP_CLASS_METHODS_END

SP_CLASS_TYPE_SPTR(GNode,GNode,Node)