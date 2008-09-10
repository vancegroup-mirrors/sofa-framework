/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 3      *
*                (c) 2006-2008 MGH, INRIA, USTL, UJF, CNRS                    *
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
#include <sofa/component/visualmodel/OglVariable.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace visualmodel
{

/** SINGLE INT VARIABLE **/
SOFA_DECL_CLASS(OglIntVariable)
SOFA_DECL_CLASS(OglInt2Variable)
SOFA_DECL_CLASS(OglInt3Variable)
SOFA_DECL_CLASS(OglInt4Variable)

//Register OglIntVariable in the Object Factory
int OglIntVariableClass = core::RegisterObject("OglIntVariable")
        .add< OglIntVariable >()
        ;
//Register OglInt2Variable in the Object Factory
int OglInt2VariableClass = core::RegisterObject("OglInt2Variable")
        .add< OglInt2Variable >()
        ;
//Register OglInt3Variable in the Object Factory
int OglInt3VariableClass = core::RegisterObject("OglInt3Variable")
        .add< OglInt3Variable >()
        ;
//Register OglInt4Variable in the Object Factory
int OglInt4VariableClass = core::RegisterObject("OglInt4Variable")
        .add< OglInt4Variable >()
        ;

OglIntVariable::OglIntVariable()
    : value(initData(&value, (int) 0, "value", "Set a int value"))
{

}

OglInt2Variable::OglInt2Variable()
    : value(initData(&value, (defaulttype::Vec<2, int>) defaulttype::Vec<2, int>(0,0) , "value", "Set a Vec2i value"))
{

}

OglInt3Variable::OglInt3Variable()
    : value(initData(&value, (defaulttype::Vec<3, int>) defaulttype::Vec<3, int>(0,0,0) , "value", "Set a Vec3i value"))
{

}

OglInt4Variable::OglInt4Variable()
    : value(initData(&value, (defaulttype::Vec<4, int>) defaulttype::Vec<4, int>(0,0,0,0) , "value", "Set a Vec4i value"))
{

}

void OglIntVariable::initVisual()
{
    shader->setInt(indexShader.getValue(), id.getValue().c_str(), value.getValue());
}


void OglInt2Variable::initVisual()
{
    shader->setInt2(indexShader.getValue(), id.getValue().c_str(), value.getValue()[0], value.getValue()[1]);
}

void OglInt3Variable::initVisual()
{
    shader->setInt3(indexShader.getValue(), id.getValue().c_str(), value.getValue()[0], value.getValue()[1], value.getValue()[2]);
}

void OglInt4Variable::initVisual()
{
    shader->setInt4(indexShader.getValue(), id.getValue().c_str(), value.getValue()[0], value.getValue()[1], value.getValue()[2], value.getValue()[3]);
}

/** SINGLE FLOAT VARIABLE **/

SOFA_DECL_CLASS(OglFloatVariable)
SOFA_DECL_CLASS(OglFloat2Variable)
SOFA_DECL_CLASS(OglFloat3Variable)
SOFA_DECL_CLASS(OglFloat4Variable)

//Register OglFloatVariable in the Object Factory
int OglFloatVariableClass = core::RegisterObject("OglFloatVariable")
        .add< OglFloatVariable >()
        ;
//Register OglFloat2Variable in the Object Factory
int OglFloat2VariableClass = core::RegisterObject("OglFloat2Variable")
        .add< OglFloat2Variable >()
        ;
//Register OglFloat3Variable in the Object Factory
int OglFloat3VariableClass = core::RegisterObject("OglFloat3Variable")
        .add< OglFloat3Variable >()
        ;
//Register OglFloat4Variable in the Object Factory
int OglFloat4VariableClass = core::RegisterObject("OglFloat4Variable")
        .add< OglFloat4Variable >()
        ;

OglFloatVariable::OglFloatVariable()
    : value(initData(&value, (float) 0.0, "value", "Set a float value"))
{

}

OglFloat2Variable::OglFloat2Variable()
    : value(initData(&value, (defaulttype::Vec2f) defaulttype::Vec2f(0.0,0.0) , "value", "Set a Vec2f value"))
{

}

OglFloat3Variable::OglFloat3Variable()
    : value(initData(&value, (defaulttype::Vec3f) defaulttype::Vec3f(0.0,0.0,0.0) , "value", "Set a Vec3f value"))
{

}

OglFloat4Variable::OglFloat4Variable()
    : value(initData(&value, (defaulttype::Vec4f) defaulttype::Vec4f(0.0,0.0,0.0,0.0) , "value", "Set a Vec4f value"))
{

}

void OglFloatVariable::initVisual()
{
    shader->setFloat(indexShader.getValue(), id.getValue().c_str(), value.getValue());
}


void OglFloat2Variable::initVisual()
{
    shader->setFloat2(indexShader.getValue(), id.getValue().c_str(), value.getValue()[0], value.getValue()[1]);
}

void OglFloat3Variable::initVisual()
{
    shader->setFloat3(indexShader.getValue(), id.getValue().c_str(), value.getValue()[0], value.getValue()[1], value.getValue()[2]);
}

void OglFloat4Variable::initVisual()
{
    shader->setFloat4(indexShader.getValue(), id.getValue().c_str(), value.getValue()[0], value.getValue()[1], value.getValue()[2], value.getValue()[3]);
}


/** INT VECTOR VARIABLE **/
SOFA_DECL_CLASS(OglIntVectorVariable)
SOFA_DECL_CLASS(OglIntVector2Variable)
SOFA_DECL_CLASS(OglIntVector3Variable)
SOFA_DECL_CLASS(OglIntVector4Variable)

//Register OglIntVectorVariable in the Object Factory
int OglIntVectorVariableClass = core::RegisterObject("OglIntVectorVariable")
        .add< OglIntVectorVariable >()
        ;

//Register OglIntVector2Variable in the Object Factory
int OglIntVector2VariableClass = core::RegisterObject("OglIntVector2Variable")
        .add< OglIntVector2Variable >()
        ;

//Register OglIntVector3Variable in the Object Factory
int OglIntVector3VariableClass = core::RegisterObject("OglIntVector3Variable")
        .add< OglIntVector3Variable >()
        ;

//Register OglIntVector4Variable in the Object Factory
int OglIntVector4VariableClass = core::RegisterObject("OglIntVector4Variable")
        .add< OglIntVector4Variable >()
        ;

OglIntVectorVariable::OglIntVectorVariable()
    : iv(initData(&iv, (helper::vector<GLint>) helper::vector<GLint>(1), "values", "Set int array values"))
{

}

OglIntVector2Variable::OglIntVector2Variable()
{

}

OglIntVector3Variable::OglIntVector3Variable()
{

}

OglIntVector4Variable::OglIntVector4Variable()
{

}


void OglIntVectorVariable::init()
{
    OglVariable::init();
}

void OglIntVector2Variable::init()
{
    OglIntVectorVariable::init();
    helper::vector<GLint> temp = iv.getValue();
    if (iv.getValue().size() %2 != 0)
    {
        std::cerr << "The number of values is not even ; padding with one zero" << std::endl;
        temp.push_back(0);
        iv.setValue(temp);

    }
}

void OglIntVector3Variable::init()
{
    OglIntVectorVariable::init();
    helper::vector<GLint> temp = iv.getValue();

    if (iv.getValue().size() %3 != 0)
    {
        std::cerr << "The number of values is not a multiple of 3 ; padding with zero(s)" << std::endl;
        while (iv.getValue().size() %3 != 0)
            temp.push_back(0);
        iv.setValue(temp);
    }
}

void OglIntVector4Variable::init()
{
    OglIntVectorVariable::init();
    helper::vector<GLint> temp = iv.getValue();

    if (iv.getValue().size() %4 != 0)
    {
        std::cerr << "The number of values is not a multiple of 4 ; padding with zero(s)" << std::endl;
        while (iv.getValue().size() %4 != 0)
            temp.push_back(0);
        iv.setValue(temp);
    }
}

void OglIntVectorVariable::initVisual()
{
    shader->setIntVector(indexShader.getValue(), id.getValue().c_str(), iv.getValue().size(), &(iv.getValue()[0]));
}

void OglIntVector2Variable::initVisual()
{
    shader->setIntVector2(indexShader.getValue(), id.getValue().c_str(), iv.getValue().size()/2, &(iv.getValue()[0]));
}

void OglIntVector3Variable::initVisual()
{
    shader->setIntVector3(indexShader.getValue(), id.getValue().c_str(), iv.getValue().size()/3, &(iv.getValue()[0]));
}

void OglIntVector4Variable::initVisual()
{
    shader->setIntVector4(indexShader.getValue(), id.getValue().c_str(), iv.getValue().size()/4, &(iv.getValue()[0]));
}


/** FLOAT VECTOR VARIABLE **/
SOFA_DECL_CLASS(OglFloatVectorVariable)
SOFA_DECL_CLASS(OglFloatVector2Variable)
SOFA_DECL_CLASS(OglFloatVector3Variable)
SOFA_DECL_CLASS(OglFloatVector4Variable)

//Register OglFloatVectorVariable in the Object Factory
int OglFloatVectorVariableClass = core::RegisterObject("OglFloatVectorVariable")
        .add< OglFloatVectorVariable >()
        ;

//Register OglFloatVector2Variable in the Object Factory
int OglFloatVector2VariableClass = core::RegisterObject("OglFloatVector2Variable")
        .add< OglFloatVector2Variable >()
        ;

//Register OglFloatVector3Variable in the Object Factory
int OglFloatVector3VariableClass = core::RegisterObject("OglFloatVector3Variable")
        .add< OglFloatVector3Variable >()
        ;
//Register OglFloatVector4Variable in the Object Factory
int OglFloatVector4VariableClass = core::RegisterObject("OglFloatVector4Variable")
        .add< OglFloatVector4Variable >()
        ;

OglFloatVectorVariable::OglFloatVectorVariable()
    : fv(initData(&fv, (helper::vector<float>) helper::vector<float>(1), "values", "Set float array values"))
{

}

OglFloatVector2Variable::OglFloatVector2Variable()
{

}

OglFloatVector3Variable::OglFloatVector3Variable()
{

}

OglFloatVector4Variable::OglFloatVector4Variable()
{

}


void OglFloatVectorVariable::init()
{
    OglVariable::init();
}

void OglFloatVector2Variable::init()
{
    OglFloatVectorVariable::init();
    helper::vector<float> temp = fv.getValue();
    if (fv.getValue().size() %2 != 0)
    {
        std::cerr << "The number of values is not even ; padding with one zero" << std::endl;
        temp.push_back(0.0);
        fv.setValue(temp);
    }
}

void OglFloatVector3Variable::init()
{
    OglFloatVectorVariable::init();
    helper::vector<float> temp = fv.getValue();

    if (fv.getValue().size() %3 != 0)
    {
        std::cerr << "The number of values is not a multiple of 3 ; padding with zero(s)" << std::endl;
        while (fv.getValue().size() %3 != 0)
            temp.push_back(0.0);
        fv.setValue(temp);
    }
}

void OglFloatVector4Variable::init()
{
    OglFloatVectorVariable::init();
    helper::vector<float> temp = fv.getValue();

    if (fv.getValue().size() %4 != 0)
    {
        std::cerr << "The number of values is not a multiple of 4 ; padding with zero(s)" << std::endl;
        while (fv.getValue().size() %4 != 0)
            temp.push_back(0.0);
        fv.setValue(temp);
    }
}

void OglFloatVectorVariable::initVisual()
{
    shader->setFloatVector(indexShader.getValue(), id.getValue().c_str(), fv.getValue().size(), &(fv.getValue()[0]));
}

void OglFloatVector2Variable::initVisual()
{
    shader->setFloatVector2(indexShader.getValue(), id.getValue().c_str(), fv.getValue().size()/2, &(fv.getValue()[0]));
}

void OglFloatVector3Variable::initVisual()
{
    shader->setFloatVector3(indexShader.getValue(), id.getValue().c_str(), fv.getValue().size()/3, &(fv.getValue()[0]));
}

void OglFloatVector4Variable::initVisual()
{
    shader->setFloatVector4(indexShader.getValue(), id.getValue().c_str(), fv.getValue().size()/4, &(fv.getValue()[0]));
}


}

}

}
