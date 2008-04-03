//
// C++ Interface: Shader
//
// Description: 
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2007
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef SOFA_CORE_SHADER_H
#define SOFA_CORE_SHADER_H


namespace sofa
{

namespace core
{

/**
 *  \brief A basic interface for the Shader different classes (OpenGL, ...).
 *
 *  
 *
 */
class Shader : public virtual objectmodel::BaseObject 
{
public:
	virtual ~Shader() { };

	virtual void start() = 0;

	virtual void stop() = 0;

};

} // namespace core

} // namespace sofa

#endif //SOFA_CORE_SHADER_H
