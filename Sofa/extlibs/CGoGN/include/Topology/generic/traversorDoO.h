/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* version 0.1                                                                  *
* Copyright (C) 2009-2012, IGG Team, LSIIT, University of Strasbourg           *
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
* Web site: http://cgogn.unistra.fr/                                           *
* Contact information: cgogn@unistra.fr                                        *
*                                                                              *
*******************************************************************************/

#ifndef __TRAVERSOR_DOO_H__
#define __TRAVERSOR_DOO_H__

#include "Topology/generic/dart.h"
#include "Topology/generic/traversorGen.h"

namespace CGoGN
{

template <typename MAP, unsigned int ORBIT>
class TraversorDartsOfOrbit
{
private:
	std::vector<Dart>::iterator m_current ;
	std::vector<Dart> m_vd ;

public:
	TraversorDartsOfOrbit(const MAP& map, Dart d, unsigned int thread = 0) ;
    TraversorDartsOfOrbit(const TraversorDartsOfOrbit& tradoo);

    inline Dart begin() ;

    inline const Dart& end() {return NIL;}

    inline Dart next() ;

    bool applyFunctor(FunctorType& f)
    {
        for (Dart d = begin(); d != end(); d = next())
        {
                if (f(d))
                    return true;
        }
        return false;
    }
} ;



template <typename MAP, unsigned int ORBIT>
class VTraversorDartsOfOrbit : public Traversor
{
private:
	std::vector<Dart>::iterator m_current ;
	std::vector<Dart> m_vd ;

public:
	VTraversorDartsOfOrbit(const MAP& map, Dart d, unsigned int thread = 0) ;

	Dart begin() ;

	Dart end() ;

	Dart next() ;
} ;



} // namespace CGoGN

#include "Topology/generic/traversorDoO.hpp"

#endif
