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

namespace CGoGN
{

/***************************************************
 *             ATTRIBUTES MANAGEMENT               *
 ***************************************************/

template <typename T, unsigned int ORBIT>
AttributeHandler_IHM<T, ORBIT> ImplicitHierarchicalMap2::addAttribute(const std::string& nameAttr)
{
	bool addNextLevelCell = false ;
	if(!isOrbitEmbedded<ORBIT>())
		addNextLevelCell = true ;

	AttributeHandler<T, ORBIT> h = Map2::addAttribute<T, ORBIT>(nameAttr) ;

	if(addNextLevelCell)
	{
		AttributeContainer& cellCont = m_attribs[ORBIT] ;
		AttributeMultiVector<unsigned int>* amv = cellCont.addAttribute<unsigned int>("nextLevelCell") ;
		m_nextLevelCell[ORBIT] = amv ;
		for(unsigned int i = cellCont.begin(); i < cellCont.end(); cellCont.next(i))
			amv->operator[](i) = EMBNULL ;
	}

	return AttributeHandler_IHM<T, ORBIT>(this, h.getDataVector()) ;
}

template <typename T, unsigned int ORBIT>
AttributeHandler_IHM<T, ORBIT> ImplicitHierarchicalMap2::getAttribute(const std::string& nameAttr)
{
	AttributeHandler<T, ORBIT> h = Map2::getAttribute<T, ORBIT>(nameAttr) ;
	return AttributeHandler_IHM<T, ORBIT>(this, h.getDataVector()) ;
}


inline void ImplicitHierarchicalMap2::update_topo_shortcuts()
{
	Map2::update_topo_shortcuts();
	m_dartLevel = Map2::getAttribute<unsigned int, DART>("dartLevel") ;
	m_edgeId = Map2::getAttribute<unsigned int, DART>("edgeId") ;

	//AttributeContainer& cont = m_attribs[DART] ;
	//m_nextLevelCell = cont.getDataVector<unsigned int>(cont.getAttributeIndex("nextLevelCell")) ;
}

/***************************************************
 *                 MAP TRAVERSAL                   *
 ***************************************************/

inline Dart ImplicitHierarchicalMap2::newDart()
{
	Dart d = Map2::newDart() ;
	m_dartLevel[d] = m_curLevel ;
	if(m_curLevel > m_maxLevel)			// update max level
		m_maxLevel = m_curLevel ;		// if needed
	return d ;
}

inline Dart ImplicitHierarchicalMap2::phi1(Dart d) const
{
	assert(m_dartLevel[d] <= m_curLevel || !"Access to a dart introduced after current level") ;
    bool finished = false ;
	unsigned int edgeId = m_edgeId[d] ;
	Dart it = d ;
	do
	{
		it = Map2::phi1(it) ;
		if(m_dartLevel[it] <= m_curLevel)
			finished = true ;
		else
		{
			while(m_edgeId[it] != edgeId)
                it = Map2::phi1(Map2::phi2(it)) ;
		}
	} while(!finished) ;
	return it ;
}

inline Dart ImplicitHierarchicalMap2::phi_1(Dart d) const
{
	assert(m_dartLevel[d] <= m_curLevel || !"Access to a dart introduced after current level") ;
	bool finished = false ;
	Dart it = Map2::phi_1(d) ;
	unsigned int edgeId = m_edgeId[it] ;
	do
	{
		if(m_dartLevel[it] <= m_curLevel)
			finished = true ;
		else
		{
			it = Map2::phi_1(it) ;
			while(m_edgeId[it] != edgeId)
				it = Map2::phi_1(Map2::phi2(it)) ;
		}
	} while(!finished) ;
	return it ;
}

inline Dart ImplicitHierarchicalMap2::phi2(Dart d) const
{
	assert(m_dartLevel[d] <= m_curLevel || !"Access to a dart introduced after current level") ;
	if(Map2::phi2(d) == d)
		return d ;
    return Map2::phi2(Map2::phi_1(phi1(d))) ;
}

inline Dart ImplicitHierarchicalMap2::alpha0(Dart d) const
{
	return phi2(d) ;
}

inline Dart ImplicitHierarchicalMap2::alpha1(Dart d) const
{
	return Map2::alpha1(d) ;
}

inline Dart ImplicitHierarchicalMap2::alpha_1(Dart d) const
{
	return Map2::alpha_1(d) ;
}

inline Dart ImplicitHierarchicalMap2::begin() const
{
	Dart d = Map2::begin() ;
	while(d != Map2::end() && m_dartLevel[d] > m_curLevel)
		Map2::next(d) ;
	return d ;
}

inline Dart ImplicitHierarchicalMap2::end() const
{
	return Map2::end() ;
}

inline void ImplicitHierarchicalMap2::next(Dart& d) const
{
	do
	{
		Map2::next(d) ;
	} while(d != Map2::end() && m_dartLevel[d] > m_curLevel) ;
}

inline bool ImplicitHierarchicalMap2::foreach_dart_of_vertex(Dart d, FunctorType& f, unsigned int /*thread*/) const
{
	Dart dNext = d;
	do
	{
		if (f(dNext))
			return true;
		dNext = alpha1(dNext);
 	} while (dNext != d);
 	return false;
}

inline bool ImplicitHierarchicalMap2::foreach_dart_of_edge(Dart d, FunctorType& f, unsigned int /*thread*/) const
{
	if (f(d))
		return true;

	Dart d2 = phi2(d);
	if (d2 != d)
		return f(d2);
	else
		return false;
}

inline bool ImplicitHierarchicalMap2::foreach_dart_of_oriented_face(Dart d, FunctorType& f, unsigned int /*thread*/) const
{
	Dart dNext = d ;
	do
	{
		if (f(dNext))
			return true ;
		dNext = phi1(dNext) ;
	} while (dNext != d) ;
	return false ;
}

inline bool ImplicitHierarchicalMap2::foreach_dart_of_face(Dart d, FunctorType& f, unsigned int thread) const
{
	return foreach_dart_of_oriented_face(d, f, thread) ;
}

inline bool ImplicitHierarchicalMap2::foreach_dart_of_oriented_volume(Dart d, FunctorType& f, unsigned int thread) const
{
	DartMarkerStore mark(*this, thread);	// Lock a marker
	bool found = false;				// Last functor return value

	std::list<Dart> visitedFaces;	// Faces that are traversed
	visitedFaces.push_back(d);		// Start with the face of d
	std::list<Dart>::iterator face;

	// For every face added to the list
	for (face = visitedFaces.begin(); !found && face != visitedFaces.end(); ++face)
	{
		if (!mark.isMarked(*face))		// Face has not been visited yet
		{
			// Apply functor to the darts of the face
			found = foreach_dart_of_oriented_face(*face, f, thread);

			// If functor returns false then mark visited darts (current face)
			// and add non visited adjacent faces to the list of face
			if (!found)
			{
				Dart dNext = *face ;
				do
				{
					mark.mark(dNext);					// Mark
					Dart adj = phi2(dNext);				// Get adjacent face
					if (adj != dNext && !mark.isMarked(adj))
						visitedFaces.push_back(adj);	// Add it
					dNext = phi1(dNext);
				} while(dNext != *face);
			}
		}
	}
	return found;
}

inline bool ImplicitHierarchicalMap2::foreach_dart_of_volume(Dart d, FunctorType& f, unsigned int thread) const
{
	return foreach_dart_of_oriented_volume(d, f, thread) ;
}

inline bool ImplicitHierarchicalMap2::foreach_dart_of_cc(Dart d, FunctorType& f, unsigned int thread) const
{
	return foreach_dart_of_oriented_volume(d, f, thread) ;
}

/***************************************************
 *               MAP MANIPULATION                  *
 ***************************************************/

inline void ImplicitHierarchicalMap2::splitFace(Dart d, Dart e)
{
	EmbeddedMap2::splitFace(d, e) ;
	if(isOrbitEmbedded<FACE>())
	{
		unsigned int cur = m_curLevel ;
		m_curLevel = m_maxLevel ;
		this->setOrbitEmbedding<FACE>(d, this->getEmbedding<FACE>(d)) ;
		this->setOrbitEmbedding<FACE>(e, this->getEmbedding<FACE>(e)) ;
		m_curLevel = cur ;
	}
}

/***************************************************
 *              LEVELS MANAGEMENT                  *
 ***************************************************/

inline unsigned int ImplicitHierarchicalMap2::getCurrentLevel()
{
	return m_curLevel ;
}

inline void ImplicitHierarchicalMap2::setCurrentLevel(unsigned int l)
{
	m_curLevel = l ;
}

inline void ImplicitHierarchicalMap2::incCurrentLevel()
{
	if(m_curLevel < m_maxLevel)
		++m_curLevel ;
	else
		CGoGNout << "incCurrentLevel : already at maximum resolution level" << CGoGNendl ;
}

inline void ImplicitHierarchicalMap2::decCurrentLevel()
{
	if(m_curLevel > 0)
		--m_curLevel ;
	else
		CGoGNout << "decCurrentLevel : already at minimum resolution level" << CGoGNendl ;
}

inline unsigned int ImplicitHierarchicalMap2::getMaxLevel()
{
	return m_maxLevel ;
}

inline unsigned int ImplicitHierarchicalMap2::getDartLevel(Dart d)
{
	return m_dartLevel[d] ;
}

inline void ImplicitHierarchicalMap2::setDartLevel(Dart d, unsigned int l)
{
	m_dartLevel[d] = l ;
}

inline void ImplicitHierarchicalMap2::setMaxLevel(unsigned int l)
{
	m_maxLevel = l;
}


/***************************************************
 *             EDGE ID MANAGEMENT                  *
 ***************************************************/

inline unsigned int ImplicitHierarchicalMap2::getNewEdgeId()
{
	return m_idCount++ ;
}

inline unsigned int ImplicitHierarchicalMap2::getEdgeId(Dart d)
{
	return m_edgeId[d] ;
}

inline void ImplicitHierarchicalMap2::setEdgeId(Dart d, unsigned int i)
{
	m_edgeId[d] = i ;
}

inline unsigned int ImplicitHierarchicalMap2::getTriRefinementEdgeId(Dart d)
{
	unsigned int dId = getEdgeId(phi_1(d));
	unsigned int eId = getEdgeId(phi1(d));

	unsigned int id = dId + eId;

	if(id == 0)
		return 1;
	else if(id == 1)
		return 2;
	else if(id == 2)
	{
		if(dId == eId)
			return 0;
		else
			return 1;
	}

	//else if(id == 3)
	return 0;
}

inline unsigned int ImplicitHierarchicalMap2::getQuadRefinementEdgeId(Dart d)
{
	unsigned int eId = getEdgeId(phi1(d));

	if(eId == 0)
		return 1;

	//else if(eId == 1)
	return 0;
}

/***************************************************
 *               CELLS INFORMATION                 *
 ***************************************************/

inline unsigned int ImplicitHierarchicalMap2::faceDegree(Dart d)
{
    unsigned int count = 0 ;
    Dart it = d ;
    do
    {
        ++count ;
        it = phi1(it) ;
    } while (it != d) ;
    return count ;
}



inline unsigned int ImplicitHierarchicalMap2::vertexInsertionLevel(Dart d)
{
	assert(m_dartLevel[d] <= m_curLevel || !"Access to a dart introduced after current level") ;
	return m_dartLevel[d] ;
}

//inline unsigned int ImplicitHierarchicalMap2::edgeLevel(Dart d)
//{
//	assert(m_dartLevel[d] <= m_curLevel || !"Access to a dart introduced after current level") ;
//	unsigned int ld = m_dartLevel[d] ;
////	unsigned int ldd = m_dartLevel[phi2(d)] ;	// the level of an edge is the maximum of the
//	unsigned int ldd = m_dartLevel[phi1(d)] ;
//	return ld < ldd ? ldd : ld ;				// insertion levels of its two darts
//}

/***************************************************
 *               ATTRIBUTE HANDLER                 *
 ***************************************************/

template <typename T, unsigned int ORBIT>
T& AttributeHandler_IHM<T, ORBIT>::operator[](Dart d)
{
	ImplicitHierarchicalMap2* m = reinterpret_cast<ImplicitHierarchicalMap2*>(this->m_map) ;
	assert(m->m_dartLevel[d] <= m->m_curLevel || !"Access to a dart introduced after current level") ;
	assert(m->vertexInsertionLevel(d) <= m->m_curLevel || !"Access to the embedding of a vertex inserted after current level") ;

	unsigned int orbit = this->getOrbit() ;
	unsigned int nbSteps = m->m_curLevel - m->vertexInsertionLevel(d) ;
	unsigned int index = m->getEmbedding<ORBIT>(d) ;

	if(index == EMBNULL)
	{
		index = m->setOrbitEmbeddingOnNewCell<ORBIT>(d) ;
		m->m_nextLevelCell[orbit]->operator[](index) = EMBNULL ;
	}

	AttributeContainer& cont = m->getAttributeContainer<ORBIT>() ;
	unsigned int step = 0 ;
	while(step < nbSteps)
	{
		step++ ;
		unsigned int nextIdx = m->m_nextLevelCell[orbit]->operator[](index) ;
		if (nextIdx == EMBNULL)
		{
			nextIdx = m->newCell<ORBIT>() ;
			m->copyCell<ORBIT>(nextIdx, index) ;
			m->m_nextLevelCell[orbit]->operator[](index) = nextIdx ;
			m->m_nextLevelCell[orbit]->operator[](nextIdx) = EMBNULL ;
			cont.refLine(index) ;
		}
		index = nextIdx ;
	}
	return this->m_attrib->operator[](index);
}

template <typename T, unsigned int ORBIT>
const T& AttributeHandler_IHM<T, ORBIT>::operator[](Dart d) const
{
	ImplicitHierarchicalMap2* m = reinterpret_cast<ImplicitHierarchicalMap2*>(this->m_map) ;
	assert(m->m_dartLevel[d] <= m->m_curLevel || !"Access to a dart introduced after current level") ;
	assert(m->vertexInsertionLevel(d) <= m->m_curLevel || !"Access to the embedding of a vertex inserted after current level") ;

	unsigned int orbit = this->getOrbit() ;
	unsigned int nbSteps = m->m_curLevel - m->vertexInsertionLevel(d) ;
	unsigned int index = m->getEmbedding<ORBIT>(d) ;

	unsigned int step = 0 ;
	while(step < nbSteps)
	{
		step++ ;
		unsigned int next = m->m_nextLevelCell[orbit]->operator[](index) ;
		if(next != EMBNULL) index = next ;
		else break ;
	}
	return this->m_attrib->operator[](index);
}


} //namespace CGoGN
