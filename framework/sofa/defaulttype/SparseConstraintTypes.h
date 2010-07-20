 
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
#ifndef SOFA_DEFAULTTYPE_SPARSECONSTRAINTTYPES_H
#define SOFA_DEFAULTTYPE_SPARSECONSTRAINTTYPES_H

#include <map>

namespace sofa
{

  namespace defaulttype
  {

    /// Data Structure to store lines of the matrix L.
    template <class T>
      class SparseConstraint
      {
        typedef std::map< unsigned int, T> data_t;
      public:    
        typedef typename data_t::const_iterator const_data_iterator;
        typedef typename data_t::iterator data_iterator;
        typedef T Data;   
        
      public:
        void add( unsigned int index, const T &value)
        {
          _data[index] += value;
        }

        void set( unsigned int index, const T &value)
        {
          _data[index] = value;
        }

        T& operator[] (unsigned int index)
        {
            return _data[index];
        }

        const T& operator[] (unsigned int index) const
        {
            return _data[index];
        }

        const_data_iterator find( unsigned int index) const
        {
          return _data.find(index);
        }

        data_iterator find( unsigned int index)
        {
          return _data.find(index);
        }


        void clear()
        {
          _data.clear();
        }

        //Iterators        
        std::pair< const_data_iterator,
          const_data_iterator> data() const
            {
              return std::make_pair(_data.begin(), _data.end());
            }
        
        std::pair< data_iterator,
          data_iterator> data()
            {
              return std::make_pair(_data.begin(), _data.end());
            }

		bool empty() const { return _data.empty(); }

		 /// write to an output stream
		inline friend std::ostream& operator << ( std::ostream& out, const SparseConstraint<T>& /*sc*/ )
		{
			return out;
		}

		/// read from an input stream
		inline friend std::istream& operator >> ( std::istream& in, SparseConstraint<T>& /*sc*/ )
		{
			return in;
		}

      protected:
        data_t _data;
      };

  } // namespace defaulttype

} // namespace sofa

#endif
