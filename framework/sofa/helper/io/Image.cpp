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
#include <sofa/helper/io/Image.h>
#include <sofa/helper/Factory.inl>

namespace sofa
{

namespace helper
{

template class Factory<std::string, sofa::helper::io::Image, std::string>;

namespace io
{



Image::Image()
: width(0), height(0), nbBits(0), data(NULL)
{
}

Image::~Image()
{
	if (data) free(data);
}

void Image::init(int w, int h, int nbb)
{
	clear();
	width = w;
	height = h;
	nbBits = nbb;
	data = (unsigned char*) malloc(((nbb+7)/8)*width*height);
}

void Image::clear()
{
	if (data!=NULL) free(data);
	width = 0;
	height = 0;
	nbBits = 0;
	data = NULL;
}

Image* Image::Create(std::string filename)
{
	std::string loader="default";
	std::string::size_type p = filename.rfind('.');
	if (p!=std::string::npos)
		loader = std::string(filename, p+1);
	return Factory::CreateObject(loader, filename);
}

} // namespace io

} // namespace helper

} // namespace sofa

