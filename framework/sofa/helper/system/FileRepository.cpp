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
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/SetDirectory.h>

#include <sys/types.h>
#include <sys/stat.h>
#ifndef WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <direct.h>
#endif

#include <string.h>
#include <stdlib.h>
#include <iostream>

namespace sofa
{

namespace helper
{

namespace system
{

#if defined (WIN32)
FileRepository DataRepository("SOFA_DATA_PATH", "../share;../examples");
#elif defined (__APPLE__)
FileRepository DataRepository("SOFA_DATA_PATH", "../share:../examples:../Resources/examples:../Resources:../../../../examples:../../../../share");
#else
FileRepository DataRepository("SOFA_DATA_PATH", "../share:../examples");
#endif

FileRepository::FileRepository(const char* envVar, const char* relativePath)
{
    if (envVar != NULL && envVar[0]!='\0')
    {
        const char* envpath = getenv(envVar);
        if (envpath != NULL && envpath[0]!='\0')
            addFirstPath(envpath);
    }
    if (relativePath != NULL && relativePath[0]!='\0')
    {
        std::string path = relativePath;
        size_t p0 = 0;
        size_t p1;
        while ( p0 < path.size() )
        {
            p1 = path.find(entrySeparator(),p0);
            if (p1 == std::string::npos) p1 = path.size();
            if (p1>p0+1)
            {
                std::string p = path.substr(p0,p1-p0);
                addLastPath(SetDirectory::GetRelativeFromProcess(p.c_str()));
            }
            p0 = p1+1;
        }
    }
    //print();
}

FileRepository::~FileRepository()
{
}

void FileRepository::addFirstPath(const std::string& path)
{
    std::vector<std::string> entries;
    size_t p0 = 0;
    size_t p1;
    while ( p0 < path.size() )
    {
        p1 = path.find(entrySeparator(),p0);
        if (p1 == std::string::npos) p1 = path.size();
        if (p1>p0+1)
        {
            entries.push_back(path.substr(p0,p1-p0));
        }
        p0 = p1+1;
    }
    vpath.insert(vpath.begin(), entries.begin(), entries.end());
}

void FileRepository::addLastPath(const std::string& path)
{
    std::vector<std::string> entries;
    size_t p0 = 0;
    size_t p1;
    while ( p0 < path.size() )
    {
        p1 = path.find(entrySeparator(),p0);
        if (p1 == std::string::npos) p1 = path.size();
        if (p1>p0+1)
        {
            entries.push_back(path.substr(p0,p1-p0));
        }
        p0 = p1+1;
    }
    vpath.insert(vpath.end(), entries.begin(), entries.end());
//     std::cout << path << std::endl;
}

std::string FileRepository::getFirstPath()
{
    if (vpath.size() > 0)
        return vpath[0];
    else return "";
}

bool FileRepository::findFileIn(std::string& filename, const std::string& path)
{
    if (filename.empty()) return false; // no filename
    struct stat s;
    std::string newfname = SetDirectory::GetRelativeFromDir(filename.c_str(), path.c_str());
    //std::cout << "Looking for " << newfname <<std::endl;
    if (!stat(newfname.c_str(),&s))
    {
        // File found
        //std::cout << "File "<<filename<<" found in "<<path.substr(p0,p1-p0)<<std::endl;
        filename = newfname;
        return true;
    }
    return false;
}

bool FileRepository::findFile(std::string& filename, const std::string& basedir, std::ostream* errlog)
{
    if (filename.empty()) return false; // no filename
    if (basedir.empty())
    {
        if (findFileIn(filename, SetDirectory::GetCurrentDir())) return true;
    }
    else
    {
        if (findFileIn(filename, SetDirectory::GetRelativeFromDir(basedir.c_str(),SetDirectory::GetCurrentDir().c_str()))) return true;
    }
    if (SetDirectory::IsAbsolute(filename)) return false; // absolute file path
    if (filename.substr(0,2)=="./" || filename.substr(0,3)=="../") return false; // local file path
    for (std::vector<std::string>::const_iterator it = vpath.begin(); it != vpath.end(); ++it)
        if (findFileIn(filename, *it)) return true;
    if (errlog)
    {
        (*errlog) << "File "<<filename<<" NOT FOUND in "<<basedir;
        for (std::vector<std::string>::const_iterator it = vpath.begin(); it != vpath.end(); ++it)
            (*errlog) << ':'<<*it;
        (*errlog)<<std::endl;
    }
    return false;
}

bool FileRepository::findFileFromFile(std::string& filename, const std::string& basefile, std::ostream* errlog)
{
    return findFile(filename, SetDirectory::GetParentDir(basefile.c_str()), errlog);
}

void FileRepository::print()
{
    for (std::vector<std::string>::const_iterator it = vpath.begin(); it != vpath.end(); ++it)
        std::cout << *it << std::endl;
}

} // namespace system

} // namespace helper

} // namespace sofa

