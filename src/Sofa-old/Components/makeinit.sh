#!/bin/bash

cp -f init.cpp init.cpp~
awk ' /BEGIN CLASS LIST/ { print ; exit } { print }' init.cpp~ > init.cpp
find -name "*.cpp" -exec grep SOFA_DECL_CLASS \{\} \; | sed 's/SOFA_DECL_CLASS/SOFA_LINK_CLASS/' | sort >> init.cpp
