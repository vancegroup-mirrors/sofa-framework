#!/bin/bash
if [ $# -lt 8 ]; then
echo usage: $0 filebasename subdivlevel center_x center_y center_z radius_x radius_y radius_z
echo It will generate filebasename.obj as the surface
echo                  filebasename.stl as a gmsh readable surface
echo                  filebasename.msh as a gmsh script
echo                  filebasename-tetra.msh as a 3D tesselation
echo
echo Example to regenerate eye_lens_3: $0 eye_lens_3 3 -0.02829 5.01921 -3.629455 1.43296 1.43552 0.381505
exit
fi
#GMSH=gmsh
GMSH=/Applications/gmsh-*/Gmsh.app/Contents/MacOS/Gmsh
MESHCONV=meshconv

echo subdivide surface $2 times
$MESHCONV -S -a $2 icosahedron.obj $1.obj
echo transform mesh
$MESHCONV -s "$6 $7 $8" -t "$3 $4 $5" -n $1.obj $1.obj
echo convert to stl
$MESHCONV $1.obj $1.stl
echo create geo script
echo 'Merge "'$1'.stl";
Surface Loop(1) = {1};
Volume(1) = {1};
' > $1.geo
echo meshing...
$GMSH -format msh -3 $1.geo
echo Done!
