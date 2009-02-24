#!/bin/bash
if [ $# -lt 12 ]; then
echo usage: $0 filebasename subdivlevel_in subdivlevel_out center_x center_y center_z radius_in_x radius_in_y radius_in_z radius_out_x radius_out_y radius_out_z center_factor
echo It will generate filebasename-{in,out}.obj as the surface
echo                  filebasename-{in,out}.stl as a gmsh readable surface
echo                  filebasename.geo as a gmsh script
echo                  filebasename.msh as a 3D tesselation
echo
echo Example to regenerate eye_lens_cortex_3:
echo ./gen-lens-cortex.sh eye_lens_cortex_3 3 3 -0.02829 5.01921 -3.629455 1.23 1.23 0.26 1.433 1.433 0.38 2
echo ./gen-ellipsoid.sh eye_lens_kernel_3 3 -0.02829 5.01921 -3.629455 1.23 1.23 0.26 2
exit
fi
if [ $(uname) == "Darwin" ]; then
GMSH=/Applications/gmsh-*/Gmsh.app/Contents/MacOS/Gmsh
else
GMSH=gmsh
fi
MESHCONV=meshconv

BASE=$1
shift
IN_LEV=$1
shift
OUT_LEV=$1
shift
CENTER="$1 $2 $3"
shift 3
IN_R="$1 $2 $3"
shift 3
OUT_R="$1 $2 $3"
shift 3
CFACTOR=${1:-1}
shift

echo == INSIDE ==

echo subdivide surface $IN_LEV times
echo $MESHCONV -S -a $IN_LEV icosahedron.obj $BASE-in.obj
$MESHCONV -S -a $IN_LEV icosahedron.obj $BASE-in.obj
echo transform mesh
echo $MESHCONV -s "1 1 $CFACTOR" -S -n $BASE-in.obj $BASE-in.obj
$MESHCONV -s "1 1 $CFACTOR" -S -n $BASE-in.obj $BASE-in.obj
echo $MESHCONV -s "$IN_R" -t "$CENTER" -n $BASE-in.obj $BASE-in.obj
$MESHCONV -s "$IN_R" -t "$CENTER" -n $BASE-in.obj $BASE-in.obj
echo convert to stl
echo $MESHCONV $BASE-in.obj $BASE-in.stl
$MESHCONV $BASE-in.obj $BASE-in.stl

echo == OUSIDE ==

echo subdivide surface $OUT_LEV times
echo $MESHCONV -S -a $OUT_LEV icosahedron.obj $BASE-out.obj
$MESHCONV -S -a $OUT_LEV icosahedron.obj $BASE-out.obj
echo transform mesh
echo $MESHCONV -s "1 1 $CFACTOR" -S -n $BASE-out.obj $BASE-out.obj
$MESHCONV -s "1 1 $CFACTOR" -S -n $BASE-out.obj $BASE-out.obj
echo $MESHCONV -s "$OUT_R" -t "$CENTER" -n $BASE-out.obj $BASE-out.obj
$MESHCONV -s "$OUT_R" -t "$CENTER" -n $BASE-out.obj $BASE-out.obj
echo convert to stl
echo $MESHCONV $BASE-out.obj $BASE-out.stl
$MESHCONV $BASE-out.obj $BASE-out.stl

echo == VOLUME ==

echo create geo script
echo 'Merge "'$BASE'-in.stl";
Merge "'$BASE'-out.stl";
Surface Loop(1) = {-1};
Surface Loop(2) = {2};
Volume(1) = {1,2};
' > $BASE.geo
echo meshing...
echo $GMSH -format msh -3 $BASE.geo
$GMSH -format msh -3 $BASE.geo
echo Done!
