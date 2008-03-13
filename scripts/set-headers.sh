#!/bin/bash
find framework \( -iname '*.h' -o -iname '*.cpp' -o -iname '*.inl' -o -iname '*.c' -o -iname '*.cu' \) -exec scripts/set-header.sh \{\} scripts/header-LGPL.txt \;
find modules \( -iname '*.h' -o -iname '*.cpp' -o -iname '*.inl' -o -iname '*.c' -o -iname '*.cu' \) -exec scripts/set-header.sh \{\} scripts/header-LGPL.txt \;
find applications \( -iname '*.h' -o -iname '*.cpp' -o -iname '*.inl' -o -iname '*.c' -o -iname '*.cu' \) -exec scripts/set-header.sh \{\} scripts/header-GPL.txt \;
