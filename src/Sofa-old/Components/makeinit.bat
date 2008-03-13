set PATH=..\..\..\Tools\bin;%PATH%
copy init.cpp init.cpp~
gawk " /BEGIN CLASS LIST/ { print ; exit } { print }" init.cpp > init0.tmp
find -name "*.cpp" -exec grep SOFA_DECL_CLASS "{}" ";" > init1.tmp
sed "s/SOFA_DECL_CLASS/SOFA_LINK_CLASS/" init1.tmp > init2.tmp
sort init2.tmp > init3.tmp
copy init0.tmp+init3.tmp init.cpp
del init0.tmp init1.tmp init2.tmp init3.tmp
