for n in 4 8 16 32; do
for scene in spring-euler spring-rk4 spring-implicit fem-implicit; do
echo
echo Scene:,$scene
echo Objects:,$n
echo -n Size:
for s in 2 4 6 8 10; do
echo -n ,$s
done
echo
for t in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16; do
echo -n $t
for s in 2 4 6 8 10; do
echo -n ,
if [ -f Bar${s}-${scene}-${n}-log-${t}.txt ]; then echo -n `awk '{ print $4 }' Bar${s}-${scene}-${n}-log-${t}.txt`; fi
done
echo
done
done
done
