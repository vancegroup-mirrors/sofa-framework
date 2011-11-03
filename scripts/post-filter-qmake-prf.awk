#!/usr/bin/awk -f
# This script removes sub-projects in a dependencies-based qmake project file
# corresponding to files that are no longer present (removed by other scripts)

# If you never used awk, a good introduction is available at http://www.cs.hmc.edu/qref/awk.html

# Usage: post-filter-qmake-prf.awk -v features=features pass=1 myfile.prf pass=2 myfile.prf

# init
BEGIN {
    FS=","
    OFS=","
}

END {
    if (blk_after != 0) print "# ERROR: unmatched brackets"
}

/^[ \t#]*declare/ {
  name=$1;
  gsub(/.*\(/,"",name); gsub(/ /,"",name); gsub(/\t/,"",name);
  path=$2;
  gsub(/).*$/,"",path); gsub(/ /,"",path); gsub(/\t/,"",path);
  deps=$3;
  gsub(/).*$/,"",deps); gsub("\t"," ",deps); gsub(/^[ ]*/,"",deps); gsub(/[ ]*$/,"",deps);
  #print "#PROJECT <<<" name "|" path "|" deps ">>>" > "/dev/stderr";
  projects[name] = path;
  if (system("test -e " path) != 0 && (!features || system("test -e " features "/sofa/" path ) != 0) )
  {
      if (pass != 2) print "# " name ": PATH " path " NOT FOUND" > "/dev/stderr";
      projMissing[name] = path;
      next;
  }
  else
      projFound[name] = path;
  if (pass != 1)
  {
      split(deps,deparray, " ");
      for (d in deparray)
      {
          dep = deparray[d];
          if (!(dep in projFound))
          {
              print "# " name ": DEPENDENCY " dep " NOT FOUND" > "/dev/stderr";
              sub(dep,"",$3);
          }
      }
  }
}

pass!=1 && /^[ \t#]*enable/ {
  name=$0;
  gsub(/.*\(/,"",name); gsub(/,.*$/,"",name); gsub(/).*$/,"",name); gsub(/ /,"",name);
  if (!(name in projFound)) next;
  path = projFound[name];
}

# other: simply print the line
pass!=1 { print }