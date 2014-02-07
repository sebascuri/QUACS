FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/ardrone_control/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/ardrone_control/srv/__init__.py"
  "../src/ardrone_control/srv/_Signal.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
