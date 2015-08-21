FILE(REMOVE_RECURSE
  "../src/ariaClientDriver/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/ariaClientDriver/msg/__init__.py"
  "../src/ariaClientDriver/msg/_AriaNavData.py"
  "../src/ariaClientDriver/msg/_AriaCommandData.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
