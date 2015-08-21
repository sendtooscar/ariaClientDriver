FILE(REMOVE_RECURSE
  "../src/ariaClientDriver/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/ariaClientDriver/AriaNavData.h"
  "../msg_gen/cpp/include/ariaClientDriver/AriaCommandData.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
