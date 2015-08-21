FILE(REMOVE_RECURSE
  "../src/ariaClientDriver/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/AriaNavData.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_AriaNavData.lisp"
  "../msg_gen/lisp/AriaCommandData.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_AriaCommandData.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
