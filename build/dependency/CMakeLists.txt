include(${CMAKE_BINARY_DIR}/_deps/xcmake_tools/Common.cmake)

# Fetch dependencies
FetchAndPopulate("https://github.com/LIONant-depot/xbits.git" "main")
FetchAndPopulate("https://github.com/LIONant-depot/xserializer.git" "main")
FetchAndPopulate("https://github.com/LIONant-depot/xresource_pipeline_v2.git" "main")
FetchAndPopulate("https://github.com/LIONant-depot/xstrtool.git" "main" )
FetchAndPopulate("https://github.com/LIONant-depot/xscheduler.git" "main")
FetchAndPopulate("https://github.com/LIONant-depot/xAnimation.git" "main")
FetchAndPopulate("https://github.com/LIONant-depot/xcollision.git" "main")


DefineInterfaceComponent(xragdoll_compiler_ "dependencies"
  "source/xragdoll.h"
  "source/xragdoll_ref.h"
  "source/xragdoll_descriptor.h"
  "source/xragdoll_details.h"
  "source/xragdoll_generate.h"
  "source/compiler/xragdoll_compiler.cpp"
  "source/compiler/xragdoll_compiler.h"
)
