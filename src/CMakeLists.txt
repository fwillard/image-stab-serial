set ( IMAGE_STAB_SOURCES
    main.cpp
    )

#add_executable(dpgen main.cpp graph.hpp graph.cpp node.hpp node.cpp edge.hpp edge.cpp Parser.cpp Parser.hpp Component.cpp Component.hpp)
add_executable(ImageStab ${IMAGE_STAB_SOURCES})