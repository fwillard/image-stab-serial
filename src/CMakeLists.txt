set ( IMAGE_STAB_SOURCES
    main.cpp
    util.hpp
    tracking.cpp
    tracking.hpp
    )

add_executable(ImageStab ${IMAGE_STAB_SOURCES})
target_link_libraries(ImageStab ${OpenCV_LIBS})
