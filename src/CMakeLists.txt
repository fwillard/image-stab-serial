set ( IMAGE_STAB_SOURCES
    main.cpp
    )

add_executable(ImageStab ${IMAGE_STAB_SOURCES})
target_link_libraries(ImageStab ${OpenCV_LIBS})