cmake_minimum_required(VERSION 3.8)

if(DEFINED PROJECT_NAME)
    set(PAN_TOMPKINS_QRS_USED_AS_SUBPROJECT ON)
else()
    set(PAN_TOMPKINS_QRS_USED_AS_SUBPROJECT OFF)
endif()

project(pantompkinsqrs C)

include(GNUInstallDirs)

option(PAN_TOMPKINS_QRS_BUILD_EXAMPLES "Build examples." ON)

set(PAN_TOMPKINS_QRS_INCLUDE_DIR "${PROJECT_SOURCE_DIR}")

add_library(pantompkinsqrs panTompkins.c)

add_library(pantompkinsqrs::pantompkinsqrs ALIAS pantompkinsqrs)

target_include_directories(
    pantompkinsqrs
    PUBLIC
        $<BUILD_INTERFACE:${PAN_TOMPKINS_QRS_INCLUDE_DIR}>
)

if(PAN_TOMPKINS_QRS_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(NOT PAN_TOMPKINS_QRS_USED_AS_SUBPROJECT)
    install(
        DIRECTORY ${PAN_TOMPKINS_QRS_INCLUDE_DIR}/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING
            PATTERN *.h
    )

    install(
        TARGETS pantompkinsqrs
        EXPORT pantompkinsqrs-targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
    
    install(
        EXPORT pantompkinsqrs-targets
        FILE pantompkinsqrs-config.cmake
        NAMESPACE pantompkinsqrs::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pantompkinsqrs
    )
endif()
