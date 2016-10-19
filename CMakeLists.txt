cmake_minimum_required(VERSION 2.8.12)
project(Hydra-OSVR)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(osvr)

set(SIXENSE_POSSIBLE_PATHS
        ${SIXENSE_ROOT}
        $ENV{SIXENSE_ROOT}
        "C:/Program Files/Steam/steamapps/common/sixense sdk/SixenseSDK"
        "C:/Program Files (x86)/Steam/steamapps/common/sixense sdk/SixenseSDK"
        "C:/Program Files/SixenseSDK"
        "C:/Program Files (x86)/SixenseSDK"
        ~/Library/Frameworks
        /Library/Frameworks
        /usr/local/
        /usr/
        /sw # Fink
        /opt/local/ # DarwinPorts
        /opt/csw/ # Blastwave
        /opt/
)

find_path(SIXENSE_INCLUDE_DIR 
        NAMES           sixense.h
        PATH_SUFFIXES   include
        PATHS           ${SIXENSE_POSSIBLE_PATHS}
)

include_directories(${SIXENSE_INCLUDE_DIR})
add_definitions(-DSIXENSE_BUILDING_DLL)

include(CheckSymbolExists)
if (WIN32)
	check_symbol_exists("_M_AMD64" "" SIXENSE_X64)
else (WIN32)
	check_symbol_exists("__x86_64__" "" SIXENSE_X64)
endif (WIN32)

if (SIXENSE_X64)
	set(TARGET_NAME sixense_x64)
else (SIXENSE_X64)
	set(TARGET_NAME sixense)
endif (SIXENSE_X64)

add_library(${TARGET_NAME} SHARED Hydra-OSVR.cpp)

target_link_libraries(${TARGET_NAME} osvr::osvrClientKitCpp)

install(TARGETS ${TARGET_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX})

if(WIN32)
	include(CopyImportedTarget)
	list(APPEND targets osvr::osvrClientKit osvr::osvrClient osvr::osvrUtil osvr::osvrCommon)
	foreach(target IN LISTS targets)
		install_imported_target(${target} DESTINATION ${CMAKE_INSTALL_PREFIX})
	endforeach()
endif()