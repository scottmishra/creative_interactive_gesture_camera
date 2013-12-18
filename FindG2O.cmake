# - Try to find G2O++
# Once done, this will define
#
#  G2O_FOUND - system has G2O
#  G2O_INCLUDE_DIRS - the G2O include directories
#  G2O_LIBRARIES - link these to use G2O

find_package(PkgConfig)
pkg_check_modules(G2O QUIET libG2O)
set(G2O_DEFINITIONS ${G2O_CFLAGS_OTHER})

find_path(G2O_INCLUDE_DIR g2o/base_edge.h
          HINTS /home/common/catkin_ws/src/g2o
          PATH_SUFFIXES G2O )

find_library(G2O_LIBRARY 
             NAMES G2O
             HINTS /home/common/catkin_ws/src/g2o/lib )

set(G2O_LIBRARIES ${G2O_LIBRARY} )
set(G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set G2O_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(G2O  DEFAULT_MSG
                                  G2O_LIBRARY G2O_INCLUDE_DIR)

mark_as_advanced(G2O_INCLUDE_DIR G2O_LIBRARY )

