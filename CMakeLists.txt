project (erle_computer_vision)

cmake_minimum_required(VERSION 2.8)

SET( DEPS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Deps) # Directorio donde se encuentran las dependencias


# MACRO to get the list of subdirectories
macro(list_subdirectories retval curdir return_relative)
  file(GLOB sub-dir RELATIVE ${curdir} *)
  set(list_of_dirs "")
  foreach(dir ${sub-dir})
    if(IS_DIRECTORY ${curdir}/${dir})
      if (${return_relative})
        if (NOT ${dir} MATCHES "CMakeFiles" AND NOT ${dir} MATCHES ".svn")
                set(list_of_dirs ${list_of_dirs} ${dir})
        endif()
      else()
        set(list_of_dirs ${list_of_dirs} ${curdir}/${dir})
      endif()
    endif()
  endforeach()
  set(${retval} ${list_of_dirs})
endmacro()

# MACRO to get the list subdir of other path distinct to current path
macro(list_subdirectories2 retval curdir return_relative)
  file(GLOB sub-dir RELATIVE ${curdir} ${curdir}*)
  set(list_of_dirs "")
  foreach(dir ${sub-dir})
    if(IS_DIRECTORY ${curdir}/${dir})
      if (${return_relative})        
        if (NOT ${dir} MATCHES "CMakeFiles" AND NOT ${dir} MATCHES ".svn")
                set(list_of_dirs ${list_of_dirs} ${dir})
        endif()
      else()
        set(list_of_dirs ${list_of_dirs} ${curdir}/${dir})
      endif()
    endif()
  endforeach()
  set(${retval} ${list_of_dirs})
endmacro()


include(${DEPS_DIR}/opencv/CMakeLists.txt)

add_subdirectory (${CMAKE_CURRENT_SOURCE_DIR}/src)

