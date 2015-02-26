add_subdirectory (${CMAKE_CURRENT_SOURCE_DIR}/stable)

IF(ERLE_TESTING)
	add_subdirectory (${CMAKE_CURRENT_SOURCE_DIR}/testing ${CMAKE_CURRENT_SOURCE_DIR}/testing)
ENDIF()
