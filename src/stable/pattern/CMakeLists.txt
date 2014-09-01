SET(Erle_PatternDetector PatternDetector.cpp)
add_library(erle_patternDetector SHARED ${Erle_PatternDetector})
TARGET_LINK_LIBRARIES(erle_patternDetector ${OPENCV_LIBRARIES})

