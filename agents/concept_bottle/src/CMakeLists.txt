# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
  ../src/PointsViewer.h
)

find_package(OpenCV 4 REQUIRED)
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIR} )

LIST(APPEND LIBS ${LIBS} ${OpenCV_LIBS} QGLViewer-qt6 Qt6OpenGLWidgets)

add_compile_options(
    
     -O3 
    -march=native 
    -flto=auto
    -fno-plt
)
add_definitions(-DNDEBUG -fmax-errors=1 -I/usr/include/x86_64-linux-gnu/qt6/QtOpenGLWidgets)
add_link_options(-flto=auto)

