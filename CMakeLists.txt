# Assignment 4
cmake_minimum_required( VERSION 2.8 )
project( Assignment4 )

include_directories( .
  /usr/local/include
  /usr/local/include/opencv
  /usr/include/eigen3 )

link_directories( /usr/local/lib )

set( OpenCV_LIBS opencv_core
  opencv_ml
  opencv_video
  opencv_objdetect
  opencv_photo
  opencv_legacy
  opencv_stitching
  opencv_videostab
  opencv_highgui
  opencv_imgproc
  opencv_calib3d
  opencv_contrib
  opencv_nonfree
  opencv_features2d
  opencv_flann )

# OpenCV code
#add_executable( getData GetDataApp.cpp GetData.cpp GetData.h )
#target_link_libraries( getData ${OpenCV_LIBS} )

add_executable(  Assignment4 Assignment4.cpp Tensor3D.cpp Tensor3D.h )
target_link_libraries( Assignment4 ${OpenCV_LIBS} )
