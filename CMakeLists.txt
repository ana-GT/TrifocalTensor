# Assignment 4
cmake_minimum_required( VERSION 2.8 )
project( Assignment4 )

include_directories( .
  /usr/local/include
  /usr/local/include/opencv
  /usr/include/eigen3 )

link_directories( /usr/local/lib )

set( billibot_srcs billibotTest.cpp trifocalTensor.cpp )
set( billibot_hdrs trifocalTensor.h )
set( personal_srcs personalTest.cpp trifocalTensor.cpp )
set( personal_hdrs trifocalTensor.h )

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
add_executable( getData GetDataApp.cpp GetData.cpp GetData.h )
target_link_libraries( getData ${OpenCV_LIBS} )

add_executable(  billibotTest ${billibot_srcs} ${billibot_hdrs} )
target_link_libraries( billibotTest ${OpenCV_LIBS} )
  
#add_executable(  personalTest ${personal_srcs} ${personal_hdrs} )
#target_link_libraries( personalTest ${OpenCV_LIBS} )
