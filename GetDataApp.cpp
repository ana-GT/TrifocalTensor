/**
 * @file GetDataApp.cpp
 */
#include <stdio.h>
#include "GetData.h"

char rgbWindow[50] = "RGB Window";
char matchWindow_01[50] = "Match Window 01";
char matchWindow_02[50] = "Match Window 02";
char keypointWindow_0[50] = "Keypoint Window 0";
char keypointWindow_1[50] = "Keypoint Window 1";
char keypointWindow_2[50] = "Keypoint Window 2";

// Function declaration
void onMouse_01( int _evt, int _x, int _y, int _flags, void* _param );
void onMouse_02( int _evt, int _x, int _y, int _flags, void* _param );

void onMouse_0( int _evt, int _x, int _y, int _flags, void* _param );
void onMouse_1( int _evt, int _x, int _y, int _flags, void* _param );
void onMouse_2( int _evt, int _x, int _y, int _flags, void* _param );

/**
 * @function main
 */
int main( int argc, char* argv[] )  {
  
  GetData gd( argv[1] );
  gd.matchAllFrames( );

  cv::imshow( keypointWindow_0, gd.getKeypointsDraw( 0 ) ); 
  cv::imshow( keypointWindow_1, gd.getKeypointsDraw( 1 ) ); 
  cv::imshow( keypointWindow_2, gd.getKeypointsDraw( 2 ) );     
  //cv::imshow( matchWindow_01, gd.getMatchesDraw( 0, 1 ) ); 
  //cv::imshow( matchWindow_02, gd.getMatchesDraw( 0, 2 ) );     

  // Set Mousecallbacks
//  cv::setMouseCallback( matchWindow_01, onMouse_01 );
//  cv::setMouseCallback( matchWindow_02, onMouse_02 );
cv::setMouseCallback( keypointWindow_0, onMouse_0 );
cv::setMouseCallback( keypointWindow_1, onMouse_1 );
cv::setMouseCallback( keypointWindow_2, onMouse_2 );

  int key;
  int ind = 0;
  while( true ) {
    key = cv::waitKey(30);

    // If a key is pressed
    if( key != -1 ) {
      printf("Pressing detected \n");
      if( key == 27 ) {
	break;
      }
      
   
    } // end if( key != -1 )
  }
  
  return 0;
}

/**
 * @function onMouse_01
 */
void onMouse_01( int _evt, int _x, int _y, int _flags, void* _param ) {

  switch( _evt ) {
  case CV_EVENT_MOUSEMOVE: 
    break;

  case CV_EVENT_LBUTTONDOWN:
    printf("[01] Mouse down in %d %d  \n", _x, _y );
  } 
}

/**
 * @function onMouse_02
 */
void onMouse_02( int _evt, int _x, int _y, int _flags, void* _param ) {

  switch( _evt ) {
  case CV_EVENT_MOUSEMOVE: 
    break;

  case CV_EVENT_LBUTTONDOWN:
    printf("[02] Mouse down in %d %d \n ", _x, _y );
  } 
}

/**
 * @function onMouse_0
 */
void onMouse_0( int _evt, int _x, int _y, int _flags, void* _param ) {

  switch( _evt ) {
  case CV_EVENT_MOUSEMOVE: 
    break;

  case CV_EVENT_LBUTTONDOWN:
    printf("[0] Keypoints: Mouse down in %d %d \n ", _x, _y );
  } 
}

/**
 * @function onMouse_1
 */
void onMouse_1( int _evt, int _x, int _y, int _flags, void* _param ) {

  switch( _evt ) {
  case CV_EVENT_MOUSEMOVE: 
    break;

  case CV_EVENT_LBUTTONDOWN:
    printf("[1] Keypoints: Mouse down in %d %d \n ", _x, _y );
  } 
}

/**
 * @function onMouse_2
 */
void onMouse_2( int _evt, int _x, int _y, int _flags, void* _param ) {

  switch( _evt ) {
  case CV_EVENT_MOUSEMOVE: 
    break;

  case CV_EVENT_LBUTTONDOWN:
    printf("[2] Keypoints: Mouse down in %d %d \n ", _x, _y );
  } 
}
