/**
 * @file trifocalTensor.cpp
 */
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "trifocalTensor.h"

/**
 * @function trifocalTensor
 * @brief Constructor
 */
trifocalTensor::trifocalTensor() {

}

/**
 * @function ~trifocalTensor
 * @brief Destructor
 */
trifocalTensor::~trifocalTensor() {

}

/**
 * @function loadImages
 * @brief Load images from .txt file
 */
void trifocalTensor::loadImages( char* _filenames ) {

  std::vector<std::string> imageFiles;

  std::ifstream ifs( _filenames );

  // Read
  std::string temp;
  while( getline( ifs, temp ) ) {
    imageFiles.push_back( temp );
  }

  // Load them
  mRgbImages.resize(0);

  for( int i = 0; i < imageFiles.size(); ++i ) {
    printf("[i] Loading image %s \n", imageFiles[i].c_str() );
    cv::Mat image = cv::imread( imageFiles[i], 1 );
    mRgbImages.push_back( image );
  }

}

/**
 * @function loadCorrespondences
 */
void trifocalTensor::loadCorrespondences( char* _filenames ) {

  std::vector<std::string> correspondenceType;
  std::vector<std::string> correspondenceData;

  std::ifstream ifs( _filenames );

  // Read
  std::string temp;
  while( getline( ifs, temp ) ) {
    correspondenceType.push_back( temp );
    getline( ifs, temp );
    correspondenceData.push_back( temp );
  }

  // Load them
  //  mRgbImages.resize(0);
  if( correspondenceType.size() == correspondenceData.size() ) {
    for( int i = 0; i < correspondenceType.size(); ++i ) {

      if( correspondenceType[i] == "PPP" ) {
	readAndStore_PPP( correspondenceData[i] );
      }
      else if( correspondenceType[i] == "LLL" ) {
	readAndStore_LLL( correspondenceData[i] );
      }
      else if( correspondenceType[i] == "PLP" ) {
	readAndStore_PLP( correspondenceData[i] );
      }
      else if( correspondenceType[i] == "PLL" ) {
	readAndStore_PLL( correspondenceData[i] );
      }

    }
  }
  else {
    printf("Error: Size type and data correspondence does not match \n");
  }
}

/**
 * @function readAndStore_PPP
 */
bool trifocalTensor::readAndStore_PPP( std::string _dataLine ) {

  std::vector<cv::Point3f> c;
  cv::Point3f P1, P2, P3;

  std::stringstream ss( _dataLine );
  ss >> P1.x; ss >> P1.y;
  ss >> P2.x; ss >> P2.y;
  ss >> P3.x; ss >> P3.y;

  c.push_back( P1 );
  c.push_back( P2 );
  c.push_back( P3 );
  printf( "[PPP] Saved (%.3f %.3f) (%.3f %.3f) (%.3f %.3f) \n", 
	  c[0].x, c[0].y, 
	  c[1].x, c[1].y, 
	  c[2].x, c[2].y );
  mPPP.push_back( c );
 
}

/**
 * @function readAndStore_LLL
 */
bool trifocalTensor::readAndStore_LLL( std::string _dataLine ) {

  std::vector<cv::Point3f> c;
  cv::Point3f L1, L2, L3;

  std::stringstream ss( _dataLine );
  ss >> L1.x; ss >> L1.y; ss >> L1.z;
  ss >> L2.x; ss >> L2.y; ss >> L2.z;
  ss >> L3.x; ss >> L3.y; ss >> L3.z;

  c.push_back( L1 );
  c.push_back( L2 );
  c.push_back( L3 );
  printf( "[LLL] Saved (%.3f, %.3f, %.3f) (%.3f, %.3f, %.3f) (%.3f, %.3f, %.3f) \n", 
	  c[0].x, c[0].y, c[0].z, 
	  c[1].x, c[1].y, c[1].z, 
	  c[2].x, c[2].y, c[2].z );
  mLLL.push_back( c );
 
}

/**
 * @function readAndStore_PLP
 */
bool trifocalTensor::readAndStore_PLP( std::string _dataLine ) {

  std::vector<cv::Point3f> c;
  cv::Point3f P1;
  cv::Point3f L2;
  cv::Point3f P3;

  std::stringstream ss( _dataLine );
  ss >> P1.x; ss >> P1.y;
  ss >> L2.x; ss >> L2.y; ss >> L2.z;
  ss >> P3.x; ss >> P3.y;

  c.push_back( P1 );
  c.push_back( L2 );
  c.push_back( P3 );
  printf( "[PLP] Saved (%.3f %.3f) (%.3f %.3f %.3f) (%.3f %.3f) \n", 
	  c[0].x, c[0].y, 
	  c[1].x, c[1].y, c[1].z, 
	  c[2].x, c[2].y );
  mPPP.push_back( c );
 
}

/**
 * @function readAndStore_PLL
 */
bool trifocalTensor::readAndStore_PLL( std::string _dataLine ) {

  std::vector<cv::Point3f> c;
  cv::Point3f P1;
  cv::Point3f L2;
  cv::Point3f L3;

  std::stringstream ss( _dataLine );
  ss >> P1.x; ss >> P1.y;
  ss >> L2.x; ss >> L2.y; ss >> L2.z;
  ss >> L3.x; ss >> L3.y; ss >> L3.z;

  c.push_back( P1 );
  c.push_back( L2 );
  c.push_back( L3 );
  printf("[PLL] Saved (%.3f %.3f) (%.3f %.3f %.3f) (%.3f %.3f %.3f) \n", 
	 c[0].x, c[0].y, 
	 c[1].x, c[1].y, c[1].z,
	 c[2].x, c[2].y, c[2].z );
  mPPP.push_back( c );
 
}

/**
 * @function showImages
 */
void trifocalTensor::showImages() {

  for( int i = 0; i < mRgbImages.size(); ++i ) {
    char windowName[50];
    int n = sprintf( windowName, "Image-%d", i );
    cv::namedWindow( windowName, CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
    cv::imshow( windowName, mRgbImages[i] );
  }

  // Press a key to keep with the program
  int key;
  while(  true ) {
    key = cv::waitKey(30);
    if( key != -1 ) {
      break;
    }
  }

}


/**
 * @function getImage
 */  
cv::Mat trifocalTensor::getImage( int _ind ) {
  
  if( _ind < 0 || _ind >= mRgbImages.size() ) {
    printf("Image of index %d does not exist. Returning NULL \n", _ind );
    cv::Mat empty;
    return empty;
  }
  else {
    return mRgbImages[_ind];
  }
}
