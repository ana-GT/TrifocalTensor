/**
 * @file GetData.cpp
 * @author A. Huaman
 */

#include "GetData.h"
#include <time.h>
#include <Eigen/Core>
#include <stdio.h>
#include <iostream>
#include <fstream>

/**
 * @function GetData
 * @brief Constructor
 */
GetData::GetData( char *_filenames ) {
  
  // Initialize a few things
  mMinHessian = 400;
  mRadiusFactor = 3.0; // 3.0

  // For random generation
  srand( time(NULL) );

  // Read images
  if( !readImagesData( _filenames, mRgbImages ) ) {
    printf( "[X] No Rgb Data read! \n" );
  }

  mNumFrames = mRgbImages.size();
  printf( "Loaded %d images \n", mNumFrames );

  // Get gray
  getGrayImages( mRgbImages, mGrayImages );
  printf( "Loaded %d Gray images \n", mGrayImages.size() );  

  mHeight = mRgbImages[0].rows;
  mWidth = mRgbImages[0].cols;

}

/**
 * @function ~GetData
 * @brief Destructor
 */
GetData::~GetData() {

}

///////////// GET DATA //////////////////


/**
 * @function getPointsFromMatch
 */
void GetData::getPointsFromMatch( int _ind1, int _ind2, 
				const cv::DMatch &_match,
				float &_x1, float &_y1, 
				float &_x2, float &_y2 ) {

  int idx1; int idx2;
  cv::Point2f p1; cv::Point2f p2;
  int h1, w1; int h2, w2;
  
  // Get both 2D Points from keypoint info
  idx1 = _match.queryIdx;
  idx2 = _match.trainIdx;
  
  p1 = mKeypoints[_ind1][idx1].pt; 
  _y1 = p1.y; _x1 = p1.x;
  
  p2 = mKeypoints[_ind2][idx2].pt; 
  _y2 = p2.y; _x2 = p2.x;
  
}

/**
 * @function getPointsFromMatch
 */
void GetData::getPointsFromMatch( int _ind1, int _ind2, 
				 const cv::DMatch &_match,
				 cv::Point2f &_p1, 
				 cv::Point2f &_p2 ) {

  int idx1; int idx2;
  
  // Get both 2D Points from keypoint info
  idx1 = _match.queryIdx;
  idx2 = _match.trainIdx;
  
  _p1 = mKeypoints[_ind1][idx1].pt; 
  _p2 = mKeypoints[_ind2][idx2].pt; 
  
}


//////////////////// MATCHING //////////////////

/**
 * @function matchAllFrames
 */
void GetData::matchAllFrames() {
 
  printf("Match all frames \n");
  cv::SURF surf( mMinHessian );
  
  // Detect keypoints and generate descriptors
  for( int i = 0; i < mNumFrames; ++i ) {

    std::vector<cv::KeyPoint> keypoint;
    cv::Mat descriptor;
    surf( mGrayImages[i], cv::Mat(), keypoint, descriptor, false );
    mKeypoints.push_back( keypoint );
    mDescriptors.push_back( descriptor );
    printf("[%d] Num keypoints: %d \n", i, keypoint.size() );
  }

  // Create a FlannBasedMatcher
  cv::FlannBasedMatcher matcher;
  std::vector< std::vector< std::vector< cv::DMatch > > > tempMatches;

  // Match each descriptor successively
  printf( " Start matching process \n" );
  for( int i = 0; i < mNumFrames; ++i ) {

    std::vector< std::vector< cv::DMatch > > matches_iToAll;
    for( int j = 0; j < mNumFrames; ++j ) {
      std::vector< cv::DMatch> matches_iToj;
      matcher.match( mDescriptors[i], mDescriptors[j], matches_iToj );
      matches_iToAll.push_back( matches_iToj );
    }
    tempMatches.push_back( matches_iToAll );
  }

  // Resize
  mMatches.resize( mNumFrames );
  for( int i = 0; i < mNumFrames; ++i ) {
    mMatches[i].resize( mNumFrames );
  }

  // Quick calculation of max and min distances
  double max_dist; 
  double min_dist;
  double dist;

  for( int i = 0; i < mNumFrames; ++i ) {
    for( int j = 0; j < mNumFrames; ++j ) {

      max_dist = 0; min_dist = 100;
      for( int k = 0; k < tempMatches[i][j].size(); ++k ) {
	dist = tempMatches[i][j][k].distance;
	if( dist < min_dist ) { min_dist = dist; }
	if( dist > max_dist ) { max_dist = dist; }
      }

      // Get only good matches
      for( int k = 0; k < tempMatches[i][j].size(); ++k ) {
	if( tempMatches[i][j][k].distance <= mRadiusFactor*min_dist ) {
	  mMatches[i][j].push_back( tempMatches[i][j][k] );
	}
      }

    }
  }

  printf("End matching process with these results: \n");
  for( int i = 0; i < mNumFrames; ++ i ) {
    printf("<%d> ", i );
    for( int j = 0; j < mNumFrames; ++j ) {
      printf(":[%d] %d  ", j, mMatches[i][j].size() );
    }
    printf("\n");
  }

}

/**
 * @function getMatchesDraw
 */
cv::Mat GetData::getMatchesDraw( int _ind1, int _ind2  ) {

  //-- Draw matches
  cv::Mat matchesImage;

  cv::drawMatches( mRgbImages[_ind1], 
		   mKeypoints[_ind1], 
		   mRgbImages[_ind2], 
		   mKeypoints[_ind2], 
		   mMatches[_ind1][_ind2], 
		   matchesImage,
		   cv::Scalar::all(-1),
		   cv::Scalar::all(-1),
		   std::vector<char>(),
		   2 ); // 2: NOT_DRAW_SINGLE_POINTS  
  
  return matchesImage;
}

/**
 * @function getSomeMatchesDraw
 */
cv::Mat GetData::getSomeMatchesDraw( int _ind1, int _ind2,
				    const std::vector<int> &matchesIndices ) {

  cv::Mat matchesImage;
  std::vector<cv::DMatch> matches;
  float x1, y1, x2, y2;
  for( int i = 0; i < matchesIndices.size(); ++i ) {
    matches.push_back( mMatches[_ind1][_ind2][ matchesIndices[i] ] );

    getPointsFromMatch( _ind1, _ind2, matches[i],
			x1, y1, x2, y2 ); 
    printf("Drawing match between: (%.3f, %.3f) and  (%.3f, %.3f) \n", x1, y1, x2, y2 );
  }

  cv::drawMatches( mRgbImages[_ind1],
		   mKeypoints[_ind1],
		   mRgbImages[_ind2],
		   mKeypoints[_ind2],
		   matches,
		   matchesImage,
		   cv::Scalar::all(-1),
		   cv::Scalar::all(-1),
		   std::vector<char>(),
		   2 ); // 2: NOT_DRAW_SINGLE_POINTS 
  return matchesImage;
}

/**
 * @function getKeypointsDraw
 */
cv::Mat GetData::getKeypointsDraw( int _ind ) {

  cv::Mat keypointsDraw;
  cv::drawKeypoints( mRgbImages[_ind],  mKeypoints[_ind], 
		     keypointsDraw, cv::Scalar::all(-1), 
		     cv::DrawMatchesFlags::DEFAULT );
  return keypointsDraw;
}
    

/////////////////// DATA ACQUISITION //////////////////////////////

/**
 * @function readImagesData
 */
bool GetData::readImagesData( char* _filenames, 
			     std::vector<cv::Mat> &_images ) {

  std::vector<std::string> imageFiles;
  
  // Open Stream
  _images.resize(0);
  std::ifstream ifs( _filenames );
  
  // Read
  std::string temp;
  while( getline( ifs, temp ) ) {
    imageFiles.push_back( temp );
  }
  
  // Load them 
  for( int i = 0; i < imageFiles.size(); ++i ) {

    cv::Mat image;

    // Read 3-channel image
    image = cv::imread( imageFiles[i], 1 );
    if( !image.data ) {
      std::cout<<"[X] Could not read file "<< imageFiles[i]<< std::endl;
     _images.resize(0);
      return false;
    }    

    _images.push_back( image );
  }

  return true;
 
}

/**
 * @function getGrayImages
 * @brief 
 */
bool GetData::getGrayImages( const std::vector<cv::Mat> &_rgbImages,
			    std::vector<cv::Mat> &_grayImages ) {

  _grayImages.resize(0);

  for( int i = 0; i < _rgbImages.size(); ++i ) {

    cv::Mat grayImage;
    _rgbImages[i].convertTo( grayImage, CV_8UC1 );
    _grayImages.push_back( grayImage );
  }

  return true;
}

