/**
 * @file GetData.h
 * @author A. Huaman :D
 * @date  2012/10/07
 */

#ifndef __GET_DATA_H__
#define __GET_DATA_H__

//-- OpenCV headers
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
// For SURF
#include "opencv2/nonfree/features2d.hpp"

#include <vector>

/**
 * @class GetData
 */
class GetData {

 public:

  // Constructor and destructor
  GetData( char *_filenames );
  ~GetData();

  //-- Get Data
  void getPointsFromMatch( int _ind1, int _ind2, 
			   const cv::DMatch &_match,
			   float &_x1, float &_y1, 
			   float &_x2, float &_y2 );
  void getPointsFromMatch( int _ind1, int _ind2, 
			   const cv::DMatch &_match,
			   cv::Point2f &_p1, 
			   cv::Point2f &_p2 );

  //-- Matching
  void matchAllFrames();
  cv::Mat getMatchesDraw( int _ind1, int _ind2  );
  cv::Mat getSomeMatchesDraw( int _ind1, int _ind2,
			      const std::vector<int> &matchesIndices );
  cv::Mat getKeypointsDraw( int _ind );

  //-- Data acquisition  
  bool readImagesData( char* _filenames, 
		       std::vector<cv::Mat> &_images );
  bool getGrayImages( const std::vector<cv::Mat> &_rgbImages,
		      std::vector<cv::Mat> &_grayImages );

  //-- Get functions
  inline cv::Mat getRgb( int _ind );
  inline int getNumFrames();


 private:
  // Image  data variables
  std::vector<cv::Mat> mRgbImages;
  std::vector<cv::Mat> mGrayImages;

  int mNumFrames;
  int mHeight;
  int mWidth;
  
  // SURF Keypoint detection and descriptor
  int mMinHessian;
  float mRadiusFactor;
  int mFactor;

  std::vector< std::vector<cv::KeyPoint> > mKeypoints; 
  std::vector< cv::Mat > mDescriptors;
  std::vector< std::vector< std::vector< cv::DMatch > > > mMatches;

};
  
////////////////// INLINE FUNCTIONS /////////////////////////////////////// 

/**
 * @function getRgb
 */
inline cv::Mat GetData::getRgb( int _ind ) {
  
  if( _ind < mNumFrames ) {
      return mRgbImages[_ind];
  }
}


/**
 * @function getNumFrames
 */
inline int GetData::getNumFrames() {
  return mNumFrames;
}


#endif /** __GET_DATA_H__ */
  
