/**
 * @file trifocalTensor.h
 * @author A. Huaman
 */

#ifndef _TRIFOCAL_TENSOR_H_
#define _TRIFOCAL_TENSOR_H_

// OpenCV headers
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/nonfree/features2d.hpp"
// General stuff
#include <vector>

/**
 * @class trifocalTensor
 */
class trifocalTensor {

 public:
  trifocalTensor();
  ~trifocalTensor();
  void loadImages( char* _filenames );
  void showImages();
  void loadCorrespondences( char* _filenames );

  bool readAndStore_PPP( std::string _dataLine ); 
  bool readAndStore_LLL( std::string _dataLine );
  bool readAndStore_PLP( std::string _dataLine );
  bool readAndStore_PLL( std::string _dataLine );
  // Getters
  cv::Mat getImage( int _ind );

 private:
  // Images
  std::vector<cv::Mat> mRgbImages;
  // Correspondences
  // Point - Point - Point
  std::vector<std::vector<cv::Point3f> > mPPP;
  // Line -Line - Line
  std::vector<std::vector<cv::Point3f> > mLLL;
  // Point - Line - Point
  std::vector<std::vector<cv::Point3f> > mPLP;
  // Point - Line - Line
  std::vector<std::vector<cv::Point3f> > mPLL;
};

#endif /** _TRIFOCAL_TENSOR_H_ */
