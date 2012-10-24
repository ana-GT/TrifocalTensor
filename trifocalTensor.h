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
#include <stdlib.h>
#include <stdio.h>
#include <Eigen/Core>

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
  void showCorrespondences();

  bool readAndStore_PPP( std::string _dataLine ); 
  bool readAndStore_LLL( std::string _dataLine );
  bool readAndStore_PLP( std::string _dataLine );
  bool readAndStore_PLL( std::string _dataLine );

  // Fill equations
  void fillEq();
  void fillEq_PPP( cv::Point3f A, cv::Point3f B, cv::Point3f C );
  void fillEq_LLL( cv::Point3f A, cv::Point3f B, cv::Point3f C );
  void fillEq_PLP( cv::Point3f A, cv::Point3f B, cv::Point3f C );
  void fillEq_PLL( cv::Point3f A, cv::Point3f B, cv::Point3f C );

  float epsilon( int _ind1, int _ind2, int _ind3 );

  void calculateTrifocalTensor();

  int ToIndex1( int i, int j, int k  );
  int ToIndex0( int i, int j, int k );


  // Getters
  cv::Mat getImage( int _ind );

 private:
  // Images
  std::vector<cv::Mat> mRgbImages;
  std::vector<cv::Mat> mCorrImages;


  // Correspondences
  // Point - Point - Point
  std::vector<std::vector<cv::Point3f> > mPPP;
  // Line -Line - Line
  std::vector<std::vector<cv::Point3f> > mLLL;
  // Point - Line - Point
  std::vector<std::vector<cv::Point3f> > mPLP;
  // Point - Line - Line
  std::vector<std::vector<cv::Point3f> > mPLL;

  // Trifocal tensor stuff
  int mPointer;
  int mNumTotalEq;
  Eigen::MatrixXf mT123;
  std::vector< Eigen::MatrixXf > mT;
  Eigen::MatrixXf mT2;
  Eigen::MatrixXf mT3;
  Eigen::VectorXf mB;

  cv::RNG mRng;
  int mDrawingPointRadius;
  int mDrawingLineThickness;


  Eigen::MatrixXf mEq;
};

#endif /** _TRIFOCAL_TENSOR_H_ */
