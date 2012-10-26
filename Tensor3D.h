/**
 * @file Tensor3D.h
 * @author A. Huaman
 */

#ifndef _TENSOR_3D_H_
#define _TENSOR_3D_H_

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
 * @class Tensor3D
 */
class Tensor3D {

 public:
  Tensor3D();
  ~Tensor3D();
  void loadImages( char* _filenames );
  void loadCorrespondences( char* _filenames );

  bool readAndStore_PPP( std::fstream &_wstream ); 
  bool readAndStore_LLL( std::fstream &_wstream ); 
  bool readAndStore_PLP( std::fstream &_wstream ); 
  bool readAndStore_PLL( std::fstream &_wstream ); 

  // Fill equations
  void testPPP();
  Eigen::MatrixXd getEq_PPP( std::vector<Eigen::VectorXd> _ppp );
  float epsilon( int i, int j, int k );

  // Utils
  void printCorrespondences();
  void showImages();
  void showCorrespondences();
  void drawPoint( Eigen::VectorXd _point,
		  cv::Mat &_img,
		  cv::Scalar _color );
  void drawLine( Eigen::VectorXd _line,
		 cv::Mat &_img,
		 cv::Scalar _color ); 

  // Getters
  cv::Mat getImage( int _ind );


 private:
  // Images
  std::vector<cv::Mat> mRgbImages;
  std::vector<cv::Mat> mCorrImages;


  // Correspondences
  // Point - Point - Point
  std::vector<std::vector<Eigen::VectorXd> > mPPP;
  // Line -Line - Line
  std::vector<std::vector<Eigen::VectorXd> > mLLL;
  // Point - Line - Point
  std::vector<std::vector<Eigen::VectorXd> > mPLP;
  // Point - Line - Line
  std::vector<std::vector<Eigen::VectorXd> > mPLL;

  // Trifocal tensor stuff
  Eigen::MatrixXf mTt;
  std::vector< Eigen::MatrixXf > mT;

  cv::RNG mRng;
  int mDrawingPointRadius;
  int mDrawingLineThickness;
  int mWidth;
  int mHeight;

  Eigen::MatrixXd mEq;
};

#endif /** _TENSOR_3D_H_ */
