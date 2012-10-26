/**
 * @file Tensor3D.cpp
 */
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "Tensor3D.h"
#include <Eigen/SVD>
#include <Eigen/Dense>

/**
 * @function Tensor3D
 * @brief Constructor
 */
Tensor3D::Tensor3D() {

  mRng = cv::RNG(12345);
  mDrawingPointRadius = 5;
  mDrawingLineThickness = 2;
}

/**
 * @function ~Tensor3D
 * @brief Destructor
 */
Tensor3D::~Tensor3D() {

}

/**
 * @function loadImages
 * @brief Load images from .txt file
 */
void Tensor3D::loadImages( char* _filenames ) {

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
    cv::Mat image = cv::imread( imageFiles[i], 1 );
    mRgbImages.push_back( image );
  }

  mWidth = mRgbImages[0].cols;
  mHeight = mRgbImages[0].rows;
}

/**
 * @function loadCorrespondences
 */
void Tensor3D::loadCorrespondences( char* _filenames ) {

  std::string fullPath( _filenames );
  std::fstream wstream( fullPath.c_str(), std::ios::in );
  std::string str;

  while( !wstream.eof() ) {
    std::string str;
    wstream >> str;
    printf("String read:  %s \n", str.c_str() );
    if( str == "PPP" ) {
      readAndStore_PPP( wstream );
    }
    else if( str == "LLL" ) {
      readAndStore_LLL( wstream );
    }
    else if( str == "PLP" ) {
      readAndStore_PLP( wstream );
    }
    else if( str == "PLL" ) {
      readAndStore_PLL( wstream );
    }
    else {
      printf("Unknown string - Skipping \n");
    }
  } // end while
 
  printCorrespondences();
}

/**
 * @function readAndStore_PPP
 */
bool Tensor3D::readAndStore_PPP( std::fstream &_wstream ) {

  std::vector<Eigen::VectorXd> PPP(3);
  Eigen::VectorXd p(3);

  for( int i = 0; i < 3; ++i ) {

    _wstream >> p(0); 
    _wstream >> p(1); 
    p(2) = 1;

    PPP[i] = p;
  }

  mPPP.push_back( PPP );
  return true;
}



/**
 * @function readAndStore_LLL
 */
bool Tensor3D::readAndStore_LLL( std::fstream &_wstream ) {

  std::vector<Eigen::VectorXd> LLL(3);
  Eigen::VectorXd l(3);

  for( int i = 0; i < 3; ++i ) {

    _wstream >> l(0); 
    _wstream >> l(1); 
    _wstream >> l(2);

    LLL[i] = l;
  }

  mLLL.push_back( LLL );
  return true;
}

/**
 * @function readAndStore_PLP
 */
bool Tensor3D::readAndStore_PLP( std::fstream &_wstream ) {

  std::vector<Eigen::VectorXd> PLP(3);
  Eigen::VectorXd p(3);
  Eigen::VectorXd l(3);
  
  _wstream >> p(0); _wstream >> p(1); p(2) = 1;
  PLP[0] = p;
  _wstream >> l(0); _wstream >> l(1); _wstream >> l(2);
  PLP[1] = l;
  _wstream >> p(0); _wstream >> p(1); p(2) = 1;
  PLP[2] = p;

  mPLP.push_back( PLP );
  return true;
}

/**
 * @function readAndStore_PLL
 */
bool Tensor3D::readAndStore_PLL( std::fstream &_wstream ) {

  std::vector<Eigen::VectorXd> PLL(3);
  Eigen::VectorXd p(3);
  Eigen::VectorXd l(3);

  _wstream >> p(0); _wstream >> p(1); p(2) = 1;
  PLL[0] = p;
  _wstream >> l(0); _wstream >> l(1); _wstream >> l(2);
  PLL[1] = l;
  _wstream >> l(0); _wstream >> l(1); _wstream >> l(2);
  PLL[2] = l;
  
  mPLL.push_back( PLL );
  return true;
}

/**
 * @function testPPP
 */
void Tensor3D::testPPP() {

  Eigen::MatrixXd stacked = Eigen::MatrixXd::Zero( 9*mPPP.size(), 27);

  for( int i = 0; i < mPPP.size(); ++i ) {
    Eigen::MatrixXd block;
    block =  getEq_PPP( mPPP[i] );
    // Check rank
    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(block);
    int rank = lu_decomp.rank();
    printf("Rank of block %d is %d \n", i, rank);

    stacked.block( 9*i,0, 9,27 ) = block;
    
    // Stack
    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp2(stacked);
    int rank2 = lu_decomp2.rank();
    printf("Rank of blocks stacked so far (until %d) is %d \n", i, rank2 );
  }
}

/**
 * @function getEq_PPP
 * @brief  Fill PPP equations (9 Eq - 4 DOF)
 */
Eigen::MatrixXd Tensor3D::getEq_PPP( std::vector<Eigen::VectorXd> _ppp ) {

  Eigen::VectorXd A(3), B(3), C(3); 
  A(0) = _ppp[0](0); A(1) = _ppp[0](1); A(2) = _ppp[0](2);
  B(0) = _ppp[1](0); B(1) = _ppp[1](1); B(2) = _ppp[1](2);
  C(0) = _ppp[2](0); C(1) = _ppp[2](1); C(2) = _ppp[2](2);


  int numEq = 9;
  Eigen::MatrixXd ppp = Eigen::MatrixXd::Zero(numEq,27);
  
  int R[] = {0,0,0,1,1,1,2,2,2};
  int S[] = {0,1,2,0,1,2,0,1,2};
  int r; int s;
  
  for( int ind = 0; ind < numEq; ++ind ) {
    r = R[ind];
    s = S[ind];

    for( int i = 0; i < 3; ++i ) {
      for( int p = 0; p < 3; ++p ) {
	for( int q = 0; q < 3; ++q ) {
	  for( int j = 0; j < 3; ++j ) {
	    for( int k = 0; k < 3; ++k ) {
		ppp( ind, 9*i + 3*p + q ) += A(i)*B(j)*epsilon(j, p, r)*C(k)*epsilon(k,q,s); 
	      
	    } // k
	  } // j
	} // q
      } // p
    } // i
    
  } // ind
 
  return ppp;

}


/**
 * @function epsilon
 */
float Tensor3D::epsilon( int i, int j, int k ) {

  // Levi-Civita

  // 0 unless all three are distinct
  if( i == j ) {
    return 0;
  }
  if( j == k ) {
    return 0;
  }
  if( k == i ) {
    return 0;
  }

  float even = 1; float odd = -1;
  // +1 if (i,j,k) is (0,1,2) (2,0,1), (1,2,0)
  if( i == 0 && j == 1 && k == 2 ) {
    return even;
  } 
  if( i == 2 && j == 0 && k == 1 ) {
    return even;
  } 
  if( i == 1 && j == 2 && k == 0) {
    return even;
  }

  // -1 if (i,j,k) is (0,2,1) (2,1,0) (1,0,2)
  if( i == 0 && j == 2 && k == 1) {
    return odd;
  } 
  if( i == 2 && j == 1 && k == 0 ) {
    return odd;
  } 
  if( i == 1 && j == 0 && k == 2 ) {
    return odd;
  }
  printf("[epsilon] None of the cases above, weird! \n");
  return 0;
}

/////////////////// UTILS //////////////////////

/**
 * @function printCorrespondences
 */
void Tensor3D::printCorrespondences() {

  for( int i = 0; i < mPPP.size(); ++i ) {
    printf("PPP[%d]: (%f %f %f) (%f %f %f) (%f %f %f) \n", i,
	   mPPP[i][0](0), mPPP[i][0](1), mPPP[i][0](2),
	   mPPP[i][1](0), mPPP[i][1](1), mPPP[i][1](2),
	   mPPP[i][2](0), mPPP[i][2](1), mPPP[i][2](2) );
  }

  for( int i = 0; i < mLLL.size(); ++i ) {
    printf("LLL[%d]: (%f %f %f) (%f %f %f) (%f %f %f) \n", i,
	   mLLL[i][0](0), mLLL[i][0](1), mLLL[i][0](2),
	   mLLL[i][1](0), mLLL[i][1](1), mLLL[i][1](2),
	   mLLL[i][2](0), mLLL[i][2](1), mLLL[i][2](2) );
  }

  for( int i = 0; i < mPLP.size(); ++i ) {
    printf("PLP[%d]: (%f %f %f) (%f %f %f) (%f %f %f) \n", i,
	   mPLP[i][0](0), mPLP[i][0](1), mPLP[i][0](2),
	   mPLP[i][1](0), mPLP[i][1](1), mPLP[i][1](2),
	   mPLP[i][2](0), mPLP[i][2](1), mPLP[i][2](2) );
  }

  for( int i = 0; i < mPLL.size(); ++i ) {
    printf("PLL[%d]: (%f %f %f) (%f %f %f) (%f %f %f) \n", i,
	   mPLL[i][0](0), mPLL[i][0](1), mPLL[i][0](2),
	   mPLL[i][1](0), mPLL[i][1](1), mPLL[i][1](2),
	   mPLL[i][2](0), mPLL[i][2](1), mPLL[i][2](2) );
  }
}


/**
 * @function showImages
 */
void Tensor3D::showImages() {

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
 * @function showCorrespondences
 */
void Tensor3D::showCorrespondences() {
  
  // Create images
  mCorrImages.resize(0);

  for( int i = 0; i < mRgbImages.size(); ++i ) {
    cv::Mat image = mRgbImages[i].clone();
    mCorrImages.push_back(image);
  }

  cv::Scalar color; 

  // Draw PPP
  for( int i = 0; i < mPPP.size(); ++i ) {
    color = cv::Scalar( mRng.uniform(0, 255), mRng.uniform(0,255), mRng.uniform(0,255) );
    drawPoint( mPPP[i][0], mCorrImages[0], color );
    drawPoint( mPPP[i][1], mCorrImages[1], color );
    drawPoint( mPPP[i][2], mCorrImages[2], color );
  }
  
  // Draw LLL 
  for( int i = 0; i < mLLL.size(); ++i ) {
    color = cv::Scalar( mRng.uniform(0, 255), mRng.uniform(0,255), mRng.uniform(0,255) );
    drawLine( mLLL[i][0], mCorrImages[0], color );
    drawLine( mLLL[i][1], mCorrImages[1], color );
    drawLine( mLLL[i][2], mCorrImages[2], color );
  }

  // Draw PLP  
  for( int i = 0; i < mPLP.size(); ++i ) {
    color = cv::Scalar( mRng.uniform(0, 255), mRng.uniform(0,255), mRng.uniform(0,255) );
    drawPoint( mPLP[i][0], mCorrImages[0], color );
    drawLine( mPLP[i][1], mCorrImages[1], color );
    drawPoint( mPLP[i][2], mCorrImages[2], color );
  }

  // Draw PLL 
  for( int i = 0; i < mPLL.size(); ++i ) {
    color = cv::Scalar( mRng.uniform(0, 255), mRng.uniform(0,255), mRng.uniform(0,255) );
    drawPoint( mPLL[i][0], mCorrImages[0], color );
    drawLine( mPLL[i][1], mCorrImages[1], color );
    drawLine( mPLL[i][2], mCorrImages[2], color );
  }
  
  // Draw CorrImages
  for( int i = 0; i < mCorrImages.size(); ++i ) {
    char windowName[50];
    int n = sprintf( windowName, "Corresp-%d", i );
    cv::namedWindow( windowName, CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO );
    cv::imshow( windowName, mCorrImages[i] );
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
 * @function drawPoint
 */
void Tensor3D::drawPoint( Eigen::VectorXd _point,
			  cv::Mat &_img,
			  cv::Scalar _color ) {
  cv::Point p;
  p.x = _point(0); p.y = _point(1);
  cv::circle( _img, p, mDrawingPointRadius, 
	      _color, -1, 8 );
}

/**
 * @function drawLine
 * @brief ax + by + c = 0. y = (-c-ax)/b
 */
void Tensor3D::drawLine( Eigen::VectorXd _line,
			 cv::Mat &_img,
			 cv::Scalar _color ) {
  cv::Point p1, p2;

  p1.x = 0 + 20;
  p1.y = ( -_line(2)  - _line(0)*p1.x ) / _line(1);

  p2.x = mWidth - 20;
  p2.y = ( -_line(2)  - _line(0)*p2.x ) / _line(1);

  cv::line( _img, p1, p2, _color, mDrawingLineThickness, 8 );
}



/**
 * @function getImage
 */  
cv::Mat Tensor3D::getImage( int _ind ) {

  if( _ind < 0 || _ind >= mRgbImages.size() ) {
    printf("Image of index %d does not exist. Returning NULL \n", _ind );
    cv::Mat empty;
    return empty;
  }
  else {
    return mRgbImages[_ind];
  }
 
}
