/**
 * @file trifocalTensor.cpp
 */
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "trifocalTensor.h"
#include <Eigen/SVD>
#include <Eigen/Dense>

/**
 * @function trifocalTensor
 * @brief Constructor
 */
trifocalTensor::trifocalTensor() {

  mRng(12345);
  mDrawingPointRadius = 5;
  mDrawingLineThickness = 2;
  mPointer = 0;
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
  mPPP.resize(0); mLLL.resize(0);
  mPLP.resize(0); mPLL.resize(0);

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
  ss >> P1.x; ss >> P1.y; P1.z = 1;
  ss >> P2.x; ss >> P2.y; P2.z = 1;
  ss >> P3.x; ss >> P3.y; P3.z = 1;

  c.push_back( P1 );
  c.push_back( P2 );
  c.push_back( P3 );
  printf( "[PPP] Saved (%.3f %.3f) (%.3f %.3f) (%.3f %.3f) \n", 
	  c[0].x, c[0].y, 
	  c[1].x, c[1].y, 
	  c[2].x, c[2].y );
  mPPP.push_back( c );
 
  return true;
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
  ss >> P1.x; ss >> P1.y; P1.z = 1;
  ss >> L2.x; ss >> L2.y; ss >> L2.z;
  ss >> P3.x; ss >> P3.y; P3.z = 1;

  c.push_back( P1 );
  c.push_back( L2 );
  c.push_back( P3 );
  printf( "[PLP] Saved (%.3f %.3f) (%.3f %.3f %.3f) (%.3f %.3f) \n", 
	  c[0].x, c[0].y, 
	  c[1].x, c[1].y, c[1].z, 
	  c[2].x, c[2].y );
  mPLP.push_back( c );
 
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
  ss >> P1.x; ss >> P1.y; P1.z = 1;
  ss >> L2.x; ss >> L2.y; ss >> L2.z;
  ss >> L3.x; ss >> L3.y; ss >> L3.z;

  c.push_back( P1 );
  c.push_back( L2 );
  c.push_back( L3 );
  printf("[PLL] Saved (%.3f %.3f) (%.3f %.3f %.3f) (%.3f %.3f %.3f) \n", 
	 c[0].x, c[0].y, 
	 c[1].x, c[1].y, c[1].z,
	 c[2].x, c[2].y, c[2].z );
  mPLL.push_back( c );
 
}

/**
 * @function fillEq
 */
void trifocalTensor::fillEq() {

  mPointer = 0;
  mNumTotalEq = 9*mPPP.size() + 3*mLLL.size() + 3*mPLP.size() + 1*mPLL.size();

  printf("Total number of equations (some of them redundant): %d \n", mNumTotalEq );
  mEq = Eigen::MatrixXf::Zero( mNumTotalEq, 27 );
  mB = Eigen::VectorXf::Zero( mNumTotalEq );

  Eigen::FullPivLU<Eigen::MatrixXf> lu(mEq);
  printf("* [Beginning] Rank of mEq is: %d \n", lu.rank() );

  // Fill Point-Point-Point ( 9 Equations - 4 DOF )
  for( int i = 0; i < mPPP.size();  i++ ) {
    fillEq_PPP( mPPP[i][0], mPPP[i][1], mPPP[i][2] );
    //Eigen::MatrixXf yam = mEq.block(mPointer - 9, 0,  9, mEq.cols() );
	  Eigen::MatrixXf yam = mEq;
    Eigen::FullPivLU<Eigen::MatrixXf> lu(yam);
    printf("* [PPP][%d] Rank of mEq(%d x %d) is: %d \n", i, yam.rows(), yam.cols(), lu.rank() );
  }

  Eigen::FullPivLU<Eigen::MatrixXf> luA(mEq);
  printf("* After [PPP] Rank of mEq is: %d \n",luA.rank() );

  // Fill Line -Line - Line ( 3 Equations - 2 DOF )
  for( int j = 0; j < mLLL.size(); j++ ) {
    fillEq_LLL( mLLL[j][0], mLLL[j][1], mLLL[j][2] );
    Eigen::FullPivLU<Eigen::MatrixXf> lu(mEq);
    printf("* [LLL][%d] Rank of mEq is: %d \n", j, lu.rank() );
  }

  // Fill Point - Line - Line ( 1 Equations - 1 DOF )
  for( int i = 0; i < mPLL.size(); ++i ) {
    fillEq_PLL( mPLL[i][0], mPLL[i][1], mPLL[i][2] );
    Eigen::FullPivLU<Eigen::MatrixXf> lu(mEq);
    printf("* [PLL][%d] Rank of mEq is: %d \n", i, lu.rank() );
  }

  // Fill Point - Line - Point ( 3 Equations - 2 DOF )
  for( int i = 0; i < mPLP.size(); ++i ) {
    fillEq_PLP( mPLP[i][0], mPLP[i][1], mPLP[i][2] );
    Eigen::FullPivLU<Eigen::MatrixXf> lu(mEq);
    printf("* [PLP][%d] Rank of mEq is: %d \n", i, lu.rank() );
  }




}

/**
 * @function fillEq_LLL
 * @brief  Fill LLL equations ( 3 Equations, 2 DOF )
 */
void trifocalTensor::fillEq_LLL( cv::Point3f _A, 
				 cv::Point3f _B,
				 cv::Point3f _C ) {

  Eigen::Vector3f A; A << _A.x, _A.y, _A.z;
  Eigen::Vector3f B; B << _B.x, _B.y, _B.z;
  Eigen::Vector3f C; C << _C.x, _C.y, _C.z;
  
  std::cout << "[LLL] Correspondence: " << A.transpose() <<"," <<B.transpose() <<"," <<C.transpose() << std::endl;
  // 3 equations
  for( int s = 0; s < 3; ++s ) {
    // T0, T1, T2
    for( int i = 0; i < 3; ++i ) {
      for( int j = 0; j < 3; ++j ) {
	for( int k = 0; k < 3; ++k ) {
	  for( int r = 0; r < 3; ++r ) {	      
	    mEq( mPointer, 9*i + 3*j + k) += A(r)*B(j)*C(k)*epsilon(r,i,s);
	  } // r
	} // k
      } // j
    } // i
    
    mPointer++;
  } // s

}


/**
 * @function fillEq_PLP
 * @brief  Fill PLP equations ( 3 Eq - 2 DOF )
 */
void trifocalTensor::fillEq_PLP( cv::Point3f _A, 
				 cv::Point3f _B,
				 cv::Point3f _C ) {

  Eigen::Vector3f A; A << _A.x, _A.y, _A.z;
  Eigen::Vector3f B; B << _B.x, _B.y, _B.z;
  Eigen::Vector3f C; C << _C.x, _C.y, _C.z;
  std::cout << "Correspondence PLP: " << A.transpose() <<"," <<B.transpose() <<"," <<C.transpose() << std::endl;
  // 3 equations
  for( int s = 0; s < 3; ++s ) {
    // T0, T1, T2
    for( int i = 0; i < 3; ++i ) {
      for( int j = 0; j < 3; ++j ) {
	for( int q = 0; q < 3; ++q ) {
	  for( int k = 0; k < 3; ++k ) {
	    if( epsilon(k,q,s) != 0 ) {
	      mEq( mPointer, 9*i + 3*j + q) = A(i)*B(j)*C(k)*epsilon(k,q,s);
	    }
	  } // k
	} // q
      } // j
    } // i
    
    mPointer++;
  } // s
  
  
}

/**
 * @function fillEq_PLL 
 * @brief  Fill PLL equations ( 1 Equation - 1 DOF )
 */
void trifocalTensor::fillEq_PLL( cv::Point3f _A, 
				 cv::Point3f _B,
				 cv::Point3f _C ) {

  Eigen::Vector3f A; A << _A.x, _A.y, _A.z;
  Eigen::Vector3f B; B << _B.x, _B.y, _B.z;
  Eigen::Vector3f C; C << _C.x, _C.y, _C.z;
  std::cout << "PLL Correspondence: " << A.transpose() <<"," <<B.transpose() <<"," <<C.transpose() << std::endl;

    // T0, T1, T2
    for( int i = 0; i < 3; ++i ) {
      for( int j = 0; j < 3; ++j ) {
	for( int k = 0; k < 3; ++k ) {
	    mEq( mPointer, 9*i + 3*j + k) += A(i)*B(j)*C(k);
	} // k
      } // j
    } // i
    mPointer++;

}

/**
 * @function fillEq_PPP
 * @brief  Fill PPP equations (9 Eq - 4 DOF)
 */
void trifocalTensor::fillEq_PPP( cv::Point3f _A, 
				 cv::Point3f _B,
				 cv::Point3f _C ) {

  Eigen::Vector3f A; A << _A.x, _A.y, _A.z;
  Eigen::Vector3f B; B << _B.x, _B.y, _B.z;
  Eigen::Vector3f C; C << _C.x, _C.y, _C.z;

  printf("[PPP] Correspondence ( %f %f %f ), (%f %f %f) (%f %f %f) \n", A(0), A(1), A(2),  B(0), B(1), B(2), C(0), C(1), C(2));
  // 3x3 equations


  for( int r = 0; r < 3; r++ ) {
    for( int s = 0; s < 3; s++ ) {
      printf("[PPP] rs: (%d %d) Pointer: %d  \n", r, s, mPointer);
      Eigen::MatrixXf T0 = Eigen::MatrixXf::Zero(3,3);
      Eigen::MatrixXf T1 = Eigen::MatrixXf::Zero(3,3);
      Eigen::MatrixXf T2 = Eigen::MatrixXf::Zero(3,3);
      
      std::vector<Eigen::MatrixXf> tempT;
      tempT.push_back(T0);
      tempT.push_back(T1);
      tempT.push_back(T2);
      // T0, T1, T2
      for( int i = 0; i < 3; i++ ) {
	for( int p = 0; p < 3; p++ ) {
	  for( int q = 0; q < 3; q++ ) {
	    for( int j = 0; j < 3; j++ ) {
	      for( int k = 0; k < 3; k++ ) {
		
		if( epsilon(j,p,r) != 0 && epsilon( k, q, s ) != 0 ) {
		  tempT[i](p,q) += A(i)*B(j)*epsilon(j,p,r)*epsilon(k,q,s)*C(k);
		  }
		/*
		float val = A(i)*( (B(j)*epsilon(j,p,r))*(epsilon(k,q,s)*C(k)) );
		float temp = tempT[i](p,q) + val;
		tempT[i](p,q) = temp; */
	      } // k
	    } // j
	  } // q
	} // p
      } // i
      
      // Fill matrix
      int index = 0;
      for( int a = 0; a < 3; ++a ) {
	for( int b = 0; b < 3; ++b ) {
	  for( int c = 0; c < 3; ++c ) {
	    mEq( mPointer, index) = tempT[a](b,c);
	    index++;
	  }
	}
      }

      mPointer++;
    } // s
  } // r
  

}

/**
 * @function ToIndex1
 */
int trifocalTensor::ToIndex1( int i, int j, int k  ) {
  int index;
  index = 9*(i-1) + 3*(j-1) + (k-1);
  return index;
}

/**
 * @function ToIndex0
 */
int trifocalTensor::ToIndex0( int i, int j, int k ) {
  int index;
  index = 9*i + 3*j + k;
  return index;
}



/**
 * @function epsilon
 */
float trifocalTensor::epsilon( int i, int j, int k ) {

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

/**
 * @function calculateTrifocalTensor
 */
void trifocalTensor::calculateTrifocalTensor() {

  Eigen::JacobiSVD<Eigen::MatrixXf> svd( mEq, Eigen::ComputeThinU | Eigen::ComputeThinV );
  Eigen::MatrixXf V = svd.matrixV();
  printf("* V has %d rows and %d cols \n", V.rows(), V.cols() );


  Eigen::FullPivLU<Eigen::MatrixXf> lu(mEq);
  printf("* Rank of mEq is: %d \n", lu.rank() );
  //std::cout << "Columns are nullspace : " << std::endl;
  //std::cout<< lu.kernel() << std::endl;
  //Eigen::MatrixXf kernel = lu.kernel();
  //mmEq(mPointer, ToIndex1(3,1,2)=;3 = kernel.col( kernel.cols() - 1 );
  
  // Eigen::MatrixXf Vt = V.transpose(); mmEq(mPointer, ToIndex1(3,1,2)=;3 = Vt.col( Vt.cols() - 1 );
  mT123 = V.col( V.cols() - 1 );
  printf("mT123: Rows: %d  cols: %d \n", mT123.rows(), mT123.cols() );

  // Saving them properly
  mT.resize(0);
  Eigen::MatrixXf T1(3,3);
  T1(0,0) = mT123(0,0); T1(0,1) = mT123(1,0); T1(0,2) = mT123(2,0);
  T1(1,0) = mT123(3,0); T1(1,1) = mT123(4,0); T1(1,2) = mT123(5,0);
  T1(2,0) = mT123(6,0); T1(2,1) = mT123(7,0); T1(2,2) = mT123(8,0);

  mT.push_back(T1);
  printf("Saved T1 \n");

  Eigen::MatrixXf T2(3,3);
  T2(0,0) = mT123(9,0); T2(0,1) = mT123(10,0); T2(0,2) = mT123(11,0);
  T2(1,0) = mT123(12,0); T2(1,1) = mT123(13,0); T2(1,2) = mT123(14,0);
  T2(2,0) = mT123(15,0); T2(2,1) = mT123(16,0); T2(2,2) = mT123(17,0);

  mT.push_back(T2);
  printf("Saved T2 \n");

  Eigen::MatrixXf T3(3,3);
  T3(0,0) = mT123(18,0); T3(0,1) = mT123(19,0); T3(0,2) = mT123(20,0);
  T3(1,0) = mT123(21,0); T3(1,1) = mT123(22,0); T3(1,2) = mT123(23,0);
  T3(2,0) = mT123(24,0); T3(2,1) = mT123(25,0); T3(2,2) = mT123(26,0);

  mT.push_back(T3);
  printf("Saved T3 \n");

  // Checking
  Eigen::MatrixXf res = mEq*mT123;
  std::cout << "Checking mEq*T: \n"<< res.transpose() << std::endl;

  // Making it with last guy = 1
  // Normalizing
  
  for( int i = 0; i < mT.size(); ++i ) {

    float temp = mT[i](2,2);

    for( int j = 0; j < 3; ++j ) {
      for( int k = 0; k < 3; ++k ) {

	float orig = mT[i](j,k);
	mT[i](j,k) = orig / temp;

      }
    }
  }
  
  // Visualize
  for( int i = 0; i < mT.size(); ++i ) {
    std::cout << "T("<<i<<"): \n" << mT[i] << std::endl;
  }

  // Test lines
  for( int i = 0; i < mLLL.size(); ++i ) {
    Eigen::VectorXf A(3);
    Eigen::VectorXf B(3);
    Eigen::VectorXf C(3);
    Eigen::VectorXf Ap(3);

    A(0) = mLLL[i][0].x; 
    A(1) = mLLL[i][0].y; 
    A(2) = mLLL[i][0].z;

    B(0) = mLLL[i][1].x; 
    B(1) = mLLL[i][1].y; 
    B(2) = mLLL[i][1].z;
 
    C(0) = mLLL[i][2].x; 
    C(1) = mLLL[i][2].y; 
    C(2) = mLLL[i][2].z;


    Eigen::MatrixXf r0, r1, r2;
    Eigen::MatrixXf Tt;
    Tt = mT[0];
    r0 = ( B.transpose() )*Tt*C; 
    Ap(0) = r0(0,0);
    Tt = mT[1];
    r1 = ( B.transpose() )*Tt*C; 
    Ap(1) = r1(0,0);
    Tt = mT[2];
    r2 = ( B.transpose() )*Tt*C; 
    Ap(2) = r2(0,0);

    // Normalize Ap
    float temp = A(2) / Ap(2);
    float num;
    num = Ap(0)*temp; Ap(0) = num;
    num = Ap(1)*temp; Ap(1) = num;
    num = Ap(2)*temp; Ap(2) = num;

    std::cout <<" ("<<i<<") " <<" A:  " << A.transpose()  << std::endl;
    std::cout <<" ("<<i<<") " <<" Ap: " << Ap.transpose()  << std::endl;
  }
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
 * @function showCorrespondences
 */
void trifocalTensor::showCorrespondences() {

  // Create images
  mCorrImages.resize(0);
  cv::Scalar color;
  cv::Point p, start, end;
  int refx1 = 50; int refx2 = 450;

  for( int i = 0; i < mRgbImages.size(); ++i ) {
    cv::Mat image = mRgbImages[i].clone();
    mCorrImages.push_back(image);
  }


  // Draw PPP
  for( int i = 0; i < mPPP.size(); ++i ) {
    color = cv::Scalar( mRng.uniform(0,255), mRng.uniform(0,255), mRng.uniform(0,255) );
    for( int j = 0; j < 3; ++j ) {
      p.x =  (mPPP[i][j].x); p.y = (mPPP[i][j].y);
			    
      cv::circle( mCorrImages[j],
		  p,
		  mDrawingPointRadius, color, -1, 8 );
    }
  }


  // Draw LLL  ax + by + c = 0  y = (-c - ax ) / b *** x = (-c - by) / a  
    for( int i = 0; i < mLLL.size(); ++i ) {
      color = cv::Scalar( mRng.uniform(0,255), mRng.uniform(0,255), mRng.uniform(0,255) );
      for( int j = 0; j < 3; ++j ) {

	start.x = refx1; 
	start.y = (int) ( (-1.0*mLLL[i][j].z -1.0*mLLL[i][j].x*refx1 ) / mLLL[i][j].y );

	end.x = refx2;
	end.y = (int) ( (-1.0*mLLL[i][j].z -1.0*mLLL[i][j].x*refx2 ) / mLLL[i][j].y );

	line( mCorrImages[j], 
	      start, end,
	      color, mDrawingLineThickness, 8 );
	
      }
    }

  // Draw PLP 
    for( int i = 0; i < mPLP.size(); ++i ) {

      color = cv::Scalar( mRng.uniform(0,255), mRng.uniform(0,255), mRng.uniform(0,255) );
      // P
      p.x = (mPLP[i][0].x); 
      p.y = (mPLP[i][0].y);			    
      cv::circle( mCorrImages[0], p, mDrawingPointRadius, color, -1, 8 );
      
      // L
      start.x = refx1; 
      start.y = (int) ( (-1.0*mPLP[i][1].z -1.0*mPLP[i][1].x*refx1 ) / mPLP[i][1].y );

      end.x = refx2;
      end.y = (int) ( (-1.0*mPLP[i][1].z -1.0*mPLP[i][1].x*refx2 ) / mPLP[i][1].y );

      line( mCorrImages[1], 
	    start, end,
	    color, mDrawingLineThickness, 8 );
      // P
      p.x = (mPLP[i][2].x); p.y = (mPLP[i][2].y);			    
      cv::circle( mCorrImages[2], p, mDrawingPointRadius, color, -1, 8 );
    }

  // Draw PLL 
    for( int i = 0; i < mPLL.size(); ++i ) {

      color = cv::Scalar( mRng.uniform(0,255), mRng.uniform(0,255), mRng.uniform(0,255) );
      // P
      p.x = (mPLL[i][0].x); 
      p.y = (mPLL[i][0].y);			    
      cv::circle( mCorrImages[0], p, mDrawingPointRadius, color, -1, 8 );
      
      // L
      start.x = refx1; 
      start.y = (int) ( (-1.0*mPLL[i][1].z -1.0*mPLL[i][1].x*refx1 ) / mPLL[i][1].y );

      end.x = refx2;
      end.y = (int) ( (-1.0*mPLL[i][1].z -1.0*mPLL[i][1].x*refx2 ) / mPLL[i][1].y );

      line( mCorrImages[1], 
	    start, end,
	    color, mDrawingLineThickness, 8 );
      // L
      start.x = refx1; 
      start.y = (int) ( (-1.0*mPLL[i][2].z -1.0*mPLL[i][2].x*refx1 ) / mPLL[i][2].y );

      end.x = refx2;
      end.y = (int) ( (-1.0*mPLL[i][2].z -1.0*mPLL[i][2].x*refx2 ) / mPLL[i][2].y );

      line( mCorrImages[2], 
	    start, end,
	    color, mDrawingLineThickness, 8 );

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
