/**
 * @file billibotTest.cpp
 * @author A. Huaman
 */

#include <stdio.h>
#include "trifocalTensor.h"

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  printf("Creating trifocal Tensor object \n");
  trifocalTensor tt;
  tt.loadImages( argv[1] );
  tt.showImages();
  
  // Hard-code: Data given
  tt.loadCorrespondences( argv[2] );


  printf("Testing \n");
  return 0;
}
