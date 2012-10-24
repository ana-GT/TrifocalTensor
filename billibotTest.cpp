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
  // Load correspondences
  tt.loadCorrespondences( argv[2] );
  // Show correspondences
  tt.showCorrespondences();
  // Fill Eq
  tt.fillEq();
  // Find T1, T2 and T3
  tt.calculateTrifocalTensor();

  printf("Testing \n");
  return 0;
}
