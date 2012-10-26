#include "Tensor3D.h"

int main( int argc, char** argv ) {
  
  Tensor3D t3;
  t3.loadImages( argv[1] );
  t3.loadCorrespondences( argv[2] );
  t3.showCorrespondences();
  t3.testPPP();
  return 0;
}
