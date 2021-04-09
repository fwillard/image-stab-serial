//
//  util.hpp
//  ImageStab
//
//  Created by Finn Willard on 4/9/21.
//

#ifndef util_hpp
#define util_hpp

#include <stdio.h>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

#endif /* util_hpp */
