/*
  ==============================================================================

    Utils.cpp
    Created: 19 Feb 2018 10:57:39am
    Author:  Joseph Dinius

  ==============================================================================
*/

#include "Utils.h"

/*
 * create a multivariate normally distributed random vector
 * with specific mean and variance
*/
Eigen::Vector2d mvnrnd(Eigen::Vector2d mean, Eigen::Matrix2d var, Eigen::Vector2d z) {
    
    return mean + var.llt().matrixL() * z;
}