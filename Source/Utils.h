/*
  ==============================================================================

    Utils.h
    Created: 19 Feb 2018 10:57:39am
    Author:  Joseph Dinius

  ==============================================================================
*/

#pragma once

#include "Eigen/Dense"
#include <random>

Eigen::Vector2d mvnrnd(Eigen::Vector2d mean, Eigen::Matrix2d var, Eigen::Vector2d z);