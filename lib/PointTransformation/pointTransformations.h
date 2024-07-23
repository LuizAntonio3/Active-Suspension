#pragma once

#ifndef POINT_TRANSFORMATIONS
#define POINT_TRANSFORMATIONS

#include <BasicLinearAlgebra.h>
#include <ThreePhaseEncoder.h>

using namespace BLA;

const Matrix<2, 1> lowerFixationOrigin = {146.224, 30.65};
const Matrix<2, 1> upperFixationOrigin = {35.55, 200.148};
const Matrix<2, 1> upperFixationOriginNegative = {-35.55, -200.148};
const Matrix<2, 1> upperAxisOrigin = {41.026, 191.775};
const Matrix<2, 1> unsprungMassOrigin = {250.863, 0};

// Calculates the distance between two points
double calculateDistance(Matrix<2, 1> pointA, Matrix<2, 1> pointB) {
    return sqrt(pow(pointA(0, 0) - pointB(0, 0), 2) + 
                pow(pointA(1, 0) - pointB(1, 0), 2));
}

// Lower Arm angle - Theta - in rad
Matrix<2, 1> getLocationLowerFixation(float theta) {
    Matrix<2,2> RotationEncoderArm;

    RotationEncoderArm = {cos(theta), - sin(theta), sin(theta), cos(theta)};

    return RotationEncoderArm * lowerFixationOrigin; 
}

// Single Link angle - Theta - in rad
Matrix<2, 1> getLocationUpperFixation(float theta) {
    Matrix<2,2> RotationSingleLink;

    RotationSingleLink = {cos(theta), - sin(theta), sin(theta), cos(theta)};
    
    return upperFixationOrigin+(RotationSingleLink*(upperFixationOriginNegative+upperFixationOrigin));
}

// Lower Arm angle - Theta - in rad
Matrix<2, 1> getLocationUnsprungMass(float theta) {
    Matrix<2,2> RotationEncoderArm;

    RotationEncoderArm = {cos(theta), - sin(theta), sin(theta), cos(theta)};

    return RotationEncoderArm * unsprungMassOrigin;

}

#endif