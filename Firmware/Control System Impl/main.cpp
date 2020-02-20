#ifndef ARDUINO
#include <iostream>
#endif

#include "matrixmath.hpp"
#include "quaternion.hpp"
#include "pos_est.h"
#include "att_est.h"

int main() {
    // Square<2> eye = Square<2>::eye();
    // Square<2> other = eye;
    // other.at(1,0) = 2;
    // Square<2> res = other*eye;
    // Vec<2> res2 = other*Mat<2,1>(1);
    // auto res3 = (other-10*eye+3*(-res.transpose()));
    // auto res4 = inv(res3);

    Control::AttitudeEstimator est;
    est.setReference(vec3(0,0,1));
    // est.predict(vec3(0,1,0),1);
    for(int i = 0; i < 100; ++i)
        est.predict(vec3(0,0.1,0.1),0.1);
    auto modelledx = est.attitude;
    auto modelledcov = est.covar;
    auto cos1 = est.cosineTheta;
    est.update(vec3(0,0,1));
    auto measurex = est.attitude;
    auto measurecov = est.covar;
    auto cos2 = est.cosineTheta;

    // Quaternion iden(1,0,0,0);
    // Quaternion rotx90(1,1,0,0);
    // Quaternion roty90(1,0,1,0);
    // rotx90.normalize();
    // roty90.normalize();

    // Vec<3> z(0);
    // z.at(2,0) = 1;

    // Vec<3> zr1 = rotx90.rotate(z);
    // Vec<3> zr2comp = roty90.rotate(zr1);
    // Quaternion rotxy = roty90*rotx90;
    // Vec<3> zr2mult = rotxy.rotate(z);


    
#ifndef ARDUINO
    // std::cout << eye << other << res << res2 << normsquare(res2) << std::endl << res3 << res4;
    // std::cout << std::endl;
    std::cout << modelledx << modelledcov << cos1;
    std::cout << std::endl;
    std::cout << measurex << measurecov << cos2;
    std::cout << std::endl;
    // std::cout << iden << rotx90 << roty90 << rotxy << std::endl;
    // std::cout << z << zr2comp << zr2mult;

#endif
}