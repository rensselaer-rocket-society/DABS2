#include "matrixmath.hpp"

namespace Control {

void on_IMU(const Vec<3>& a, const Vec<3>& g, const Vec<3>& m);
void on_Altimeter(float altitude);

void Tasks();

}