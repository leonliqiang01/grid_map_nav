#ifndef __QUADRATIC_CALCULATOR_H__
#define __QUADRATIC_CALCULATOR_H__

#include <vector>
#include "potential_calculator.h"

namespace global_planner {

class QuadraticCalculator : public PotentialCalculator {
    public:
        QuadraticCalculator(int nx, int ny): PotentialCalculator(nx,ny) {}

        float calculatePotential(float* potential, unsigned char cost, int n, float prev_potential);
};


} //end namespace global_planner
#endif
