/*
 * OpenPlanner C++ Core Implementation (op_local_planner_src/include/op_frenet_planner.h)
 * Adapted from OpenPlanner C++ Libraries for local_path_planner
 */

#ifndef OP_FRENET_PLANNER_H
#define OP_FRENET_PLANNER_H

#include <vector>
#include <cmath>
#include <algorithm>

namespace PlannerHNS {

class QuinticPolynomialCore {
public:
    double a0, a1, a2, a3, a4, a5;

    QuinticPolynomialCore(double xs, double vxs, double axs, double xe, double vxe, double axe, double T) {
        a0 = xs;
        a1 = vxs;
        a2 = axs / 2.0;
        T = std::max(1.0, T);
        double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;

        double b0 = xe - a0 - a1 * T - a2 * T2;
        double b1 = vxe - a1 - 2.0 * a2 * T;
        double b2 = axe - 2.0 * a2;

        a3 = (10.0 * b0 / T3) - (4.0 * b1 / T2) + (0.5 * b2 / T);
        a4 = (-15.0 * b0 / T4) + (7.0 * b1 / T3) - (b2 / T2);
        a5 = (6.0 * b0 / T5) - (3.0 * b1 / T4) + (0.5 * b2 / T3);
    }

    double calc(double t) const {
        double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
        return a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5;
    }
};

struct WayPointCore {
    double x, y, z, yaw, v, cost;
    int id;
    WayPointCore(double _x=0, double _y=0, double _z=0, double _yaw=0, double _v=0)
        : x(_x), y(_y), z(_z), yaw(_yaw), v(_v), cost(0.0), id(0) {}
};

struct ObstacleCore {
    double x, y, r;
};

} // namespace PlannerHNS

#endif // OP_FRENET_PLANNER_H
