#ifndef FILTER_INTERFACE_H
#define FILTER_INTERFACE_H

#include "utils.hpp"

class AttitudeEstimator {
    protected:
        float q0, q1, q2, q3; /* quaternion internal state */
    
    public:
        virtual quaternion filter(const Vec3 &gyr, const Vec3 &acc, float dt) = 0;
        
        /* Use this with care !!! Only if you know exactly what you are doing ! */
        void override_state(float qw, float qx, float qy, float qz) {
            q0 = qw;
            q1 = qx;
            q2 = qy;
            q3 = qz;
            normalizeQuat(&q0, &q1, &q2, &q3);
        };
};

#endif