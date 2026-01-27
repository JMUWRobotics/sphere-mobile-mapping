#ifndef AUTOGAIN_H
#define AUTOGAIN_H

#include "filter.hpp"
#include "utils.hpp"

class AutogainFilter : public AttitudeEstimator {
    private:
        float gain_;
        float gain_min; 
        float alpha;
        float autogain;

    public:
        AutogainFilter(float gain_min, float alpha, float autogain) : 
            gain_(gain_min), gain_min(gain_min), alpha(alpha), autogain(autogain) 
        {}; 
        
        quaternion filter(const Vec3 &gyr, const Vec3 &acc, float dt) override;
};

#endif /* AUTOGAIN_H */