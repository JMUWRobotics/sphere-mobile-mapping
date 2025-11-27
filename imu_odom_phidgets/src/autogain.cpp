#include "autogain.hpp"

quaternion AutogainFilter::filter(const Vec3 &gyr, const Vec3 &acc, float dt)
{
    float gx = gyr.x, gy = gyr.y, gz = gyr.z;
    float ax = acc.x, ay = acc.y, az = acc.z;

    // Helper variables
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // calculating the factors describign if there is active rotation.
    float factorX = (0.05 * gx) * (0.05 * gx);
    float factorY = (0.05 * gy) * (0.05 * gy);
    float factorZ = (0.05 * gz) * (0.05 * gz);
    if (factorX > 1) factorX = 1;
    if (factorY > 1) factorY = 1;
    if (factorZ > 1) factorZ = 1;

    // adapt gains automatically
    if (!autogain <= 0)
    {
        float maxFac = std::max(std::max(factorY, factorZ), factorX);
        // alpha is one magnitude less, as the standard values for both are 0.2 and 0.02
        gain_ = autogain * maxFac;
        gain_ = std::max(gain_, gain_min);
        if (maxFac < 0.1)
            maxFac = 0;
        alpha = autogain * (1 - maxFac);
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) recipNorm = 1;
    else recipNorm = 1.0 / sqrt(ax * ax + ay * ay + az * az);
    // Normalize acc
    float axNom = ax * recipNorm;
    float ayNom = ay * recipNorm;
    float azNom = az * recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * axNom + _4q0 * q1q1 - _2q1 * ayNom;
    s1 = _4q1 * q3q3 - _2q3 * axNom + 4.0f * q0q0 * q1 - _2q0 * ayNom - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * azNom;
    s2 = 4.0f * q0q0 * q2 + _2q0 * axNom + _4q2 * q3q3 - _2q3 * ayNom - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * azNom;
    s3 = 4.0f * q1q1 * q3 - _2q1 * axNom + 4.0f * q2q2 * q3 - _2q2 * ayNom;
    recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= gain_ * s0;
    qDot2 -= gain_ * s1;
    qDot3 -= gain_ * s2;
    qDot4 -= gain_ * s3;

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    normalizeQuat(&q0, &q1, &q2, &q3);

    // Complementary Filter
    float acc_roll = atan2f(ayNom, azNom);
    float acc_pitch = atan2f(-axNom, sqrt(ayNom * ayNom + azNom * azNom));

    // get RPY from quaternion

    // acc_roll= atan(copysign(1.0,az)*ay/sqrt(0.01*ax*ax+az*az));
    float roll, pitch, yaw;
    eulerFromQuat(&roll, &pitch, &yaw, q0, q1, q2, q3);

    // the actual complementary step

    // gimbal lock avoidence
    if (fabs(q2 * q1 + q0 * q3) - 0.5 < 0.01) acc_roll = roll;

    float q00, q11, q22, q33;
    quatFromEuler(&q00, &q11, &q22, &q33, acc_roll, acc_pitch, yaw);

    quaternion qAcc, qCur, qNex;

    qAcc.w = q00;
    qAcc.x = q11;
    qAcc.y = q22;
    qAcc.z = q33;

    qCur.w = q0;
    qCur.x = q1;
    qCur.y = q2;
    qCur.z = q3;

    quaternion_slerp(&qCur, &qAcc, &qNex, alpha);

    q0 = qNex.w;
    q1 = qNex.x;
    q2 = qNex.y;
    q3 = qNex.z;

    normalizeQuat(&q0, &q1, &q2, &q3);

    return quaternion(q0, q1, q2, q3);
}

