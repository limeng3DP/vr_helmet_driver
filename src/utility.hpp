#ifndef UTILITY_HPP
#define UTILITY_HPP

class UtilityMath
{
public:
    // Utility math functions:
    static void normalizeVector(double& x, double& y, double& z);

    static void normalizeQuaternion(double& q0, double& q1, double& q2, double& q3);

    static void scaleQuaternion(double gain,double& dq0, double& dq1, double& dq2, double& dq3);

    static void invertQuaternion(double q0, double q1, double q2, double q3,
                                 double& q0_inv, double& q1_inv, double& q2_inv, double& q3_inv);

    static void quaternionMultiplication(double p0, double p1, double p2, double p3,
                                         double q0, double q1, double q2, double q3,
                                         double& r0, double& r1, double& r2, double& r3);

    static void rotateVectorByQuaternion(double x, double y, double z,
                                         double q0, double q1, double q2, double q3,
                                         double& vx, double& vy, double& vz);
};


void UtilityMath::normalizeVector(double& x, double& y, double& z)
{
    double norm = sqrt(x*x + y*y + z*z);
    x /= norm;
    y /= norm;
    z /= norm;
}

void UtilityMath::normalizeQuaternion(double& q0, double& q1, double& q2, double& q3)
{
    double norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}

void UtilityMath::invertQuaternion(double q0, double q1, double q2, double q3,
                                   double& q0_inv, double& q1_inv, double& q2_inv, double& q3_inv)
{
    // Assumes quaternion is normalized.
    q0_inv = q0;
    q1_inv = -q1;
    q2_inv = -q2;
    q3_inv = -q3;
}

void UtilityMath::scaleQuaternion(double gain,double& dq0, double& dq1, double& dq2, double& dq3)
{
    if (dq0 < 0.0)//0.9
    {
        // Slerp (Spherical linear interpolation):
        double angle = acos(dq0);
        double A = sin(angle*(1.0 - gain))/sin(angle);
        double B = sin(angle * gain)/sin(angle);
        dq0 = A + B * dq0;
        dq1 = B * dq1;
        dq2 = B * dq2;
        dq3 = B * dq3;
    }
    else
    {
        // Lerp (Linear interpolation):
        dq0 = (1.0 - gain) + gain * dq0;
        dq1 = gain * dq1;
        dq2 = gain * dq2;
        dq3 = gain * dq3;
    }

    normalizeQuaternion(dq0, dq1, dq2, dq3);
}

void quaternionMultiplication(double p0, double p1, double p2, double p3,
                              double q0, double q1, double q2, double q3,
                              double& r0, double& r1, double& r2, double& r3)
{
    // r = p q
    r0 = p0*q0 - p1*q1 - p2*q2 - p3*q3;
    r1 = p0*q1 + p1*q0 + p2*q3 - p3*q2;
    r2 = p0*q2 - p1*q3 + p2*q0 + p3*q1;
    r3 = p0*q3 + p1*q2 - p2*q1 + p3*q0;
}

void rotateVectorByQuaternion( double x, double y, double z,
                               double q0, double q1, double q2, double q3,
                               double& vx, double& vy, double& vz)
{
    vx = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*x + 2*(q1*q2 - q0*q3)*y + 2*(q1*q3 + q0*q2)*z;
    vy = 2*(q1*q2 + q0*q3)*x + (q0*q0 - q1*q1 + q2*q2 - q3*q3)*y + 2*(q2*q3 - q0*q1)*z;
    vz = 2*(q1*q3 - q0*q2)*x + 2*(q2*q3 + q0*q1)*y + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*z;
}

#endif // UTILITY_HPP
