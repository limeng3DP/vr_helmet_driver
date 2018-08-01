#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP
#include <cmath>
#include "utility.hpp"
class PoseEstimatorMadgwick
{
public:
    PoseEstimatorMadgwick(float frequency):sample_freq(frequency)
    {}
    void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
    {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
        {
            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

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
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * (1.0f / sample_freq);
        q1 += qDot2 * (1.0f / sample_freq);
        q2 += qDot3 * (1.0f / sample_freq);
        q3 += qDot4 * (1.0f / sample_freq);

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }


    void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float hx, hy;
        float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
            MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
            return;
        }

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Normalise magnetometer measurement
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0mx = 2.0f * q0 * mx;
            _2q0my = 2.0f * q0 * my;
            _2q0mz = 2.0f * q0 * mz;
            _2q1mx = 2.0f * q1 * mx;
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _2q0q2 = 2.0f * q0 * q2;
            _2q2q3 = 2.0f * q2 * q3;
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;

            // Reference direction of Earth's magnetic field
            hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * (1.0f / sample_freq);
        q1 += qDot2 * (1.0f / sample_freq);
        q2 += qDot3 * (1.0f / sample_freq);
        q3 += qDot4 * (1.0f / sample_freq);

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }
    void getPose(float &q_w,float &q_x,float &q_y,float &q_z)
    {
        q_w = q0;
        q_x = q1;
        q_y = q2;
        q_z = q3;
    }

private:
    // Fast inverse square-root
    // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
    const int instability_fix = 1;
    float q0 = 1.0;
    float q1 = 0.0;
    float q2 = 0.0;
    float q3 = 0.0;
    float sample_freq = 1000.0; // sample frequency in Hz
    float beta =	1.0f;	// 2 * proportional gain


    float invSqrt(float x)
    {
        if (instability_fix == 0)
        {
            /* original code */
            float halfx = 0.5f * x;
            float y = x;
            long i = *(long*)&y;
            i = 0x5f3759df - (i>>1);
            y = *(float*)&i;
            y = y * (1.5f - (halfx * y * y));
            return y;
        }
        else if (instability_fix == 1)
        {
            /* close-to-optimal  method with low cost from http://pizer.wordpress.com/2008/10/12/fast-inverse-square-root */
            unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
            float tmp = *(float*)&i;
            return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
        }
        else
        {
            /* optimal but expensive method: */
            return 1.0f / sqrtf(x);
        }
    }
};

//code from ros imu_tools package
//Note: in function getMeasurement(),the original code doesn't consider lx < 0 when computing q_mag

class ComplementaryFilter
{
  public:
    ComplementaryFilter();
    virtual ~ComplementaryFilter();

    bool setGainAcc(double gain);
    bool setGainMag(double gain);
    double getGainAcc() const;
    double getGainMag() const;

    bool setBiasAlpha(double bias_alpha);
    double getBiasAlpha() const;

    // When the filter is in the steady state, bias estimation will occur (if the
    // parameter is enabled).
    bool getSteadyState() const;

    void setDoBiasEstimation(bool do_bias_estimation);
    bool getDoBiasEstimation() const;

    void setDoAdaptiveGain(bool do_adaptive_gain);
    bool getDoAdaptiveGain() const;

    double getAngularVelocityBiasX() const;
    double getAngularVelocityBiasY() const;
    double getAngularVelocityBiasZ() const;

    // Set the orientation, as a Hamilton Quaternion, of the body frame wrt the
    // fixed frame.
    void setOrientation(double q0, double q1, double q2, double q3);

    // Get the orientation, as a Hamilton Quaternion, of the body frame wrt the
    // fixed frame.
    void getOrientation(double& q0, double& q1, double& q2, double& q3) const;

    // Update from accelerometer and gyroscope data.
    // [ax, ay, az]: Normalized gravity vector.
    // [wx, wy, wz]: Angular veloctiy, in rad / s.
    // dt: time delta, in seconds.
    void update(double ax, double ay, double az,
                double wx, double wy, double wz,
                double dt);

    // Update from accelerometer, gyroscope, and magnetometer data.
    // [ax, ay, az]: Normalized gravity vector.
    // [wx, wy, wz]: Angular veloctiy, in rad / s.
    // [mx, my, mz]: Magnetic field, units irrelevant.
    // dt: time delta, in seconds.
    void update(double ax, double ay, double az,
                double wx, double wy, double wz,
                double mx, double my, double mz,
                double dt);
private:
    static const double kGravity;
    static const double gamma_;
    // Bias estimation steady state thresholds
    static const double kAngularVelocityThreshold;
    static const double kAccelerationThreshold;
    static const double kDeltaAngularVelocityThreshold;

    // Gain parameter for the complementary filter, belongs in [0, 1].
    double gain_acc_;
    double gain_mag_;

    // Bias estimation gain parameter, belongs in [0, 1].
    double bias_alpha_;

    // Parameter whether to do bias estimation or not.
    bool do_bias_estimation_;

    // Parameter whether to do adaptive gain or not.
    bool do_adaptive_gain_;

    bool initialized_;
    bool steady_state_;

    // The orientation as a Hamilton quaternion (q0 is the scalar). Represents
    // the orientation of the fixed frame wrt the body frame.
    double q0_, q1_, q2_, q3_;

    // Bias in angular velocities;
    double wx_prev_, wy_prev_, wz_prev_;

    // Bias in angular velocities;
    double wx_bias_, wy_bias_, wz_bias_;

    void updateBiases(double ax, double ay, double az,
                      double wx, double wy, double wz);

    bool checkState(double ax, double ay, double az,
                    double wx, double wy, double wz) const;

    void getPrediction(double wx, double wy, double wz, double dt,
                       double& q0_pred, double& q1_pred, double& q2_pred, double& q3_pred) const;

    void getMeasurement(double ax, double ay, double az,
                        double& q0_meas, double& q1_meas, double& q2_meas, double& q3_meas);

    void getMeasurement(double ax, double ay, double az,
                        double mx, double my, double mz,
                        double& q0_meas, double& q1_meas, double& q2_meas, double& q3_meas);

    void getAccCorrection(double ax, double ay, double az,
                          double p0, double p1, double p2, double p3,
                          double& dq0, double& dq1, double& dq2, double& dq3);

    void getMagCorrection(double mx, double my, double mz,
                          double p0, double p1, double p2, double p3,
                          double& dq0, double& dq1, double& dq2, double& dq3);

    double getAdaptiveGain(double alpha, double ax, double ay, double az);
};

ComplementaryFilter::ComplementaryFilter() :
    gain_acc_(0.01),
    gain_mag_(0.01),
    bias_alpha_(0.01),
    do_bias_estimation_(true),
    do_adaptive_gain_(true),
    initialized_(false),
    steady_state_(false),
    q0_(1), q1_(0), q2_(0), q3_(0),
    wx_prev_(0), wy_prev_(0), wz_prev_(0),
    wx_bias_(0), wy_bias_(0), wz_bias_(0)
{ }

ComplementaryFilter::~ComplementaryFilter() { }

void ComplementaryFilter::setDoBiasEstimation(bool do_bias_estimation)
{
    do_bias_estimation_ = do_bias_estimation;
}

bool ComplementaryFilter::getDoBiasEstimation() const
{
    return do_bias_estimation_;
}

void ComplementaryFilter::setDoAdaptiveGain(bool do_adaptive_gain)
{
    do_adaptive_gain_ = do_adaptive_gain;
}

bool ComplementaryFilter::getDoAdaptiveGain() const
{
    return do_adaptive_gain_;
}

bool ComplementaryFilter::setGainAcc(double gain)
{
    if (gain >= 0 && gain <= 1.0)
    {
        gain_acc_ = gain;
        return true;
    }
    else
        return false;
}
bool ComplementaryFilter::setGainMag(double gain)
{
  if (gain >= 0 && gain <= 1.0)
  {
      gain_mag_ = gain;
      return true;
  }
  else
      return false;
}

double ComplementaryFilter::getGainAcc() const
{
    return gain_acc_;
}

double ComplementaryFilter::getGainMag() const
{
    return gain_mag_;
}

bool ComplementaryFilter::getSteadyState() const
{
    return steady_state_;
}

bool ComplementaryFilter::setBiasAlpha(double bias_alpha)
{
    if (bias_alpha >= 0 && bias_alpha <= 1.0)
    {
        bias_alpha_ = bias_alpha;
        return true;
    }
    else
        return false;
}

double ComplementaryFilter::getBiasAlpha() const
{
    return bias_alpha_;
}

void ComplementaryFilter::setOrientation(double q0, double q1, double q2, double q3)
{
    // Set the state to inverse (state is fixed wrt body).
    UtilityMath::invertQuaternion(q0, q1, q2, q3, q0_, q1_, q2_, q3_);
}
double ComplementaryFilter::getAngularVelocityBiasX() const
{
    return wx_bias_;
}
double ComplementaryFilter::getAngularVelocityBiasY() const
{
    return wy_bias_;
}

double ComplementaryFilter::getAngularVelocityBiasZ() const
{
    return wz_bias_;
}

void ComplementaryFilter::update(double ax, double ay, double az,
                                 double wx, double wy, double wz,
                                 double dt)
{
    if (!initialized_)
    {
        // First time - ignore prediction:
        getMeasurement(ax, ay, az,q0_, q1_, q2_, q3_);
        initialized_ = true;
        return;
    }
    // Bias estimation.
    if (do_bias_estimation_)
        updateBiases(ax, ay, az, wx, wy, wz);

    // Prediction.
    double q0_pred, q1_pred, q2_pred, q3_pred;
    getPrediction(wx, wy, wz, dt, q0_pred, q1_pred, q2_pred, q3_pred);

    // Correction (from acc):
    // q_ = q_pred * [(1-gain) * qI + gain * dq_acc]
    // where qI = identity quaternion
    double dq0_acc, dq1_acc, dq2_acc, dq3_acc;
    getAccCorrection(ax, ay, az, q0_pred, q1_pred, q2_pred, q3_pred,
                     dq0_acc, dq1_acc, dq2_acc, dq3_acc);
    double gain;
    if (do_adaptive_gain_)
    {
        gain = getAdaptiveGain(gain_acc_, ax, ay, az);
    }
    else
    {
        gain = gain_acc_;
    }

    UtilityMath::scaleQuaternion(gain, dq0_acc, dq1_acc, dq2_acc, dq3_acc);

    quaternionMultiplication(q0_pred, q1_pred, q2_pred, q3_pred,
                             dq0_acc, dq1_acc, dq2_acc, dq3_acc,
                             q0_, q1_, q2_, q3_);

    UtilityMath::normalizeQuaternion(q0_, q1_, q2_, q3_);
}

void ComplementaryFilter::update(double ax, double ay, double az,
                                 double wx, double wy, double wz,
                                 double mx, double my, double mz,
                                 double dt)
{
    if (!initialized_)
    {
        // First time - ignore prediction:
        getMeasurement(ax, ay, az,mx, my, mz,q0_, q1_, q2_, q3_);
        initialized_ = true;
        return;
    }
    // Bias estimation.
    if (do_bias_estimation_)
        updateBiases(ax, ay, az, wx, wy, wz);
    // Prediction.
    double q0_pred, q1_pred, q2_pred, q3_pred;
    getPrediction(wx, wy, wz, dt, q0_pred, q1_pred, q2_pred, q3_pred);
    // Correction (from acc):
    // q_temp = q_pred * [(1-gain) * qI + gain * dq_acc]
    // where qI = identity quaternion
    double dq0_acc, dq1_acc, dq2_acc, dq3_acc;
    getAccCorrection(ax, ay, az,q0_pred, q1_pred, q2_pred, q3_pred,
                     dq0_acc, dq1_acc, dq2_acc, dq3_acc);
    double alpha = gain_acc_;
    if (do_adaptive_gain_)
        alpha = getAdaptiveGain(gain_acc_, ax, ay, az);
    UtilityMath::scaleQuaternion(alpha, dq0_acc, dq1_acc, dq2_acc, dq3_acc);

    double q0_temp, q1_temp, q2_temp, q3_temp;
    quaternionMultiplication(q0_pred, q1_pred, q2_pred, q3_pred,
                             dq0_acc, dq1_acc, dq2_acc, dq3_acc,
                             q0_temp, q1_temp, q2_temp, q3_temp);

    // Correction (from mag):
    // q_ = q_temp * [(1-gain) * qI + gain * dq_mag]
    // where qI = identity quaternion
    double dq0_mag, dq1_mag, dq2_mag, dq3_mag;
    getMagCorrection(mx, my, mz, q0_temp, q1_temp, q2_temp, q3_temp,
                     dq0_mag, dq1_mag, dq2_mag, dq3_mag);

    UtilityMath::scaleQuaternion(gain_mag_, dq0_mag, dq1_mag, dq2_mag, dq3_mag);

    quaternionMultiplication(q0_temp, q1_temp, q2_temp, q3_temp,
                             dq0_mag, dq1_mag, dq2_mag, dq3_mag,
                             q0_, q1_, q2_, q3_);

    UtilityMath::normalizeQuaternion(q0_, q1_, q2_, q3_);
}

bool ComplementaryFilter::checkState(double ax, double ay, double az,
                                     double wx, double wy, double wz) const
{
    double acc_magnitude = sqrt(ax*ax + ay*ay + az*az);
    if (fabs(acc_magnitude - kGravity) > kAccelerationThreshold)
        return false;
    if (fabs(wx - wx_prev_) > kDeltaAngularVelocityThreshold ||
            fabs(wy - wy_prev_) > kDeltaAngularVelocityThreshold ||
            fabs(wz - wz_prev_) > kDeltaAngularVelocityThreshold)
        return false;

    if (fabs(wx - wx_bias_) > kAngularVelocityThreshold ||
            fabs(wy - wy_bias_) > kAngularVelocityThreshold ||
            fabs(wz - wz_bias_) > kAngularVelocityThreshold)
        return false;

    return true;
}

void ComplementaryFilter::updateBiases(double ax, double ay, double az,
                                       double wx, double wy, double wz)
{
    steady_state_ = checkState(ax, ay, az, wx, wy, wz);
    if (steady_state_)
    {
        wx_bias_ += bias_alpha_ * (wx - wx_bias_);
        wy_bias_ += bias_alpha_ * (wy - wy_bias_);
        wz_bias_ += bias_alpha_ * (wz - wz_bias_);
    }
    wx_prev_ = wx;
    wy_prev_ = wy;
    wz_prev_ = wz;
}

void ComplementaryFilter::getPrediction(double wx, double wy, double wz, double dt,
                                        double& q0_pred, double& q1_pred, double& q2_pred, double& q3_pred) const
{
    double wx_unb = wx - wx_bias_;
    double wy_unb = wy - wy_bias_;
    double wz_unb = wz - wz_bias_;

    q0_pred = q0_ + 0.5*dt*( wx_unb*q1_ + wy_unb*q2_ + wz_unb*q3_);
    q1_pred = q1_ + 0.5*dt*(-wx_unb*q0_ - wy_unb*q3_ + wz_unb*q2_);
    q2_pred = q2_ + 0.5*dt*( wx_unb*q3_ - wy_unb*q0_ - wz_unb*q1_);
    q3_pred = q3_ + 0.5*dt*(-wx_unb*q2_ + wy_unb*q1_ - wz_unb*q0_);

    UtilityMath::normalizeQuaternion(q0_pred, q1_pred, q2_pred, q3_pred);
}

void ComplementaryFilter::getMeasurement(double ax, double ay, double az,
                                         double mx, double my, double mz,
                                         double& q0_meas, double& q1_meas, double& q2_meas, double& q3_meas)
{
    // q_acc is the quaternion obtained from the acceleration vector representing
    // the orientation of the Global frame wrt the Local frame with arbitrary yaw
    // (intermediary frame). q3_acc is defined as 0.
    double q0_acc, q1_acc, q2_acc, q3_acc;

    // Normalize acceleration vector.
    UtilityMath::normalizeVector(ax, ay, az);
    if (az >=0)
    {
        q0_acc =  sqrt((az + 1) * 0.5);
        q1_acc = -ay/(2.0 * q0_acc);
        q2_acc =  ax/(2.0 * q0_acc);
        q3_acc = 0;
    }
    else
    {
        double X = sqrt((1 - az) * 0.5);
        q0_acc = -ay/(2.0 * X);
        q1_acc = X;
        q2_acc = 0;
        q3_acc = ax/(2.0 * X);
    }

    // [lx, ly, lz] is the magnetic field reading, rotated into the intermediary
    // frame by the inverse of q_acc.
    // l = R(q_acc)^-1 m
    double lx = (q0_acc*q0_acc + q1_acc*q1_acc - q2_acc*q2_acc)*mx +
            2.0 * (q1_acc*q2_acc)*my - 2.0 * (q0_acc*q2_acc)*mz;
    double ly = 2.0 * (q1_acc*q2_acc)*mx + (q0_acc*q0_acc - q1_acc*q1_acc +
                                            q2_acc*q2_acc)*my + 2.0 * (q0_acc*q1_acc)*mz;
    /* need to consider the case lx < 0 */



    // q_mag is the quaternion that rotates the Global frame (North West Up) into
    // the intermediary frame. q1_mag and q2_mag are defined as 0.
    double gamma = lx*lx + ly*ly;
    double q0_mag,q3_mag,beta;
    if(lx >= 0)
    {
        beta = sqrt(gamma + lx*sqrt(gamma));
        q0_mag = beta / (sqrt(2.0 * gamma));
        q3_mag = ly / (sqrt(2.0) * beta);
    }
    else
    {
        beta = sqrt(gamma - lx * sqrt(gamma));
        q0_mag = ly / (sqrt(2.0) * beta);
        q3_mag = beta / (sqrt(2.0 * gamma));
    }
    // The quaternion multiplication between q_acc and q_mag represents the
    // quaternion, orientation of the Global frame wrt the local frame.
    // q = q_acc times q_mag
    quaternionMultiplication(q0_acc, q1_acc, q2_acc, q3_acc,
                             q0_mag, 0, 0, q3_mag,
                             q0_meas, q1_meas, q2_meas, q3_meas );
    //q0_meas = q0_acc*q0_mag;
    //q1_meas = q1_acc*q0_mag + q2_acc*q3_mag;
    //q2_meas = q2_acc*q0_mag - q1_acc*q3_mag;
    //q3_meas = q0_acc*q3_mag;
}


void ComplementaryFilter::getMeasurement(double ax, double ay, double az,
                                         double& q0_meas, double& q1_meas, double& q2_meas, double& q3_meas)
{
    // q_acc is the quaternion obtained from the acceleration vector representing
    // the orientation of the Global frame wrt the Local frame with arbitrary yaw
    // (intermediary frame). q3_acc is defined as 0.
    // Normalize acceleration vector.
    UtilityMath::normalizeVector(ax, ay, az);

    if (az >=0)
    {
        q0_meas =  sqrt((az + 1) * 0.5);
        q1_meas = -ay/(2.0 * q0_meas);
        q2_meas =  ax/(2.0 * q0_meas);
        q3_meas = 0;
    }
    else
    {
        double X = sqrt((1 - az) * 0.5);
        q0_meas = -ay/(2.0 * X);
        q1_meas = X;
        q2_meas = 0;
        q3_meas = ax/(2.0 * X);
    }
}

void ComplementaryFilter::getAccCorrection(double ax, double ay, double az,
                                           double p0, double p1, double p2, double p3,
                                           double& dq0, double& dq1, double& dq2, double& dq3)
{
    // Normalize acceleration vector.
    UtilityMath::normalizeVector(ax, ay, az);

    // Acceleration reading rotated into the world frame by the inverse predicted
    // quaternion (predicted gravity):
    double gx, gy, gz;
    rotateVectorByQuaternion(ax, ay, az,p0, -p1, -p2, -p3,gx, gy, gz);

    // Delta quaternion that rotates the predicted gravity into the real gravity:
    dq0 =  sqrt((gz + 1) * 0.5);
    dq1 = -gy/(2.0 * dq0);
    dq2 =  gx/(2.0 * dq0);
    dq3 =  0.0;
}

void ComplementaryFilter::getMagCorrection(double mx, double my, double mz,
                                           double p0, double p1, double p2, double p3,
                                           double& dq0, double& dq1, double& dq2, double& dq3)
{
    // Magnetic reading rotated into the world frame by the inverse predicted
    // quaternion:
    double lx, ly, lz;
    rotateVectorByQuaternion(mx, my, mz,p0, -p1, -p2, -p3,lx, ly, lz);

    // Delta quaternion that rotates the l so that it lies in the xz-plane
    // (points north):
    double gamma = lx*lx + ly*ly;
    double beta = sqrt(gamma + lx*sqrt(gamma));
    dq0 = beta / (sqrt(2.0 * gamma));
    dq1 = 0.0;
    dq2 = 0.0;
    dq3 = ly / (sqrt(2.0) * beta);
}

void ComplementaryFilter::getOrientation(double& q0, double& q1, double& q2, double& q3) const
{
    // Return the inverse of the state (state is fixed wrt body).
    UtilityMath::invertQuaternion(q0_, q1_, q2_, q3_, q0, q1, q2, q3);
}

double ComplementaryFilter::getAdaptiveGain(double alpha, double ax, double ay, double az)
{
    double a_mag = sqrt(ax*ax + ay*ay + az*az);
    double error = fabs(a_mag - kGravity)/kGravity;
    double factor;
    double error1 = 0.1;
    double error2 = 0.2;
    double m = 1.0/(error1 - error2);
    double b = 1.0 - m*error1;
    if (error < error1)
        factor = 1.0;
    else if (error < error2)
        factor = m*error + b;
    else
        factor = 0.0;
    //printf("FACTOR: %f \n", factor);
    return factor*alpha;
}

const double ComplementaryFilter::kGravity = 9.81;
const double ComplementaryFilter::gamma_ = 0.01;
// Bias estimation steady state thresholds
const double ComplementaryFilter::kAngularVelocityThreshold = 0.2;
const double ComplementaryFilter::kAccelerationThreshold = 0.1;
const double ComplementaryFilter::kDeltaAngularVelocityThreshold = 0.01;

#endif //POSE_ESTIMATOR_HPP
