#ifndef IMU_NODE_HPP
#define IMU_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "vr_helmet_driver/LevelLog.h"
#include "pose_estimator.hpp"
extern "C"{
#include <stdlib.h>
#include <stdint.h>
#include <stdlib.h>
#include <hidapi/hidapi.h>
}

#define HOLOLENS_SENSORS_PACKET_REPORT_SIZE 497
#define HOLOLENS_SENSORS_CONTROL_REPORT_SIZE 33
#define HOLOLENS_SENSORS_VID 0x045e
#define HOLOLENS_SENSORS_PID 0x0659

#define IMU_ACC_COV 0.00001
#define IMU_GYRO_COV 0.00001

class ImuNode
{
/*
 * Raw IMU sample - a single measurement of acceleration, angular
 * velocity, and sample time. Units are hardware dependent and may
 * or may not be calibrated.
 */

struct raw_imu_sample
{
    uint64_t time;
    int32_t acc[3];
    int32_t gyro[3];
    int32_t mag[3];
};

/*
* Raw IMU sample - a single measurement of acceleration (in m/s²),
* angular velocity (in rad/s), magnetic field, and temperature,
* and sample time.
*/
struct imu_sample
{
    float acceleration[3];
    float angular_velocity[3];
    float magnetic_field[3];
    float temperature;
    double time;
};

enum hololens_sensors_report_id
{
    HOLOLENS_SENSORS_PACKET_REPORT_ID = 0x01,
    HOLOLENS_SENSORS_CONTROL_REPORT_ID = 0x02
};

struct hololens_imu_message
{
    uint8_t code;
    uint8_t text[57];
} __attribute__((packed));


struct hololens_sensors_packet_report
{
    uint8_t id; //1 bytes
    uint16_t temperature[4]; //8 bytes
    uint64_t gyro_timestamp[4];
    int16_t gyro[3][32];
    uint64_t accel_timestamp[4];
    int32_t accel[3][4];
    uint64_t magnet_timestamp[4];
    int16_t magnet[3][4];
    uint64_t frame_id;
    uint8_t code[4];
    hololens_imu_message message[2];
} __attribute__((packed));

struct hololens_sensors_control_report
{
    uint8_t id;
    uint8_t code;
    uint8_t len;
    uint8_t data[30];
} __attribute__((packed));

enum hololens_sensors_command
{
    HOLOLENS_SENSORS_COMMAND_CONFIG_START = 0x0b,
    HOLOLENS_SENSORS_COMMAND_CONFIG_META = 0x06,
    HOLOLENS_SENSORS_COMMAND_CONFIG_DATA = 0x04,
    HOLOLENS_SENSORS_COMMAND_CONFIG_READ = 0x08,
    HOLOLENS_SENSORS_COMMAND_START_IMU = 0x07
};

public:

struct wmr_imu
{
    hid_device* hmd_imu;
    hid_device* hmd_cfg;
    imu_sample data;
    uint64_t last_timestamp;
};
    ImuNode(ros::NodeHandle &_nh):nh(_nh)
    {
        imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_sample",3);
    }
    void operator ()()
    {
        imu_read();
    }
    void imu_read()
    {
        wmr_imu imu_dev;
        int ret = 0;
        unsigned char buffer[HOLOLENS_SENSORS_PACKET_REPORT_SIZE];
        ret = wmr_open_device(&imu_dev);

        while(ros::ok())
        {
            int size = hid_read(imu_dev.hmd_imu, buffer, HOLOLENS_SENSORS_PACKET_REPORT_SIZE);

            if (size < 0)
            {
                printf("error reading from device\n");
                return;
            }
            else if (size == 0)
            {
                continue; // No more messages, return.
            }
            // currently the only message type the hardware supports (I think)
            if (buffer[0] == HOLOLENS_SENSORS_PACKET_REPORT_ID)
            {
                imu_sample samples[4];
                wmr_handle_imu_report(&imu_dev, (hololens_sensors_packet_report *)buffer,samples);
                for(int i = 0; i < 4; ++i)
                {
                    imu_sample & sample = samples[i];
                    sensor_msgs::Imu imu_msg;
                    imu_msg.angular_velocity.x = sample.angular_velocity[0];
                    imu_msg.angular_velocity.y = sample.angular_velocity[1];
                    imu_msg.angular_velocity.z = sample.angular_velocity[2];

                    imu_msg.linear_acceleration.x = sample.acceleration[0];
                    imu_msg.linear_acceleration.y = sample.acceleration[1];
                    imu_msg.linear_acceleration.z = sample.acceleration[2];

                    imu_msg.angular_velocity_covariance = {IMU_GYRO_COV,0.0,0.0,0.0,IMU_GYRO_COV,0.0,0.0,0.0,IMU_GYRO_COV};
                    imu_msg.linear_acceleration_covariance = {IMU_ACC_COV,0.0,0.0,0.0,IMU_ACC_COV,0.0,0.0,0.0,IMU_ACC_COV};
                    imu_msg.header.stamp = ros::Time(sample.time);
                    imu_msg.header.frame_id="/imu";
                    if(!pose_est_initialized)
                    {
                        timestamp_prev = sample.time;
                        pose_est_initialized = true;

                        imu_msg.orientation.w = 1.0;
                        imu_msg.orientation.x = 0.0;
                        imu_msg.orientation.y = 0.0;
                        imu_msg.orientation.z = 0.0;
                        imu_msg.orientation_covariance = {-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
                        imu_pub.publish(imu_msg);
                        continue;
                    }
                    // determine dt: either constant, or from IMU timestamp
                    double dt;
                    if (constant_dt > 0)
                      dt = constant_dt;
                    else
                    {
                        dt = sample.time - timestamp_prev;
                        timestamp_prev = sample.time;
                    }

                    // Update the filter.
                    pose_est.update(sample.acceleration[0], sample.acceleration[1], sample.acceleration[2],
                                    sample.angular_velocity[0],sample.angular_velocity[1],sample.angular_velocity[2],dt);

                    // Get the orientation:
                    double q0, q1, q2, q3;
                    pose_est.getOrientation(q0, q1, q2, q3);
                    imu_msg.orientation.w = q0;
                    imu_msg.orientation.x = q1;
                    imu_msg.orientation.y = q2;
                    imu_msg.orientation.z = q3;
                    imu_msg.orientation_covariance = {-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
                    if(remove_gyro_bias)
                    {
                        imu_msg.angular_velocity.x -= pose_est.getAngularVelocityBiasX();
                        imu_msg.angular_velocity.y -= pose_est.getAngularVelocityBiasY();
                        imu_msg.angular_velocity.z -= pose_est.getAngularVelocityBiasZ();
                    }
                    imu_pub.publish(imu_msg);
                    //printf("%f s,  %f C\n", sample.time,sample.temperature);
                    //printf("Mag : %.6f,%.6f,%.6f\n", sample.magnetic_field[0], sample.magnetic_field[1], sample.magnetic_field[2]);
                }
            }
            else
            {
                printf("unknown message type: %u\n", buffer[0]);
            }
        }
        ret = wmr_close_device(&imu_dev);
    }

    int wmr_open_device(wmr_imu* imu)
    {
        if (!imu)
            return -1;
        int idx = 0;
        // Open the HMD device
        imu->hmd_imu = wmr_open_device_idx(HOLOLENS_SENSORS_VID, HOLOLENS_SENSORS_PID, 0, 1, idx);
        imu->hmd_cfg = imu->hmd_imu;//wmr_open_device_idx(HOLOLENS_SENSORS_VID, HOLOLENS_SENSORS_PID, 0, 1, idx + 1);

        if (!imu->hmd_imu)
        {
            goto cleanup;
        }
        if (!imu->hmd_cfg)
        {
            goto cleanup;
        }

        if (hid_set_nonblocking(imu->hmd_imu, 1) == -1)
        {
            printf("failed to set non-blocking on device");
            goto cleanup;
        }

        // turn the IMU on
        wmr_imu_start(imu);

        //wmr_imu_stop(imu);
        //uint8_t config_meta[66];
        //wmr_imu_send_command(imu, HOLOLENS_SENSORS_COMMAND_START_IMU, config_meta, sizeof(config_meta));
        return 0;
    cleanup:
        return -1;
    }

    int wmr_close_device(wmr_imu* imu)
    {
        printf("closing Microsoft HoloLens Sensors device");
        hid_close(imu->hmd_imu);
        //hid_close(imu->hmd_cfg);
        return 0;
    }

    hid_device* wmr_open_device_idx(int manufacturer, int product, int iface, int iface_tot, int device_index)
    {
        struct hid_device_info* devs = hid_enumerate(manufacturer, product);
        struct hid_device_info* cur_dev = devs;

        int idx = 0;
        int iface_cur = 0;
        hid_device* ret = NULL;

        while (cur_dev)
        {
            printf("%04x:%04x %s\n", manufacturer, product, cur_dev->path);

            if (idx == device_index && iface == iface_cur)
            {
                ret = hid_open_path(cur_dev->path);
            }
            cur_dev = cur_dev->next;
            iface_cur++;
            if (iface_cur >= iface_tot)
            {
                idx++;
                iface_cur = 0;
            }
        }
        hid_free_enumeration(devs);
        return ret;
    }

private:
    ros::NodeHandle nh;
    ros::Publisher imu_pub;
    ComplementaryFilter pose_est;
    const double constant_dt = -1.0;
    bool pose_est_initialized = false;
    double timestamp_prev = 0.0;
    const bool use_mag = false;
    const bool remove_gyro_bias = false;

private:
    int wmr_imu_read_config(wmr_imu *imu, uint8_t command,uint8_t *buf, size_t count)
    {
        hololens_sensors_control_report report;
        unsigned int offset = 0;
        int ret;

        ret = wmr_imu_send_command(imu, HOLOLENS_SENSORS_COMMAND_CONFIG_START,(uint8_t*)&report,sizeof(report));
        if (ret < 0)
        {
            return ret;
        }
        if (report.code != 0x04) {
            printf("Unexpected reply 0x%02x\n",report.code);
            return -1;
        }

        ret = wmr_imu_send_command(imu, command, (uint8_t*)&report, sizeof(report));
        if (ret < 0)
        {
            return ret;
        }
        for (;;)
        {
            ret = wmr_imu_send_command(imu, HOLOLENS_SENSORS_COMMAND_CONFIG_READ, (uint8_t*)&report,sizeof(report));
            if (ret < 0)
            {
                return ret;
            }
            if (report.code == 0x02)
            {
                break;
            }
            if (report.code != 0x01 || report.len > 30)
            {
                printf("Unexpected reply 0x%02x\n", report.code);
                return -1;
            }
            if (offset + report.len > count)
            {
                printf("Out of space at %u+%u/%lu\n", offset, report.len, count);
                return offset;
            }
            memcpy(buf + offset, report.data, report.len);
            offset += report.len;
        }

        return offset;
    }

    int wmr_imu_send_command(wmr_imu* imu, unsigned char type, unsigned char* buf, int len)
    {
        unsigned char cmd[64] = { 0x02, type };
        int ret = 0;
        ret = hid_write(imu->hmd_cfg, cmd, sizeof(cmd));
        usleep(1000);
        do {
            int size = hid_read(imu->hmd_cfg, buf, len);
            if (size == -1)
            {
                return -1;
            }
            if (buf[0] == 0x02)
            {
                return size;
            }
        } while (buf[0] != 0x02);

        return -1;
    }

    int wmr_imu_start(wmr_imu *imu)
    {

        uint8_t config_meta[66];

        uint16_t config_len;
        uint8_t *config;
        int ret;
         wmr_imu_send_command(imu, HOLOLENS_SENSORS_COMMAND_START_IMU, config_meta,sizeof(config_meta));

        return 0;
    }

    int wmr_imu_stop(wmr_imu *imu)
    {
        unsigned char buf[64];
        unsigned char cmd[64] = { 0x02, 0x0b ,
            0xe3 , 0xa7  ,
            0x8a , 0x0a ,
            0x00 , 0x00  , 0x01 , 0x00 ,
            0x00 , 0x00  , 0x02 , 0x00 ,
            0x00 , 0x00  , 0x00 , 0x00 ,
            0x00 , 0x00  , 0x00 , 0x00 ,
            0x00 , 0x00  , 0x00 , 0x00 ,
            0x00 , 0x00  , 0x00 , 0x00 ,
            0x00 , 0x00 };
        int ret = 0;
        ret = hid_write(imu->hmd_cfg, cmd, sizeof(cmd));
        do {
            int size = hid_read(imu->hmd_cfg, buf, 33);
            if (size == -1)
            {
                return -1;
            }
            if (buf[0] == 0x02)
            {
                return size;
            }
        } while (buf[0] == 0x01);


        return 0;
    }

    void wmr_handle_imu_report(wmr_imu* imu, hololens_sensors_packet_report *report,
                               imu_sample *package_samples)
    {
        for (int i = 0; i < 4; i++)
        {
            struct raw_imu_sample raw;
            imu_sample& imu_sam = package_samples[i];
            int temperature;
            int64_t dt;

            temperature = (report->temperature[i]);
            /* Time in 10⁻⁷ s @ 1 kHz */
            raw.time = (report->gyro_timestamp[i]);
            /* Acceleration in 10⁻³ m/s² @ 1 kHz */
            raw.acc[0] = (int32_t)(report->accel[0][i]);
            raw.acc[1] = (int32_t)(report->accel[1][i]);
            raw.acc[2] = (int32_t)(report->accel[2][i]);

            raw.mag[0] = (int32_t)(report->magnet[0][i]);
            raw.mag[1] = (int32_t)(report->magnet[1][i]);
            raw.mag[2] = (int32_t)(report->magnet[2][i]);

            /* Angular velocity in 10⁻³ rad/s @ 8 kHz */
            raw.gyro[0] = (int16_t)(report->gyro[0][8 * i + 0]) +
                (int16_t)(report->gyro[0][8 * i + 1]) +
                (int16_t)(report->gyro[0][8 * i + 2]) +
                (int16_t)(report->gyro[0][8 * i + 3]) +
                (int16_t)(report->gyro[0][8 * i + 4]) +
                (int16_t)(report->gyro[0][8 * i + 5]) +
                (int16_t)(report->gyro[0][8 * i + 6]) +
                (int16_t)(report->gyro[0][8 * i + 7]);
            raw.gyro[1] = (int16_t)(report->gyro[1][8 * i + 0]) +
                (int16_t)(report->gyro[1][8 * i + 1]) +
                (int16_t)(report->gyro[1][8 * i + 2]) +
                (int16_t)(report->gyro[1][8 * i + 3]) +
                (int16_t)(report->gyro[1][8 * i + 4]) +
                (int16_t)(report->gyro[1][8 * i + 5]) +
                (int16_t)(report->gyro[1][8 * i + 6]) +
                (int16_t)(report->gyro[1][8 * i + 7]);
            raw.gyro[2] = (int16_t)(report->gyro[2][8 * i + 0]) +
                (int16_t)(report->gyro[2][8 * i + 1]) +
                (int16_t)(report->gyro[2][8 * i + 2]) +
                (int16_t)(report->gyro[2][8 * i + 3]) +
                (int16_t)(report->gyro[2][8 * i + 4]) +
                (int16_t)(report->gyro[2][8 * i + 5]) +
                (int16_t)(report->gyro[2][8 * i + 6]) +
                (int16_t)(report->gyro[2][8 * i + 7]);

            //telemetry_send_raw_imu_sample(self->dev.id, &raw);
            dt = raw.time - imu->last_timestamp;

            /*
            * Transform from IMU coordinate system into common coordinate
            * system:
            *
            *   -y                                y
            *    |          ⎡ 0 -1  0 ⎤ ⎡x⎤       |
            *    +-- -x ->  ⎢-1  0  0 ⎥ ⎢y⎥  ->   +-- x
            *   /           ⎣ 0  0 -1 ⎦ ⎣z⎦      /
            * -z                                z
            *
            */
            imu_sam.acceleration[0] = raw.acc[1] * -1e-3;
            imu_sam.acceleration[1] = raw.acc[0] * -1e-3;
            imu_sam.acceleration[2] = raw.acc[2] * -1e-3;
            imu_sam.angular_velocity[0] = raw.gyro[1] * -(1e-3 / 8.0);
            imu_sam.angular_velocity[1] = raw.gyro[0] * -(1e-3 / 8.0);
            imu_sam.angular_velocity[2] = raw.gyro[2] * -(1e-3 / 8.0);


            imu_sam.magnetic_field[0] = raw.mag[1] * 0.15;
            imu_sam.magnetic_field[1] = raw.mag[0] * 0.15;
            imu_sam.magnetic_field[2] = raw.mag[2] * 0.15;


            imu_sam.temperature = (float)temperature /326.8+25;
            imu_sam.time = raw.time * 1e-7;

            //imu->data = imu_sam;

           // printf("%f\n", 1.0/ (dt* 1e-7));
            //printf("%f,%f,%f\n", imu_sam.acceleration[0], imu_sam.acceleration[1], imu_sam.acceleration[2]);
            //telemetry_send_imu_sample(self->dev.id, &imu);

            //pose_update(1e-7 * dt, &self->imu.pose, &imu);

            //telemetry_send_pose(self->dev.id, &self->imu.pose);

            //imu->last_timestamp = raw.time;
        }

        /*
        if (report->message[0])
        {
            printf("[%02x] %s\n", report->message[0].code, report->message[0].text);
        }
        if (report->message[1])
        {
            printf("[%02x] %s\n", report->message[1].code, report->message[1].text);
        }

        */
    }

};

#endif // IMU_NODE_HPP
