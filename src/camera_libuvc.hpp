#ifndef CAMERA_LIBUVC_HPP
#define CAMERA_LIBUVC_HPP
extern "C"{
#include <libuvc/libuvc.h>
#include <libusb-1.0/libusb.h>
}
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "imu_node.hpp"

#define MICROSOFT_HOLOLENS_VID 0x045e
#define MICROSOFT_HOLOLENS_PID 0x0659

#define IMAGE_WIDTH 1280
#define IMAGE_HEIGHT 481

struct Publishers
{
    Publishers(ros::Publisher& _left_pub, ros::Publisher &_right_pub)
        :left_pub(_left_pub),right_pub(_right_pub)
    {}
    ros::Publisher left_pub;
    ros::Publisher right_pub;
};

void callback(uvc_frame_t *frame, void *ptr)
{
    ros::Time time_stamp = ros::Time(frame->capture_time.tv_usec * 1e-6);
    Publishers *pubs = (Publishers*)ptr;
    cv::Mat stereo_image(frame->height,frame->width,CV_8UC1);
    static int frame_size = frame->width * frame->height;
    static int col_range = frame->width / 2;
    memcpy((void*)stereo_image.data,frame->data,frame_size);

    //shallow copy
    cv::Mat left_img = stereo_image.colRange(0,col_range);
    cv::Mat right_img = stereo_image.colRange(col_range,frame->width);

    if(pubs->left_pub.getNumSubscribers() > 0)
    {
        cv_bridge::CvImage cv_image_left;
        cv_image_left.header.frame_id = "left_cam";
        cv_image_left.header.stamp = time_stamp;
        cv_image_left.encoding = "mono8";
        cv_image_left.image = left_img;

        sensor_msgs::Image image_msgs_left;
        cv_image_left.toImageMsg(image_msgs_left);
        pubs->left_pub.publish(image_msgs_left);
    }
    if(pubs->right_pub.getNumSubscribers() > 0)
    {
        cv_bridge::CvImage cv_image_right;
        cv_image_right.header.frame_id = "right_cam";
        cv_image_right.header.stamp = time_stamp;
        cv_image_right.encoding = "mono8";
        cv_image_right.image = right_img;

        sensor_msgs::Image image_msgs_right;
        cv_image_right.toImageMsg(image_msgs_right);
        pubs->right_pub.publish(image_msgs_right);
    }
}

struct exposure_gain_cmd{
    uint8_t header[4];
    uint32_t len;
    uint16_t  unknow;
    uint16_t  cam_id_0;
    uint16_t  exposure;
    uint16_t  gain;
    uint16_t  cam_id_1;
}__attribute__((packed));

class CameraNode
{
public:
    CameraNode(ros::NodeHandle& _nh,ImuNode * pImu)
        :nh(_nh),image_width(IMAGE_WIDTH),image_height(IMAGE_HEIGHT),imu_dev(pImu)
    {
        left_pub = nh.advertise<sensor_msgs::Image>("/camera/left/image_raw",3);
        right_pub = nh.advertise<sensor_msgs::Image>("/camera/right/image_raw",3);
        //gain_exposure_control(gain,exposure);
    }
    void fill_exposure_gain(uint8_t *buffer,int cam_id,int exposure,int gain)
    {
        //44 6c 6f 2b  12 00 00 00  80 00 00 00  70 17 32 00  00 00

        struct exposure_gain_cmd * ptr_eg = (struct exposure_gain_cmd*)buffer;

        ptr_eg->header[0]=0x44;ptr_eg->header[1]=0x6c;ptr_eg->header[2]=0x6f;ptr_eg->header[3]=0x2b;
        ptr_eg->len=18;
        ptr_eg->unknow=0x0080;
        ptr_eg->cam_id_0=cam_id;
        ptr_eg->exposure=exposure;
        ptr_eg->gain=gain;
        ptr_eg->cam_id_1=cam_id;

    }
    bool gain_exposure_control(int gain,int exposure)
    {
        int ret=0;
        uint8_t ep_out=0x05;
        uint8_t buf[32];
        ret = libusb_init(NULL);
        if (ret < 0)
        {
            printf("Can not set exposure and gain!\n");
            return false;
        }
        libusb_device_handle *handle = libusb_open_device_with_vid_pid(NULL,MICROSOFT_HOLOLENS_VID,
                                                                       MICROSOFT_HOLOLENS_PID);
        if (handle == NULL)
        {
            printf("Failed to open device with libusb!\n");
            return false;
        }

        int iface = 3;
        printf("\nlibusb claiming interface %d\n", iface);
        ret = libusb_claim_interface(handle, iface);
        ImuNode::wmr_imu imu;
        ret = imu_dev->wmr_open_device(&imu);
        int size=0;
        fill_exposure_gain(buf,0,exposure,gain);
        ret = libusb_bulk_transfer(handle, ep_out, (unsigned char*)buf, 18, &size, 1000);

        fill_exposure_gain(buf,1,exposure,gain);
        ret = libusb_bulk_transfer(handle, ep_out, (unsigned char*)buf, 18, &size, 1000);
        imu_dev->wmr_close_device(&imu);
        libusb_exit(NULL);
        return true;
    }

    void operator ()()
    {
        image_retrieve();
    }
    void image_retrieve()
    {
        uvc_context_t *ctx;
        uvc_device_t *dev;
        uvc_device_handle_t *devh;
        uvc_stream_ctrl_t ctrl;
        uvc_error_t res;
        /* Initialize a UVC service context. Libuvc will set up its own libusb
         * context. Replace NULL with a libusb_context pointer to run libuvc
         * from an existing libusb context. */
        res = uvc_init(&ctx, NULL);
        if (res < 0)
        {
            uvc_perror(res, "uvc_init");
            return;
        }
        puts("UVC service context initialized");
        /* Locates the attached UVC device by vid and pid, stores in dev */
        /* filter devices: vendor_id, product_id, "serial_num" */
        res = uvc_find_device(ctx, &dev,MICROSOFT_HOLOLENS_VID, MICROSOFT_HOLOLENS_PID, NULL);
        if (res < 0)
        {
            /* no devices found */
            uvc_perror(res, "uvc_find_device");
        }
        else
        {
            puts("Device found");
            /* Try to open the device: requires exclusive access */
            res = uvc_open(dev, &devh);
            if (res < 0)
            {
                uvc_perror(res, "uvc_open"); /* unable to open device */
            }
            else
            {
                puts("Device opened");
                /* Print out a message containing all the information that libuvc
                 * knows about the device */
                uvc_print_diag(devh, stderr);
                std::cout<<"width: "<<image_width<<"\t height: "<<image_height<<std::endl;
                res = uvc_get_stream_ctrl_format_size(devh, &ctrl, /* result stored in ctrl */
                                                      UVC_FRAME_FORMAT_GRAY8,
                                                      image_width, image_height, 30 /* width, height, fps */
                                                      );
                /* Print out the result */
                uvc_print_stream_ctrl(&ctrl, stderr);
                if (res < 0)
                {
                    uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
                }
                else
                {
                    /* Start the video stream. The library will call user function cb:
                     *   cb(frame, (void*) 12345)
                     */
                    Publishers pubs(left_pub,right_pub);

                    res = uvc_start_streaming(devh, &ctrl,callback,(void*)&pubs, 0);
                    if (res < 0)
                    {
                        uvc_perror(res, "start_streaming"); /* unable to start stream */
                    }
                    else
                    {
                        puts("start streaming...");
                        uvc_set_ae_mode(devh, 1); /* e.g., turn on auto exposure */
                        ros::Rate rate(100);
                        while (ros::ok())
                        {
                            ros::spinOnce();
                            rate.sleep();
                        }
                        /* End the stream. Blocks until last callback is serviced */
                        uvc_stop_streaming(devh);
                        puts("Done streaming.");
                    }
                }
                /* Release our handle on the device */
                uvc_close(devh);
                puts("Device closed");
            }
            /* Release the device descriptor */
            uvc_unref_device(dev);
        }
        /* Close the UVC context. This closes and cleans up any existing device handles,
         * and it closes the libusb context if one was not provided. */
        uvc_exit(ctx);
        puts("UVC exited");
    }
private:
    ros::NodeHandle nh;
    ImuNode *imu_dev;
    int image_width,image_height;
    ros::Publisher left_pub;
    ros::Publisher right_pub;
    const int gain = 60;//scale: 60/16
    const int exposure  = 6000;
};




#endif // CAMERA_LIBUVC_HPP
