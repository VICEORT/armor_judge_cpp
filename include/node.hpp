#pragma once

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include "Inference.h"
#include "Detection.h"
#include "Visualization.h"
#include "mode_manager.hpp"
#include "YawController.h"
#include "time_associator.hpp"
#include "extrinsic_projection.hpp"
#include "cloud_refine.hpp"
#include "armor_bbox_counter.hpp"
#include "yaw.h"
#include "pitch.h"

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#pragma pack(push,1)
struct PcReceive
{
    uint8_t sof;
    uint8_t reboot;
    uint8_t reserved1[2];

    float target_pitch_angle_d;
    float target_yaw_angle_d;

    float yaw_speed_error;

    uint8_t shoot_mode;
    uint8_t reserved2;
    uint8_t aiming_state;

    uint8_t eof;
};
#pragma pack(pop)

#pragma pack(push,1)
struct PcSend
{
    uint8_t sof;
    uint8_t task_mode;
    uint8_t self_team;
    uint8_t _reserved;

    float bullet_speed;
    float gimbal_roll_d;
    float gimbal_yaw_d;
    float gimbal_pitch_d;
    float gimbal_yaw_dps;

    uint8_t reserved[3];
    uint8_t eof;
};
#pragma pack(pop)

class ArmorJudgeNode : public rclcpp::Node
{
public:
    ArmorJudgeNode();

private:

    void offlineTick();

    // ===== 串口通信 =====
    int serial_fd_{-1};
    bool serial_enabled_{true};
    std::string serial_port_{"/dev/ttyACM0"};

    // ===== 离线模式 =====
    bool offline_mode_{false};
    std::string offline_video_path_;
    double offline_fps_{60.0};
    cv::VideoCapture offline_cap_;
    rclcpp::TimerBase::SharedPtr offline_timer_;

    float gimbalYaw_{0.0f};
    float gimbalPitch_{0.0f};
    float bulletSpeed_{0.0f};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void processFrame(cv::Mat &frame);   // 原来的 while 循环
    bool readSerial();
    void sendAimData(float yaw, float pitch);

    // 模块
    std::shared_ptr<Inference> inference;
    std::shared_ptr<Detector> detector;
    std::shared_ptr<Visualization> visualizer;
    std::shared_ptr<hero_pkg::aim::TimeAssociator> time_associator_;
    std::shared_ptr<hero_pkg::aim::ExtrinsicProjection> projector_;
    std::shared_ptr<hero_pkg::aim::CloudRefine> cloud_refiner_;
    std::shared_ptr<hero_pkg::aim::ArmorBBoxCounter> armor_bbox_counter_;

    ModeManager modeManager;
    YawController yawController;


    // 状态变量
    int frameIdx;
    int armorCount;
    int ballCount;

    double autoAimYaw{0.0};    // AUTO_AIM 目标 yaw
    double autoAimPitch{0.0};  // AUTO_AIM 目标 pitch

    std::vector<double> frameTimes;
    std::vector<hero_pkg::aim::ArmorBBoxInfo> lastArmorBoxes;

    std::vector<DetectionResult> lastDetections;
    
    int framesNoBall{0};
    bool disappearanceHandled{false};
    std::string lastPosition{""};
    std::string lastHorizontal{""};
    float lastDiff{0.0f};
    bool lastDiffValid{false};
    
    std::vector<TrajectoryPoint> prevTrajectory;
    cv::Point2f lastDisappearancePoint{-1, -1};
    
    const int DISAPPEAR_FRAMES = 3;
    
    cv::Point2f lastArmorCenter{-1, -1};
    
    // 完整轨迹记录（用于分析）
    std::vector<std::vector<TrajectoryPoint>> allTrajectories;

    DetectionConfig config;

};