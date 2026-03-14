#include "node.hpp"
#include <algorithm>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>

ArmorJudgeNode::ArmorJudgeNode()
: Node("armor_judge_node"),
  yawController(0.3),
  frameIdx(0),
  armorCount(0),
  ballCount(0)
{
    RCLCPP_INFO(this->get_logger(),"Armor Judge Node Started");

    std::string modelPath = "best.onnx";
    std::string device = "GPU";

    inference = std::make_shared<Inference>(modelPath, device);

    config.trackerType = "CSRT";
    detector = std::make_shared<Detector>(config);

    visualizer = std::make_shared<Visualization>();

// ================= 时间关联初始化 =================
hero_pkg::aim::TimeAssociatorConfig ta_cfg;

ta_cfg.buffer_size = this->declare_parameter("aim.buffer_size",30);
ta_cfg.camera_fps = this->declare_parameter("aim.camera_fps",60.0);
ta_cfg.integrate_window = this->declare_parameter("aim.integrate_window",0.1);

// 自动推导 max_dt
double auto_dt =
std::min(0.5 * ta_cfg.integrate_window,
         1.2 / ta_cfg.camera_fps);

ta_cfg.max_dt = std::max(auto_dt,0.03);

time_associator_ =
std::make_shared<hero_pkg::aim::TimeAssociator>(ta_cfg);


// ================= 投影初始化 =================
hero_pkg::aim::ExtrinsicProjectionConfig proj_cfg;

proj_cfg.fx = this->declare_parameter("camera.fx",0.0);
proj_cfg.fy = this->declare_parameter("camera.fy",0.0);
proj_cfg.cx = this->declare_parameter("camera.cx",0.0);
proj_cfg.cy = this->declare_parameter("camera.cy",0.0);

proj_cfg.k1 = this->declare_parameter("camera.k1",0.0);
proj_cfg.k2 = this->declare_parameter("camera.k2",0.0);
proj_cfg.p1 = this->declare_parameter("camera.p1",0.0);
proj_cfg.p2 = this->declare_parameter("camera.p2",0.0);

auto ext_r = this->declare_parameter<std::vector<double>>("extrinsic.extrinsic_r");
    for(size_t i=0; i<9 && i<ext_r.size(); ++i) proj_cfg.extrinsic_r[i] = ext_r[i];

    auto ext_t = this->declare_parameter<std::vector<double>>("extrinsic.extrinsic_t");
    for(size_t i=0; i<3 && i<ext_t.size(); ++i) proj_cfg.extrinsic_t[i] = ext_t[i];

projector_ =
std::make_shared<hero_pkg::aim::ExtrinsicProjection>(proj_cfg);

armor_bbox_counter_ =
std::make_shared<hero_pkg::aim::ArmorBBoxCounter>();

hero_pkg::aim::CloudRefineConfig refine_cfg;

cloud_refiner_ =
std::make_shared<hero_pkg::aim::CloudRefine>(refine_cfg);

offline_mode_ = this->declare_parameter("offline.enable", false);
offline_video_path_ = this->declare_parameter("offline.video_path", std::string("2.7.ChangZhou.mp4"));
offline_fps_ = this->declare_parameter("offline.fps", 60.0);

serial_enabled_ = this->declare_parameter("serial.enable", true);
serial_port_ = this->declare_parameter("serial.port", std::string("/dev/ttyACM0"));

if (offline_mode_)
{
    RCLCPP_WARN(this->get_logger(), "Offline mode enabled: no camera/cloud subscriptions, no serial communication");

    serial_enabled_ = false;

    offline_cap_.open(offline_video_path_);
    if (!offline_cap_.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open offline video: %s", offline_video_path_.c_str());
        throw std::runtime_error("offline video open failed");
    }

    const int timer_ms = std::max(1, static_cast<int>(1000.0 / std::max(offline_fps_, 1.0)));
    offline_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_ms),
        std::bind(&ArmorJudgeNode::offlineTick, this)
    );

    RCLCPP_INFO(this->get_logger(), "Offline video source: %s", offline_video_path_.c_str());
}
else
{
    image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw",
        10,
        std::bind(&ArmorJudgeNode::imageCallback, this, std::placeholders::_1)
    );
    cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points_raw",
        10,
        std::bind(&ArmorJudgeNode::cloudCallback, this, std::placeholders::_1)
    );
}

// ================= 串口初始化 =================
if (serial_enabled_)
{
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);

    if (serial_fd_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Serial open failed: %s", serial_port_.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Serial opened: %s", serial_port_.c_str());
    }
}
else
{
    RCLCPP_INFO(this->get_logger(), "Serial disabled (offline or config)");
}
}

void ArmorJudgeNode::offlineTick()
{
    if (!offline_cap_.isOpened())
    {
        return;
    }

    cv::Mat frame;
    if (!offline_cap_.read(frame))
    {
        // 播放到末尾后循环，便于调试
        offline_cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
        if (!offline_cap_.read(frame))
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Offline video read failed");
            return;
        }
    }

    processFrame(frame);
}

void ArmorJudgeNode::sendAimData(float yaw, float pitch)
{
    if (!serial_enabled_) return;
    if (serial_fd_ < 0) return;

    PcReceive data{};

    data.sof = 0x33;
    data.eof = 0xEE;

    // 只需要这两个
    data.target_pitch_angle_d = pitch;
    data.target_yaw_angle_d = yaw;

    write(serial_fd_, &data, sizeof(data));
}

bool ArmorJudgeNode::readSerial()
{
    if (!serial_enabled_)
    {
        // 离线/禁用串口时给出可用默认值
        bulletSpeed_ = 15.0f;
        gimbalYaw_ = 0.0f;
        gimbalPitch_ = 0.0f;
        return true;
    }

    if(serial_fd_ < 0) return false;

    PcSend data;

    int n = read(serial_fd_, &data, sizeof(data));

    if(n == sizeof(data) && data.sof == 0x33 && data.eof == 0xEE)
    {
        bulletSpeed_ = data.bullet_speed;
        gimbalYaw_ = data.gimbal_yaw_d;
        gimbalPitch_ = data.gimbal_pitch_d;
        return true;
    }
    return false;
}


void ArmorJudgeNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat frame;

    try
    {
        frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch(cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(),"cv_bridge error");
        return;
    }

    processFrame(frame);

    // 推入时间缓冲
    hero_pkg::aim::FramePacket packet;
    packet.stamp = msg->header.stamp;
    packet.image_mat = frame;
    packet.has_cv_mat = true;

    time_associator_->pushFrame(packet);
}

void ArmorJudgeNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

    readSerial();
    // 只有 AUTO_AIM 才运行外参反投影
    if (modeManager.getMode() != Mode::AUTO_AIM)
        return;

    pcl::PointCloud<pcl::PointXYZ> cloud;

    pcl::fromROSMsg(*msg, cloud);

    // 找最近图像帧
    auto match_result = time_associator_->match(msg->header.stamp);

    if (!match_result.has_value() || !match_result->matched)
    {
        RCLCPP_WARN(this->get_logger(),"No matched frame");
        return;
    }

    auto frame = match_result->frame;

    // 点云投影
    auto proj = projector_->project(*msg, frame);

hero_pkg::aim::PointCloudRefineInput refine_input;

refine_input.ok = proj.ok;

/* 拷贝点 */
for (const auto& p : proj.points)
{
    hero_pkg::aim::ProjectedPointInfo q;

    q.x_cam = p.x_cam;
    q.y_cam = p.y_cam;
    q.z_cam = p.z_cam;

    q.u = p.u;
    q.v = p.v;

    q.armor_score = p.armor_score;
    q.armor_box_index = p.armor_box_index;

    q.finite_point = p.finite_point;
    q.front_of_camera = p.front_of_camera;
    q.valid_projection = p.valid_projection;

    refine_input.points.push_back(q);
}

/* 拷贝bbox */
for (const auto& b : proj.armor_boxes)
{
    hero_pkg::aim::ArmorBBoxInfo box;

    box.box = b.box;
    box.confidence = b.confidence;
    box.detection_index = b.detection_index;

    refine_input.armor_boxes.push_back(box);
}

    auto bbox_result =
    armor_bbox_counter_->count(refine_input);

    RCLCPP_INFO(this->get_logger(),
    "armor points = %ld",
    bbox_result.num_armor_points);

auto refine_result =
cloud_refiner_->process(refine_input);

RCLCPP_INFO(this->get_logger(),
"refine input=%ld filter=%ld cluster=%ld",
refine_result.num_input_points,
refine_result.num_after_outlier_filter,
refine_result.num_after_cluster);

if (refine_result.ok)
{
    double target_x = refine_result.mean_x;
    double target_y = refine_result.mean_y;
    double target_z = refine_result.mean_z;

    RCLCPP_INFO(this->get_logger(),
    "target xyz = %.3f %.3f %.3f",
    target_x,target_y,target_z);

    RCLCPP_INFO(this->get_logger(),
    "armor distance = %.3f m",
    refine_result.mean_distance);

    // ================= yaw =================

    double yaw =
    calculateDeltaYawwithLidar(
        target_x,
        target_y,
        target_z,
        0.0,0.0,0.0   // lidar坐标
    );

    // ================= pitch =================

    double horizontal_distance =
        std::sqrt(target_x*target_x +
                  target_y*target_y);

    double pitch =
    calculateLaunchAngleBisection(
        horizontal_distance,
        target_z,
        bulletSpeed_,   // 初速度 m/s
        0.0,    // yaw平面角
        0.003,  // 质量 kg
        0.02,   // 半径 m
        0.47,   // 阻力系数
        0.1     // 升力系数
    );

    RCLCPP_INFO(this->get_logger(),
    "AUTO AIM yaw=%.3f pitch=%.3f",
    yaw,pitch);
    sendAimData(yaw, pitch);
}
}

void ArmorJudgeNode::processFrame(cv::Mat &frame)
{
    float fps = 0.0f;
    auto t0 = std::chrono::high_resolution_clock::now();
            
        int frameHeight = frame.rows;
        int frameWidth = frame.cols;
            
            // 1. 预处理和推理
            float scale = 1.0f;
            int padX = 0, padY = 0;
            ov::Tensor inputTensor = inference->preprocess(frame, scale, padX, padY);
            ov::Tensor outputTensor = inference->infer(inputTensor);
            
            // 2. 解析检测结果
            const float* outputData = outputTensor.data<float>();
            ov::Shape outputShape = outputTensor.get_shape();
            
            // 输出形状: (1, 4+classes, N)
            size_t numClasses = outputShape[1] - 4;
            size_t numDetections = outputShape[2];
            
            std::vector<DetectionResult> detections = detector->parseDetections(
                outputData, numDetections, numClasses, 
                scale, padX, padY, frameWidth, frameHeight);
            
            // 3. NMS
            std::vector<DetectionResult> nmsDetections = detector->nonMaxSuppression(detections);
            
            // ================= 模式控制 =================
            //0键 → IDLE   1键 → AUTO_AIM   2键 → CALIB
            Mode currentMode = modeManager.getMode();

            // ======IDLE模式开始 ======
            if (currentMode == Mode::IDLE)
            {
            // IDLE模式仅显示画面
             cv::imshow("Armor Judge - Detection", frame);

                 int key = cv::waitKey(1) & 0xFF;
                 modeManager.updateFromKeyboard(key);

                 if (key == 'q')
                 {
                     rclcpp::shutdown();
                     return;
            }}

            // ====== AUTO_AIM模式开始 ======
            else if (currentMode == Mode::AUTO_AIM)
            {
            // AUTO_AIM模式正常运行检测和跟踪

            }

            // ====== CALIB模式开始 ======
            else if (currentMode == Mode::CALIB)
            {
            // 4. 分离armor和ball
            std::vector<DetectionResult> armorDets, ballDets;
            for (const auto& det : nmsDetections) {
                if (det.classID == config.armorID) {
                    armorDets.push_back(det);
                } else if (det.classID == config.ballID) {
                    ballDets.push_back(det);
                }
            }
            
            // 5. 选择最佳检测
            DetectionResult* bestArmor = nullptr;
            DetectionResult* yoloBall = nullptr;
            
            if (!armorDets.empty()) {
                bestArmor = &(*std::max_element(armorDets.begin(), armorDets.end(),
                    [](const DetectionResult& a, const DetectionResult& b) {
                        return a.confidence < b.confidence;
                    }));
                armorCount++;
            }
            
            if (!ballDets.empty()) {
                yoloBall = &(*std::max_element(ballDets.begin(), ballDets.end(),
                    [](const DetectionResult& a, const DetectionResult& b) {
                        return a.confidence < b.confidence;
                    }));
            }
            
            // 6. 轨迹过滤
            DetectionResult* filteredYoloBall = nullptr;
            if (yoloBall != nullptr) {
                TrajectoryPoint newPt(yoloBall->center.x, yoloBall->center.y, frameIdx + 1);
                std::string reason;
                bool isValid = detector->validateBallTrajectory(
                    detector->getTrajectory(), &newPt, &reason);
                
                // 额外检查：与armor的水平距离
                bool horizCheck = true;
                if (bestArmor != nullptr) {
                    float horizDist = std::abs(yoloBall->center.x - bestArmor->center.x);
                    horizCheck = (horizDist <= 500);
                }
                
                // 额外检查：ball面积必须在合理范围内
                bool areaCheck = true;
                if (bestArmor != nullptr) {
                    float ballArea = yoloBall->bbox.area();
                    float armorArea = bestArmor->bbox.area();
                    if (armorArea > 0) {
                        float areaRatio = ballArea / armorArea;
                        // ball面积不能超过armor的一半，也不能小于armor的五分之一
                        areaCheck = (areaRatio <= config.maxBallArmorAreaRatio && 
                                    areaRatio >= config.minBallArmorAreaRatio);
                    }
                }
                
                if (isValid && horizCheck && areaCheck) {
                    filteredYoloBall = yoloBall;
                }
            }
            
            frameIdx++;
            
            // 更新armor中心
            if (bestArmor != nullptr) {
                lastArmorCenter = bestArmor->center;
            }
            
            // 7. 融合检测和跟踪
            DetectionResult* bestBall = nullptr;
            std::string detectionSource = "none";
            cv::Scalar ballColor;
            
            HybridBallTracker& tracker = detector->getTracker();
            
            // 跟踪器更新
            cv::Rect trackBbox;
            float trackConf = 0.0f;
            bool trackSuccess = false;
            
            if (tracker.isInitialized()) {
                trackSuccess = tracker.update(frame, trackBbox, trackConf);
            }
            
            if (filteredYoloBall != nullptr) {
                if (trackSuccess && trackConf > config.trackerConfidenceThres) {
                    // 融合检测和跟踪
                    cv::Point2f trackCenter(trackBbox.x + trackBbox.width / 2.0f,
                                           trackBbox.y + trackBbox.height / 2.0f);
                    float distance = cv::norm(filteredYoloBall->center - trackCenter);
                    
                    if (distance < 100) {
                        // 融合
                        float w = config.hybridFusionWeight;
                        cv::Point2f fusedCenter = filteredYoloBall->center * w + trackCenter * (1 - w);
                        float fusedW = filteredYoloBall->bbox.width * w + trackBbox.width * (1 - w);
                        float fusedH = filteredYoloBall->bbox.height * w + trackBbox.height * (1 - w);
                        
                        DetectionResult fusedDet;
                        fusedDet.center = fusedCenter;
                        fusedDet.bbox = cv::Rect(fusedCenter.x - fusedW/2, fusedCenter.y - fusedH/2,
                                                 fusedW, fusedH);
                        fusedDet.confidence = (filteredYoloBall->confidence + trackConf) / 2;
                        fusedDet.classID = config.ballID;
                        
                        static DetectionResult fusedDetStatic;
                        fusedDetStatic = fusedDet;
                        bestBall = &fusedDetStatic;
                        detectionSource = "hybrid";
                        ballColor = cv::Scalar(0, 255, 255);  // 黄色
                        
                        // 绘制连接线
                        visualizer->drawConnectionLine(frame, filteredYoloBall->center, 
                                                     trackCenter, distance);
                        
                        // 重新初始化跟踪器
                        tracker.initialize(frame, bestBall->bbox);
                    } else {
                        bestBall = filteredYoloBall;
                        detectionSource = "yolo";
                        ballColor = cv::Scalar(0, 0, 255);  // 红色
                        tracker.initialize(frame, bestBall->bbox);
                    }
                } else {
                    bestBall = filteredYoloBall;
                    detectionSource = "yolo";
                    ballColor = cv::Scalar(0, 0, 255);  // 红色
                    tracker.initialize(frame, bestBall->bbox);
                }
                
                ballCount++;
            } else if (trackSuccess && trackConf > config.trackerConfidenceThres) {
                // 只有跟踪结果
                static DetectionResult trackDet;
                trackDet.bbox = trackBbox;
                trackDet.center = cv::Point2f(trackBbox.x + trackBbox.width / 2.0f,
                                             trackBbox.y + trackBbox.height / 2.0f);
                trackDet.confidence = trackConf * 0.8f;
                trackDet.classID = config.ballID;
                
                bestBall = &trackDet;
                detectionSource = "tracker";
                ballColor = cv::Scalar(255, 0, 0);  // 蓝色
            }
            
            // 8. 处理ball检测结果
            if (bestBall != nullptr) {
                prevTrajectory.clear();
                lastDisappearancePoint = cv::Point2f(-1, -1);
                
                // 更新卡尔曼滤波器
                detector->updateKalman(bestBall->center);
                
                // 添加到轨迹
                TrajectoryPoint newPt(bestBall->center.x, bestBall->center.y, frameIdx);
                std::string reason;
                bool isValid = detector->validateBallTrajectory(
                    detector->getTrajectory(), &newPt, &reason);
                
                if (isValid) {
                    detector->addToTrajectory(newPt);
                } else {
                    // 保存当前轨迹到历史记录
                    if (detector->getTrajectory().size() > 5) {
                        allTrajectories.push_back(detector->getTrajectory());
                    }
                    detector->clearTrajectory();
                    detector->addToTrajectory(newPt);
                    detector->resetKalman(bestBall->center);
                }
                
                // 增加轨迹长度限制，确保完整轨迹
                while (detector->getTrajectory().size() > 300) {
                    auto& traj = const_cast<std::vector<TrajectoryPoint>&>(detector->getTrajectory());
                    traj.erase(traj.begin());
                }
                
                disappearanceHandled = false;
                framesNoBall = 0;
                
                // 绘制ball检测框
                std::string label = "Ball " + std::to_string(static_cast<int>(bestBall->confidence * 100)) + "%";
                visualizer->drawDetection(frame, *bestBall, label, ballColor);
                
                // 计算horizontal
                if (bestArmor != nullptr) {
                    float bx = bestBall->center.x;
                    lastHorizontal = (bx >= bestArmor->bbox.x && 
                                     bx <= bestArmor->bbox.x + bestArmor->bbox.width) ? 
                                     "true" : "Fault";
                    lastDiff = bx - bestArmor->center.x;
                    lastDiffValid = true;
                    // ====== 新增 yaw 调整 ======

                    readSerial(); // 读取下位机发送的基准 yaw/pitch

    float receivedYawFromMCU = gimbalYaw_;
    float receivedPitchFromMCU = gimbalPitch_;
                   // 接收到下位机基准 yaw/pitch 的地方
    yawController.setBaseYaw(receivedYawFromMCU);
    yawController.setBasePitch(receivedPitchFromMCU);

    // CALIB 模块循环
    yawController.update(lastDiff, lastDiffValid, lastPosition);

    float sendYaw = 0.0f;
    float sendPitch = 0.0f;

    if (yawController.isHit(lastDiff)) {
        // 命中 → 发送 pitch
        sendPitch = static_cast<float>(yawController.getCorrectedPitch());
    } else {
        // 未命中 → 发送 yaw
        sendYaw = static_cast<float>(yawController.getCorrectedYaw());
    }

    // 发送到下位机
    sendAimData(sendYaw, sendPitch);
    }
            } else {
                framesNoBall++;
                
                // 绘制预测点
                cv::Point2f pred = detector->predictBallPosition();
                if (pred.x > 0 && pred.y > 0) {
                    visualizer->drawPrediction(frame, pred);
                }
            }
            
            // 9. 处理armor检测
            if (bestArmor != nullptr) {
                // 绘制armor检测框
                std::string label = "Armor " + std::to_string(static_cast<int>(bestArmor->confidence * 100)) + "%";
                visualizer->drawDetection(frame, *bestArmor, label, cv::Scalar(0, 255, 0));
                
                // 检查ball是否消失
                if (framesNoBall >= DISAPPEAR_FRAMES && !disappearanceHandled && 
                    !detector->getTrajectory().empty()) {
                    
                    const auto& lastPt = detector->getTrajectory().back();
                    lastDisappearancePoint = cv::Point2f(lastPt.x, lastPt.y);
                    
                    // 根据消失点相对于armor下边缘判断Position
                    int armorBottom = bestArmor->bbox.y + bestArmor->bbox.height;
                    if (lastPt.y < armorBottom) {
                        lastPosition = "Back";
                    } else {
                        lastPosition = "Front";
                    }
                    
                    prevTrajectory = detector->getTrajectory();
                    disappearanceHandled = true;
                    
                    // 保存完整轨迹到历史记录
                    if (prevTrajectory.size() > 5) {
                        allTrajectories.push_back(prevTrajectory);
                    }
                    
                    detector->clearTrajectory();
                }
                
                // 绘制状态文本
                std::vector<std::pair<std::string, cv::Scalar>> statusTexts;
                
                if (detectionSource != "none") {
                    cv::Scalar srcColor = (detectionSource == "yolo") ? cv::Scalar(0, 255, 0) :
                                         (detectionSource == "hybrid") ? cv::Scalar(255, 255, 0) :
                                         cv::Scalar(0, 200, 255);
                    statusTexts.push_back(std::make_pair("Source: " + detectionSource, srcColor));
                    if (bestBall) {
                        statusTexts.push_back(std::make_pair("Conf: " + std::to_string(bestBall->confidence).substr(0, 4), srcColor));
                    }
                }
                
                // 跟踪器状态
                std::string trackerStatus = tracker.isInitialized() ? "Active" : "Inactive";
                cv::Scalar trackerColor = tracker.isInitialized() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
                statusTexts.push_back(std::make_pair("Tracker: " + trackerStatus + " (C:" + 
                                      std::to_string(tracker.getConfidence()).substr(0, 4) + ")", trackerColor));
                
                if (lastDiffValid) {
                    statusTexts.push_back(std::make_pair("Diff: " + std::to_string(lastDiff).substr(0, 6), cv::Scalar(0, 255, 0)));
                }
                
                if (!lastHorizontal.empty()) {
                    cv::Scalar hColor = (lastHorizontal == "true") ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
                    statusTexts.push_back(std::make_pair("Horizontal: " + lastHorizontal, hColor));
                }
                
                if (!lastPosition.empty()) {
                    cv::Scalar pColor = (lastPosition == "Front") ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
                    statusTexts.push_back(std::make_pair("Position: " + lastPosition, pColor));
                } else {
                    statusTexts.push_back(std::make_pair("Position: Waiting", cv::Scalar(200, 200, 200)));
                }
                
                visualizer->drawStatusTexts(frame, statusTexts, 10, 30, 20, 0.6);
                
                // 绘制轨迹
                if (!prevTrajectory.empty()) {
                    visualizer->drawTrajectory(frame, prevTrajectory, cv::Scalar(160, 160, 160), 2, false);
                    if (lastDisappearancePoint.x >= 0) {
                        visualizer->drawDisappearancePoint(frame, lastDisappearancePoint);
                    }
                }
                
                if (!detector->getTrajectory().empty()) {
                    visualizer->drawTrajectory(frame, detector->getTrajectory(), cv::Scalar(0, 255, 255), 2, false);
                }
            }
            
            // 10. 显示FPS
            auto t1 = std::chrono::high_resolution_clock::now();
            double frameTime = std::chrono::duration<double>(t1 - t0).count();
            frameTimes.push_back(frameTime);
            float fps = 1.0f / frameTime;
        } // ====== CALIB模式结束 ======

            visualizer->drawFPS(frame, fps);
            
            
            // 显示帧
            cv::imshow("Armor Judge - Detection", frame);
            
            // 统计信息
            if (frameIdx % 200 == 0) {
                std::cout << "[STAT] frame " << frameIdx 
                         << ": armor=" << armorCount 
                         << ", ball=" << ballCount << std::endl;
                std::cout << "[TRACK] Tracker status: initialized=" << detector->getTracker().isInitialized()
                         << ", confidence=" << detector->getTracker().getConfidence()
                         << ", miss_count=" << detector->getTracker().getMissCount() << std::endl;
            }
            
            // 按键处理
            int key = cv::waitKey(1) & 0xFF;

            // ====== 新增 ======
            modeManager.updateFromKeyboard(key);
            if (key == 'q') {
                rclcpp::shutdown();
            }
            else if (key == 'r') {
                detector->getTracker().reset();
                std::cout << "[INFO] Tracker reset" << std::endl;
            }
        }