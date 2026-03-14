#include "Inference.h"
#include "Detection.h"
#include "Visualization.h"
#include "mode_manager.hpp"
#include "YawController.h"
#include <iostream>
#include <chrono>
#include <cstring>
#include <fstream>
#include <libgen.h>
#include <unistd.h>
#include <linux/limits.h>

// 获取可执行文件所在目录
std::string getExecutableDir() {
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    if (count != -1) {
        result[count] = '\0';
        char* dir = dirname(result);
        return std::string(dir);
    }
    return ".";
}

// 查找模型文件
std::string findModelPath(const std::string& modelName) {
    // 尝试不同的路径
    std::vector<std::string> searchPaths = {
        modelName,                                          // 当前目录
        "../" + modelName,                                  // 上级目录
        getExecutableDir() + "/" + modelName,               // 可执行文件目录
        getExecutableDir() + "/../" + modelName             // 可执行文件上级目录
    };
    
    for (const auto& path : searchPaths) {
        std::ifstream file(path);
        if (file.good()) {
            std::cout << "[INFO] Found model at: " << path << std::endl;
            return path;
        }
    }
    
    return modelName; // 返回默认名称
}

int main(int argc, char* argv[]) {
    // 默认配置
    std::string device = "GPU";
    std::string videoPath = "";
    std::string modelName = "best.onnx";
    std::string trackerType = "CSRT";
    std::string outputVideo = "Output.mp4";  // 默认输出视频路径
    
    // 解析命令行参数（简化版）
    if (argc >= 2) {
        // 第一个参数是视频源（文件路径或相机编号）
        videoPath = argv[1];
        
        // 可选参数
        for (int i = 2; i < argc; ++i) {
            if (strcmp(argv[i], "--device") == 0 && i + 1 < argc) {
                device = argv[++i];
                std::transform(device.begin(), device.end(), device.begin(), ::toupper);
            } else if (strcmp(argv[i], "--model") == 0 && i + 1 < argc) {
                modelName = argv[++i];
            } else if (strcmp(argv[i], "--tracker") == 0 && i + 1 < argc) {
                trackerType = argv[++i];
            } else if (strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
                outputVideo = argv[++i];
            }
        }
    }
    
    // 如果没有提供视频源，显示使用说明
    if (videoPath.empty()) {
        std::cout << "===========================================\n";
        std::cout << "  Armor Judge - 装甲板识别系统 C++版本\n";
        std::cout << "===========================================\n\n";
        std::cout << "使用方法:\n";
        std::cout << "  " << argv[0] << " <视频源> [选项]\n\n";
        std::cout << "参数说明:\n";
        std::cout << "  <视频源>        视频文件路径或相机编号(0,1,2...)\n";
        std::cout << "                  示例: video.mp4 或 0\n\n";
        std::cout << "可选参数:\n";
        std::cout << "  --model <路径>  模型文件 (默认: best.onnx)\n";
        std::cout << "  --device <设备> CPU或GPU (默认: GPU)\n";
        std::cout << "  --tracker <类型> CSRT或KCF (默认: CSRT)\n";
        std::cout << "  --output <路径> 输出视频文件 (可选)\n\n";
        std::cout << "使用示例:\n";
        std::cout << "  " << argv[0] << " video.mp4\n";
        std::cout << "  " << argv[0] << " 0\n";
        std::cout << "  " << argv[0] << " video.mp4 --device CPU\n";
        std::cout << "  " << argv[0] << " video.mp4 --tracker KCF\n\n";
        std::cout << "运行时按键:\n";
        std::cout << "  q - 退出程序\n";
        std::cout << "  r - 重置跟踪器\n\n";
        return 0;
    }
    
    // 查找模型文件
    std::string modelPath = findModelPath(modelName);
    
    std::cout << "[INFO] 设备: " << device << std::endl;
    std::cout << "[INFO] 视频: " << videoPath << std::endl;
    std::cout << "[INFO] 模型: " << modelPath << std::endl;
    std::cout << "[INFO] 跟踪器: " << trackerType << std::endl;
    std::cout << std::endl;
    
    // 初始化模块
    try {
        Inference inference(modelPath, device);
        
        DetectionConfig config;
        config.trackerType = trackerType;
        Detector detector(config);
        
        Visualization visualizer;
        
        ModeManager modeManager;
        YawController yawController(0.3);// 调整Yaw步长0.3度，提升响应速度
        
        // ========== 上下位机通信示例 ==========
        // 1. 从单片机接收基准yaw和pitch（这里使用模拟值）
        // 实际应用中，从串口或网络接收单片机发送的数据
        double receivedYaw = 0.0;    // 单片机发送的yaw值
        double receivedPitch = 45.0; // 单片机发送的pitch值
        
        // 2. 设置接收到的基准值
        yawController.setBaseYaw(receivedYaw);
        yawController.setBasePitch(receivedPitch);
        
        // 注意：在实际应用中，每次从单片机接收到新的yaw/pitch时，
        // 都需要调用setBaseYaw()和setBasePitch()来更新基准值
        // ======================================
        
        // 打开视频
        cv::VideoCapture cap;
        
        // 判断是相机编号还是文件路径
        // 如果是纯数字（0-9）则作为相机编号，否则作为文件路径
        bool isCamera = false;
        if (videoPath.length() <= 2) {  // 相机编号通常是1-2位数字
            bool allDigits = true;
            for (char c : videoPath) {
                if (!std::isdigit(c)) {
                    allDigits = false;
                    break;
                }
            }
            if (allDigits) {
                isCamera = true;
            }
        }
        
        if (isCamera) {
            // 作为相机编号打开
            int cameraId = std::stoi(videoPath);
            cap.open(cameraId);
            if (cap.isOpened()) {
                std::cout << "[INFO] 打开相机: " << cameraId << std::endl;
            } else {
                std::cerr << "[ERROR] 无法打开相机: " << cameraId << std::endl;
                std::cerr << "        请检查相机是否连接" << std::endl;
                return -1;
            }
        } else {
            // 作为文件路径打开
            cap.open(videoPath);
            if (cap.isOpened()) {
                std::cout << "[INFO] 打开视频文件: " << videoPath << std::endl;
            } else {
                std::cerr << "[ERROR] 无法打开视频文件: " << videoPath << std::endl;
                std::cerr << "        请检查文件是否存在，路径是否正确" << std::endl;
                std::cerr << "        提示: 使用绝对路径或相对于当前目录的路径" << std::endl;
                return -1;
            }
        }
        
        // 运行时状态
        int frameIdx = 0;
        int armorCount = 0, ballCount = 0;
        std::vector<double> frameTimes;
        
        int framesNoBall = 0;
        bool disappearanceHandled = false;
        std::string lastPosition = "";
        std::string lastHorizontal = "";
        float lastDiff = 0.0f;
        bool lastDiffValid = false;
        
        std::vector<TrajectoryPoint> prevTrajectory;
        cv::Point2f lastDisappearancePoint(-1, -1);
        
        const int DISAPPEAR_FRAMES = 3;
        
        cv::Point2f lastArmorCenter(-1, -1);
        
        // 完整轨迹记录（用于分析）
        std::vector<std::vector<TrajectoryPoint>> allTrajectories;
        
        // 视频写入器
        cv::VideoWriter videoWriter;
        if (!outputVideo.empty()) {
            int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
            double fps = cap.get(cv::CAP_PROP_FPS);
            if (fps <= 0) fps = 30.0;  // 默认30fps
            cv::Size frameSize(cap.get(cv::CAP_PROP_FRAME_WIDTH), 
                             cap.get(cv::CAP_PROP_FRAME_HEIGHT));
            videoWriter.open(outputVideo, fourcc, fps, frameSize);
            if (!videoWriter.isOpened()) {
                std::cerr << "[WARN] 无法创建输出视频文件: " << outputVideo << std::endl;
            } else {
                std::cout << "[INFO] 输出视频: " << outputVideo << " (FPS: " << fps << ")" << std::endl;
            }
        }
        
        std::cout << "[INFO] 开始处理视频..." << std::endl;
        std::cout << "[提示] 按 'q' 退出, 按 'r' 重置跟踪器" << std::endl;
        std::cout << std::endl;
        
        cv::Mat frame;
        HybridBallTracker& tracker = detector.getTracker();
        
        while (cap.read(frame)) {
            auto t0 = std::chrono::high_resolution_clock::now();
            
            int frameHeight = frame.rows;
            int frameWidth = frame.cols;
            
            // 1. 预处理和推理
            float scale = 1.0f;
            int padX = 0, padY = 0;
            ov::Tensor inputTensor = inference.preprocess(frame, scale, padX, padY);
            ov::Tensor outputTensor = inference.infer(inputTensor);
            
            // 2. 解析检测结果
            const float* outputData = outputTensor.data<float>();
            ov::Shape outputShape = outputTensor.get_shape();
            
            // 输出形状: (1, 4+classes, N)
            size_t numClasses = outputShape[1] - 4;
            size_t numDetections = outputShape[2];
            
            std::vector<DetectionResult> detections = detector.parseDetections(
                outputData, numDetections, numClasses, 
                scale, padX, padY, frameWidth, frameHeight);
            
            // 3. NMS
            std::vector<DetectionResult> nmsDetections = detector.nonMaxSuppression(detections);
            
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

                 if (key == 'q') break;
                     continue;
            }

            // ====== AUTO_AIM模式开始 ======
            if (currentMode == Mode::AUTO_AIM)
            {
            // AUTO_AIM模式正常运行检测和跟踪
            }

            // ====== CALIB模式开始 ======
            if (currentMode == Mode::CALIB)
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
                bool isValid = detector.validateBallTrajectory(
                    detector.getTrajectory(), &newPt, &reason);
                
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
                        visualizer.drawConnectionLine(frame, filteredYoloBall->center, 
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
                detector.updateKalman(bestBall->center);
                
                // 添加到轨迹
                TrajectoryPoint newPt(bestBall->center.x, bestBall->center.y, frameIdx);
                std::string reason;
                bool isValid = detector.validateBallTrajectory(
                    detector.getTrajectory(), &newPt, &reason);
                
                if (isValid) {
                    detector.addToTrajectory(newPt);
                } else {
                    // 保存当前轨迹到历史记录
                    if (detector.getTrajectory().size() > 5) {
                        allTrajectories.push_back(detector.getTrajectory());
                    }
                    detector.clearTrajectory();
                    detector.addToTrajectory(newPt);
                    detector.resetKalman(bestBall->center);
                }
                
                // 增加轨迹长度限制，确保完整轨迹
                while (detector.getTrajectory().size() > 300) {
                    auto& traj = const_cast<std::vector<TrajectoryPoint>&>(detector.getTrajectory());
                    traj.erase(traj.begin());
                }
                
                disappearanceHandled = false;
                framesNoBall = 0;
                
                // 绘制ball检测框
                std::string label = "Ball " + std::to_string(static_cast<int>(bestBall->confidence * 100)) + "%";
                visualizer.drawDetection(frame, *bestBall, label, ballColor);
                
                // 计算horizontal
                if (bestArmor != nullptr) {
                    float bx = bestBall->center.x;
                    lastHorizontal = (bx >= bestArmor->bbox.x && 
                                     bx <= bestArmor->bbox.x + bestArmor->bbox.width) ? 
                                     "true" : "Fault";
                    lastDiff = bx - bestArmor->center.x;
                    lastDiffValid = true;
                    // ====== 新增 yaw 调整 ======
                    yawController.update(lastDiff, lastDiffValid, lastPosition);
                    
                    // ====== 发送修正后的值回单片机 ======
                    // 获取修正后的绝对值（基准值 + 修正增量）
                    double correctedYaw = yawController.getCorrectedYaw();
                    double correctedPitch = yawController.getCorrectedPitch();
                    
                    // 实际应用中，通过串口或网络发送这些值给单片机
                    // 例如：serialPort.send(correctedYaw, correctedPitch);
                    std::cout << "[SEND] 发送给单片机 -> Yaw: " << correctedYaw 
                              << " deg, Pitch: " << correctedPitch << " deg" << std::endl;
                    // =====================================
                }
            } else {
                framesNoBall++;
                
                // 绘制预测点
                cv::Point2f pred = detector.predictBallPosition();
                if (pred.x > 0 && pred.y > 0) {
                    visualizer.drawPrediction(frame, pred);
                }
            }
            
            // 9. 处理armor检测
            if (bestArmor != nullptr) {
                // 绘制armor检测框
                std::string label = "Armor " + std::to_string(static_cast<int>(bestArmor->confidence * 100)) + "%";
                visualizer.drawDetection(frame, *bestArmor, label, cv::Scalar(0, 255, 0));
                
                // 检查ball是否消失
                if (framesNoBall >= DISAPPEAR_FRAMES && !disappearanceHandled && 
                    !detector.getTrajectory().empty()) {
                    
                    const auto& lastPt = detector.getTrajectory().back();
                    lastDisappearancePoint = cv::Point2f(lastPt.x, lastPt.y);
                    
                    // 根据消失点相对于armor下边缘判断Position
                    int armorBottom = bestArmor->bbox.y + bestArmor->bbox.height;
                    if (lastPt.y < armorBottom) {
                        lastPosition = "Back";
                    } else {
                        lastPosition = "Front";
                    }
                    
                    prevTrajectory = detector.getTrajectory();
                    disappearanceHandled = true;
                    
                    // 保存完整轨迹到历史记录
                    if (prevTrajectory.size() > 5) {
                        allTrajectories.push_back(prevTrajectory);
                    }
                    
                    detector.clearTrajectory();
                }
                
                // 绘制状态文本
                std::vector<std::pair<std::string, cv::Scalar>> statusTexts;
                
                if (detectionSource != "none") {
                    cv::Scalar srcColor = (detectionSource == "yolo") ? cv::Scalar(0, 255, 0) :
                                         (detectionSource == "hybrid") ? cv::Scalar(255, 255, 0) :
                                         cv::Scalar(0, 200, 255);
                    statusTexts.push_back({"Source: " + detectionSource, srcColor});
                    if (bestBall) {
                        statusTexts.push_back({"Conf: " + std::to_string(bestBall->confidence).substr(0, 4), srcColor});
                    }
                }
                
                // 跟踪器状态
                std::string trackerStatus = tracker.isInitialized() ? "Active" : "Inactive";
                cv::Scalar trackerColor = tracker.isInitialized() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
                statusTexts.push_back({"Tracker: " + trackerStatus + " (C:" + 
                                      std::to_string(tracker.getConfidence()).substr(0, 4) + ")", trackerColor});
                
                if (lastDiffValid) {
                    statusTexts.push_back({"Diff: " + std::to_string(lastDiff).substr(0, 6), cv::Scalar(0, 255, 0)});
                }
                
                if (!lastHorizontal.empty()) {
                    cv::Scalar hColor = (lastHorizontal == "true") ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
                    statusTexts.push_back({"Horizontal: " + lastHorizontal, hColor});
                }
                
                if (!lastPosition.empty()) {
                    cv::Scalar pColor = (lastPosition == "Front") ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
                    statusTexts.push_back({"Position: " + lastPosition, pColor});
                } else {
                    statusTexts.push_back({"Position: Waiting", cv::Scalar(200, 200, 200)});
                }
                
                visualizer.drawStatusTexts(frame, statusTexts, 10, 30, 20, 0.6);
                
                // 绘制轨迹
                if (!prevTrajectory.empty()) {
                    visualizer.drawTrajectory(frame, prevTrajectory, cv::Scalar(160, 160, 160), 2, false);
                    if (lastDisappearancePoint.x >= 0) {
                        visualizer.drawDisappearancePoint(frame, lastDisappearancePoint);
                    }
                }
                
                if (!detector.getTrajectory().empty()) {
                    visualizer.drawTrajectory(frame, detector.getTrajectory(), cv::Scalar(0, 255, 255), 2, false);
                }
            }
            
            // 10. 显示FPS
            auto t1 = std::chrono::high_resolution_clock::now();
            double frameTime = std::chrono::duration<double>(t1 - t0).count();
            frameTimes.push_back(frameTime);
            float fps = 1.0f / frameTime;
            
            visualizer.drawFPS(frame, fps);
        } // ====== CALIB模式结束 ======
            
            // 保存到输出视频
            if (videoWriter.isOpened()) {
                videoWriter.write(frame);
            }
            
            // 显示帧
            cv::imshow("Armor Judge - Detection", frame);
            
            // 统计信息
            if (frameIdx % 200 == 0) {
                std::cout << "[STAT] frame " << frameIdx 
                         << ": armor=" << armorCount 
                         << ", ball=" << ballCount << std::endl;
                std::cout << "[TRACK] Tracker status: initialized=" << tracker.isInitialized()
                         << ", confidence=" << tracker.getConfidence()
                         << ", miss_count=" << tracker.getMissCount() << std::endl;
            }
            
            // 按键处理
            int key = cv::waitKey(1) & 0xFF;

            // ====== 新增 ======
            modeManager.updateFromKeyboard(key);
            if (key == 'q') {
                break;
            } else if (key == 'r') {
                tracker.reset();
                std::cout << "[INFO] Tracker reset" << std::endl;
            }
        }
        
        // 保存最后一个活动轨迹
        if (detector.getTrajectory().size() > 5) {
            allTrajectories.push_back(detector.getTrajectory());
        }
        
        // 保存最后一个活动轨迹
        if (detector.getTrajectory().size() > 5) {
            allTrajectories.push_back(detector.getTrajectory());
        }
        
        // 性能统计
        if (!frameTimes.empty()) {
            double totalTime = 0;
            for (double t : frameTimes) {
                totalTime += t;
            }
            double avgFps = frameTimes.size() / totalTime;
            double avgFrameMs = (totalTime / frameTimes.size()) * 1000.0;
            
            std::cout << "\n[PERF] Device=" << device << ", Tracker=" << trackerType << std::endl;
            std::cout << "       Total frames: " << frameTimes.size() << std::endl;
            std::cout << "       Total time: " << totalTime << "s" << std::endl;
            std::cout << "       Average FPS: " << avgFps << std::endl;
            std::cout << "       Average frame time: " << avgFrameMs << "ms" << std::endl;
            std::cout << "       Ball detections: " << ballCount << std::endl;
            std::cout << "       Armor detections: " << armorCount << std::endl;
            
            // 轨迹统计
            std::cout << "\n[TRAJECTORY] 轨迹统计:" << std::endl;
            std::cout << "       Total trajectories: " << allTrajectories.size() << std::endl;
            int totalPoints = 0;
            int maxLength = 0;
            for (const auto& traj : allTrajectories) {
                totalPoints += traj.size();
                maxLength = std::max(maxLength, (int)traj.size());
            }
            if (!allTrajectories.empty()) {
                std::cout << "       Total trajectory points: " << totalPoints << std::endl;
                std::cout << "       Average trajectory length: " << (totalPoints / allTrajectories.size()) << std::endl;
                std::cout << "       Max trajectory length: " << maxLength << std::endl;
            }
        }
        
        // 关闭视频写入器
        if (videoWriter.isOpened()) {
            videoWriter.release();
            std::cout << "\n[INFO] 输出视频已保存: " << outputVideo << std::endl;
        }
        
        cap.release();
        cv::destroyAllWindows();
        
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
