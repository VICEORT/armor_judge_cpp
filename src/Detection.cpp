#include "Detection.h"
#include <algorithm>
#include <cmath>
#include <iostream>

// ============ BallKalmanFilter 实现 ============
BallKalmanFilter::BallKalmanFilter() {
    // 初始化状态向量 [x, y, vx, vy, ax, ay]
    state.fill(0.0f);
    
    // 初始化矩阵为零
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            F_[i][j] = 0.0f;
            P_[i][j] = 0.0f;
            Q_[i][j] = 0.0f;
        }
    }
    
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 6; ++j) {
            H_[i][j] = 0.0f;
        }
        for (int j = 0; j < 2; ++j) {
            R_[i][j] = 0.0f;
        }
    }
    
    // 状态转移矩阵
    float dt = 1.0f;
    for (int i = 0; i < 6; ++i) {
        F_[i][i] = 1.0f;
    }
    F_[0][2] = dt;
    F_[0][4] = 0.5f * dt * dt;
    F_[1][3] = dt;
    F_[1][5] = 0.5f * dt * dt;
    F_[2][4] = dt;
    F_[3][5] = dt;
    
    // 观测矩阵
    H_[0][0] = 1.0f;
    H_[1][1] = 1.0f;
    
    // 协方差矩阵
    for (int i = 0; i < 6; ++i) {
        P_[i][i] = 100.0f;
    }
    
    // 过程噪声 - 增大以适应快速移动
    Q_[0][0] = 0.05f;  // x位置
    Q_[1][1] = 0.05f;  // y位置
    Q_[2][2] = 0.1f;   // x速度
    Q_[3][3] = 0.1f;   // y速度
    Q_[4][4] = 0.5f;   // x加速度
    Q_[5][5] = 0.5f;   // y加速度
    
    // 观测噪声 - 降低以提高响应速度
    R_[0][0] = 3.0f;
    R_[1][1] = 3.0f;
}

void BallKalmanFilter::matrixMultiply66(const std::array<std::array<float, 6>, 6>& A,
                                         const std::array<std::array<float, 6>, 6>& B,
                                         std::array<std::array<float, 6>, 6>& result) {
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            result[i][j] = 0.0f;
            for (int k = 0; k < 6; ++k) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void BallKalmanFilter::matrixMultiply6x6Vector(const std::array<std::array<float, 6>, 6>& A,
                                                const std::array<float, 6>& v,
                                                std::array<float, 6>& result) {
    for (int i = 0; i < 6; ++i) {
        result[i] = 0.0f;
        for (int j = 0; j < 6; ++j) {
            result[i] += A[i][j] * v[j];
        }
    }
}

void BallKalmanFilter::predict() {
    // state = F * state
    std::array<float, 6> newState;
    matrixMultiply6x6Vector(F_, state, newState);
    state = newState;
    
    // P = F * P * F^T + Q
    std::array<std::array<float, 6>, 6> FP, FPFt;
    matrixMultiply66(F_, P_, FP);
    
    // F^T
    std::array<std::array<float, 6>, 6> Ft;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            Ft[i][j] = F_[j][i];
        }
    }
    
    matrixMultiply66(FP, Ft, FPFt);
    
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            P_[i][j] = FPFt[i][j] + Q_[i][j];
        }
    }
}

void BallKalmanFilter::update(const cv::Point2f& measurement) {
    // 简化的卡尔曼更新（仅更新x和y位置）
    std::array<float, 2> z = {measurement.x, measurement.y};
    
    // y = z - H * state
    std::array<float, 2> y;
    y[0] = z[0] - state[0];
    y[1] = z[1] - state[1];
    
    // 自适应增益：快速移动时增大增益以提高响应速度
    float innovation = std::sqrt(y[0]*y[0] + y[1]*y[1]);
    float gain = (innovation > 20.0f) ? 0.7f : 0.5f;
    
    state[0] += gain * y[0];
    state[1] += gain * y[1];
    
    // 更新速度估计 - 提高响应速度
    if (state[2] == 0.0f && state[3] == 0.0f) {
        // 首次更新，初始化速度
        state[2] = y[0];
        state[3] = y[1];
    } else {
        // 速度更新 - 快速移动时更敏感
        float alpha = (innovation > 20.0f) ? 0.5f : 0.3f;
        state[2] = state[2] * (1.0f - alpha) + y[0] * alpha;
        state[3] = state[3] * (1.0f - alpha) + y[1] * alpha;
    }
}

cv::Point2f BallKalmanFilter::getPrediction() const {
    return cv::Point2f(state[0], state[1]);
}

void BallKalmanFilter::reset(const cv::Point2f& initPos) {
    state.fill(0.0f);
    state[0] = initPos.x;
    state[1] = initPos.y;
    
    for (int i = 0; i < 6; ++i) {
        P_[i][i] = 100.0f;
    }
}

// ============ HybridBallTracker 实现 ============
HybridBallTracker::HybridBallTracker(const std::string& type)
    : trackerType_(type), isInitialized_(false), 
      missCount_(0), trackerConfidence_(0.0f) {
}

bool HybridBallTracker::initialize(const cv::Mat& frame, const cv::Rect& bbox) {
    try {
        if (trackerType_ == "CSRT") {
            tracker_ = cv::TrackerCSRT::create();
        } else if (trackerType_ == "KCF") {
            tracker_ = cv::TrackerKCF::create();
        } else if (trackerType_ == "MOSSE") {
            // MOSSE已弃用，使用KCF替代
            tracker_ = cv::TrackerKCF::create();
        } else {
            tracker_ = cv::TrackerCSRT::create();
        }
        
        tracker_->init(frame, bbox);
        isInitialized_ = true;
        missCount_ = 0;
        lastValidBbox_ = bbox;
        trackerConfidence_ = 1.0f;
        
        positionHistory_.clear();
        sizeHistory_.clear();
        velocityHistory_.clear();
        
        cv::Point2f center(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);
        positionHistory_.push_back(center);
        sizeHistory_.push_back(cv::Size(bbox.width, bbox.height));
        
        return isInitialized_;
    } catch (const std::exception& e) {
        std::cerr << "[WARNING] Tracker initialization failed: " << e.what() << std::endl;
        isInitialized_ = false;
        return false;
    }
}

bool HybridBallTracker::update(const cv::Mat& frame, cv::Rect& bbox, float& confidence) {
    if (!isInitialized_ || tracker_.empty()) {
        return false;
    }
    
    try {
        bool success = tracker_->update(frame, bbox);
        
        if (success) {
            confidence = calculateTrackerConfidence(bbox);
            trackerConfidence_ = confidence;
            
            if (confidence < 0.5f) {  // TRACKER_CONFIDENCE_THRES
                success = false;
                missCount_++;
            } else {
                lastValidBbox_ = bbox;
                missCount_ = 0;
                
                // 更新历史
                cv::Point2f center(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);
                sizeHistory_.push_back(cv::Size(bbox.width, bbox.height));
                if (sizeHistory_.size() > 10) {
                    sizeHistory_.erase(sizeHistory_.begin());
                }
                
                positionHistory_.push_back(center);
                if (positionHistory_.size() > 10) {
                    if (positionHistory_.size() > 1) {
                        cv::Point2f lastCenter = positionHistory_[positionHistory_.size() - 2];
                        cv::Point2f velocity = center - lastCenter;
                        velocityHistory_.push_back(velocity);
                        if (velocityHistory_.size() > 5) {
                            velocityHistory_.erase(velocityHistory_.begin());
                        }
                    }
                    positionHistory_.erase(positionHistory_.begin());
                }
            }
        } else {
            missCount_++;
            trackerConfidence_ = std::max(0.0f, trackerConfidence_ - 0.2f);
        }
        
        return success;
    } catch (const std::exception& e) {
        std::cerr << "[WARNING] Tracker update failed: " << e.what() << std::endl;
        missCount_++;
        return false;
    }
}

float HybridBallTracker::calculateTrackerConfidence(const cv::Rect& bbox) {
    float confidence = 1.0f;
    
    // 检查bbox合理性
    if (bbox.width <= 0 || bbox.height <= 0 || bbox.width > 200 || bbox.height > 200) {
        return 0.0f;
    }
    
    // 检查尺寸稳定性
    if (!sizeHistory_.empty()) {
        float avgW = 0, avgH = 0;
        for (const auto& sz : sizeHistory_) {
            avgW += sz.width;
            avgH += sz.height;
        }
        avgW /= sizeHistory_.size();
        avgH /= sizeHistory_.size();
        
        float sizeChange = std::abs(bbox.width - avgW) / avgW + 
                          std::abs(bbox.height - avgH) / avgH;
        confidence *= std::max(0.0f, 1.0f - sizeChange);
    }
    
    // 检查运动一致性
    if (velocityHistory_.size() >= 2) {
        float avgVx = 0, avgVy = 0;
        for (const auto& v : velocityHistory_) {
            avgVx += v.x;
            avgVy += v.y;
        }
        avgVx /= velocityHistory_.size();
        avgVy /= velocityHistory_.size();
        
        float stdVx = 0, stdVy = 0;
        for (const auto& v : velocityHistory_) {
            stdVx += (v.x - avgVx) * (v.x - avgVx);
            stdVy += (v.y - avgVy) * (v.y - avgVy);
        }
        stdVx = std::sqrt(stdVx / velocityHistory_.size());
        stdVy = std::sqrt(stdVy / velocityHistory_.size());
        
        float motionVariance = (stdVx + stdVy) / std::max(1.0f, std::abs(avgVx) + std::abs(avgVy));
        confidence *= std::max(0.0f, 1.0f - motionVariance * 0.5f);
    }
    
    return std::max(0.0f, std::min(1.0f, confidence));
}

void HybridBallTracker::reset() {
    tracker_.release();
    isInitialized_ = false;
    missCount_ = 0;
    trackerConfidence_ = 0.0f;
    positionHistory_.clear();
    sizeHistory_.clear();
    velocityHistory_.clear();
}

bool HybridBallTracker::isReliable() const {
    return isInitialized_ && missCount_ < 5 && trackerConfidence_ > 0.5f;
}

// ============ Detector 实现 ============
Detector::Detector(const DetectionConfig& config) 
    : config_(config), tracker_(config.trackerType) {
}

Detector::~Detector() {
}

std::vector<DetectionResult> Detector::parseDetections(
    const float* output,
    size_t numDetections,
    size_t numClasses,
    float scale,
    int padX,
    int padY,
    int frameWidth,
    int frameHeight) {
    
    std::vector<DetectionResult> detections;
    
    // 输出格式: (cx, cy, w, h, class_scores...)
    // 转置为每个检测一行
    for (size_t i = 0; i < numDetections; ++i) {
        float cx = output[i];
        float cy = output[numDetections + i];
        float w = output[2 * numDetections + i];
        float h = output[3 * numDetections + i];
        
        // 找到最大置信度类别
        int classID = -1;
        float maxConf = 0.0f;
        for (size_t c = 0; c < numClasses; ++c) {
            float conf = output[(4 + c) * numDetections + i];
            if (conf > maxConf) {
                maxConf = conf;
                classID = c;
            }
        }
        
        // 过滤类别
        if (classID != config_.armorID && classID != config_.ballID) {
            continue;
        }
        
        // 应用置信度阈值
        float confThres = (classID == config_.armorID) ? 
                          config_.armorConfThres : config_.ballConfThres;
        if (maxConf < confThres) {
            continue;
        }
        
        // 转换坐标到原图
        float cxImg = (cx - padX) / scale;
        float cyImg = (cy - padY) / scale;
        float wImg = w / scale;
        float hImg = h / scale;
        
        int x1 = std::max(0, static_cast<int>(cxImg - wImg / 2));
        int y1 = std::max(0, static_cast<int>(cyImg - hImg / 2));
        int x2 = std::min(frameWidth - 1, static_cast<int>(cxImg + wImg / 2));
        int y2 = std::min(frameHeight - 1, static_cast<int>(cyImg + hImg / 2));
        
        DetectionResult det;
        det.bbox = cv::Rect(x1, y1, x2 - x1, y2 - y1);
        det.center = cv::Point2f((x1 + x2) / 2.0f, (y1 + y2) / 2.0f);
        det.confidence = maxConf;
        det.classID = classID;
        det.originalConf = maxConf;
        
        detections.push_back(det);
    }
    
    return detections;
}

float Detector::calculateIOU(const cv::Rect& box1, const cv::Rect& box2) {
    cv::Rect intersection = box1 & box2;
    float interArea = intersection.area();
    float unionArea = box1.area() + box2.area() - interArea;
    
    if (unionArea <= 0) return 0.0f;
    return interArea / unionArea;
}

std::vector<DetectionResult> Detector::nonMaxSuppression(
    const std::vector<DetectionResult>& detections,
    float iouThreshold) {
    
    if (detections.empty()) return {};
    
    // 分类别处理
    std::vector<DetectionResult> armors, balls;
    for (const auto& det : detections) {
        if (det.classID == config_.armorID) {
            armors.push_back(det);
        } else if (det.classID == config_.ballID) {
            balls.push_back(det);
        }
    }
    
    // 对ball使用更宽松的NMS（减少误抑制）
    auto nmsClass = [this](std::vector<DetectionResult>& dets, float iou) {
        if (dets.empty()) return;
        
        std::sort(dets.begin(), dets.end(), 
                  [](const DetectionResult& a, const DetectionResult& b) {
                      return a.confidence > b.confidence;
                  });
        
        std::vector<bool> suppressed(dets.size(), false);
        std::vector<DetectionResult> result;
        
        for (size_t i = 0; i < dets.size(); ++i) {
            if (suppressed[i]) continue;
            result.push_back(dets[i]);
            
            for (size_t j = i + 1; j < dets.size(); ++j) {
                if (suppressed[j]) continue;
                float overlap = calculateIOU(dets[i].bbox, dets[j].bbox);
                if (overlap > iou) {
                    suppressed[j] = true;
                }
            }
        }
        dets = result;
    };
    
    nmsClass(armors, iouThreshold);
    nmsClass(balls, config_.nmsThreshold);  // ball使用单独阈值
    
    // 合并结果
    std::vector<DetectionResult> result;
    result.insert(result.end(), armors.begin(), armors.end());
    result.insert(result.end(), balls.begin(), balls.end());
    
    return result;
}

bool Detector::isTrajectoryVertical(const std::vector<TrajectoryPoint>& trajectory) {
    if (trajectory.size() < 3) return true;
    
    const auto& first = trajectory.front();
    const auto& last = trajectory.back();
    
    float dx = last.x - first.x;
    float dy = last.y - first.y;
    
    if (std::abs(dy) < 5 && std::abs(dx) < 5) return true;
    
    float angleRad = std::atan2(dx, dy);
    float angleDeg = std::abs(angleRad * 180.0f / M_PI);
    
    return angleDeg <= config_.maxTrajectoryAngle;
}

bool Detector::isTrajectoryDownward(const std::vector<TrajectoryPoint>& trajectory) {
    if (trajectory.size() < 3) return true;
    
    const auto& first = trajectory.front();
    const auto& last = trajectory.back();
    
    float dyTotal = last.y - first.y;
    if (dyTotal <= 0) return false;
    
    int downCount = 0;
    for (size_t i = 1; i < trajectory.size(); ++i) {
        if (trajectory[i].y >= trajectory[i-1].y) {
            downCount++;
        }
    }
    
    return downCount >= static_cast<int>((trajectory.size() - 1) / 2);
}

bool Detector::checkHorizontalMovement(const std::vector<TrajectoryPoint>& trajectory) {
    if (trajectory.size() < 2) return true;
    
    for (size_t i = 1; i < trajectory.size(); ++i) {
        float dx = std::abs(trajectory[i].x - trajectory[i-1].x);
        if (dx > config_.maxHorizontalMove) {
            return false;
        }
    }
    
    float totalDrift = std::abs(trajectory.back().x - trajectory.front().x);
    if (totalDrift > config_.maxHorizontalMove * 2) {
        return false;
    }
    
    return true;
}

bool Detector::checkStagnation(const std::vector<TrajectoryPoint>& trajectory) {
    if (trajectory.size() < static_cast<size_t>(config_.stagnantFrames)) return true;
    
    // 检查最近N帧
    size_t startIdx = trajectory.size() - config_.stagnantFrames;
    float centerX = 0, centerY = 0;
    
    for (size_t i = startIdx; i < trajectory.size(); ++i) {
        centerX += trajectory[i].x;
        centerY += trajectory[i].y;
    }
    centerX /= config_.stagnantFrames;
    centerY /= config_.stagnantFrames;
    
    // 检查所有点是否在半径内
    bool allWithinRadius = true;
    for (size_t i = startIdx; i < trajectory.size(); ++i) {
        float dist = std::sqrt(std::pow(trajectory[i].x - centerX, 2) + 
                              std::pow(trajectory[i].y - centerY, 2));
        if (dist > config_.stagnantRadius) {
            allWithinRadius = false;
            break;
        }
    }
    
    if (allWithinRadius) {
        // 检查垂直移动
        float minY = trajectory[startIdx].y;
        float maxY = trajectory[startIdx].y;
        for (size_t i = startIdx + 1; i < trajectory.size(); ++i) {
            minY = std::min(minY, trajectory[i].y);
            maxY = std::max(maxY, trajectory[i].y);
        }
        
        float verticalMovement = maxY - minY;
        if (verticalMovement < config_.minVerticalSpeed * (config_.stagnantFrames - 1)) {
            return false;
        }
    }
    
    return true;
}

bool Detector::checkAngleConsistency(const std::vector<TrajectoryPoint>& trajectory) {
    if (trajectory.size() < 2) return true;
    
    for (size_t i = 1; i < trajectory.size(); ++i) {
        size_t startIdx = (i >= 5) ? (i - 5) : 0;
        
        float dx = trajectory[i].x - trajectory[startIdx].x;
        float dy = trajectory[i].y - trajectory[startIdx].y;
        
        if (std::abs(dy) >= 5) {
            float angleRad = std::atan2(dx, dy);
            float angleDeg = std::abs(angleRad * 180.0f / M_PI);
            if (angleDeg > config_.maxTrajectoryAngle) {
                return false;
            }
        }
    }
    
    return true;
}

bool Detector::validateBallTrajectory(
    const std::vector<TrajectoryPoint>& trajectory,
    const TrajectoryPoint* newPoint,
    std::string* reason) {
    
    std::vector<TrajectoryPoint> testTrajectory = trajectory;
    if (newPoint != nullptr) {
        testTrajectory.push_back(*newPoint);
    }
    
    if (testTrajectory.size() < 2) {
        if (reason) *reason = "Too short";
        return true;
    }
    
    if (!isTrajectoryVertical(testTrajectory)) {
        if (reason) *reason = "Not vertical";
        return false;
    }
    
    if (!isTrajectoryDownward(testTrajectory)) {
        if (reason) *reason = "Not downward";
        return false;
    }
    
    if (!checkHorizontalMovement(testTrajectory)) {
        if (reason) *reason = "Horizontal movement";
        return false;
    }
    
    if (!checkStagnation(testTrajectory)) {
        if (reason) *reason = "Stagnation detected";
        return false;
    }
    
    if (!checkAngleConsistency(testTrajectory)) {
        if (reason) *reason = "Angle inconsistency";
        return false;
    }
    
    if (reason) *reason = "Valid";
    return true;
}

cv::Point2f Detector::predictBallPosition() {
    return kalmanFilter_.getPrediction();
}

void Detector::updateKalman(const cv::Point2f& measurement) {
    kalmanFilter_.update(measurement);
}

void Detector::resetKalman(const cv::Point2f& initPos) {
    kalmanFilter_.reset(initPos);
}
