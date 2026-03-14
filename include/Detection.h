#ifndef DETECTION_H
#define DETECTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <vector>
#include <memory>
#include <array>

// 配置参数
struct DetectionConfig {
    int inputSize = 640;
    float armorConfThres = 0.25f;
    float ballConfThres = 0.25f;  // 进一步降低阈值提高检出率
    float nmsThreshold = 0.45f;   // NMS阈值
    int armorID = 0;
    int ballID = 1;
    
    float maxTrajectoryAngle = 15.0f;
    float maxHorizontalMove = 20.0f;
    int stagnantFrames = 3;
    float stagnantRadius = 30.0f;
    float minVerticalSpeed = 2.0f;
    float maxBallArmorAreaRatio = 0.5f;
    float minBallArmorAreaRatio = 0.3f;  // ball面积不能小于armor的0.333倍，ball拉长为答辩形状后体积偏大，不再限制下限
    
    float vertDeviationThresh = 10.0f;
    int vertTolFrames = 3;
    float horizDisconnectThresh = 50.0f;
    float predTolerance = 30.0f;
    float predXTolerance = 40.0f;
    
    std::string trackerType = "CSRT";
    int trackerMaxMiss = 5;
    float trackerConfidenceThres = 0.5f;
    float hybridFusionWeight = 0.85f;  // 提高YOLO权重，改善跟踪效果
};

// 检测结果结构
struct DetectionResult {
    cv::Rect bbox;
    cv::Point2f center;
    float confidence;
    int classID;
    float originalConf;
    
    DetectionResult() : confidence(0), classID(-1), originalConf(0) {}
    DetectionResult(cv::Rect b, float conf, int cls) 
        : bbox(b), confidence(conf), classID(cls), originalConf(conf) {
        center = cv::Point2f(b.x + b.width / 2.0f, b.y + b.height / 2.0f);
    }
};

// 轨迹点
struct TrajectoryPoint {
    float x, y;
    int frameIdx;
    
    TrajectoryPoint(float x_ = 0, float y_ = 0, int f = 0) 
        : x(x_), y(y_), frameIdx(f) {}
};

// 卡尔曼滤波器（恒定加速度模型）
class BallKalmanFilter {
public:
    BallKalmanFilter();
    void predict();
    void update(const cv::Point2f& measurement);
    cv::Point2f getPrediction() const;
    void reset(const cv::Point2f& initPos);
    
    std::array<float, 6> state;  // [x, y, vx, vy, ax, ay]
    
private:
    std::array<std::array<float, 6>, 6> F_;  // 状态转移矩阵
    std::array<std::array<float, 6>, 2> H_;  // 观测矩阵
    std::array<std::array<float, 6>, 6> P_;  // 协方差矩阵
    std::array<std::array<float, 6>, 6> Q_;  // 过程噪声
    std::array<std::array<float, 2>, 2> R_;  // 观测噪声
    
    // 辅助矩阵运算函数
    void matrixMultiply66(const std::array<std::array<float, 6>, 6>& A, 
                          const std::array<std::array<float, 6>, 6>& B,
                          std::array<std::array<float, 6>, 6>& result);
    void matrixMultiply6x6Vector(const std::array<std::array<float, 6>, 6>& A,
                                 const std::array<float, 6>& v,
                                 std::array<float, 6>& result);
};

// 混合跟踪器
class HybridBallTracker {
public:
    HybridBallTracker(const std::string& type = "CSRT");
    bool initialize(const cv::Mat& frame, const cv::Rect& bbox);
    bool update(const cv::Mat& frame, cv::Rect& bbox, float& confidence);
    void reset();
    bool isReliable() const;
    
    bool isInitialized() const { return isInitialized_; }
    float getConfidence() const { return trackerConfidence_; }
    int getMissCount() const { return missCount_; }
    
private:
    cv::Ptr<cv::Tracker> tracker_;
    std::string trackerType_;
    bool isInitialized_;
    int missCount_;
    cv::Rect lastValidBbox_;
    float trackerConfidence_;
    
    std::vector<cv::Point2f> positionHistory_;
    std::vector<cv::Size> sizeHistory_;
    std::vector<cv::Point2f> velocityHistory_;
    
    float calculateTrackerConfidence(const cv::Rect& bbox);
};

// 识别模块
class Detector {
public:
    Detector(const DetectionConfig& config = DetectionConfig());
    ~Detector();
    
    /**
     * @brief 解析YOLO输出
     * @param output 推理输出tensor (1, C, N)
     * @param scale letterbox缩放比例
     * @param padX 横向填充
     * @param padY 纵向填充
     * @param frameWidth 原始帧宽度
     * @param frameHeight 原始帧高度
     * @return 检测结果
     */
    std::vector<DetectionResult> parseDetections(
        const float* output, 
        size_t numDetections,
        size_t numClasses,
        float scale, 
        int padX, 
        int padY,
        int frameWidth,
        int frameHeight);
    
    /**
     * @brief NMS非极大值抑制
     * @param detections 输入检测列表
     * @param iouThreshold IOU阈值
     * @return 过滤后的检测
     */
    std::vector<DetectionResult> nonMaxSuppression(
        const std::vector<DetectionResult>& detections,
        float iouThreshold = 0.4f);
    
    /**
     * @brief 验证ball轨迹有效性
     * @param trajectory 轨迹点列表
     * @param newPoint 新点（可选）
     * @param reason 输出原因
     * @return 是否有效
     */
    bool validateBallTrajectory(
        const std::vector<TrajectoryPoint>& trajectory,
        const TrajectoryPoint* newPoint = nullptr,
        std::string* reason = nullptr);
    
    /**
     * @brief 卡尔曼预测
     */
    cv::Point2f predictBallPosition();
    
    /**
     * @brief 更新卡尔曼滤波器
     */
    void updateKalman(const cv::Point2f& measurement);
    
    /**
     * @brief 重置卡尔曼滤波器
     */
    void resetKalman(const cv::Point2f& initPos);
    
    /**
     * @brief 获取/设置轨迹
     */
    const std::vector<TrajectoryPoint>& getTrajectory() const { return ballTrajectory_; }
    void addToTrajectory(const TrajectoryPoint& pt) { ballTrajectory_.push_back(pt); }
    void clearTrajectory() { ballTrajectory_.clear(); }
    
    /**
     * @brief 混合跟踪器
     */
    HybridBallTracker& getTracker() { return tracker_; }
    
private:
    DetectionConfig config_;
    BallKalmanFilter kalmanFilter_;
    HybridBallTracker tracker_;
    std::vector<TrajectoryPoint> ballTrajectory_;
    
    // 辅助函数
    bool isTrajectoryVertical(const std::vector<TrajectoryPoint>& trajectory);
    bool isTrajectoryDownward(const std::vector<TrajectoryPoint>& trajectory);
    bool checkHorizontalMovement(const std::vector<TrajectoryPoint>& trajectory);
    bool checkStagnation(const std::vector<TrajectoryPoint>& trajectory);
    bool checkAngleConsistency(const std::vector<TrajectoryPoint>& trajectory);
    
    float calculateIOU(const cv::Rect& box1, const cv::Rect& box2);
};

#endif // DETECTION_H
