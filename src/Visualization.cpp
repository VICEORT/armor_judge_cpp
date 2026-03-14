#include "Visualization.h"

Visualization::Visualization() {
    colorGreen_ = cv::Scalar(0, 255, 0);
    colorRed_ = cv::Scalar(0, 0, 255);
    colorBlue_ = cv::Scalar(255, 0, 0);
    colorYellow_ = cv::Scalar(0, 255, 255);
    colorCyan_ = cv::Scalar(255, 255, 0);
    colorGray_ = cv::Scalar(160, 160, 160);
    colorWhite_ = cv::Scalar(255, 255, 255);
}

Visualization::~Visualization() {
}

void Visualization::drawDetection(cv::Mat& frame,
                                   const DetectionResult& detection,
                                   const std::string& label,
                                   const cv::Scalar& color) {
    // 绘制矩形框
    cv::rectangle(frame, detection.bbox, color, 2);
    
    // 绘制标签
    int labelY = std::max(0, detection.bbox.y - 6);
    cv::putText(frame, label, 
                cv::Point(detection.bbox.x, labelY),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
}

void Visualization::drawTrajectory(cv::Mat& frame,
                                    const std::vector<TrajectoryPoint>& trajectory,
                                    const cv::Scalar& color,
                                    int thickness,
                                    bool drawPoints) {
    if (trajectory.size() < 2) return;
    
    // 绘制连线
    for (size_t i = 1; i < trajectory.size(); ++i) {
        cv::Point pt1(static_cast<int>(trajectory[i-1].x), 
                     static_cast<int>(trajectory[i-1].y));
        cv::Point pt2(static_cast<int>(trajectory[i].x), 
                     static_cast<int>(trajectory[i].y));
        cv::line(frame, pt1, pt2, color, thickness);
    }
    
    // 绘制点
    if (drawPoints) {
        for (const auto& pt : trajectory) {
            cv::circle(frame, cv::Point(static_cast<int>(pt.x), 
                                       static_cast<int>(pt.y)), 
                      3, color, -1);
        }
    }
    
    // 绘制最新点
    if (!trajectory.empty()) {
        const auto& lastPt = trajectory.back();
        cv::circle(frame, cv::Point(static_cast<int>(lastPt.x), 
                                    static_cast<int>(lastPt.y)),
                  4, colorGreen_, -1);
    }
}

void Visualization::drawDisappearancePoint(cv::Mat& frame,
                                            const cv::Point2f& point,
                                            const std::string& label) {
    cv::Point pt(static_cast<int>(point.x), static_cast<int>(point.y));
    
    // 绘制红色圆圈
    cv::circle(frame, pt, 6, colorRed_, -1);
    
    // 绘制标签
    cv::putText(frame, label, 
                cv::Point(pt.x + 8, pt.y - 8),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, colorRed_, 1);
}

void Visualization::drawStatusTexts(cv::Mat& frame,
                                     const std::vector<std::pair<std::string, cv::Scalar>>& texts,
                                     int x, int y, int lineHeight, double fontScale) {
    for (size_t i = 0; i < texts.size(); ++i) {
        int yi = y + i * lineHeight;
        cv::putText(frame, texts[i].first,
                   cv::Point(x, yi),
                   cv::FONT_HERSHEY_SIMPLEX, fontScale, texts[i].second, 2);
    }
}

void Visualization::drawPrediction(cv::Mat& frame, const cv::Point2f& prediction) {
    cv::Point pt(static_cast<int>(prediction.x), static_cast<int>(prediction.y));
    cv::circle(frame, pt, 4, cv::Scalar(0, 0, 200), -1);
}

void Visualization::drawFPS(cv::Mat& frame, float fps) {
    std::string fpsText = "FPS: " + std::to_string(static_cast<int>(fps));
    int x = frame.cols - 120;
    int y = 30;
    cv::putText(frame, fpsText, cv::Point(x, y),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, colorCyan_, 2);
}

void Visualization::drawConnectionLine(cv::Mat& frame,
                                        const cv::Point2f& pt1,
                                        const cv::Point2f& pt2,
                                        float distance) {
    cv::Point p1(static_cast<int>(pt1.x), static_cast<int>(pt1.y));
    cv::Point p2(static_cast<int>(pt2.x), static_cast<int>(pt2.y));
    
    // 绘制连线
    cv::line(frame, p1, p2, colorCyan_, 1);
    
    // 绘制距离文本
    cv::Point midPt((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
    std::string distText = "Dist:" + std::to_string(static_cast<int>(distance));
    cv::putText(frame, distText, midPt,
               cv::FONT_HERSHEY_SIMPLEX, 0.4, colorCyan_, 1);
}

void Visualization::drawArmorBottomLine(cv::Mat& frame, const cv::Rect& armorBox) {
    int y = armorBox.y + armorBox.height;
    cv::line(frame, cv::Point(armorBox.x, y), 
             cv::Point(armorBox.x + armorBox.width, y),
             colorGreen_, 1, cv::LINE_AA);
}
