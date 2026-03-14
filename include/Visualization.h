#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "Detection.h"

/**
 * @brief 可视化模块：封装所有绘制功能
 */
class Visualization {
public:
    Visualization();
    ~Visualization();
    
    /**
     * @brief 绘制检测框
     * @param frame 图像帧
     * @param detection 检测结果
     * @param label 标签文本
     * @param color 颜色
     */
    void drawDetection(cv::Mat& frame, 
                       const DetectionResult& detection,
                       const std::string& label,
                       const cv::Scalar& color);
    
    /**
     * @brief 绘制轨迹
     * @param frame 图像帧
     * @param trajectory 轨迹点列表
     * @param color 颜色
     * @param thickness 线条粗细
     * @param drawPoints 是否绘制点
     */
    void drawTrajectory(cv::Mat& frame,
                        const std::vector<TrajectoryPoint>& trajectory,
                        const cv::Scalar& color,
                        int thickness = 2,
                        bool drawPoints = false);
    
    /**
     * @brief 绘制消失点
     * @param frame 图像帧
     * @param point 消失点坐标
     * @param label 标签
     */
    void drawDisappearancePoint(cv::Mat& frame,
                                const cv::Point2f& point,
                                const std::string& label = "Disappear");
    
    /**
     * @brief 绘制状态文本
     * @param frame 图像帧
     * @param texts 文本和颜色列表
     * @param x 起始X坐标
     * @param y 起始Y坐标
     * @param lineHeight 行高
     * @param fontScale 字体缩放
     */
    void drawStatusTexts(cv::Mat& frame,
                         const std::vector<std::pair<std::string, cv::Scalar>>& texts,
                         int x = 10,
                         int y = 30,
                         int lineHeight = 22,
                         double fontScale = 0.7);
    
    /**
     * @brief 绘制卡尔曼预测点
     * @param frame 图像帧
     * @param prediction 预测位置
     */
    void drawPrediction(cv::Mat& frame, const cv::Point2f& prediction);
    
    /**
     * @brief 绘制FPS
     * @param frame 图像帧
     * @param fps FPS值
     */
    void drawFPS(cv::Mat& frame, float fps);
    
    /**
     * @brief 绘制连接线（检测和跟踪的距离）
     * @param frame 图像帧
     * @param pt1 点1
     * @param pt2 点2
     * @param distance 距离值
     */
    void drawConnectionLine(cv::Mat& frame,
                           const cv::Point2f& pt1,
                           const cv::Point2f& pt2,
                           float distance);
    
    /**
     * @brief 绘制armor下边缘参考线
     * @param frame 图像帧
     * @param armorBox armor检测框
     */
    void drawArmorBottomLine(cv::Mat& frame, const cv::Rect& armorBox);
    
private:
    cv::Scalar colorGreen_;
    cv::Scalar colorRed_;
    cv::Scalar colorBlue_;
    cv::Scalar colorYellow_;
    cv::Scalar colorCyan_;
    cv::Scalar colorGray_;
    cv::Scalar colorWhite_;
};

#endif // VISUALIZATION_H
