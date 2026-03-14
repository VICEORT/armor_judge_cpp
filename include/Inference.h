#ifndef INFERENCE_H
#define INFERENCE_H

#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

/**
 * @brief 推理模块：封装OpenVINO推理功能
 */
class Inference {
public:
    /**
     * @brief 构造函数
     * @param modelPath ONNX模型路径
     * @param device 推理设备 ("CPU" 或 "GPU")
     * @param inputSize 输入图像大小 (默认640x640)
     */
    Inference(const std::string& modelPath, 
              const std::string& device = "CPU",
              int inputSize = 640);
    
    /**
     * @brief 析构函数
     */
    ~Inference();
    
    /**
     * @brief 预处理图像 - letterbox + 归一化
     * @param frame 输入图像
     * @param scale 输出缩放比例
     * @param padX 输出横向填充
     * @param padY 输出纵向填充
     * @return 预处理后的tensor
     */
    ov::Tensor preprocess(const cv::Mat& frame, float& scale, int& padX, int& padY);
    
    /**
     * @brief 执行推理
     * @param inputTensor 输入tensor
     * @return 推理结果 (1, C, N) 其中C是类别+4坐标，N是检测框数量
     */
    ov::Tensor infer(const ov::Tensor& inputTensor);
    
    /**
     * @brief 获取输入尺寸
     */
    int getInputSize() const { return inputSize_; }
    
private:
    ov::Core core_;
    std::shared_ptr<ov::CompiledModel> compiledModel_;
    ov::InferRequest inferRequest_;
    std::string device_;
    int inputSize_;
    
    // Letterbox填充
    cv::Mat letterbox(const cv::Mat& img, cv::Size newShape, 
                      float& scale, int& padX, int& padY, 
                      const cv::Scalar& color = cv::Scalar(114, 114, 114));
};

#endif // INFERENCE_H
