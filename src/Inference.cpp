#include "Inference.h"
#include <iostream>

Inference::Inference(const std::string& modelPath, 
                     const std::string& device,
                     int inputSize)
    : device_(device), inputSize_(inputSize) {
    
    try {
        // 编译模型
        std::cout << "[INFO] Loading model: " << modelPath << std::endl;
        compiledModel_ = std::make_shared<ov::CompiledModel>(
            core_.compile_model(modelPath, device_)
        );
        std::cout << "[INFO] Compiled model on device: " << device_ << std::endl;
        
        // 创建推理请求
        inferRequest_ = compiledModel_->create_infer_request();
        
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to compile model on " << device_ 
                  << ": " << e.what() << std::endl;
        throw;
    }
}

Inference::~Inference() {
}

cv::Mat Inference::letterbox(const cv::Mat& img, cv::Size newShape, 
                              float& scale, int& padX, int& padY,
                              const cv::Scalar& color) {
    int h = img.rows;
    int w = img.cols;
    
    // 计算缩放比例
    scale = std::min(static_cast<float>(newShape.height) / h, 
                     static_cast<float>(newShape.width) / w);
    
    int nh = static_cast<int>(h * scale);
    int nw = static_cast<int>(w * scale);
    
    // 调整大小
    cv::Mat imgResized;
    cv::resize(img, imgResized, cv::Size(nw, nh));
    
    // 计算padding
    int padW = newShape.width - nw;
    int padH = newShape.height - nh;
    int top = padH / 2;
    int left = padW / 2;
    padX = left;
    padY = top;
    
    // 添加边框
    cv::Mat imgPadded;
    cv::copyMakeBorder(imgResized, imgPadded, top, padH - top, left, padW - left,
                       cv::BORDER_CONSTANT, color);
    
    return imgPadded;
}

ov::Tensor Inference::preprocess(const cv::Mat& frame, float& scale, int& padX, int& padY) {
    // Letterbox - 直接使用原始画面
    cv::Mat imgLb = letterbox(frame, cv::Size(inputSize_, inputSize_), 
                              scale, padX, padY);
    
    // BGR to RGB
    cv::Mat imgRgb;
    cv::cvtColor(imgLb, imgRgb, cv::COLOR_BGR2RGB);
    
    // 归一化
    cv::Mat imgFloat;
    imgRgb.convertTo(imgFloat, CV_32F, 1.0 / 255.0);
    
    // HWC to CHW
    std::vector<cv::Mat> channels(3);
    cv::split(imgFloat, channels);
    
    // 创建tensor (1, 3, inputSize, inputSize)
    ov::Shape tensorShape = {1, 3, static_cast<size_t>(inputSize_), 
                             static_cast<size_t>(inputSize_)};
    ov::Tensor inputTensor(ov::element::f32, tensorShape);
    
    float* tensorData = inputTensor.data<float>();
    
    // 复制数据 CHW格式
    for (int c = 0; c < 3; ++c) {
        std::memcpy(tensorData + c * inputSize_ * inputSize_, 
                    channels[c].data, 
                    inputSize_ * inputSize_ * sizeof(float));
    }
    
    return inputTensor;
}

ov::Tensor Inference::infer(const ov::Tensor& inputTensor) {
    // 设置输入
    inferRequest_.set_input_tensor(inputTensor);
    
    // 执行推理
    inferRequest_.infer();
    
    // 获取输出
    return inferRequest_.get_output_tensor(0);
}
