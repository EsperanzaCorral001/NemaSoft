#ifndef INFERENCEWORKER_H
#define INFERENCEWORKER_H

#include <QObject>
#include <QMutex>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>


#define CONFIDENCE_THRESHOLD 0.1f
#define OVERLAP_THRESHOLD 0.1f
#define TILE_FACTOR 4

class InferenceWorker : public QObject {
    Q_OBJECT

public:
    explicit InferenceWorker(int frameWidth, int frameHeight, cv::Mat& img);
    explicit InferenceWorker(int frameWidth, int frameHeight, cv::Mat& img, const std::vector<cv::Rect>& manualBoxes); // New constructor
    ~InferenceWorker();

    void clearInput() {
        QMutexLocker locker(&m_mutex);
        m_inputFrame.release();
        m_outputFrame.release();
    }

    void readClassNames();
    void initializeONNXRuntime();

	// single image processing
	// TODO: also show path for single image processing
    std::vector<float> preprocessImage(const cv::Mat& image);
    void processOutput(Ort::Value& output, cv::Mat& originalImage);
    cv::Rect createBox(float cx, float cy, float w, float h, float scale_x, float scale_y);
    void runModel(cv::Mat& input);

	// batched image processing
    std::vector<cv::Mat> splitImageIntoQuadrants(const cv::Mat& image);
    std::vector<float> preprocessBatchedImages(const std::vector<cv::Mat>& images);
    std::vector<cv::Rect> processBatchedOutput(Ort::Value& output, cv::Mat& originalImage);
    cv::Rect createBoxForQuadrant(float cx, float cy, float w, float h, float scale_x, float scale_y, int quadrant, int quadWidth, int quadHeight);
    std::vector<cv::Rect> runBatchedModel(cv::Mat& input);

	// common processing
    static std::vector<cv::Rect> shortestPath(std::vector<cv::Rect>& centroids);
    // Draw boxes into a provided image instead of always drawing into member frame
    std::vector<cv::Rect> drawBoxes(cv::Mat& frameToDraw, std::vector<cv::Rect>& boxes, std::vector<int>& classIds,
        std::vector<float>& confidences, std::vector<int>& indices);
    
public slots:
    void predict();
    void predictWithManualBoxes();

signals:
    void frameProcessed(const cv::Mat& frame, const std::vector<cv::Rect>& boxCentroids);

private:

    std::vector<cv::Rect> m_manualBoxes; // Store manual boxes
    bool m_useManualBoxes = false;


    // ONNX Runtime components
    std::unique_ptr<Ort::Env> m_env;
    std::unique_ptr<Ort::Session> m_session;
    Ort::AllocatorWithDefaultOptions m_allocator;
    Ort::MemoryInfo m_memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    // Model info
    std::string m_inputName;
    std::string m_outputName;
    std::vector<int64_t> m_inputShape;
    int64_t m_inputHeight;
    int64_t m_inputWidth;

    // Qt and OpenCV components
    QMutex m_mutex;
    int m_frameWidth;
    int m_frameHeight;
    cv::Mat m_inputFrame;
    cv::Mat m_outputFrame;
    std::vector<std::string> m_classNames;
};


#endif // INFERENCEWORKER_H