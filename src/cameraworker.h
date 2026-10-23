#ifndef CAMERAWORKER_H
#define CAMERAWORKER_H

#include <QObject>
#include <QImage>
#include <QMutex>
#include <opencv2/opencv.hpp>

#include "utils.h"

class CameraWorker : public QObject {
    Q_OBJECT

public:
    explicit CameraWorker(int camIndex = IMG, int camType = NONE, int frameWidth = 1080, int frameHeight= 720, int fps = 30, QObject* parent = nullptr);
    ~CameraWorker();
    void stop();
    void start();
    bool isOpened() const {
        return m_cap.isOpened();
    }   

    int getFrameWidth() { return m_frameWidth; };
    int getFrameHeight() { return m_frameHeight; };

    void setCaptureImg(bool val) { m_captureImg = val; }
    bool getCaptureImg() const { return m_captureImg; }

    void clearCapturedFrame() { m_capturedFrame.release(); }

	// TODO: even after capture macro img reads from the camera, we still need this to show the video feed captured frame to the user
    void setCapturedFrame(cv::Mat& frame) { m_capturedFrame = frame.clone(); }
    cv::Mat getCaturedFrame() { QMutexLocker lock(&m_mutex); return m_capturedFrame; }
    cv::Mat getCurrentFrame() const { QMutexLocker locker(&m_frameMutex); return m_currentFrame.clone(); }
 
public slots:
    void process();

signals:
    void frameReady(const QImage& image, int cameraType);

private:
    cv::VideoCapture m_cap;
    bool m_running;
    QMutex m_mutex;
    int m_cameraIndex;
    int m_cameraType;
    int m_frameWidth;
    int m_frameHeight;
    bool m_captureImg;
    cv::Mat m_capturedFrame;
    mutable QMutex m_frameMutex;
    cv::Mat m_currentFrame;
};


#endif // CAMERAWORKER_H