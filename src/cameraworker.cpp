#include "cameraworker.h"
#include <QThread>
#include "logger.h"
#include <filesystem>
#include "utils.h"

CameraWorker::CameraWorker(int camIndex, int camType,
    int frameWidth, int frameHeight, int fps,
    QObject* parent)
    : QObject(parent),
      m_running(false), //give variables valid values even if opening camera fails 
      m_frameWidth(frameWidth),
      m_frameHeight(frameHeight),
      m_captureImg(false)
{
    m_cameraIndex = camIndex;
    m_cameraType = camType;
    Logger::info(QString("CameraWorker initialized with camera index: %1 and type: %2").arg(m_cameraIndex).arg(m_cameraType));
    if (camIndex != IMG) {
        m_cap.open(m_cameraIndex);
        // Apply user-specified settings
        m_cap.set(cv::CAP_PROP_FRAME_WIDTH, frameWidth);
        m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, frameHeight);
        m_cap.set(cv::CAP_PROP_FPS, fps);
        if (!m_cap.isOpened()) {
            Logger::critical(QString("Failed to open camera with index: %1").arg(m_cameraIndex));
            return;
        }
        m_frameWidth = m_cap.get(cv::CAP_PROP_FRAME_WIDTH);
        m_frameHeight = m_cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        Logger::info(QString("Camera opened with resolution: %1x%2 @ %3 FPS").arg(m_frameWidth).arg(m_frameHeight).arg(fps));
    }
    else {
        m_frameWidth = frameWidth;
        m_frameHeight = frameHeight;
    }
    m_running = true;
}

CameraWorker::~CameraWorker() {
    Logger::info("deleting CameraWorker object");
    stop();
    if (m_cap.isOpened()) {
        m_cap.release();
    }
    clearCapturedFrame();
}

void CameraWorker::stop() {
    QMutexLocker locker(&m_mutex);
    m_running = false;
}

void CameraWorker::start() {
    QMutexLocker locker(&m_mutex);
    //setCaptureImg(false); // reset the flag after restart
    m_running = true;
}

void CameraWorker::process() {
    while (true) {
        {
            QMutexLocker locker(&m_mutex);
            if (!m_running) break;
        }

        cv::Mat frame;

        // Always capture new frames from camera or image
        if (m_cameraIndex == IMG) {
            std::string imgPath = "test_img.png";
            if (!std::filesystem::exists(imgPath)) {
                Logger::critical(QString("Test Image file does not exist: %1")
                    .arg(QString::fromStdString(imgPath)));
                QThread::msleep(1000);
                continue;
            }
            frame = cv::imread(imgPath);
        }
        else {
            m_cap >> frame;
        }

        if (frame.empty()) {
            Logger::warning("Empty frame captured from camera");
            QThread::msleep(50);
            continue;
        }

        // ALWAYS update m_currentFrame (thread-safe)
        {
            QMutexLocker locker(&m_frameMutex);
            m_currentFrame = frame.clone();
        }

        // Handle capture flag
        {
            QMutexLocker locker(&m_mutex);
            if (m_captureImg) {
                Logger::info("Frame captured and stored");
                m_capturedFrame = frame.clone();
                m_captureImg = false; // Reset flag after capturing
            }
        }

        // Emit the frame for display
        QImage qImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_BGR888);
        emit frameReady(qImage.copy(), m_cameraType);

        QThread::msleep(50);
    }

    Logger::info("CameraWorker process loop exited");
}