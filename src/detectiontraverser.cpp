#include "detectiontraverser.h"
#include "utils.h"
#include <vector>
#include "mainwindow.h"
#include <QString>
#include <QObject>
#include <QTextEdit>
#include "logger.h"

DetectionTraverser::DetectionTraverser(XYZStage* xyzStage, QObject* parent)
    : QObject(parent), m_xyzStage(xyzStage), m_paused(false), m_aborted(false)
{
    Logger::info("appendLog working in detectiontraverser.");
}

void DetectionTraverser::setTraversalData(const std::vector<cv::Rect>& path, const cv::Mat& transformMatrix)
{
    m_macroImgPath = path;
    m_transformMatrix = transformMatrix;
}

void DetectionTraverser::abortTraversal()
{
    QMutexLocker locker(&m_mutex);
    m_aborted = true;
    if (m_paused) {
        m_pauseCondition.wakeAll(); // Wake it up to allow it to exit
    }
}

void DetectionTraverser::userConfirmedAdjustment()
{
    QMutexLocker locker(&m_mutex);
    if (m_paused) {
        Logger::info("Resume state.");
        m_paused = false;
        m_pauseCondition.wakeAll(); // Wake up the processing loop
    }
}

void DetectionTraverser::process()
{
    try {
        emit traversalStarted();
        m_aborted = false;

        // Convert all image coordinates to real coordinates at once
        std::vector<cv::Point2f> realCoordinates;
        for (const auto& box : m_macroImgPath) {
            cv::Point2f imageCenter(box.x + box.width / 2.0f, box.y + box.height / 2.0f);
            std::vector<cv::Point2f> imagePoints = { imageCenter };
            std::vector<cv::Point2f> transformedPoints;
            cv::transform(imagePoints, transformedPoints, m_transformMatrix);
            realCoordinates.push_back(transformedPoints[0]);
        }

        Logger::info(QString("Starting traversal of: %1 detected points.").arg(realCoordinates.size()));

        for (size_t i = 0; i < realCoordinates.size(); ++i) {
            {
                QMutexLocker locker(&m_mutex);
                if (m_aborted) {
                    Logger::info("Trivarsal aborted");
                    emit traversalFinished("Traversal aborted.");
                    return;
                }
            }

            emit updateProgress(i + 1, realCoordinates.size());
            const cv::Point2f& targetPoint = realCoordinates[i];
            if (!std::isfinite(targetPoint.x) ||
                !std::isfinite(targetPoint.y) ||
                targetPoint.x < globle_vars.min_x ||
                targetPoint.x > globle_vars.max_x ||
                targetPoint.y < globle_vars.min_y ||
                targetPoint.y > globle_vars.max_y) {
                Logger::warning(QString("Point %1 is out of bound. Skipping!").arg(i + 1));
                continue;  // Skip to next point
            }

            // Move to absolute XY targets. Leave Z unchanged for manual focus.
            const double currentX = globle_vars.current_x.load();

            m_xyzStage->move_and_wait(
                targetPoint.x - currentX, 0, 0
            );

            // Read Y immediately before issuing its movement.
            const double currentY = globle_vars.current_y.load();

            m_xyzStage->move_and_wait(
                0, targetPoint.y - currentY, 0
            );

            Logger::info(QString("Arrived at point %1 ...Waiting for user adjustment.").arg(i + 1));
            emit waitingForUserAdjustment();
            // Pause execution and wait for the user to click "Confirm" or for abort
            {
                QMutexLocker locker(&m_mutex);
                m_paused = true;
                while (m_paused && !m_aborted) {
                    // Wait in 1s increments so abort can be handled promptly
                    m_pauseCondition.wait(&m_mutex, 1000);
                }
                if (m_aborted) {
                    Logger::info("Traversal aborted during pause");
                    emit traversalFinished("Traversal aborted.");
                    return;
                }
                // After waking up and m_paused==false, continue to next point
            }
        }

        emit traversalFinished("Traversal completed successfully.");
    }
    catch (const std::exception& e) {
        Logger::critical(QString("Traversal exception: %1").arg(e.what()));
        emit traversalFinished(QString("Traversal failed: %1").arg(e.what()));
    }
    catch (...) {
        Logger::critical("Traversal unknown exception");
        emit traversalFinished("Traversal failed with unknown error.");
    }
}