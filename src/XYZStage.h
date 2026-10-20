#pragma once
#include <windows.h>
#include <iostream>
#include <string>
#include <cstring>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <QTextEdit>
#include "utils.h"
#include <QString>
#include <functional>
#include <vector>
#include <sstream>
#include <cctype>
#include <QDebug>

// Global variables structure
struct GlobalVars {
    // Make position values atomic to avoid data races when read from multiple threads
    std::atomic<double> current_x{100.0};
    std::atomic<double> current_y{100.0};
    std::atomic<double> current_z{100.0};
    int max_x = 1000000;
    int max_y = 1500000;
    int max_z = 390000;
    int min_x = -2000;
    int min_y = -2000;
    int min_z = 0;
    std::atomic<bool> is_moving{false};
};

// Global instance declaration
extern GlobalVars globle_vars;

class XYZStage {
private:
    void parsePositionResponse(const std::string& response);

    struct Position {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct MoveCommand {
        double dx;
        double dy;
        double dz;
        double vx;
        double vy;
        double vz;
    };

    struct Scale {
        double x = 88.0 / 1000.0;
        double y = 88.0 / 1000.0;
        double z = 1260.0 / 1000.0;
    };

    HANDLE m_serialHandle;
    Position position;
    std::string port;
    Scale scale;

    // Worker thread management
    std::thread m_workerThread;
    std::queue<MoveCommand> m_commandQueue;
    std::mutex m_queueMutex;
    std::condition_variable m_condition;
    std::atomic<bool> m_stopWorker;

    bool m_commandInProgress = false; // Protected by m_queueMutex

    std::thread m_homeThread;

    std::atomic<bool> m_homing{false};
    std::atomic<bool> m_cancelHome{false};
    std::atomic<bool> m_positionUncertain{false};

    std::recursive_mutex m_serialMutex;
    std::mutex m_logMutex;

    // Sync for move_and_wait
    std::mutex m_syncMutex;
    std::condition_variable m_syncCondition;
    std::atomic<bool> m_isWaitingForMoveCompletion;

    // Logging callback
    std::function<void(const QString&, const QString&)> m_logCallback;

    // Helper to log
void log(const QString& message, const QString& level = "INFO") {
    try {
        std::function<void(const QString&, const QString&)> callback;

        {
            std::lock_guard<std::mutex> lock(m_logMutex);
            callback = m_logCallback;
        }

        if (callback) {
            callback(message, level);
        }
    }
    catch (const std::exception& e) {
        qDebug() << "Log callback threw an exception:" << e.what();
    }
    catch (...) {
        qDebug() << "Log callback threw an unknown exception.";
    }
}

    // Worker thread function
    void worker();

    // Private helper methods
    HANDLE getSerial();
    std::string readResponse(HANDLE hSerial, int maxWaitMs = 1000);

    // Core movement function
    Position _move(
        double x, double y, double z,
        double vx, double vy, double vz,
        char direction
    );

    void _home();

    std::string exchangeHomeCommand(const std::string& command);
public:
    XYZStage(const std::string& portName = "COM5");
    ~XYZStage();

    void shutdown();

    void setLogCallback(
        const std::function<void(const QString&, const QString&)>& callback
    ) {
        std::lock_guard<std::mutex> lock(m_logMutex);
        m_logCallback = callback;
    }

    // Public move methods
    bool move(
        double dx, double dy, double dz,
        double velocity_x = 10000,
        double velocity_y = 10000,
        double velocity_z = 10000
    );

    bool home();

    void stopHoming() {
        m_cancelHome.store(true);
    }

    bool isHoming() const {
        return m_homing.load();
    }

    bool positionUncertain() const {
        return m_positionUncertain.load();
    }

    // Blocking move method that waits for movement to complete
    void move_and_wait(double dx, double dy, double dz, double velocity_x = 10000, double velocity_y = 10000, double velocity_z = 10000);

    // Getter for current position
    Position getPosition();

    // Getter for port
    std::string getPort() const { return port; }

    // Setter for port
    void setPort(const std::string& newPort) { port = newPort; }

    // Get serial handle
    HANDLE getSerialHandle() { return m_serialHandle; }
};