#include "XYZStage.h"
#include "logger.h"
#include "HomingSequence.h"

// Global variable definition
GlobalVars globle_vars;

// Constructor
XYZStage::XYZStage(const std::string& portName)
    : port(portName)
    , m_stopWorker(false)
    , m_isWaitingForMoveCompletion(false)
    , m_serialHandle(INVALID_HANDLE_VALUE)
{
    log(QString("XYZStage initializing on port: %1").arg(QString::fromStdString(port)), "INFO");

    // Open serial connection
    m_serialHandle = getSerial();

    if (m_serialHandle != INVALID_HANDLE_VALUE) {
        log("Serial connection established", "INFO");

        // Query initial position
        getPosition();
    }
    else {
        log("Serial connection failed - running in simulation mode", "WARNING");
    }

    // Start the worker thread
    log("Starting worker thread...", "INFO");
    m_workerThread = std::thread(&XYZStage::worker, this);
    log("Worker thread started successfully", "INFO");
}

// Destructor

XYZStage::~XYZStage() {
    shutdown();
}

void XYZStage::shutdown() {
    log("Shutting down XYZStage...", "INFO");

    stopHoming();

    // Signal worker thread to stop
    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        m_stopWorker = true;
    }

    // Wake up worker thread
    m_condition.notify_one();

    // Finish the homing thread before closing its serial connection.
    if (m_homeThread.joinable()) {
        m_homeThread.join();
    }

    // Wait for worker thread to finish
    if (m_workerThread.joinable()) {
        log("Waiting for worker thread to finish...", "INFO");
        m_workerThread.join();
        log("Worker thread stopped", "INFO");
    }

    // Close serial connection
    if (m_serialHandle != INVALID_HANDLE_VALUE) {
        CloseHandle(m_serialHandle);
        m_serialHandle = INVALID_HANDLE_VALUE;
    }

    log("XYZStage shutdown complete", "INFO");
}

// Worker thread function
void XYZStage::worker() {
    log("Worker thread running", "INFO");

    while (true) {
        try {
            MoveCommand currentCommand;

            {
                std::unique_lock<std::mutex> lock(m_queueMutex);

                // Wait for command or stop signal
                m_condition.wait(lock, [this] {
                    return !m_commandQueue.empty() || m_stopWorker;
                    });

                // Exit if stopping and queue is empty
                if (m_stopWorker && m_commandQueue.empty()) {
                    log("Worker thread exiting", "INFO");
                    return;
                }

                // Get command from queue
                currentCommand = m_commandQueue.front();
                m_commandQueue.pop();
                m_commandInProgress = true;

                log(QString("Worker dequeued command: dx=%1, dy=%2, dz=%3")
                    .arg(currentCommand.dx)
                    .arg(currentCommand.dy)
                    .arg(currentCommand.dz), "INFO");
            }

            // Execute the move (outside the lock)
            char direction = (currentCommand.dx >= 0 && currentCommand.dy >= 0 && currentCommand.dz >= 0) ? 'P' : 'D';

            // Wrap low level move in try/catch so worker thread doesn't terminate on exceptions
            try {
                _move(std::abs(currentCommand.dx),
                    std::abs(currentCommand.dy),
                    std::abs(currentCommand.dz),
                    currentCommand.vx,
                    currentCommand.vy,
                    currentCommand.vz,
                    direction);
            } catch (const std::exception& e) {
                log(QString("Exception in _move(): %1").arg(e.what()), "CRITICAL");
            } catch (...) {
                log("Unknown exception in _move()", "CRITICAL");
            }

            {
                std::lock_guard<std::mutex> lock(m_queueMutex);
                m_commandInProgress = false;
            }

            // Notify if blocking call is waiting
            if (m_isWaitingForMoveCompletion.load()) {
                {
                    std::lock_guard<std::mutex> lock(m_syncMutex);
                    m_isWaitingForMoveCompletion = false;
                }
                m_syncCondition.notify_one();
            }
        }
        catch (const std::exception& e) {
            log(QString("Worker thread caught exception: %1").arg(e.what()), "CRITICAL");
            // small sleep to avoid busy-loop in case of persistent failure
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        catch (...) {
            log("Worker thread caught unknown exception", "CRITICAL");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
    }
}

// Public move method (non-blocking, queues command)
bool XYZStage::move(double dx, double dy, double dz, double velocity_x, double velocity_y, double velocity_z) {
    {
        std::lock_guard<std::mutex> lock(m_queueMutex);

        if (m_stopWorker || m_homing || m_positionUncertain) {
            log(
                "Move rejected: stage is closing, homing, or needs "
                "a successful home after an interrupted search",
                "WARNING"
            );
            return false;
        }

        m_commandQueue.push({
            dx, dy, dz,
            velocity_x, velocity_y, velocity_z
        });

        log(
            QString("Queued move: dx=%1, dy=%2, dz=%3")
                .arg(dx).arg(dy).arg(dz),
            "INFO"
        );
    }

    m_condition.notify_one();
    return true;
}

// Blocking move method (waits for completion)
void XYZStage::move_and_wait(double dx, double dy, double dz, double velocity_x, double velocity_y, double velocity_z) {
    std::unique_lock<std::mutex> lock(m_syncMutex);

    // Set waiting flag
    m_isWaitingForMoveCompletion = true;

    // Queue the command
    if (!move(dx, dy, dz, velocity_x, velocity_y, velocity_z)) {
        m_isWaitingForMoveCompletion.store(false);
        return;
    }

    // Wait for completion with timeout to avoid indefinite hangs
    log("move_and_wait: Waiting for completion...", "INFO");
    bool completed = m_syncCondition.wait_for(lock, std::chrono::seconds(20), [this] {
        return !m_isWaitingForMoveCompletion.load();
    });

    if (!completed) {
        log("move_and_wait: Timeout waiting for move completion", "WARNING");
        // Clear the waiting flag to avoid deadlocks
        m_isWaitingForMoveCompletion = false;
    } else {
        log("move_and_wait: Move complete", "INFO");
    }
}

// Home command
bool XYZStage::home() {
    {
        std::lock_guard<std::mutex> lock(m_queueMutex);

        if (
            m_serialHandle == INVALID_HANDLE_VALUE ||
            m_stopWorker ||
            m_homing ||
            m_commandInProgress ||
            !m_commandQueue.empty()
        ) {
            log(
                "Home rejected: check serial connection and wait "
                "for movement/traversal to finish",
                "WARNING"
            );
            return false;
        }

        m_homing.store(true);
        m_cancelHome.store(false);
    }

    if (m_homeThread.joinable()) {
        m_homeThread.join();
    }

    try {
        m_homeThread = std::thread([this] {
            try {
                _home();
            }
            catch (const std::exception& e) {
                m_positionUncertain.store(true);

                log(
                    QString("Homing exception: %1").arg(e.what()),
                    "CRITICAL"
                );
            }
            catch (...) {
                m_positionUncertain.store(true);
                log("Unexpected homing exception", "CRITICAL");
            }

            m_homing.store(false);
        });
    }
    catch (...) {
        m_homing.store(false);
        log("Could not start homing thread", "CRITICAL");
        return false;
    }

    return true;
}

// Core movement function
XYZStage::Position XYZStage::_move(double x, double y, double z, double vx, double vy, double vz, char direction) {

    std::lock_guard<std::recursive_mutex> serialLock(m_serialMutex);

    if (m_serialHandle == INVALID_HANDLE_VALUE) {
        log("SIMULATION MODE - No actual movement", "WARNING");

        // Update simulated position
        int sign = (direction == 'D') ? -1 : 1;
        // atomic<double> may not support fetch_add on all STL implementations (MSVC), use CAS loop
        {
            double oldVal = globle_vars.current_x.load();
            double newVal;
            do {
                newVal = oldVal + (x * sign);
            } while (!globle_vars.current_x.compare_exchange_weak(oldVal, newVal));
        }
        {
            double oldVal = globle_vars.current_y.load();
            double newVal;
            do {
                newVal = oldVal + (y * sign);
            } while (!globle_vars.current_y.compare_exchange_weak(oldVal, newVal));
        }
        {
            double oldVal = globle_vars.current_z.load();
            double newVal;
            do {
                newVal = oldVal + (z * sign);
            } while (!globle_vars.current_z.compare_exchange_weak(oldVal, newVal));
        }

        log(QString("Simulated position: X=%1, Y=%2, Z=%3")
            .arg(globle_vars.current_x.load())
            .arg(globle_vars.current_y.load())
            .arg(globle_vars.current_z.load()), "INFO");

        // Simulate movement time
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        return position;
    }

    int sign = (direction == 'D') ? -1 : 1;

    log(QString("Moving FROM: X=%1, Y=%2, Z=%3")
        .arg(globle_vars.current_x.load())
        .arg(globle_vars.current_y.load())
        .arg(globle_vars.current_z.load()), "INFO");

    // Read current position atomically into locals
    double currentX = globle_vars.current_x.load();
    double currentY = globle_vars.current_y.load();
    double currentZ = globle_vars.current_z.load();

    // Calculate target position
    int targetX = static_cast<int>(currentX + (x * sign));
    int targetY = static_cast<int>(currentY + (y * sign));
    int targetZ = static_cast<int>(currentZ + (z * sign));

    // Check bounds
    if (targetX < globle_vars.min_x || targetY < globle_vars.min_y || targetZ < globle_vars.min_z) {
        log("MOVE FAILED - Position below minimum bounds", "CRITICAL");
        return position;
    }

    if (targetX > globle_vars.max_x || targetY > globle_vars.max_y || targetZ > globle_vars.max_z) {
        log("MOVE FAILED - Position exceeds maximum bounds", "CRITICAL");
        return position;
    }

    // Convert to controller units
    int x_units = static_cast<int>(x * scale.x);
    int y_units = static_cast<int>(y * scale.y);
    int z_units = static_cast<int>(z * scale.z);
    int vx_units = static_cast<int>(vx * scale.x);
    int vy_units = static_cast<int>(vy * scale.y);
    int vz_units = static_cast<int>(vz * scale.z);

    // Build command string
    std::string cmd;

    if (x_units == 0 && y_units == 0 && z_units == 0) {
        log("No movement requested (all zeros)", "INFO");
        return position;
    }
    else if (x_units == 0 && y_units == 0) {
        char buffer[256];
        sprintf_s(buffer, "/1V,,%d%c,,%dR\r\n", vz_units, direction, z_units);
        cmd = buffer;
    }
    else if (x_units == 0 && z_units == 0) {
        char buffer[256];
        sprintf_s(buffer, "/1V,%d%c,%dR\r\n", vy_units, direction, y_units);
        cmd = buffer;
    }
    else if (y_units == 0 && z_units == 0) {
        char buffer[256];
        sprintf_s(buffer, "/1V%d%c%dR\r\n", vx_units, direction, x_units);
        cmd = buffer;
    }
    else if (x_units == 0) {
        char buffer[256];
        sprintf_s(buffer, "/1V,%d,%d%c,%d,%dR\r\n", vy_units, vz_units, direction, y_units, z_units);
        cmd = buffer;
    }
    else if (y_units == 0) {
        char buffer[256];
        sprintf_s(buffer, "/1V%d,,%d%c%d,,%dR\r\n", vx_units, vz_units, direction, x_units, z_units);
        cmd = buffer;
    }
    else if (z_units == 0) {
        char buffer[256];
        sprintf_s(buffer, "/1V%d,%d%c%d,%dR\r\n", vx_units, vy_units, direction, x_units, y_units);
        cmd = buffer;
    }
    else {
        char buffer[256];
        sprintf_s(buffer, "/1V%d,%d,%d%c%d,%d,%dR\r\n", vx_units, vy_units, vz_units, direction, x_units, y_units, z_units);
        cmd = buffer;
    }

    // Send command
    DWORD bytesWritten;
    if (!WriteFile(m_serialHandle, cmd.c_str(), static_cast<DWORD>(cmd.length()), &bytesWritten, NULL)) {
        log("Failed to write to serial port!", "CRITICAL");
        return position;
    }

    log(QString("Command sent: %1").arg(QString::fromStdString(cmd).trimmed()), "INFO");

    // Calculate wait time
    double sleep_time = 3.0;

    if (vx_units > 1) {
        double time = std::abs(static_cast<double>(x_units) / vx_units);
        if (time > sleep_time) sleep_time = time;
    }
    if (vy_units > 1) {
        double time = std::abs(static_cast<double>(y_units) / vy_units);
        if (time > sleep_time) sleep_time = time;
    }
    if (vz_units > 1) {
        double time = std::abs(static_cast<double>(z_units) / vz_units);
        if (time > sleep_time) sleep_time = time;
    }

    sleep_time += 0.5; // Buffer time

    log(QString("Waiting %1 seconds for movement...").arg(sleep_time, 0, 'f', 2), "INFO");

    // Wait for movement
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));

    // Query new position
    getPosition();

    return position;
}

// Home function
std::string XYZStage::exchangeHomeCommand(const std::string& command) {
    DWORD written = 0;

    log(
        QString("Home TX: %1")
            .arg(QString::fromStdString(command).trimmed()),
        "DEBUG"
    );

    if (
        !WriteFile(
            m_serialHandle,
            command.data(),
            static_cast<DWORD>(command.size()),
            &written,
            NULL
        ) ||
        written != command.size()
    ) {
        return {};
    }

    std::string reply;

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(1500);

    while (std::chrono::steady_clock::now() < deadline) {
        DWORD errors = 0;
        COMSTAT status{};

        if (
            !ClearCommError(m_serialHandle, &errors, &status) ||
            errors
        ) {
            return {};
        }

        if (status.cbInQue) {
            char buffer[256];
            DWORD received = 0;

            const DWORD requested = (std::min)(
                status.cbInQue,
                static_cast<DWORD>(sizeof(buffer))
            );

            if (
                !ReadFile(
                    m_serialHandle,
                    buffer,
                    requested,
                    &received,
                    NULL
                )
            ) {
                return {};
            }

            reply.append(buffer, received);

            if (reply.find('\x03') != std::string::npos) {
                log(
                    QString("Home RX: %1")
                        .arg(QString::fromStdString(reply)),
                    "DEBUG"
                );

                return reply;
            }
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(10)
        );
    }

    log(
        "Timed out reading homing controller response",
        "CRITICAL"
    );

    return {};
}

void XYZStage::_home() {
    std::lock_guard<std::recursive_mutex> serialLock(
        m_serialMutex
    );

    if (!PurgeComm(m_serialHandle, PURGE_RXCLEAR)) {
        log(
            "Cannot clear stale serial replies before homing",
            "CRITICAL"
        );
        return;
    }

    log(
        QString(
            "Homing XYZ using existing connection on %1; "
            "order Z, Y, X"
        ).arg(QString::fromStdString(port)),
        "INFO"
    );

    homing::IO io;

    io.exchange = [this](const std::string& cmd) {
        return exchangeHomeCommand(cmd);
    };

    io.cancelled = [this] {
        return m_cancelHome.load() || m_stopWorker.load();
    };

    io.report = [this](const std::string& text) {
        log(QString::fromStdString(text));
    };

    io.milliseconds = [] {
        return std::chrono::duration_cast<
            std::chrono::milliseconds
        >(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();
    };

    io.pause = [](int ms) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(ms)
        );
    };

    const auto result = homing::run(io);

    if (!result.success) {
        if (result.motionStarted) {
            m_positionUncertain.store(true);
        }

        log(
            QString("Home failed: %1")
                .arg(QString::fromStdString(result.message)),
            "CRITICAL"
        );

        return;
    }

    position.x =
        static_cast<double>(result.positions[0]) / scale.x;

    position.y =
        static_cast<double>(result.positions[1]) / scale.y;

    position.z =
        static_cast<double>(result.positions[2]) / scale.z;

    globle_vars.current_x.store(position.x);
    globle_vars.current_y.store(position.y);
    globle_vars.current_z.store(position.z);

    m_positionUncertain.store(false);

    log(
        "Home reference updated from controller readback. "
        "Home is a reference point, not the slide center.",
        "INFO"
    );
}

// Get position
XYZStage::Position XYZStage::getPosition() {
    std::lock_guard<std::recursive_mutex> serialLock(m_serialMutex);

    if (m_serialHandle == INVALID_HANDLE_VALUE) {
        return position;
    }

    char buffer[256];
    sprintf_s(buffer, "/1?aA\r\n");

    DWORD bytesWritten;
    if (!WriteFile(m_serialHandle, buffer, strlen(buffer), &bytesWritten, NULL)) {
        log("Failed to send position query", "WARNING");
        return position;
    }

    // Read response
    std::string response = readResponse(m_serialHandle, 2000);

    if (!response.empty()) {
        parsePositionResponse(response);
    }

    return position;
}

// Parse position response
void XYZStage::parsePositionResponse(const std::string& response) {
    size_t backtickPos = response.find('`');

    if (backtickPos == std::string::npos) {
        return;
    }

    std::string positionPart = response.substr(backtickPos + 1);
    std::vector<int> positions;
    std::stringstream ss(positionPart);
    std::string token;

    int count = 0;
    while (std::getline(ss, token, ',') && count < 3) {
        try {
            std::string cleanToken;
            for (char c : token) {
                if (std::isdigit(c) || c == '-') {
                    cleanToken += c;
                }
            }

            if (!cleanToken.empty()) {
                positions.push_back(std::stoi(cleanToken));
                count++;
            }
        }
        catch (...) {
            break;
        }
    }

    if (positions.size() >= 3) {
        globle_vars.current_x.store(static_cast<double>(positions[0]) / scale.x);
        globle_vars.current_y.store(static_cast<double>(positions[1]) / scale.y);
        globle_vars.current_z.store(static_cast<double>(positions[2]) / scale.z);

        position.x = globle_vars.current_x.load();
        position.y = globle_vars.current_y.load();
        position.z = globle_vars.current_z.load();

        log(QString("Position: X=%1, Y=%2, Z=%3")
            .arg(globle_vars.current_x.load())
            .arg(globle_vars.current_y.load())
            .arg(globle_vars.current_z.load()), "DEBUG");
    }
}

// Get serial handle
HANDLE XYZStage::getSerial() {
    HANDLE hSerial = CreateFileA(
        port.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );

    if (hSerial == INVALID_HANDLE_VALUE) {
        return INVALID_HANDLE_VALUE;
    }

    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams)) {
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    dcbSerialParams.BaudRate = CBR_9600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 2000;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 2000;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(hSerial, &timeouts)) {
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    return hSerial;
}

// Read serial response
std::string XYZStage::readResponse(HANDLE hSerial, int maxWaitMs) {
    std::string response;
    char buffer[256] = { 0 };
    DWORD bytesRead = 0;

    Sleep(100);

    int attempts = maxWaitMs / 100;

    for (int i = 0; i < attempts; i++) {
        if (ReadFile(hSerial, buffer, sizeof(buffer) - 1, &bytesRead, NULL)) {
            if (bytesRead > 0) {
                buffer[bytesRead] = '\0';
                response += std::string(buffer);

                if (response.find('\r') != std::string::npos ||
                    response.find('\n') != std::string::npos) {
                    break;
                }
            }
        }

        if (response.empty()) {
            Sleep(100);
        }
    }

    return response;
}