#pragma once

#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace homing {

// Parsed controller response.
struct Packet {
    bool valid = false;
    bool ready = false;
    unsigned int error = 0;
    std::string payload;
};

inline Packet parsePacket(const std::string& wire) {
    Packet p;

    const auto start = wire.find("/0");

    if (start == std::string::npos || start + 3 > wire.size()) {
        return p;
    }

    const auto end = wire.find('\x03', start + 3);

    // Reject incomplete replies.
    if (end == std::string::npos) {
        return p;
    }

    const auto status =
        static_cast<unsigned char>(wire[start + 2]);

    if ((status & 0xC0) != 0x40) {
        return p;
    }

    p.valid = true;
    p.ready = (status & 0x20) != 0;
    p.error = status & 0x0F;
    p.payload = wire.substr(start + 3, end - start - 3);

    return p;
}

// Read XYZ positions, accepting an optional fourth controller axis.
inline bool parsePositions(
    const Packet& p,
    std::array<long long, 3>& positions
) {
    if (!p.valid || !p.ready || p.error) {
        return false;
    }

    std::istringstream in(p.payload);
    std::string token;
    int count = 0;

    while (std::getline(in, token, ',')) {
        if (token.empty() || count >= 4) {
            return false;
        }

        try {
            std::size_t consumed = 0;

            const auto value = std::stoll(token, &consumed);

            if (
                consumed != token.size() ||
                value < (std::numeric_limits<int>::min)() ||
                value > (std::numeric_limits<int>::max)()
            ) {
                return false;
            }

            if (count < 3) {
                positions[count] = value;
            }
        }
        catch (...) {
            return false;
        }

        ++count;
    }

    return
        (count == 3 || count == 4) &&
        !p.payload.empty() &&
        p.payload.back() != ',';
}

struct Axis {
    int number;
    int velocity;
    int maxSteps;
    int polarity;
    const char* name;
};

// Settings and order copied from the original Python homing script.
// Polarity configures the sensor signal, not motor direction.
inline std::array<Axis, 3> axes() {
    return {{
        {3, 3000, 100000, 1, "Z"},
        {2, 1000, 10000, 1, "Y"},
        {1, 1000, 10000, 1, "X"}
    }};
}

inline std::string command(const Axis& axis) {
    return
        "/1aM" + std::to_string(axis.number) +
        "V" + std::to_string(axis.velocity) +
        "L10m50h30f" + std::to_string(axis.polarity) +
        "N1Z" + std::to_string(axis.maxSteps) +
        "R\r";
}

// Functions supplied by XYZStage::_home().
struct IO {
    std::function<std::string(const std::string&)> exchange;
    std::function<bool()> cancelled;
    std::function<void(const std::string&)> report;
    std::function<long long()> milliseconds;
    std::function<void(int)> pause;
};

struct Result {
    bool success = false;
    bool motionStarted = false;

    std::array<long long, 3> positions{{0, 0, 0}};

    std::string message;
};

inline Result run(const IO& io) {
    Result result;

    const auto fail = [&](const std::string& message) {
        result.message = message;

        // Request a controller stop if motion may have started.
        // This is not a hardware emergency stop.
        if (result.motionStarted) {
            const auto stopped =
                parsePacket(io.exchange("/1T\r"));

            if (!stopped.valid || stopped.error) {
                result.message +=
                    "; STOP acknowledgement missing or in error";
            }
        }

        io.report(result.message);
        return result;
    };

    const auto checked = [&](const std::string& cmd) {
        const auto p = parsePacket(io.exchange(cmd));

        if (!p.valid) {
            throw std::runtime_error(
                "Missing or malformed controller reply"
            );
        }

        if (p.error) {
            throw std::runtime_error(
                "Controller error code " +
                std::to_string(p.error)
            );
        }

        return p;
    };

    try {
        if (io.cancelled()) {
            return fail("Homing cancelled before motion");
        }

        if (!checked("/1Q\r").ready) {
            return fail(
                "Controller is busy; homing was not started"
            );
        }

        for (const auto& axis : axes()) {
            if (io.cancelled()) {
                return fail("Homing cancelled");
            }

            io.report(
                std::string("Homing ") +
                axis.name +
                " (axis " +
                std::to_string(axis.number) +
                ")"
            );

            // Even a lost acknowledgement may mean motion started.
            result.motionStarted = true;

            checked(command(axis));

            // Allow the search duration plus an additional
            // acceleration and back-out allowance.
            const auto deadline =
                io.milliseconds() +
                static_cast<long long>(
                    std::ceil(
                        1000.0 * axis.maxSteps / axis.velocity
                    )
                ) +
                30000;

            while (true) {
                if (io.cancelled()) {
                    return fail(
                        std::string("Homing cancelled on ") +
                        axis.name
                    );
                }

                if (io.milliseconds() >= deadline) {
                    return fail(
                        std::string("Homing timed out on ") +
                        axis.name
                    );
                }

                if (checked("/1Q\r").ready) {
                    break;
                }

                io.pause(100);
            }

            const auto p = checked("/1?aA\r");

            if (!parsePositions(p, result.positions)) {
                return fail(
                    "Invalid position readback after homing"
                );
            }

            if (
                std::llabs(
                    result.positions[axis.number - 1]
                ) > 1
            ) {
                return fail(
                    std::string(
                        "Controller did not report home zero on "
                    ) +
                    axis.name
                );
            }

            io.report(
                std::string(axis.name) +
                " home completed according to controller"
            );
        }

        // Verify the final XYZ readback.
        // Do not issue a command that artificially forces zero.
        for (auto value : result.positions) {
            if (std::llabs(value) > 1) {
                return fail(
                    "Final XYZ home position readback is not zero"
                );
            }
        }

        result.success = true;
        result.message =
            "XYZ home completed according to controller; "
            "position readback verified";

        io.report(result.message);

        return result;
    }
    catch (const std::exception& e) {
        return fail(e.what());
    }
}

} // namespace homing