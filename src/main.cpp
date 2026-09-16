#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "can_bus.hpp"
#include "can_codec.hpp"
#include "cooling_state_machine.hpp"

namespace {

struct CliOptions {
    double setpoint_c = 55.0;
    double kp = 5.0;
    double ki = 2.0;
    double kd = 0.5;
    double dt_seconds = 1.0;
};

void print_usage() {
    std::cout << "Usage: eae_firmware [--setpoint C] [--kp K] [--ki K] [--kd K] [--dt seconds]\n";
}

CliOptions parse_args(int argc, char** argv) {
    CliOptions opts;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        auto next_value = [&]() -> double {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << arg << "\n";
                std::exit(1);
            }
            return std::atof(argv[++i]);
        };

        if (arg == "--setpoint") {
            opts.setpoint_c = next_value();
        } else if (arg == "--kp") {
            opts.kp = next_value();
        } else if (arg == "--ki") {
            opts.ki = next_value();
        } else if (arg == "--kd") {
            opts.kd = next_value();
        } else if (arg == "--dt") {
            opts.dt_seconds = next_value();
        } else if (arg == "--help") {
            print_usage();
            std::exit(0);
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            print_usage();
            std::exit(1);
        }
    }

    return opts;
}

} // namespace

int main(int argc, char** argv) {
    CliOptions opts = parse_args(argc, argv);

    eae::CanBus sensor_bus;   // ECU node -> controller node: temp/ignition/level
    eae::CanBus actuator_bus; // controller node -> relay driver board: pump/fan
    eae::CoolingStateMachine controller(opts.setpoint_c, opts.kp, opts.ki, opts.kd);

    // Emulated field data - one row per CAN scan cycle. Matches the Section 7
    // scenario: warm-up, PID fan control band, a low-level fault, an
    // over-temperature trip, a sensor fault, and an ignition-off reset.
    const std::vector<eae::SensorFrame> scans = {
        {20.0f, false, true},
        {22.0f, true, true},
        {35.0f, true, true},
        {48.0f, true, true},
        {56.0f, true, true},
        {62.0f, true, true},
        {58.0f, true, true},
        {50.0f, true, true},
        {51.0f, true, false},
        {82.0f, true, true},
        {200.0f, true, true},
        {25.0f, false, true},
    };

    std::cout << "Setpoint=" << opts.setpoint_c << "C  Kp=" << opts.kp << " Ki=" << opts.ki
              << " Kd=" << opts.kd << " dt=" << opts.dt_seconds << "s\n\n";

    std::cout << std::left << std::setw(6) << "Scan" << std::setw(9) << "Temp(C)" << std::setw(6) << "Ign"
              << std::setw(9) << "LevelOK" << std::setw(9) << "State" << std::setw(6) << "Pump"
              << std::setw(6) << "Fan%" << std::setw(6) << "Alarm" << "\n";
    std::cout << std::string(57, '-') << "\n";

    for (std::size_t i = 0; i < scans.size(); ++i) {
        // "Send" the sensor reading from the ECU node onto the CAN bus.
        sensor_bus.send(eae::encode_sensor_frame(scans[i]));

        // The controller node "receives" the frame off the bus and decodes it.
        auto received_sensor_frame = sensor_bus.receive();
        eae::SensorFrame sensors = eae::decode_sensor_frame(*received_sensor_frame);

        eae::CoolingOutput output = controller.update(sensors, opts.dt_seconds);

        // The controller "sends" its actuator command back out over CAN...
        actuator_bus.send(eae::encode_actuator_frame(output.actuator));
        // ...and the relay/driver board "receives" and decodes it.
        auto received_actuator_frame = actuator_bus.receive();
        eae::ActuatorFrame actuator = eae::decode_actuator_frame(*received_actuator_frame);

        std::cout << std::left << std::setw(6) << i << std::setw(9) << sensors.coolant_temp_c
                  << std::setw(6) << (sensors.ignition_on ? "ON" : "off") << std::setw(9)
                  << (sensors.level_ok ? "yes" : "NO") << std::setw(9) << eae::to_string(output.state)
                  << std::setw(6) << (actuator.pump_run ? "RUN" : "off") << std::setw(6)
                  << static_cast<int>(actuator.fan_percent) << std::setw(6)
                  << (output.alarm ? "FAULT" : "ok") << "\n";
    }

    return 0;
}
