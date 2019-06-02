/**
 * @file offboard_velocity.cpp
 * @brief Example that demonstrates offboard position control in local NED and body coordinates. Based on offboard_velocity.cpp
 *
 * @authors Author: Julian Oes <julian@oes.ch>,
 *                  Shakthi Prashanth <shakthi.prashanth.m@intel.com>
                    Charles Blouin <charles.blouin@rcbenchmark.com>
 * @date 2019-06-02
 */

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include <dronecode_sdk/dronecode_sdk.h>
#include <dronecode_sdk/plugins/action/action.h>
#include <dronecode_sdk/plugins/offboard/offboard.h>
#include <dronecode_sdk/plugins/telemetry/telemetry.h>

using namespace dronecode_sdk;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::chrono::seconds;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

// Handles Action's result
inline void action_error_exit(Action::Result result, const std::string &message)
{
    if (result != Action::Result::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << message << Action::result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles Offboard's result
inline void offboard_error_exit(Offboard::Result result, const std::string &message)
{
    if (result != Offboard::Result::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << message << Offboard::result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles connection result
inline void connection_error_exit(ConnectionResult result, const std::string &message)
{
    if (result != ConnectionResult::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << message << connection_result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Logs during Offboard control
inline void offboard_log(const std::string &offb_mode, const std::string msg)
{
    std::cout << "[" << offb_mode << "] " << msg << std::endl;
}

/**
 * Does Offboard control using NED co-ordinates.
 *
 * returns true if everything went well in Offboard control, exits with a log otherwise.
 */
bool offb_ctrl_ned(std::shared_ptr<dronecode_sdk::Offboard> offboard)
{
    const std::string offb_mode = "NED";
    // Send it once before starting offboard, otherwise it will be rejected.
    offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 0.0f});

    Offboard::Result offboard_result = offboard->start();
    offboard_error_exit(offboard_result, "Offboard start failed");
    offboard_log(offb_mode, "Offboard started");
    
    float height = 0.75f;

    offboard_log(offb_mode, "Going to 0, 0, -0.0");
    offboard->set_position_ned({0.0f, 0.0f, -0.0f, 0.0f});
    sleep_for(seconds(1)); // Let yaw settle.

    offboard_log(offb_mode, "Going to 0, 0, -0.75");
    offboard->set_position_ned({0.0f, 0.0f, -height, 0.0f});
    sleep_for(seconds(4)); // Let yaw settle.
    
    offboard_log(offb_mode, "Going to 0.2, 0, -0.75");
    offboard->set_position_ned({0.2f, 0.0f, -height, 0.0f});
    sleep_for(seconds(2)); // Let yaw settle.
    
    offboard_log(offb_mode, "Going to 0, 0, -0.75");
    offboard->set_position_ned({0.0f, 0.0f, -height, 0.0f});
    sleep_for(seconds(2)); // Let yaw settle.

    // Interpolate for smooth landing
    const unsigned steps = 5;
    
    for (unsigned i = 0; i < steps; ++i) {
        offboard->set_position_ned({0.0f, 0.0f, -height + height/(float)steps*(float)i + 0.15f, 0.0f});
        offboard_log(offb_mode, std::to_string(-height + height/(float)steps*(float)i) );
        sleep_for(milliseconds(400));
    }
    
    offboard_log(offb_mode, "Going to 0, 0, 0");
    offboard->set_position_ned({0.0f, 0.0f, 0.0f, 0.0f});
    // sleep_for(seconds(1)); // Let yaw settle.


    return true;
}



void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

int main(int argc, char **argv)
{
    DronecodeSDK dc;
    std::string connection_url;
    ConnectionResult connection_result;

    if (argc == 2) {
        connection_url = argv[1];
        connection_result = dc.add_any_connection(connection_url);
    } else {
        usage(argv[0]);
        return 1;
    }

    if (connection_result != ConnectionResult::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT
                  << "Connection failed: " << connection_result_str(connection_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }
    
    // Wait for the system to connect via heartbeat
    while (!dc.is_connected()) {
        std::cout << "Wait for system to connect via heartbeat" << std::endl;
        sleep_for(seconds(1));
    }

    // System got discovered.
    System &system = dc.system();
    auto action = std::make_shared<Action>(system);
    auto offboard = std::make_shared<Offboard>(system);
    auto telemetry = std::make_shared<Telemetry>(system);
    sleep_for(seconds(1));
    dronecode_sdk::Telemetry::Health health = telemetry->health();
    
    
    if (health.gyrometer_calibration_ok) {
        std::cout << "Gyro is calibrated" << std::endl;
        sleep_for(seconds(1));
    }
    
    /*
    while (!telemetry->health_all_ok()) {
        dronecode_sdk::Telemetry::Health health = telemetry->health();
        std::cout << "Gyro: " << health.gyrometer_calibration_ok << std::endl;
        std::cout << "Local Position: " << health.local_position_ok << std::endl;
        std::cout << "Home Position: " << health.home_position_ok << std::endl;
        std::cout << "Waiting for system to be ready" << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << "System is ready" << std::endl;
*/
    Action::Result arm_result = action->arm();
    action_error_exit(arm_result, "Arming failed");
    std::cout << "Armed" << std::endl;



    //  using local NED co-ordinates
    bool ret = offb_ctrl_ned(offboard);
    if (ret == false) {
        return EXIT_FAILURE;
    }

    const Action::Result land_result = action->land();
    action_error_exit(land_result, "Landing failed");

    // Check if vehicle is still in air
    while (telemetry->in_air()) {
        std::cout << "Vehicle is landing..." << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << "Landed!" << std::endl;
    action->disarm();
    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(3));
    std::cout << "Finished..." << std::endl;

    return EXIT_SUCCESS;
}
