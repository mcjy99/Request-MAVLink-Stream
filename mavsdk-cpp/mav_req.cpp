#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h> //direct mavlink access
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <chrono>
#include <iostream>
#include <cmath>
#include <thread>
#include <cstdint>
#include <future>
#include <unistd.h>

using namespace mavsdk;
using namespace std::chrono;

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udpin://0.0.0.0:14540\n";
}

int main(int argc, char** argv, float msg_interval){
    //create instance of Mavsdk with configuration
    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::CompanionComputer}}; 
    
    ConnectionResult conn_result = mavsdk.add_any_connection(argv[1]); //define port when running in terminal as the first argument
    if (conn_result != ConnectionResult::Success){
        std::cerr << "Connection failed " << conn_result << std::endl;
        return 1;
    }
    std::cout << "Waiting for system to connect..." << std::endl;

    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }
    auto telemetry = Telemetry(system.value());
    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for system to be ready\n";
        sleep(1.0);
    }
    std::cout << "System ready \n";

    auto mavlink_passthrough = MavlinkPassthrough{system.value()}; //instantiate passthrough plugin
    MavlinkPassthrough::CommandLong command_req{};
    command_req.target_sysid = mavlink_passthrough.get_target_sysid();
    command_req.target_compid = mavlink_passthrough.get_target_compid();
    command_req.command = MAV_CMD_SET_MESSAGE_INTERVAL; 
    command_req.param1 = 147; //message ID, 147 is BATTERY_STATUS
    command_req.param2 = 5000000.0f; //interval in microseconds, define as -1 to disable streaming
    command_req.param3 = 0;
    command_req.param4 = 0;
    command_req.param5 = 0;
    command_req.param6 = 0;
    command_req.param7 = 0;
   auto result = mavlink_passthrough.send_command_long(command_req);
if (result == MavlinkPassthrough::Result::Success) {
    // Set up message handler to wait for ACK
    std::promise<bool> ack_received;
    auto future = ack_received.get_future();
    
    mavlink_passthrough.subscribe_message(
        MAVLINK_MSG_ID_COMMAND_ACK,
        [&ack_received, &command_req](const mavlink_message_t& message) {
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(&message, &ack);
            
            if (ack.command == MAV_CMD_SET_MESSAGE_INTERVAL) {
                bool success = (ack.result == MAV_RESULT_ACCEPTED);
                ack_received.set_value(success);
            }
        });

//Wait for acknowledgment with timeout
    if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
        bool success = future.get();
        if (success) {
            std::cout << "Command acknowledged and accepted!" << std::endl;
        } else {
            std::cout << "Command acknowledged but rejected!" << std::endl;
        }
    } else {
        std::cout << "Timeout waiting for command acknowledgment" << std::endl;
    }
} else {
    std::cerr << "Failed to send command: " << static_cast<int>(result) << std::endl;
}

    return 0;
}



