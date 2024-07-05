#pragma once

#include "include/Logger.hpp"
#include "include/nlohmann/json.hpp"

#include "processors/debug_plotter.hpp"
#include "processors/receiver.hpp"
#include "processors/submitter.hpp"

#include <memory>

#define RED 100
#define BLUE 101

namespace radar {

class RadarUpdater {
public:
    std::shared_ptr<RadarInformation> radar_information_;
    std::shared_ptr<serial::Serial> serial_;

    RefereeSerialReceiver referee_serial_receiver_;
    RefereeSerialSubmitter referee_serial_submitter_;
    EnemyPointsReceiver radar_points_receiver_;
    DebugPlotter plotter_;

    std::shared_ptr<Logger> logger_;
    Configs radar_config_;

    RadarUpdater()
        : radar_config_(read_config("./resources/config.json"))

    {
        logger_ = std::make_shared<Logger>(Logger::log_target::file, Logger::log_level::debug, radar_config_.log_path);
        initialize_serial();
        initialize_processors();

        logger_->INFO("[√]initialized updater node.");
    };

    void update_radar_status()
    {
        referee_serial_receiver_.receive_referee_data();
        process_data();
        referee_serial_submitter_.send_serial_data();
    }

    void process_data()
    {
        plotter_.plot();
    }

private:
    Configs read_config(std::string filename)
    {
        std::ifstream i(filename);
        nlohmann::json config_json;
        i >> config_json;

        Configs config;

        config.debug = config_json["SERIAL_DEBUG"].get<std::string>() == "true" ? true : false;
        config.friend_side = config_json["FRIEND_SIDE"].get<std::string>() == "RED" ? RED : BLUE;
        config.log_path = config_json["serial_log_path"].get<std::string>();
        config.serial_port = config_json["serial_port"].get<std::string>();
        config.delay_duration = config_json["delay_duration"].get<float>();
        return config;
    }

    void initialize_serial()
    {
        while (true) {
            try {
                serial_ = std::make_shared<serial::Serial>(radar_config_.serial_port, 115200U, serial::Timeout::simpleTimeout(50U), serial::eightbits, serial::parity_none,
                    serial::stopbits_one, serial::flowcontrol_none);
                break;
            } catch (const serial::IOException& se) {
                logger_->ERRORS(se.what());
            }
            sleep(1);
        }
    }

    void initialize_processors()
    {
        std::map<int, enemy_robot_position> empty;
        empty.clear();
        radar_information_ = std::make_shared<RadarInformation>(
            RadarInformation {
                radar_config_.friend_side,
                GameState::NOT_START,
                -1,
                400,
                false,
                false,
                0,
                0,
                0,
                false,
                empty });
        referee_serial_receiver_
            = RefereeSerialReceiver(
                radar_information_,
                serial_,
                logger_);
        radar_points_receiver_
            = EnemyPointsReceiver(
                radar_information_,
                logger_,
                radar_config_);
        referee_serial_submitter_
            = RefereeSerialSubmitter(
                radar_information_,
                logger_,
                serial_);
        plotter_
            = DebugPlotter(
                radar_information_,
                radar_config_);
    }
};

}