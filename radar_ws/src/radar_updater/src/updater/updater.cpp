#include "../updater.hpp"

namespace radar {

// constructor
Updater::Updater()
    : rclcpp::Node("updater")
    , radar_config_(read_config("./resources/config.json"))
{
    try {
        initialize_serial();
        initialize_processors();
        initialize_callbacks();
        initialize_debug_plotter();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "%s", e.what());
    }

    RCLCPP_INFO(get_logger(), "[√]initialized updater node.");
};

// destructor
Updater::~Updater()
{
    RCLCPP_INFO(get_logger(), "[-] Exited updater node.");
}

/* initializition functions */
info::Configs Updater::read_config(std::string filename)
{
    std::ifstream i(filename);
    nlohmann::json config_json;
    i >> config_json;

    info::Configs config;

    config.debug = config_json["SERIAL_DEBUG"].get<std::string>() == "true" ? true : false;
    config.friend_side = config_json["FRIEND_SIDE"].get<std::string>() == "RED" ? RED : BLUE;
    config.log_path = config_json["serial_log_path"].get<std::string>();
    config.serial_port = config_json["serial_port"].get<std::string>();
    config.delay_duration = config_json["delay_duration"].get<float>();
    return config;
}

void Updater::initialize_serial()
{
    while (true) {
        try {
            serial_ = std::make_shared<serial::Serial>(radar_config_.serial_port, 115200U, serial::Timeout::simpleTimeout(50U), serial::eightbits, serial::parity_none,
                serial::stopbits_one, serial::flowcontrol_none);
            break;
        } catch (const serial::IOException& se) {
            RCLCPP_ERROR(get_logger(), "%s", se.what());
        }
        sleep(1);
    }
}

void Updater::initialize_processors()
{
    std::map<int, info::enemy_robot_position> empty;
    empty.clear();

    std::map<int, int> enemy_catogory;
    if (radar_config_.friend_side == RED) {
        enemy_catogory = {
            { 0, 101 },
            { 1, 102 },
            { 2, 103 },
            { 3, 104 },
            { 4, 105 },
            { 5, 107 },
        };

    } else {
        enemy_catogory = {
            { 0, 1 },
            { 1, 2 },
            { 2, 3 },
            { 3, 4 },
            { 4, 5 },
            { 5, 7 },
        };
    }

    radar_information_
        = std::make_shared<info::Informations>(info::Informations {
            radar_config_.friend_side,
            info::GameState::NOT_START,
            -1,
            400,
            false,
            false,
            0,
            0,
            0,
            false,
            empty,
            enemy_catogory });
}
void Updater::initialize_callbacks()
{
    position_subscription_
        = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "car_positions", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                receive_topic_data(msg);
            });
    using namespace std::chrono_literals;
    updater_timer = this->create_wall_timer(
        200ms, [this]() { update_radar_status(); });
}
}
