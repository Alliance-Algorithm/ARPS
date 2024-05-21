#pragma once

#include "Logger.hpp"
#include "receive.hpp"

#include <bitset>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <unistd.h>

#include <opencv2/core.hpp>

#include <serial/serial.h>
#include <serial_util/crc/dji_crc.hpp>
#include <serial_util/package_receive.hpp>

#define RED 100
#define BLUE 101

namespace radar {

// 比赛状态
enum class GameState {
    NOT_START,
    PREPARATION,
    REFREE_CHECK,
    COUNTDOWN,
    STARTED,
    SETTLING
};

/* - 比赛信息 - ;
   己方颜色  friend_Side;
   比赛状态  gamestate;
   剩余时间  time_remain;
   敌方哨兵血量  enemy_sentry_hp;
   是否激活大能量机关  if_active_big_buff;
   是否已开启双倍易伤  if_double_debuff_enabled;
   可用双倍易伤次数 double_debuff_chances;
   飞镖目标 dart_target
   开启双倍易伤  enable_double_debuff;
   雷达决策命令  double_debuff_cmd;
 */
struct GameInformation {
    int friend_Side;
    GameState gamestate;
    int time_remain;
    float enemy_sentry_hp;

    bool if_active_big_buff;
    bool if_double_debuff_enabled;

    int double_debuff_chances;
    bool enable_double_debuff;
    int double_debuff_cmd;

    int dart_target;
    bool dart_change;
};
struct Enable_double_debuff_by {
    bool enemy_sentry;
    bool big_buff;
};
struct RadarMarkProgress {
    int mark_hero_progress;
    int mark_engineer_progress;
    int mark_standard_3_progress;
    int mark_standard_4_progress;
    int mark_standard_5_progress;
    int mark_sentry_progress;
};
struct IfRadarMark {
    bool if_mark_hero;
    bool if_mark_engineer;
    bool if_mark_standard_3;
    bool if_mark_standard_4;
    bool if_mark_standard_5;
    bool if_mark_sentry;
};

class DecisionMaker {
public:
    DecisionMaker() {};
    DecisionMaker(std::shared_ptr<serial::Serial> serial, Logger* logger, int friend_side)
        : radar_serial_(serial)
        , logger_(logger)
    {
        radar_information_ = GameInformation {
            friend_side, GameState::NOT_START, -1, 400, false, false, 0, 0, 0, 0, false
        };
        radar_mark_progress_ = RadarMarkProgress {
            0, 0, 0, 0, 0, 0
        };
        if_radar_mark_ = IfRadarMark {
            false, false, false, false, false, false
        };
        enable_double_debuff_by_ = Enable_double_debuff_by { 0, 0 };
        logger->INFO("[√]initialized game status receive node.");
    };

    void receive_status_data()
    {
        if (!radar_serial_->isOpen())
            return;

        if (cache_size_ >= sizeof(frame_.header)) {
            auto frame_size = sizeof(frame_.header) + sizeof(frame_.body.command_id) + frame_.header.data_length + sizeof(uint16_t);
            cache_size_ += radar_serial_->read(reinterpret_cast<uint8_t*>(&frame_) + cache_size_,
                frame_size - cache_size_);

            if (cache_size_ == frame_size) {
                cache_size_ = 0;
                if (serial_util::dji_crc::verify_crc16(&frame_, frame_size)) {
                    process_frame_info();
                } else {
                    logger_->WARNING("Body crc16 invalid");
                }
            }
        } else {
            auto result = serial_util::receive_package(
                *radar_serial_, frame_.header, cache_size_,
                static_cast<uint8_t>(0xa5),
                [](const package::receive::FrameHeader& header) {
                    return serial_util::dji_crc::verify_crc8(header);
                });
            if (result == serial_util::ReceiveResult::HEADER_INVAILD) {
                logger_->WARNING("Header start invalid");
            } else if (result == serial_util::ReceiveResult::VERIFY_INVAILD) {
                logger_->WARNING("Header crc8 invalid");
            }
        }
        make_decision();
    }

    void process_frame_info()
    {
        auto command_id = frame_.body.command_id;
        logger_->INFO("command_id: " + std::to_string(command_id));
        RCLCPP_INFO(rclcpp::get_logger("Referee serial info"), "%s", frame_.body.data);

        if (command_id == 0x0001)
            update_game_status();
        if (command_id == 0x0003)
            update_sentry_hp();
        if (command_id == 0x020C) {
            if_radar_mark_ = IfRadarMark { false, false, false, false, false, false };
            update_mark_progress();
        }
        if (command_id == 0x0101)
            update_buff_info();
        if (command_id == 0x020E)
            update_radar_info();
        if (command_id == 0x0105)
            update_dart_info();
    }

    void update_game_status()
    {
        auto& data = reinterpret_cast<package::receive::GameStatus&>(frame_.body.data);
        radar_information_.gamestate = static_cast<GameState>(data.game_stage);
        radar_information_.time_remain = static_cast<int>(data.stage_remain_time);
        logger_->INFO("receive gamestate:" + std::to_string(static_cast<int>(radar_information_.gamestate))
            + "|time_remain:" + std::to_string(radar_information_.time_remain));
    };
    void update_sentry_hp()
    {
        auto& data = reinterpret_cast<package::receive::GameRobotHp&>(frame_.body.data);
        radar_information_.enemy_sentry_hp = radar_information_.friend_Side == RED ? data.blue_7 : data.red_7;
        logger_->INFO("receive enemy sentry hp:" + std::to_string(radar_information_.enemy_sentry_hp));
    };
    void update_buff_info()
    {
        auto& data = reinterpret_cast<package::receive::EventData&>(frame_.body.data);
        std::bitset<32> event_info_val(data.event_data);
        radar_information_.if_active_big_buff = static_cast<bool>(event_info_val[5]);
        logger_->INFO("receive big buff info:" + std::to_string(radar_information_.if_active_big_buff));
    };
    void update_radar_info()
    {
        auto& data = reinterpret_cast<package::receive::RadarInfo&>(frame_.body.data);

        auto bit_0 = (data.radar_info >> 7) % 2;
        auto bit_1 = (data.radar_info >> 6) % 2;
        auto bit_2 = (data.radar_info >> 5) % 2;

        int current_double_debuff_chances = bit_0 * 2 + bit_1;

        if (current_double_debuff_chances > radar_information_.double_debuff_chances) {
            logger_->INFO("Double debuff chances add:" + std::to_string(radar_information_.double_debuff_chances));
        }
        radar_information_.double_debuff_chances = current_double_debuff_chances;

        radar_information_.if_double_debuff_enabled = static_cast<bool>(bit_2);
        logger_->INFO("receive if enable doublebuff info:" + std::to_string(radar_information_.if_double_debuff_enabled));
    };
    void update_mark_progress()
    {
        auto& data = reinterpret_cast<package::receive::RadarMarkProgress&>(frame_.body.data);
        if_radar_mark_ = IfRadarMark {
            data.mark_hero_progress > radar_mark_progress_.mark_hero_progress,
            data.mark_engineer_progress > radar_mark_progress_.mark_engineer_progress,
            data.mark_standard_3_progress > radar_mark_progress_.mark_standard_3_progress,
            data.mark_standard_4_progress > radar_mark_progress_.mark_standard_4_progress,
            data.mark_standard_5_progress > radar_mark_progress_.mark_standard_5_progress,
            data.mark_sentry_progress > radar_mark_progress_.mark_sentry_progress
        };
        radar_mark_progress_ = RadarMarkProgress {
            data.mark_hero_progress,
            data.mark_engineer_progress,
            data.mark_standard_3_progress,
            data.mark_standard_4_progress,
            data.mark_standard_5_progress,
            data.mark_sentry_progress
        };
        // logger_->INFO("receive mark info");
    }
    void update_dart_info()
    {
        auto& data = reinterpret_cast<package::receive::DartInfo&>(frame_.body.data);

        auto bit_5 = (data.dart_info >> 11) % 2;
        auto bit_6 = (data.dart_info >> 10) % 2;

        auto last_dart_target = radar_information_.dart_target;
        radar_information_.dart_target = 2 * bit_5 + bit_6;

        if (last_dart_target != radar_information_.dart_target)
            radar_information_.dart_change = true;
    }

    package::receive::Frame frame_;
    std::shared_ptr<serial::Serial> radar_serial_;
    size_t cache_size_ = 0;

    GameInformation radar_information_;
    Enable_double_debuff_by enable_double_debuff_by_;
    RadarMarkProgress radar_mark_progress_;
    IfRadarMark if_radar_mark_;
    Logger* logger_;

public:
    void make_decision()
    {
        // reset status if game not start
        if (radar_information_.gamestate == GameState::SETTLING || radar_information_.gamestate == GameState::COUNTDOWN) {
            radar_information_.enemy_sentry_hp = 400;
            radar_information_.if_active_big_buff = false;
            radar_information_.if_double_debuff_enabled = false;
            radar_information_.double_debuff_cmd = 0;
            radar_information_.double_debuff_chances = 0;
            radar_information_.dart_target = 0;
            radar_information_.enable_double_debuff = false;
            radar_information_.dart_change = false;
            enable_double_debuff_by_.big_buff = false;
            enable_double_debuff_by_.enemy_sentry = false;
        }
        // if double debuff is enabled,do nothing
        if (radar_information_.if_double_debuff_enabled)
            return;
        // if (radar_information_.gamestate == GameState::STARTED) {
        //     if (radar_information_.enemy_sentry_hp < 380 && !enable_double_debuff_by_.enemy_sentry) {
        //         radar_information_.enable_double_debuff = true;
        //         enable_double_debuff_by_.enemy_sentry = true;
        //     }

        //     if (radar_information_.if_active_big_buff && !enable_double_debuff_by_.big_buff) {
        //         sleep(5);
        //         radar_information_.enable_double_debuff = true;
        //         enable_double_debuff_by_.big_buff = true;
        //     }
        // }

        if (radar_information_.dart_change) {
            radar_information_.enable_double_debuff = true;
            logger_->INFO("-->dart change");
        }

        if (radar_information_.time_remain < 30 && radar_information_.gamestate == GameState::STARTED) {
            radar_information_.enable_double_debuff = true;
        }

        if (radar_information_.enable_double_debuff && radar_information_.double_debuff_cmd < 2) {
            radar_information_.double_debuff_cmd++;
            logger_->INFO("--->Changed cmd id: " + std::to_string(radar_information_.double_debuff_cmd));
            radar_information_.if_double_debuff_enabled = true;
        }

        radar_information_.dart_change = false;
        radar_information_.enable_double_debuff = false;
    };
};

}