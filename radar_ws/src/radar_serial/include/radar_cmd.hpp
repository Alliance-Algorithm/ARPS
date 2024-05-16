#pragma once

#include "../include/receive.hpp"
#include "Logger.hpp"
#include <bitset>
#include <opencv2/core.hpp>
#include <serial/serial.h>
#include <serial_util/crc/dji_crc.hpp>
#include <serial_util/package_receive.hpp>
#include <unistd.h>

#define RED 100
#define BLUE 101

namespace radar {

enum class GameState {
    NOT_START,
    PREPARATION,
    REFREE_CHECK,
    COUNTDOWN,
    STARTED,
    SETTLING
};

struct Radar_information {
    int friend_Side;
    GameState gamestate;
    int time_remain;
    float enemy_sentry_hp;

    bool if_active_big_buff;
    bool if_double_debuff_enabled;

    bool enable_double_debuff;
    int double_debuff_cmd;
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

class Radar_decision {
public:
    Radar_decision(serial::Serial* serial, Logger* logger, int friend_side)
        : radar_serial_(serial)
        , logger(logger)
    {
        radar_information_ = Radar_information {
            friend_side, GameState::NOT_START, -1, 400, false, false, 0, 0
        };
        radar_mark_progress = RadarMarkProgress {
            0, 0, 0, 0, 0, 0
        };
        enable_double_debuff_by = Enable_double_debuff_by { 0, 0 };
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
                    logger->WARNING("Body crc16 invalid");
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
                logger->WARNING("Header start invalid");
            } else if (result == serial_util::ReceiveResult::VERIFY_INVAILD) {
                logger->WARNING("Header crc8 invalid");
            }
        }
        make_decision();
    }

    void process_frame_info()
    {
        auto command_id = frame_.body.command_id;
        if (command_id == 0x0001)
            update_game_status();
        if (command_id == 0x0003)
            update_sentry_hp();
        if (command_id == 0x020C)
            update_mark_progress();
        if (command_id == 0x020E)
            update_radar_info();
    }
    void update_game_status()
    {
        auto& data = reinterpret_cast<package::receive::GameStatus&>(frame_.body.data);
        radar_information_.gamestate = static_cast<GameState>(data.game_stage);
        radar_information_.time_remain = static_cast<int>(data.stage_remain_time);
    };
    void update_sentry_hp()
    {
        auto& data = reinterpret_cast<package::receive::GameRobotHp&>(frame_.body.data);
        radar_information_.enemy_sentry_hp = radar_information_.friend_Side == RED ? data.blue_7 : data.red_7;
    };

    void update_buff_info()
    {
        auto& data = reinterpret_cast<package::receive::EventData&>(frame_.body.data);
        std::bitset<32> event_info_val(data.event_data);
        radar_information_.if_active_big_buff = static_cast<bool>(event_info_val[5]);
    };

    void update_radar_info()
    {
        auto& data = reinterpret_cast<package::receive::RadarInfo&>(frame_.body.data);
        std::bitset<8> radar_info_val(data.radar_info);
        radar_information_.if_double_debuff_enabled = static_cast<bool>(radar_info_val[2]);
    };

    void update_mark_progress()
    {
        auto& data = reinterpret_cast<package::receive::RadarMarkProgress&>(frame_.body.data);
        radar_mark_progress = RadarMarkProgress {
            data.mark_hero_progress,
            data.mark_engineer_progress,
            data.mark_standard_3_progress,
            data.mark_standard_4_progress,
            data.mark_standard_5_progress,
            data.mark_sentry_progress
        };
    }

    package::receive::Frame frame_;
    serial::Serial* radar_serial_;
    size_t cache_size_ = 0;

    Radar_information radar_information_;
    Enable_double_debuff_by enable_double_debuff_by;
    RadarMarkProgress radar_mark_progress;
    Logger* logger;

public:
    void make_decision()
    {
        radar_information_.enable_double_debuff = false;

        // reset status if game not start
        if (radar_information_.gamestate == GameState::SETTLING) {
            radar_information_.enemy_sentry_hp = 400;
            radar_information_.double_debuff_cmd = 0;
            radar_information_.enable_double_debuff = false;
            enable_double_debuff_by.big_buff = false;
            enable_double_debuff_by.enemy_sentry = false;
        }
        // if double debuff is enabled,do nothing
        if (radar_information_.if_double_debuff_enabled)
            return;
        if (radar_information_.gamestate == GameState::STARTED) {
            if (radar_information_.enemy_sentry_hp < 380 && !enable_double_debuff_by.enemy_sentry) {
                radar_information_.enable_double_debuff = true;
                enable_double_debuff_by.enemy_sentry = true;
            }

            if (radar_information_.if_active_big_buff && !enable_double_debuff_by.big_buff) {
                sleep(5);
                radar_information_.enable_double_debuff = true;
                enable_double_debuff_by.big_buff = true;
            }
        }

        if (radar_information_.time_remain < 30 && radar_information_.double_debuff_cmd < 2) {
            enable_double_debuff_by.big_buff = true;
        }

        if (radar_information_.enable_double_debuff)
            radar_information_.double_debuff_cmd++;
    };
};

}