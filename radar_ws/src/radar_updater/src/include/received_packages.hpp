#pragma once

#include <cstddef>
#include <cstdint>
#include <sys/cdefs.h>

namespace radar::package::receive {

constexpr size_t frame_data_max_length = 1024;

struct __attribute__((packed)) FrameHeader {
    uint8_t start;
    uint16_t data_length;
    uint8_t sequence;
    uint8_t crc8;
};

struct __attribute__((packed)) FrameBody {
    uint16_t command_id;
    uint8_t data[frame_data_max_length];
};

struct __attribute__((packed)) Frame {
    FrameHeader header;
    FrameBody body;
};

struct __attribute__((packed)) GameStatus {
    uint8_t game_type : 4;
    uint8_t game_stage : 4;
    uint16_t stage_remain_time;
    uint64_t sync_timestamp;
};

struct __attribute__((packed)) GameRobotHp {
    uint16_t red_1;
    uint16_t red_2;
    uint16_t red_3;
    uint16_t red_4;
    uint16_t red_5;
    uint16_t red_7;
    uint16_t red_outpost;
    uint16_t red_base;
    uint16_t blue_1;
    uint16_t blue_2;
    uint16_t blue_3;
    uint16_t blue_4;
    uint16_t blue_5;
    uint16_t blue_7;
    uint16_t blue_outpost;
    uint16_t blue_base;
};

struct __attribute__((packed)) RadarInfo {
    uint8_t radar_info;
};

struct __attribute__((packed)) EventData {
    uint32_t event_data;
};

struct __attribute__((packed)) RadarMarkProgress {
    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
};
struct __attribute__((packed)) DartInfo {
    uint8_t dart_remaining_time;
    uint16_t dart_info;
};

struct __attribute__((packed)) DataFromSentry {
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[70];
};

} // namespace package::receive