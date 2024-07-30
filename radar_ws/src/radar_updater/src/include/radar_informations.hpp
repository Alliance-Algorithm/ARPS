#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <string>

namespace radar::info {
struct Configs {
    bool debug;
    int friend_side;
    std::string log_path;
    std::string serial_port;
    float delay_duration;
};

struct enemy_robot_position {
    float x;
    float y;
    std::chrono::steady_clock::time_point last_updated_time;
};

struct __attribute__((packed)) enemy_robot_position_new {
    uint16_t hero_position_x;
    uint16_t hero_position_y;
    uint16_t engineer_position_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t infantry_5_position_x;
    uint16_t infantry_5_position_y;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
};

struct __attribute__((packed)) position_data_with_sentry {
    struct
    {
        float x;
        float y;
    } positions[6];
};

struct __attribute__((packed)) single_robot_position_data {
    uint16_t target_robot_id;
    float target_position_x;
    float target_position_y;
};

struct __attribute__((packed)) radar_cmd_t {
    uint8_t cmd;
};

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

   是否激活大能量机关  is_active_big_buff;
   是否已开启双倍易伤  is_double_debuff_enabled;
   可用双倍易伤次数 double_debuff_chances;
   飞镖目标 dart_target;

   开启双倍易伤  enable_double_debuff;
   雷达决策命令  double_debuff_cmd;
 */
class Informations {
public:
    int friend_Side_;
    GameState gamestate_;
    int time_remain_;
    float enemy_sentry_hp_;

    bool is_active_big_buff_;
    bool is_double_debuff_enabled_;

    int double_debuff_chances_;
    int double_debuff_cmd_;

    int dart_target_;
    bool dart_change_;

    struct is_enable_double_debuff_by_ {
        bool enemy_sentry;
        bool big_buff;
    };

    std::map<int, enemy_robot_position> enemy_robot_positions;
    std::map<int, int> enemy_catorgories;
};

}