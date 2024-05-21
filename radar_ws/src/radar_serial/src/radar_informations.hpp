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
}