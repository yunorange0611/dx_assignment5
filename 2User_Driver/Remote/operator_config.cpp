#include "Remote/operator_control.h"
namespace engineer {
const OperatorConfig kOperatorConfig = {
    false, // TODO: 通道/方向全部实物核对后再设true；当前仅确认左右拨杆索引
    1,
    0, // 实物确认：左拨杆=switches[1]，右拨杆=switches[0]
    3,
    2,
    1,
    0, // 左上下、左左右、右上下、右左右
    {1, 1, 1, 1},
    0.06f,
    100,
    100,
    300,
    45.0f,  // 张开目标，32T输出轮角，可用行程仍需实测
    0.0f,   // 闭合参考，在该姿态人工校零，张开为正
    150.0f, // 保持电流上限占位，非夹持力标定值
    2000,   // 闭合最多推进2秒，随后停止增加目标并降低电流上限
    7.0f    // 大P满杆目标推进速度deg/s；底层纠偏速度上限由arm_config.cpp设置
};
}
