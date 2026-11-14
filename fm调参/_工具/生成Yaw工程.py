"""生成唯一的Yaw专项FreeMASTER工程，结构参考project4云台调参的Yaw部分。"""
from pathlib import Path
import argparse
import importlib.util
import json
import sys
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
OUTPUT = HERE.parent
REPO = OUTPUT.parent
spec = importlib.util.spec_from_file_location('project_generator', HERE / '生成工程.py')
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)
V, G = gen.V, gen.G


def yaw_only():
    j = 'g_arm_status.joint[0].'
    d = 'g_yaw_feedback_diag.'
    command = [V('g_robot_debug_command',
                 '命令 1关控 3清故障 5仅使能Yaw 6仅失能Yaw', write=True, low=0, high=6)]
    # 模式切换、开控和输入检测单独一组；不放PID参数。
    mode_state = gen.TUNE_STATUS + [
        V('g_robot_tune_status.yaw_parameter_result', 'Yaw参数结果 0成功'),
        V('g_robot_status.remote_online', '遥控在线'),
        V('g_robot_status.updated_ms', '整车协调发布时间', 'ms'),
        V('g_arm_status.updated_ms', '机械臂发布时间', 'ms'),
        V('g_arm_status.last_result', 'Arm最近操作结果'),
        V('g_arm_status.configuration_valid', '机械臂配置合法'),
        V('g_robot_status.independent_armed', '独立底盘已解锁'),
        V('g_robot_status.yaw_direct_active', '遥控Yaw直控生效'),
        V('g_robot_status.protection_current_reasons', '当前保护原因位图'),
    ] + gen.joint_fields(0, ('online', 'calibrated', 'enabled', 'faults',
                             'feedback_age_ms', 'homing'))

    # 速度环：只放速度PID、速度目标/反馈、前馈和最终输出。
    speed_vars = gen.YAW_SPEED + [
        V('tune_yaw_velocity_kp', 'Yaw速度P', 'raw/(deg/s)', True, 0, 1000000),
        V('tune_yaw_velocity_ki', 'Yaw速度I', 'raw/deg', True, 0, 1000000),
        V('tune_yaw_voltage_limit_raw', 'Yaw测试输出上限 电压原码', 'raw', True, 0, 25000),
        V('tune_yaw_friction_ff_raw', 'Yaw静摩擦前馈 电压原码', 'raw', True, 0, 25000),
    ] + gen.variables('g_yaw_feedback_diag', [
        ('yaw_pid_dt_s', 'PID实际dt', 's'),
        ('yaw_spd_kp', '速度PID Kp', 'raw/(deg/s)'),
        ('yaw_spd_ki', '速度PID Ki', 'raw/deg'),
        ('yaw_spd_kd', '速度PID Kd', 'raw/(deg/s)'),
        ('yaw_spd_error', '速度误差', 'deg/s'),
        ('yaw_spd_max_output', '速度PID最大输出', 'raw'),
        ('yaw_spd_max_i_error', '速度积分分离阈值', 'deg/s'),
        ('yaw_spd_max_sigma', '速度误差积分上限', 'deg'),
        ('yaw_spd_p_term', '速度P输出', 'raw'),
        ('yaw_spd_i_term', '速度I输出', 'raw'),
        ('yaw_spd_d_term', '速度D输出', 'raw'),
        ('yaw_spd_sigma', '速度误差积分', 'deg'),
        ('yaw_spd_prelimit', 'PID加前馈限幅前', 'raw'),
        ('yaw_spd_output', 'PID边界限幅后输出', 'raw'),
    ]) + [
        V('g_robot_tune_status.yaw_velocity_kp', '生效速度P'),
        V('g_robot_tune_status.yaw_velocity_ki', '生效速度I'),
        V('g_robot_tune_status.yaw_voltage_limit_raw', '生效输出上限 电压原码', 'raw'),
        V('g_arm_status.yaw_friction_ff_applied', '实际静摩擦前馈', 'raw'),
    ] + gen.joint_fields(0, ('velocity_target_deg_s', 'velocity_deg_s',
                             'can_velocity_deg_s', 'encoder_velocity_deg_s',
                             'effort', 'faults', 'feedback_age_ms'))
    speed_group = G('第1步 Yaw速度环（mode5）', speed_vars, [
        ('示波器 速度目标与反馈', ['tune_yaw_speed_deg_s', j + 'velocity_target_deg_s',
                                   j + 'velocity_deg_s', j + 'can_velocity_deg_s',
                                   j + 'encoder_velocity_deg_s']),
        ('示波器 速度P I 前馈 总输出', [d + 'yaw_spd_p_term', d + 'yaw_spd_i_term',
                                       'g_arm_status.yaw_friction_ff_applied', j + 'effort']),
        ('示波器 速度误差积分与dt', [d + 'yaw_spd_sigma', d + 'yaw_pid_dt_s']),
    ])

    # 位置环：只放位置PID、位置目标/反馈和位置PID输出。
    position_vars = gen.YAW_POSITION + [
        V('tune_yaw_position_kp', 'Yaw位置P', '1/s', True, 0.001, 1000),
    ] + gen.variables('g_yaw_feedback_diag', [
        ('yaw_pid_dt_s', 'PID实际dt', 's'),
        ('yaw_pos_kp', '位置PID Kp', '1/s'),
        ('yaw_pos_ki', '位置PID Ki', '1/(deg*s)'),
        ('yaw_pos_kd', '位置PID Kd', 's/deg'),
        ('yaw_pos_error', '位置误差', 'deg'),
        ('yaw_pos_max_output', '位置PID最大输出速度', 'deg/s'),
        ('yaw_pos_max_i_error', '位置积分分离阈值', 'deg'),
        ('yaw_pos_max_sigma', '位置误差积分上限', 'deg*s'),
        ('yaw_pos_p_term', '位置P输出', 'deg/s'),
        ('yaw_pos_i_term', '位置I输出', 'deg/s'),
        ('yaw_pos_d_term', '位置D输出', 'deg/s'),
        ('yaw_pos_sigma', '位置误差积分', 'deg*s'),
        ('yaw_pos_output', '位置PID输出速度', 'deg/s'),
    ]) + [
        V('g_robot_tune_status.yaw_position_kp', '生效位置P', '1/s'),
        V('g_arm_status.yaw_friction_ff_applied', '实际静摩擦前馈', 'raw'),
    ] + gen.joint_fields(0, ('target_deg', 'reference_deg', 'position_deg',
                             'velocity_target_deg_s', 'velocity_deg_s',
                             'effort', 'faults', 'feedback_age_ms'))
    position_group = G('第2步 Yaw位置环（mode6）', position_vars, [
        ('示波器 目标角与实际角', ['tune_yaw_target_deg', j + 'target_deg',
                                   j + 'position_deg', j + 'reference_deg']),
        ('示波器 位置PID输出与速度反馈', [d + 'yaw_pos_output', j + 'velocity_target_deg_s',
                                         j + 'velocity_deg_s']),
        ('示波器 位置P I 前馈 总输出', [d + 'yaw_pos_p_term', d + 'yaw_pos_i_term',
                                      'g_arm_status.yaw_friction_ff_applied', j + 'effort']),
    ])

    # 遥控跟随：目标由摇杆产生，不包含调参PID。
    remote_vars = [V('tune_yaw_remote_lead_deg', 'Yaw遥控目标最大领先', 'deg', True, 2, 180)] + [
        V('g_robot_status.remote_axis[1]', '左横处理后输入'),
        V('g_robot_status.remote_online', '遥控在线'),
        V('g_robot_status.independent_armed', '独立底盘已解锁'),
        V('g_robot_status.yaw_direct_active', '遥控Yaw直控生效'),
    ] + gen.joint_fields(0, ('enabled', 'homing', 'target_deg', 'position_deg',
                             'velocity_target_deg_s', 'velocity_deg_s'))
    remote_group = G('第3步 遥控跟随（mode0）', remote_vars, [
        ('示波器 遥控目标与实际角', [j + 'target_deg', j + 'position_deg']),
        ('示波器 遥控目标速度与反馈', [j + 'velocity_target_deg_s', j + 'velocity_deg_s']),
    ])

    limit_group = G('第4步 零位与限位', gen.YAW_LIMIT + gen.joint_fields(
        0, ('raw_encoder', 'fixed_zero_encoder', 'homing')))

    feedback_vars = gen.variables('g_yaw_feedback_diag', [
        ('window_valid', '500ms统计有效', ''),
        ('window_seq', '统计窗口序号', ''),
        ('encoder_speed_deg_s', '500ms角差均速', 'deg/s'),
        ('rpm_mean_deg_s', '500ms原始rpm均速', 'deg/s'),
        ('zero_rpm_fraction', '零rpm比例', ''),
        ('raw_rpm', '最新原始rpm', 'rpm'),
        ('max_gap_ms', '最大帧间隔', 'ms'),
        ('gap_resets', '超时重启次数', ''),
        ('invalid_frames', '无效帧数', ''),
    ]) + gen.joint_fields(0, ('can_velocity_deg_s', 'encoder_velocity_deg_s',
                              'velocity_window_ms', 'faults', 'feedback_age_ms')) + [
        V('g_arm_stack_free_bytes', 'Arm历史最小剩余栈', 'byte'),
        V('g_stack_overflow_latched', '栈溢出锁存 0正常'),
    ]
    feedback_group = G('第5步 反馈对照与接收状态', feedback_vars, [
        ('示波器 40ms闭环速度与原始rpm', [j + 'encoder_velocity_deg_s',
                                         j + 'can_velocity_deg_s']),
        ('示波器 500ms同窗速度对照', [d + 'encoder_speed_deg_s', d + 'rpm_mean_deg_s']),
    ])

    items = [
        G('第0步 模式切换与开控检测', gen.MODE + command + mode_state),
        speed_group,
        position_group,
        remote_group,
        limit_group,
        feedback_group,
    ]
    gen.attach_scope_specs(items)
    return items


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-backup', action='store_true')
    args = parser.parse_args()
    path = OUTPUT / 'Yaw调参.pmpx'
    record = gen.create(path, 'Yaw专项调参｜project4风格', yaw_only(), not args.no_backup,
                        root_expressions=['robot_tune_mode', 'tune_yaw_speed_deg_s',
                                          'tune_yaw_target_deg', 'tune_yaw_position_kp',
                                          'tune_yaw_velocity_kp', 'tune_yaw_velocity_ki'])
    # Gains, seconds and integrals must not look like integer or single-decimal values.
    tree = ET.parse(path)
    for obj in tree.findall('./objects/object[@type="CVariable"]'):
        get = lambda n: obj.find(f'member[@name="{n}"]')
        if get('treat_as').text == '2':
            name = get('name').text
            if name.endswith('yaw_pid_dt_s'):
                digits = '6'
            elif any(k in name for k in ('kp', 'ki', 'kd', 'sigma', 'error',
                                         '_term', 'max_', 'prelimit', 'output')):
                digits = '4'
            else:
                digits = '2'
            get('show_num_afp_digs').text = digits
    tree.write(path, encoding='utf-8', xml_declaration=True)
    gen.validate(path)
    (HERE / '绑定校验_Yaw.json').write_text(
        json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f"已生成 {path.name}：{record['groups']}组，{record['variables']}变量，"
          f"{record['scopes']}示波器")


if __name__ == '__main__':
    main()
