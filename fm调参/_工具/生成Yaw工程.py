"""生成一份只面向Yaw调参的FreeMASTER工程；仍绑定当前Debug整车ELF。"""
from pathlib import Path
import importlib.util
import json
import sys

HERE = Path(__file__).resolve().parent
OUTPUT = HERE.parent
REPO = OUTPUT.parent
sys.path.insert(0, str(REPO / 'build' / 'test_deps'))

spec = importlib.util.spec_from_file_location('project_generator', HERE / '生成工程.py')
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)

V = gen.V
G = gen.G


def yaw_only():
    yaw_state = gen.joint_fields(
        0, ('online', 'calibrated', 'enabled', 'faults', 'feedback_age_ms',
            'position_deg', 'target_deg', 'reference_deg', 'velocity_deg_s',
            'velocity_target_deg_s', 'effort'))
    yaw_state += [
        V('g_arm_status.joint[0].homing', 'Yaw开控回零中'),
        V('g_arm_status.joint[0].raw_encoder', 'Yaw原始编码器 0..8191'),
        V('g_arm_status.joint[0].fixed_zero_encoder', 'Yaw固定零点 -1未填写'),
        V('g_arm_status.joint[0].temperature_c', 'Yaw温度', '℃'),
        V('g_arm_status.last_result', 'Arm最近操作结果'),
        V('g_robot_status.remote_online', '遥控在线'),
        V('g_robot_status.updated_ms', '整车协调发布时间', 'ms'),
        V('g_arm_status.updated_ms', '机械臂发布时间', 'ms'),
        V('g_robot_status.protection_current_reasons', '当前保护原因位图'),
    ]
    target = gen.MODE + gen.YAW_SPEED + gen.YAW_POSITION
    tune = gen.YAW_TUNE + gen.YAW_LIMIT
    yaw_pid = gen.variables('g_yaw_feedback_diag', [
        ('yaw_pid_dt_s','Yaw PID实际dt','s'),
        ('yaw_pos_p_term','Yaw位置P项','deg/s'),
        ('yaw_pos_i_term','Yaw位置I项','deg/s'),
        ('yaw_pos_d_term','Yaw位置D项','deg/s'),
        ('yaw_pos_sigma','Yaw位置误差积分','deg*s'),
        ('yaw_pos_output','Yaw位置PID输出速度','deg/s'),
        ('yaw_spd_p_term','Yaw速度P项','raw'),
        ('yaw_spd_i_term','Yaw速度I项','raw'),
        ('yaw_spd_d_term','Yaw速度D项','raw'),
        ('yaw_spd_sigma','Yaw速度误差积分','deg'),
        ('yaw_spd_prelimit','Yaw速度PID限幅前','raw'),
        ('yaw_spd_output','Yaw速度PID输出','raw')]) + [
        V('g_arm_status.yaw_friction_ff_applied','Yaw实际摩擦前馈','raw')]
    items = [
        G('Yaw｜模式、目标与使能', target),
        G('Yaw｜增益、限幅与静摩擦前馈', tune),
        G('Yaw｜新PID（只读）', yaw_pid),
        G('Yaw｜就绪、反馈与清零诊断', yaw_state),
        G('Yaw｜速度与输出曲线', yaw_state + gen.YAW_SPEED, [
            ('Yaw速度目标/反馈', ['g_arm_status.joint[0].velocity_target_deg_s',
                                 'g_arm_status.joint[0].velocity_deg_s']),
            ('Yaw输出 effort', ['g_arm_status.joint[0].effort']),
        ]),
        G('Yaw｜位置曲线', yaw_state + gen.YAW_POSITION, [
            ('Yaw位置/目标/参考', ['g_arm_status.joint[0].target_deg',
                                  'g_arm_status.joint[0].position_deg',
                                  'g_arm_status.joint[0].reference_deg']),
        ]),
    ]
    gen.attach_scope_specs(items)
    return items


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    items = yaw_only()
    path = OUTPUT / 'Yaw调参.pmpx'
    record = gen.create(path, 'Yaw专项调参', items, True)
    (HERE / '绑定校验_Yaw.json').write_text(
        json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f"已生成 {record['file']}：{record['groups']} 个分组，"
          f"{record['variables']} 个变量，{record['scopes']} 个示波器；"
          f"{Path(record['elf']).parent.name}")
    print('只包含Yaw目标、增益、状态、反馈和曲线；不改变其他工程。')


if __name__ == '__main__':
    main()
