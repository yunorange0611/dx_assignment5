"""从单一 Debug 整车固件 ELF 刷新三份 FreeMASTER 工程；不烧录、不连接、不写目标。"""
from pathlib import Path
import argparse
import hashlib
import json
import re
import shutil
import sys

HERE = Path(__file__).resolve().parent
OUTPUT = HERE.parent
REPO = OUTPUT.parent
sys.path.insert(0, str(REPO / 'build/test_deps'))
from elftools.elf.elffile import ELFFile
from fm_xml import generate, validate


class Symbols:
    """从 ARM ELF/DWARF 解析成员偏移，禁止借用旧工程地址或猜结构体布局。"""
    def __init__(self, path):
        self.path = path
        self.stream = path.open('rb')
        self.elf = ELFFile(self.stream)
        self.addresses = {s.name: s['st_value'] for s in self.elf.get_section_by_name('.symtab').iter_symbols()}
        self.types = {}
        for cu in self.elf.get_dwarf_info().iter_CUs():
            for die in cu.iter_DIEs():
                if die.tag != 'DW_TAG_variable' or 'DW_AT_name' not in die.attributes:
                    continue
                name = die.attributes['DW_AT_name'].value.decode('utf-8', errors='replace')
                if name in self.addresses and 'DW_AT_type' in die.attributes:
                    self.types.setdefault(name, die.get_DIE_from_attribute('DW_AT_type'))

    @staticmethod
    def unwrap(die):
        while die.tag in ('DW_TAG_typedef', 'DW_TAG_const_type', 'DW_TAG_volatile_type', 'DW_TAG_restrict_type'):
            die = die.get_DIE_from_attribute('DW_AT_type')
        return die

    def size(self, die):
        die = self.unwrap(die)
        if 'DW_AT_byte_size' in die.attributes:
            return die.attributes['DW_AT_byte_size'].value
        if die.tag == 'DW_TAG_array_type':
            n = 1
            for child in die.iter_children():
                if child.tag == 'DW_TAG_subrange_type':
                    n *= child.attributes['DW_AT_count'].value if 'DW_AT_count' in child.attributes else child.attributes['DW_AT_upper_bound'].value + 1
            return n * self.size(die.get_DIE_from_attribute('DW_AT_type'))
        raise ValueError(f'不能确定变量宽度: {die.tag}')

    def resolve(self, expr):
        base = re.match(r'\w+', expr).group()
        if base not in self.types:
            raise ValueError(f'{self.path.parent.name} 不存在带类型的符号: {base}')
        address = self.addresses[base]
        die = self.types[base]
        tail = expr[len(base):]
        for token in re.finditer(r'\.([A-Za-z_]\w*)|\[(\d+)\]', tail):
            die = self.unwrap(die)
            if token.group(1):
                member = next((c for c in die.iter_children() if c.tag == 'DW_TAG_member' and c.attributes.get('DW_AT_name') and c.attributes['DW_AT_name'].value.decode() == token.group(1)), None)
                if member is None:
                    raise ValueError(f'找不到成员 {expr}')
                offset = member.attributes['DW_AT_data_member_location'].value
                if not isinstance(offset, int):
                    raise ValueError(f'不支持动态偏移 {expr}')
                address += offset
                die = member.get_DIE_from_attribute('DW_AT_type')
            else:
                if die.tag != 'DW_TAG_array_type':
                    raise ValueError(f'并非数组: {expr}')
                element = die.get_DIE_from_attribute('DW_AT_type')
                index = int(token.group(2))
                if (index + 1) * self.size(element) > self.size(die):
                    raise ValueError(f'数组越界 {expr}')
                address += index * self.size(element)
                die = element
        die = self.unwrap(die)
        encoding = die.attributes.get('DW_AT_encoding')
        treat = 2 if encoding and encoding.value == 4 else (1 if encoding and encoding.value in (5, 6) else 0)
        return address, self.size(die), treat


def V(expr, label, unit='', write=False, low=None, high=None, note=''):
    default_note = ('四项参数与joint先填齐，最后递增本结构request_seq；状态只读。'
                    if expr.startswith('g_arm_tune_request.') else
                    '直接修改，由唯一控制owner检查后使用；切档先清目标。')
    return dict(expr=expr, label=('★ ' if write else '') + label, unit=unit,
                writable=write, min=low, max=high, description=expr + '；' + label + '；' +
                (note or (default_note if write else '只读观察，不能用它控制电机。')))


def variables(prefix, names):
    return [V(prefix + '.' + name, label, unit) for name, label, unit in names]


TUNE_STATUS = variables('g_robot_tune_status', [('updated_ms','调参状态发布时间','ms'),
              ('mode','实际调参档位',''),('epoch','切档代号',''),('reason','调参原因 0正常1就绪2未就绪3关控4非法5电流超时6保留7目标为0/未开始',''),
              ('ready','调参已就绪',''),('blocked','调参锁禁',''),('held_mask','保留保持的关节掩码','')])
MODE = [V('robot_tune_mode','调参对象 0正常1左电流2右电流3左速度4右速度5Yaw速度6Yaw位置',write=True,low=0,high=6,
          note='只选对象；切档自动清旧目标。写非零电流/速度目标即开始本次测试，写0停止；位置档选档后保持在当前角。0档退出并保留已有机械臂保持。')]
ENABLE = []  # No separate enable gate.
COMMON = MODE + ENABLE + TUNE_STATUS + [V('uwTick', '系统毫秒计数', 'ms'), V('g_dr16_status.received_ms', '遥控最近接收时刻', 'ms'),
          V('g_robot_status.updated_ms', '整车协调发布时间', 'ms'),
          V('g_robot_status.stop_epoch', '全局关控次数'),
          V('g_robot_status.remote_age_ms', '遥控接收年龄', 'ms'),
          V('g_robot_status.wait_neutral', '等待回中'),
          V('g_robot_status.protection_disabled', '机械臂保护失能'),
          V('g_robot_status.remote_online', '遥控在线'), V('g_robot_status.mode', '模式 0安全1行驶2夹取3待机'),
          V('g_robot_status.active', '机械臂协调开控'), V('g_arm_status.enabled_mask', '机械臂使能掩码'),
          V('g_robot_status.protection_current_reasons', '当前保护原因位图'),
          V('g_robot_status.protection_latched_reasons', '历史锁存原因位图'),
          V('g_robot_debug_command', '整车命令 1停2校零3清错4请求开控5仅使能Yaw6仅失能Yaw', write=True, low=0, high=6)]
CH_COMMON = variables('g_chassis_status', [('updated_ms','底盘发布时间','ms'),('ready','底盘任务就绪',''),
            ('configuration_valid','底盘配置合法',''),('hardware_confirmed','底盘实测确认',''),
            ('faults','底盘故障位图',''),('enabled','底盘出力使能',''),
            ('wheels_stationary','驱动轮静止确认',''),('tune_mode','底盘实际调参档',''),
            ('tune_blocked','底盘调参锁禁',''),('tune_result','底盘参数处理结果',''),
            ('tune_current_limit_ma','底盘测试生效限流','mA')])
REMOTE = variables('g_dr16_status', [('valid','接收帧有效',''),('consecutive_frames','连续有效帧数',''),
            ('good_frames','有效帧累计',''),('rejected_frames','拒收批次累计',''),('uart_errors','串口错误累计',''),('restart_errors','DMA重启错误累计',''),
            ('start_calls','接收初始化调用次数 正常至少1',''),('service_ms','接收维护心跳','ms'),
            ('rx_running','UART及DMA接收已运行',''),('last_start_result','最近启动结果 0成功1错误2忙3超时',''),
            ('rx_events','UART DMA接收回调累计',''),('rx_bytes','接收回调累计字节数','byte'),
            ('last_event_ms','最近接收回调时间','ms'),('last_size','最近回调字节数 合法18/36/54/72','byte'),
            ('last_uart_error','最近串口错误位 1校验2噪声4帧8溢出16DMA',''),
            ('uart_rx_state','UART接收状态 32就绪34接收中',''),('dma_remaining','当前DMA剩余计数 初始72','byte')]) + [V(f'g_dr16_status.frame.channel[{i}]', f'遥控通道{i} 中位约0') for i in range(5)] + [V(f'g_dr16_status.frame.switches[{i}]', f'拨杆原码{i} 上1中3下2') for i in range(2)]
REMOTE_MAPPING = REMOTE + [V(f'g_robot_status.remote_axis[{i}]',label,'[-1,1]') for i,label in enumerate(
    ('处理后左竖Y 当前原始channel3','处理后左横X 当前原始channel2','处理后右竖Y 当前原始channel1','处理后右横X 当前原始channel0'))] + [
    V(f'g_robot_status.remote_switch[{i}]','处理后'+side+'拨杆原码 上1中3下2') for i,side in enumerate(('左','右'))] + [
    V('uwTick','系统毫秒计数','ms'),V('g_dr16_status.received_ms','原始帧最近接收时刻','ms'),
    V('g_robot_status.updated_ms','处理后快照发布时间','ms'),V('g_robot_status.remote_online','遥控在线'),
    V('g_robot_status.remote_age_ms','原始帧年龄','ms')]
CH_CURRENT = [V('tune_current_ma','单轮电流测试目标','mA',True,-10000,10000,
                note='选档后从0小步填写；非零电流开始500ms计时，写0停止。')]
CH_MOTION = [
    V('tune_chassis_acceleration_m_s2', '行驶加速度', 'm/s^2', True, 0.001, 100),
    V('tune_chassis_deceleration_m_s2', '行驶减速度', 'm/s^2', True, 0.001, 100),
    V('tune_chassis_angular_accel_rad_s2', '转向角加速度', 'rad/s^2', True, 0.001, 100),
    V('tune_chassis_angular_decel_rad_s2', '转向角减速度', 'rad/s^2', True, 0.001, 100),
    V('tune_chassis_max_turn_rad_s', '最大转向角速度', 'rad/s', True, 0.001, 10),
]
CH_SPEED = [V('tune_speed_rpm','单轮速度目标 3508转子rpm','rpm',True,-9000,9000)]
CH_TUNE = [V('tune_wheel_kp','轮速P','mA/rpm',True,0.001,1000),
           V('tune_wheel_ki','轮速I','mA/(rpm*s)',True,0,1000),
           V('tune_wheel_current_limit_ma','本次轮组测试限流','mA',True,1,10000)] + variables('g_chassis_tune_status',
           [('result','轮参数结果 0待机1成功2非法3忙4未就绪',''),('velocity_kp','生效轮速P','mA/rpm'),
            ('velocity_ki','生效轮速I','mA/(rpm*s)'),('max_current_ma','生效运行限流','mA')])
CH_CLEAR = [V('g_chassis_debug_request.mode','本地维护 0停止1清底盘故障',write=True,low=0,high=1,
              note='仅维护请求。正常调轮不写它；清错填1后，最后递增同结构request_seq。'),
            V('g_chassis_debug_request.request_seq','仅清底盘故障时最后加1',write=True,low=0,high=2147483647,
              note='本地维护专用提交序号；不用于日常轮电流/速度目标。')] + variables('g_chassis_debug_status',
            [('consumed_seq','已消费维护序号',''),('result','维护结果 2已停3已请求清错5未就绪','')])


def wheel(i):
    side = '左ID1' if i == 0 else '右ID2'
    return [V(f'g_chassis_status.{n}[{i}]', side + label, unit) for n,label,unit in [
        ('feedback_frames','反馈累计',''),('feedback_ms','最近反馈时刻','ms'),('measured_rpm','实测转子速度','rpm'),
        ('target_rpm','目标转子速度','rpm'),('current_ma','最终提交电流','mA'),('temperature_c','电机温度','℃')]]


def joint(i):
    side = ['Yaw','肩关节','肘关节','Roll','夹爪'][i]
    return [V(f'g_arm_status.joint[{i}].{n}', side + label, unit) for n,label,unit in [
        ('online','在线',''),('calibrated','已人工校零',''),('enabled','已使能',''),('motor_state','电机状态',''),
        ('faults','故障位图',''),('feedback_age_ms','反馈年龄','ms'),('position_deg','关节角','deg'),
        ('target_deg','关节目标','deg'),('reference_deg','参考角','deg'),('velocity_deg_s','关节速度','deg/s'),
        ('can_velocity_deg_s','CAN原始rpm速度','deg/s'),('encoder_velocity_deg_s','编码器40ms速度','deg/s'),('velocity_window_ms','测速窗口','ms'),
        ('drive_position_deg','电机驱动角','deg'),('drive_target_deg','电机驱动目标','deg'),
        ('drive_velocity_deg_s','电机驱动速度','deg/s'),('velocity_target_deg_s','闭环目标速度','deg/s'),
        ('effort','控制输出','raw' if i == 0 else ('mA' if i in (3,4) else 'Nm'))]]


ARM_OTHER_HEALTH = [V(f'g_arm_status.joint[{i}].{name}', label, unit)
                    for i in (3, 4) for name, label, unit in [
                        ('online', '辅助轴在线', ''), ('calibrated', '辅助轴已校零', ''),
                        ('faults', '辅助轴故障', ''), ('drive_velocity_deg_s', '辅助轴驱动速度', 'deg/s')]]


ARM_TUNE = [V('g_arm_tune_request.joint','调参轴 0Yaw1肩2肘3Roll4爪',write=True,low=0,high=4),
            V('g_arm_tune_request.position_kp','位置P申请','1/s',True,0.001,1000),
            V('g_arm_tune_request.velocity_kp','速度P申请',write=True,low=0,high=1000000),
            V('g_arm_tune_request.velocity_ki','速度I申请',write=True,low=0,high=1000000),
            V('g_arm_tune_request.effort_limit','运行输出上限申请 DM为Nm Yaw为电压原码 M2006为mA',write=True,low=0,high=1000000),
            V('g_arm_tune_request.request_seq','关节参数提交序号 最后加1',write=True,low=0,high=2147483647)] + variables('g_arm_tune_status', [('consumed_seq','已消费关节参数序号',''),('active_seq','已生效关节参数序号',''),('result','关节参数结果 engineer::Result',''),('active_joint','最近生效轴',''),('position_kp','生效位置P','1/s'),('velocity_kp','生效速度P',''),('velocity_ki','生效速度I',''),('effort_limit','生效输出上限 注意轴单位','')])
YAW_ZERO = [
    V('g_arm_status.joint[0].raw_encoder', 'Yaw原始编码器 对正车头时抄到kYawZeroEncoder', note='只读0..8191；零点在Arm/yaw_config.h填写，重新编译烧录。'),
    V('g_arm_status.joint[0].fixed_zero_encoder', 'Yaw已配置固定零点 -1未填写'),
    V('g_arm_status.joint[0].homing', 'Yaw开控回零中'),
]
YAW_LIMIT = YAW_ZERO + [V('tune_yaw_half_range_deg','Yaw软限位半幅','deg',True,2.001,360,
              note='实物确认的线束半幅；直接填写，owner检查后生效。看result与active_half_range_deg，不写旧限位序号。')] + variables('g_yaw_limit_status',
              [('result','限位结果 0成功',''),('ready','在线限位就绪',''),('active_half_range_deg','生效半幅','deg'),
               ('min_deg','生效最小角','deg'),('max_deg','生效最大角','deg'),('max_speed_deg_s','限位基础速度上限','deg/s')])
YAW_SPEED = [V('tune_yaw_speed_deg_s','Yaw速度内环目标','deg/s',True,-100,100)]
YAW_POSITION = [V('tune_yaw_target_deg','Yaw位置双环目标','deg',True,-360,360,
                 note='切6档时自动对齐当前角。目标必须在已生效线束软限位内，先测1到2度小动作。')]
YAW_TUNE = [V('tune_yaw_position_kp','Yaw位置P','1/s',True,0.001,1000),
            V('tune_yaw_velocity_kp','Yaw速度P','raw/(deg/s)',True,0,1000000),
            V('tune_yaw_velocity_ki','Yaw速度I','raw/deg',True,0,1000000),
            V('tune_yaw_voltage_limit_raw','Yaw测试输出上限 电压原码','raw',True,0,25000),
            V('tune_yaw_friction_ff_raw','Yaw静摩擦前馈 电压原码','raw',True,0,25000),
            V('tune_yaw_remote_lead_deg','Yaw遥控目标最大领先 默认3','deg',True,2,180),
            V('g_arm_status.yaw_friction_ff_applied','Yaw实际叠加静摩擦前馈','raw')] + variables('g_robot_tune_status',
            [('yaw_parameter_result','Yaw参数结果 0成功',''),('yaw_position_kp','Yaw生效位置P','1/s'),
             ('yaw_velocity_kp','Yaw生效速度P',''),('yaw_velocity_ki','Yaw生效速度I',''),
             ('yaw_voltage_limit_raw','Yaw生效输出上限 电压原码','raw')])
POWER = variables('g_chassis_status', [('power_frames','功率帧累计',''),('power_received_ms','功率最近接收时刻','ms'),('power_w','功率计反馈','W'),('energy_j','缓冲能量','J'),('min_energy_j','本次最低缓冲能量','J'),('power_over_50_ms','原始功率超50W累计时间','ms'),('power_over_latched','曾超50W锁存',''),('power_filtered_w','滤波后功率','W'),('power_speed_scale','功率/能量限速比例',''),('power_configuration_valid','功率配置合法',''),('power_confirmed','功率实测确认',''),('power_phase','功率停车阶段',''),('power_reason','功率停车原因',''),('power_session_allowed','能量允许运行',''),('power_stop_requested','功率请求受控停车',''),('power_inhibited','功率禁止重启','')])
POWER_TUNE = [
    V('tune_chassis_power_budget_w','功率预算 50W以下','W',True,1,49,
      note='confirmed=false 时不生效；当前代码默认未启用。'),
    V('tune_chassis_power_filter_ms','功率滤波时间','ms',True,1,2000),
]
TILT_TUNE = [
    V('tune_attitude_trip_deg','翻车触发角','deg',True,1,89),
    V('tune_attitude_release_deg','翻车解除角','deg',True,0.1,88),
    V('tune_attitude_trip_ms','翻车触发时间','ms',True,1,5000),
    V('tune_attitude_release_ms','翻车解除时间','ms',True,1,5000),
]
IMU = variables('g_ins_status', [('updated_ms','IMU发布时间','ms'),('received_ms','IMU采样时刻','ms'),('frames','IMU采样累计',''),('ready','IMU任务就绪',''),('confirmed','安装与阈值已确认',''),('calibrated','偏置校准完成',''),('valid','IMU有效',''),('faults','IMU故障位图',''),('tilt_deg','倾斜角','deg'),('tipped','翻车判定',''),('calibrating','正在静态标定',''),('driver_errors','驱动错误累计','')]) + TILT_TUNE + [V('g_ins_debug_command','请求IMU静态标定 写1',write=True,low=0,high=1), V('g_vehicle_health.healthy','整车健康'), V('g_vehicle_health.reasons','整车不放行原因位图')]


def G(title, data, curves=()):
    dedup = list({v['expr']:v for v in data}.values())
    return dict(title=title, variables=dedup, scopes=[dict(title=t,exprs=e,period=.01) for t,e in curves])


def remote_mapping_group(compact=False):
    if compact:
        # 正常映射检查只对照杆量和拨杆；接收链计数留在排障工程。
        data = [v for v in REMOTE_MAPPING if v['expr'].startswith(
            ('g_dr16_status.frame.', 'g_robot_status.remote_axis[',
             'g_robot_status.remote_switch['))]
        data += [V('g_robot_status.remote_online', '遥控在线')]
    else:
        data = REMOTE_MAPPING
    return G('遥控映射检查｜原始与处理后',data)


def wheel_scope(i):
    return ('左轮转子速度与最终电流' if i == 0 else '右轮转子速度与最终电流',
            [f'g_chassis_status.{n}[{i}]' for n in ('target_rpm','measured_rpm','current_ma')])


def arm_scope(i):
    return ('Yaw目标与反馈' if i == 0 else ('肩关节跟踪与输出' if i == 1 else '肘关节跟踪与输出'),
            [f'g_arm_status.joint[{i}].{n}' for n in ('target_deg','reference_deg','position_deg','effort')])


def wheel_fields(i, names):
    return [v for v in wheel(i) if v['expr'].rsplit('.', 1)[-1].split('[')[0] in names]


def joint_fields(i, names):
    return [v for v in joint(i) if v['expr'].rsplit('.', 1)[-1] in names]


def attach_scope_specs(items):
    specs = {v['expr']: v for group in items for v in group['variables']}
    for source in (wheel(0), wheel(1), joint(0), joint(1), joint(2)):
        specs.update({v['expr']: v for v in source})
    for group in items:
        shown = {v['expr'] for v in group['variables']}
        extra = dict.fromkeys(expr for curve in group['scopes'] for expr in curve['exprs']
                              if expr not in shown)
        group['scope_variables'] = [specs[expr] for expr in extra]


def groups():
    # 一个批次一个 Group；每页只放本批要写的目标、要看的反馈和必要曲线。
    # 档位/使能只保留两行，不再加入左下右中、回中300ms、停稳等门控步骤。
    select = MODE + ENABLE
    command = [V('g_robot_debug_command',
                 '整车命令 0空闲1停2校零3清错4请求开控5仅使能Yaw6仅失能Yaw',
                 write=True, low=0, high=6)]
    wheel_params = CH_TUNE[:3]
    wheel_status = CH_TUNE[3:]
    yaw_params = YAW_TUNE[:4]
    yaw_status = YAW_TUNE[4:]
    currents = [V(f'g_chassis_status.current_ma[{i}]', '轮组提交电流', 'mA') for i in range(2)]
    tune_gate = [V('g_robot_tune_status.ready', '调参已就绪'),
                 V('g_robot_tune_status.blocked', '调参锁禁')]
    half_range = [V('tune_yaw_half_range_deg', 'Yaw软限位半幅', 'deg', True, 2.001, 360,
                    note='直接填写；看result与active_half_range_deg。')]
    tune = [
        # 0档底盘直驱：只依赖共用遥控解析和两轮反馈，不绑机械臂/Yaw/IMU/功率。
        G('0档底盘直驱｜左拨杆上', select + [
            V('g_robot_status.remote_axis[0]', '左竖 前后请求'),
            V('g_robot_status.remote_axis[3]', '右横 转向请求'),
            V('g_robot_status.remote_switch[0]', '左拨杆 上1中3下2'),
            V('g_dr16_status.valid', '遥控帧有效'),
            V('g_robot_status.remote_age_ms', '遥控接收年龄', 'ms'),
            V('g_robot_status.independent_stop_latched', '独立底盘STOP锁存 回中/命令4清除'),
            V('g_robot_status.independent_armed', '独立底盘已回中解锁'),
            V('g_chassis_status.independent', '底盘直驱中'),
            V('g_chassis_status.enabled', '底盘出力使能'),
            V('g_chassis_status.faults', '底盘故障位图'),
            V('g_chassis_status.wheels_stationary', '驱动轮静止确认'),
            V('g_chassis_status.v_m_s', '纵向速度', 'm/s'),
            V('g_chassis_status.omega_rad_s', '转向速度', 'rad/s')] +
          wheel_fields(0, ('target_rpm', 'measured_rpm', 'current_ma')) +
          wheel_fields(1, ('target_rpm', 'measured_rpm', 'current_ma')),
          [('左轮目标/实际/电流', ['g_chassis_status.target_rpm[0]',
                                  'g_chassis_status.measured_rpm[0]',
                                  'g_chassis_status.current_ma[0]']),
           ('右轮目标/实际/电流', ['g_chassis_status.target_rpm[1]',
                                  'g_chassis_status.measured_rpm[1]',
                                  'g_chassis_status.current_ma[1]'])]),
        # 第0批：只验证关控、失联和零输出。
        G('第0批 安全常驻', select + command + [
            V('uwTick', '系统毫秒计数', 'ms'),
            V('g_robot_status.stop_epoch', '全局关控次数'),
            V('g_robot_status.protection_current_reasons', '当前保护原因位图'),
            V('g_robot_status.protection_latched_reasons', '历史锁存原因位图'),
            V('g_robot_status.remote_age_ms', '遥控接收年龄', 'ms'),
            V('g_arm_status.enabled_mask', '机械臂使能掩码'),
            V('g_chassis_status.enabled', '底盘出力使能'),
            V('g_chassis_status.faults', '底盘故障位图'),
            V('g_dr16_status.valid', '遥控帧有效'),
            V('g_dr16_status.good_frames', '有效帧累计')] + currents),
        # 第1批：只认遥控通道、拨杆、两轮反馈。
        G('第1批 遥控与两轮反馈', select +
          [V(f'g_dr16_status.frame.channel[{i}]', f'遥控通道{i} 中位约0') for i in range(5)] +
          [V(f'g_dr16_status.frame.switches[{i}]', f'拨杆原码{i} 上1中3下2') for i in range(2)] +
          [V(f'g_robot_status.remote_axis[{i}]', label) for i, label in enumerate(
              ('左竖Y 处理后', '左横X 处理后', '右竖Y 处理后', '右横X 处理后'))] +
          [V(f'g_robot_status.remote_switch[{i}]', f'处理后{side}拨杆')
           for i, side in enumerate(('左', '右'))] +
          [V('g_robot_status.remote_online', '遥控在线'),
           V('g_robot_status.remote_age_ms', '遥控接收年龄', 'ms'),
           V('g_dr16_status.good_frames', '有效帧累计')] +
          wheel_fields(0, ('feedback_ms', 'measured_rpm')) +
          wheel_fields(1, ('feedback_ms', 'measured_rpm'))),
        # 第2/3批：单轮电流方向；另一轮只看提交电流为0。
        G('第2批 左轮ID1电流方向', select + CH_CURRENT + wheel_params[2:] +
          wheel_fields(0, ('measured_rpm', 'current_ma')) +
          wheel_fields(1, ('current_ma',)) +
          [V('g_chassis_status.tune_blocked', '底盘调参锁禁')] + tune_gate,
          [('左轮电流指令/实际/转速', ['tune_current_ma', 'g_chassis_status.current_ma[0]',
                                      'g_chassis_status.measured_rpm[0]'])]),
        G('第3批 右轮ID2电流方向', select + CH_CURRENT + wheel_params[2:] +
          wheel_fields(1, ('measured_rpm', 'current_ma')) +
          wheel_fields(0, ('current_ma',)) +
          [V('g_chassis_status.tune_blocked', '底盘调参锁禁')] + tune_gate,
          [('右轮电流指令/实际/转速', ['tune_current_ma', 'g_chassis_status.current_ma[1]',
                                      'g_chassis_status.measured_rpm[1]'])]),
        G('行驶加减速与转向', CH_MOTION),
        G('翻车保护｜独立底盘', select + TILT_TUNE + [
            V('g_ins_status.tilt_deg','当前倾斜角','deg'),
            V('g_ins_status.tipped','翻车判定'),
            V('g_ins_status.valid','IMU有效'),
            V('g_ins_status.calibrated','校准完成'),
            V('g_ins_debug_command','请求IMU静态标定 写1',write=True,low=0,high=1),
            V('g_chassis_status.independent','独立底盘直驱中'),
            V('g_chassis_status.enabled','底盘出力使能'),
            V('g_robot_status.independent_stop_latched','独立底盘STOP锁存 回中/命令4清除'),
            V('g_robot_status.independent_armed','独立底盘已回中解锁'),
            V('g_robot_status.protection_current_reasons','当前保护原因位图')]),
        G('Yaw直控｜左横', select + [
            V('g_robot_debug_command','命令5仅使能Yaw 6仅失能Yaw',write=True,low=0,high=6),
            V('g_robot_status.yaw_direct_active','Yaw直控中'),
            V('g_robot_status.yaw_direct_target_deg','Yaw直控目标','deg'),
            V('g_arm_status.joint[0].position_deg','Yaw反馈','deg'),
            V('g_arm_status.joint[0].target_deg','Yaw目标','deg'),
            V('g_arm_status.joint[0].velocity_deg_s','Yaw速度','deg/s'),
            V('g_arm_status.joint[0].effort','Yaw输出 电压原码','raw'),
            V('tune_yaw_friction_ff_raw','Yaw静摩擦前馈 电压原码','raw',True,0,25000),
            V('g_arm_status.joint[0].enabled','Yaw使能'),
            V('g_arm_status.joint[0].calibrated','Yaw校零'),
            V('g_arm_status.joint[0].faults','Yaw故障位图'),
            V('g_yaw_limit_status.active_half_range_deg','Yaw半幅','deg'),
            V('g_yaw_limit_status.result','Yaw限位结果')]),
        G('功率限制（当前未启用）', POWER_TUNE + [
            V('g_chassis_status.power_w','功率计瞬时功率','W'),
            V('g_chassis_status.energy_j','缓冲能量','J'),
            V('g_chassis_status.min_energy_j','本次最低缓冲能量','J'),
            V('g_chassis_status.power_over_50_ms','原始功率超50W累计时间','ms'),
            V('g_chassis_status.power_over_latched','曾超50W锁存 只读'),
            V('g_chassis_status.power_filtered_w','滤波后功率','W'),
            V('g_chassis_status.power_speed_scale','功率/能量限速比例 0..1'),
            V('g_chassis_status.power_confirmed','功率限制已确认'),
            V('g_chassis_status.power_phase','功率阶段'),
            V('g_chassis_status.power_reason','功率原因'),
            V('g_chassis_status.power_stop_requested','功率请求停车'),
            V('g_chassis_status.power_inhibited','功率锁存禁行')]),
        # 第4批：两轮速度PI；目标只上曲线，不占变量行。
        G('第4批 两轮速度PI', select + wheel_params + CH_SPEED +
          wheel_fields(0, ('measured_rpm', 'current_ma')) +
          wheel_fields(1, ('measured_rpm', 'current_ma')) + wheel_status +
          [V('g_chassis_status.tune_blocked', '底盘调参锁禁')] + tune_gate,
          [('左轮目标/实际/电流', ['g_chassis_status.target_rpm[0]',
                                  'g_chassis_status.measured_rpm[0]',
                                  'g_chassis_status.current_ma[0]']),
           ('右轮目标/实际/电流', ['g_chassis_status.target_rpm[1]',
                                  'g_chassis_status.measured_rpm[1]',
                                  'g_chassis_status.current_ma[1]'])]),
        # 第5批：抬臂与左下保持，只看两pitch和使能状态。
        G('第5批 0档抬臂左下保持', select + command +
          joint_fields(1, ('position_deg', 'target_deg', 'effort')) +
          joint_fields(2, ('position_deg', 'target_deg', 'effort')) + [
              V('g_robot_status.mode', '模式 0安全1行驶2夹取3待机'),
              V('g_robot_status.active', '机械臂协调开控'),
              V('g_arm_status.enabled_mask', '机械臂使能掩码'),
              V('g_chassis_status.enabled', '底盘出力使能')] + currents,
          [('肩目标/反馈/输出', ['g_arm_status.joint[1].target_deg',
                                'g_arm_status.joint[1].position_deg',
                                'g_arm_status.joint[1].effort']),
           ('肘目标/反馈/输出', ['g_arm_status.joint[2].target_deg',
                                'g_arm_status.joint[2].position_deg',
                                'g_arm_status.joint[2].effort'])]),
        # 第6批：Yaw中位与半幅，不加运动曲线。
        G('第6批 Yaw线束中位与软限位', YAW_ZERO + select + command + half_range + [
            V('g_yaw_limit_status.result', '限位结果 0成功'),
            V('g_yaw_limit_status.active_half_range_deg', '生效半幅', 'deg'),
            V('g_yaw_limit_status.min_deg', '生效最小角', 'deg'),
            V('g_yaw_limit_status.max_deg', '生效最大角', 'deg'),
            V('g_arm_status.last_result', 'Arm最近操作结果')] +
          joint_fields(0, ('position_deg', 'calibrated')) + [
              V('g_arm_status.enabled_mask', '机械臂使能掩码')] + tune_gate),
        # 第7批：Yaw速度内环，目标/实际/电压输出同图。
        G('第7批 Yaw速度内环PI', select + yaw_params[1:] + YAW_SPEED +
          joint_fields(0, ('velocity_deg_s', 'velocity_target_deg_s', 'effort')) + [
              yaw_status[0],
              V('g_yaw_limit_status.active_half_range_deg', '生效半幅', 'deg')] +
          currents + tune_gate,
          [('Yaw目标速度/反馈/电压输出', ['tune_yaw_speed_deg_s',
                                     'g_arm_status.joint[0].velocity_target_deg_s',
                                     'g_arm_status.joint[0].velocity_deg_s',
                                     'g_arm_status.joint[0].effort'])]),
        # 第8批：IMU、功率和整车许可，只读。
        G('第8批 IMU功率与真实整车许可', select + [
            V('g_ins_status.valid', 'IMU有效'),
            V('g_ins_status.calibrated', '偏置校准完成'),
            V('g_ins_status.faults', 'IMU故障位图'),
            V('g_ins_status.tilt_deg', '倾斜角', 'deg'),
            V('g_ins_status.tipped', '翻车判定'),
            V('g_ins_status.calibrating', '正在静态标定'),
            V('g_ins_debug_command', '请求IMU静态标定 写1', write=True, low=0, high=1),
            V('g_chassis_status.power_w', '功率计反馈', 'W'),
            V('g_chassis_status.energy_j', '缓冲能量', 'J'),
            V('g_chassis_status.min_energy_j', '本次最低缓冲能量', 'J'),
            V('g_chassis_status.power_over_50_ms', '原始功率超50W累计时间', 'ms'),
            V('g_chassis_status.power_over_latched', '曾超50W锁存'),
            V('g_chassis_status.power_phase', '功率停车阶段'),
            V('g_chassis_status.power_reason', '功率停车原因'),
            V('g_chassis_status.power_frames', '功率帧累计'),
            V('g_chassis_status.power_received_ms', '功率最近接收时刻', 'ms'),
            V('g_chassis_status.faults', '底盘故障位图'),
            V('g_vehicle_health.reasons', '整车不放行原因位图')],
          [('功率与缓冲', ['g_chassis_status.power_w', 'g_chassis_status.energy_j',
                          'g_chassis_status.min_energy_j'])]),
        # 第9批：Yaw位置双环，目标/参考/反馈/输出同图。
        G('第9批 Yaw位置双环', select + yaw_params + YAW_POSITION +
          joint_fields(0, ('target_deg', 'reference_deg', 'position_deg',
                           'velocity_deg_s', 'effort')) + [
              yaw_status[0],
              V('g_yaw_limit_status.active_half_range_deg', '生效半幅', 'deg')] + tune_gate,
          [('Yaw目标/参考/反馈/输出', ['g_arm_status.joint[0].target_deg',
                                      'g_arm_status.joint[0].reference_deg',
                                      'g_arm_status.joint[0].position_deg',
                                      'g_arm_status.joint[0].effort'])]),
        # 第10批：正常遥控，两轮目标/实际/电流。
        G('第10批 正常遥控慢速落地', select + command + [
            V('g_robot_status.mode', '模式 0安全1行驶2夹取3待机'),
            V('g_robot_status.active', '机械臂协调开控'),
            V('g_arm_status.enabled_mask', '机械臂使能掩码'),
            V('g_robot_status.remote_axis[0]', '左竖 前后请求'),
            V('g_robot_status.remote_axis[3]', '右横 转向请求'),
            V('g_chassis_status.v_m_s', '轮速估计纵向速度', 'm/s'),
            V('g_chassis_status.omega_rad_s', '轮速估计转向速度', 'rad/s'),
            V('g_chassis_status.faults', '底盘故障位图')] +
          wheel_fields(0, ('measured_rpm', 'current_ma')) +
          wheel_fields(1, ('measured_rpm', 'current_ma')),
          [('左轮目标/实际/电流', ['g_chassis_status.target_rpm[0]',
                                  'g_chassis_status.measured_rpm[0]',
                                  'g_chassis_status.current_ma[0]']),
           ('右轮目标/实际/电流', ['g_chassis_status.target_rpm[1]',
                                  'g_chassis_status.measured_rpm[1]',
                                  'g_chassis_status.current_ma[1]'])]),
        # 第11批：停止、清错和功率停车分开看。
        G('第11批 停止清错与故障恢复', select + command + [
            V('g_robot_status.protection_current_reasons', '当前保护原因位图'),
            V('g_robot_status.protection_latched_reasons', '历史锁存原因位图'),
            V('g_arm_status.enabled_mask', '机械臂使能掩码'),
            V('g_chassis_status.enabled', '底盘出力使能'),
            V('g_chassis_status.faults', '底盘故障位图'),
            V('g_ins_status.faults', 'IMU故障位图'),
            V('g_chassis_status.power_phase', '功率停车阶段'),
            V('g_chassis_status.power_reason', '功率停车原因'),
            V('g_chassis_status.power_w', '功率计反馈', 'W'),
            V('g_chassis_status.energy_j', '缓冲能量', 'J')] + currents +
          CH_CLEAR[:2] + [
              V('g_chassis_debug_status.result', '维护结果 2已停3已请求清错5未就绪')],
          [('功率与缓冲', ['g_chassis_status.power_w', 'g_chassis_status.energy_j'])]),
        # 第12批：只核对准备写回源码的生效参数。
        G('第12批 固化参数与重启复验', select + wheel_params + yaw_params + half_range +
          wheel_status + yaw_status + [
              V('g_yaw_limit_status.active_half_range_deg', '生效半幅', 'deg')]),
    ]
    diag_core = [MODE[0],
                 V('uwTick', '系统毫秒计数', 'ms'),
                 V('g_robot_status.updated_ms', '整车协调发布时间', 'ms'),
                 V('g_robot_status.mode', '模式 0安全1行驶2夹取3待机'),
                 V('g_robot_status.active', '机械臂协调开控'),
                 V('g_robot_status.protection_current_reasons', '当前保护原因位图'),
                 V('g_robot_status.protection_latched_reasons', '历史锁存原因位图'),
                 V('g_arm_status.enabled_mask', '机械臂使能掩码'),
                 V('g_chassis_status.enabled', '底盘出力使能'),
                 V('g_chassis_status.faults', '底盘故障位图'),
                 V('g_ins_status.faults', 'IMU故障位图'),
                 V('g_vehicle_health.reasons', '整车不放行原因位图')]
    diagnostics = [
        G('排查0 心跳与调参锁禁', diag_core + [
            V('g_robot_tune_status.updated_ms', '调参状态发布时间', 'ms'),
            V('g_robot_tune_status.reason', '调参原因 0正常1就绪2未就绪3关控4非法5电流超时6保留7目标为0/未开始'),
            V('g_robot_tune_status.ready', '调参已就绪'),
            V('g_robot_tune_status.blocked', '调参锁禁'),
            V('g_robot_tune_status.held_mask', '保留保持的关节掩码'),
            V('g_chassis_status.updated_ms', '底盘发布时间', 'ms'),
            V('g_chassis_status.ready', '底盘任务就绪'),
            V('g_chassis_status.configuration_valid', '底盘配置合法'),
            V('g_chassis_status.hardware_confirmed', '底盘实测确认'),
            V('g_chassis_status.tune_mode', '底盘实际调参档'),
            V('g_chassis_status.tune_blocked', '底盘调参锁禁'),
            V('g_chassis_status.tune_result', '底盘参数处理结果'),
            V('g_arm_status.updated_ms', '机械臂发布时间', 'ms'),
            V('g_arm_status.ready', '机械臂任务就绪'),
            V('g_arm_status.configuration_valid', '机械臂配置合法'),
            V('g_ins_status.updated_ms', 'IMU发布时间', 'ms'),
            V('g_ins_status.ready', 'IMU任务就绪'),
            V('g_ins_status.confirmed', 'IMU安装与阈值已确认'),
            V('g_ins_status.calibrated', 'IMU偏置校准完成'),
            V('g_ins_status.valid', 'IMU有效')]),
        G('排查1 遥控不在线或回中失败', diag_core + REMOTE + [
            V('g_robot_status.remote_online', '遥控在线'),
            V('g_robot_status.remote_age_ms', '遥控接收年龄', 'ms')] +
          [V(f'g_robot_status.remote_axis[{i}]', label) for i, label in enumerate(
              ('左竖Y 处理后', '左横X 处理后', '右竖Y 处理后', '右横X 处理后'))] +
          [V(f'g_robot_status.remote_switch[{i}]', f'处理后{side}拨杆')
           for i, side in enumerate(('左', '右'))]),
        G('遥控映射检查｜原始与处理后', [V('uwTick', '系统毫秒计数', 'ms'),
            V('g_robot_status.updated_ms', '处理后快照发布时间', 'ms'),
            V('g_dr16_status.received_ms', '原始帧最近接收时刻', 'ms'),
            V('g_dr16_status.valid', '接收帧有效'),
            V('g_dr16_status.good_frames', '有效帧累计'),
            V('g_robot_status.remote_online', '遥控在线'),
            V('g_robot_status.remote_age_ms', '原始帧年龄', 'ms')] +
          [V(f'g_dr16_status.frame.channel[{i}]', f'遥控通道{i} 中位约0') for i in range(5)] +
          [V(f'g_dr16_status.frame.switches[{i}]', f'拨杆原码{i} 上1中3下2') for i in range(2)] +
          [V(f'g_robot_status.remote_axis[{i}]', label) for i, label in enumerate(
              ('左竖Y 处理后', '左横X 处理后', '右竖Y 处理后', '右横X 处理后'))] +
          [V(f'g_robot_status.remote_switch[{i}]', f'处理后{side}拨杆')
           for i, side in enumerate(('左', '右'))]),
        G('排查2 CAN1与机械臂反馈', diag_core + [
            V('g_arm_status.faults', 'Arm总体故障'),
            V('g_arm_status.last_result', 'Arm最近操作结果'),
            V('can_tx_error_count[0]', 'CAN1发送错误累计'),
            V('g_arm_status.tx_errors[0]', 'Arm发送错误累计'),
            V('g_arm_status.rx_dropped', 'Arm接收丢帧累计')] +
          joint_fields(0, ('online', 'calibrated', 'faults', 'feedback_age_ms',
                           'position_deg', 'target_deg', 'effort')) +
          joint_fields(1, ('online', 'calibrated', 'faults', 'feedback_age_ms',
                           'position_deg', 'target_deg', 'effort')) +
          joint_fields(2, ('online', 'calibrated', 'faults', 'feedback_age_ms',
                           'position_deg', 'target_deg', 'effort')) +
          joint_fields(3, ('online', 'faults')) +
          joint_fields(4, ('online', 'faults'))),
        G('排查3 轮调参锁禁与机械臂保持', diag_core + [
            V('g_chassis_status.tune_mode', '底盘实际调参档'),
            V('g_chassis_status.tune_blocked', '底盘调参锁禁'),
            V('g_chassis_status.tune_result', '底盘参数处理结果'),
            V('g_chassis_status.tune_current_limit_ma', '底盘测试生效限流', 'mA')] +
          wheel_params + wheel_status +
          wheel_fields(0, ('feedback_ms', 'measured_rpm', 'current_ma', 'temperature_c')) +
          wheel_fields(1, ('feedback_ms', 'measured_rpm', 'current_ma', 'temperature_c')) +
          joint_fields(1, ('online', 'calibrated', 'faults', 'position_deg')) +
          joint_fields(2, ('online', 'calibrated', 'faults', 'position_deg'))),
        G('排查4 左右轮不转反转抖动', diag_core + [
            V('can_tx_error_count[1]', 'CAN2发送错误累计'),
            V('g_chassis_status.rx_dropped', 'CAN2接收丢帧累计'),
            V('g_chassis_status.v_m_s', '轮速估计纵向速度', 'm/s'),
            V('g_chassis_status.omega_rad_s', '轮速估计转向速度', 'rad/s'),
            V('g_chassis_status.wheels_stationary', '驱动轮静止确认')] +
          wheel_fields(0, ('feedback_ms', 'measured_rpm', 'target_rpm',
                           'current_ma', 'temperature_c')) +
          wheel_fields(1, ('feedback_ms', 'measured_rpm', 'target_rpm',
                           'current_ma', 'temperature_c')),
          [wheel_scope(0), wheel_scope(1)]),
        G('排查5 功率计反馈与停车阶段', diag_core + POWER),
        G('排查6 Yaw零位限位与生效参数', diag_core + YAW_LIMIT + YAW_TUNE +
          joint_fields(0, ('online', 'calibrated', 'faults', 'feedback_age_ms',
                           'position_deg', 'target_deg', 'reference_deg',
                           'velocity_deg_s', 'effort')) + [
              V('g_arm_status.last_result', 'Arm最近操作结果')]),
        G('排查7 Yaw调参不动或不能切档', diag_core + [
            V('g_robot_tune_status.reason', '调参原因 0正常1就绪2未就绪3关控4非法5电流超时6保留7目标为0/未开始'),
            V('g_robot_tune_status.ready', '调参已就绪'),
            V('g_robot_tune_status.blocked', '调参锁禁')] + YAW_LIMIT + [
                V('tune_yaw_speed_deg_s', 'Yaw速度内环目标', 'deg/s', True, -100, 100),
                V('tune_yaw_target_deg', 'Yaw位置双环目标', 'deg', True, -90, 90)] +
          joint_fields(0, ('online', 'calibrated', 'enabled', 'faults', 'feedback_age_ms',
                           'position_deg', 'target_deg', 'velocity_deg_s', 'effort'))),
        G('排查8 IMU翻车及正常遥控不放行', diag_core + IMU + [
            V('g_chassis_status.power_phase', '功率停车阶段'),
            V('g_chassis_status.power_reason', '功率停车原因'),
            V('g_chassis_status.power_session_allowed', '能量允许运行'),
            V('g_chassis_status.power_stop_requested', '功率请求受控停车'),
            V('g_chassis_status.power_inhibited', '功率禁止重启')]),
    ]
    attach_scope_specs(tune)
    attach_scope_specs(diagnostics)
    return tune, diagnostics


def arm_groups():
    # 机械臂页同样按“02_机械臂抬起与保持.pdf”的步骤分组，每步只留必要量。
    select = MODE + ENABLE
    command = [V('g_robot_debug_command',
                 '整车命令 0空闲1停2校零3清错4请求开控5仅使能Yaw6仅失能Yaw',
                 write=True, low=0, high=6)]
    items = [
        G('第0步 关控与连接', select + command + [
            V('uwTick', '系统毫秒计数', 'ms'),
            V('g_robot_status.updated_ms', '整车协调发布时间', 'ms'),
            V('g_arm_status.updated_ms', '机械臂发布时间', 'ms'),
            V('g_arm_status.ready', '机械臂任务就绪'),
            V('g_arm_status.configuration_valid', '机械臂配置合法'),
            V('g_arm_status.enabled_mask', '机械臂使能掩码'),
            V('g_chassis_status.enabled', '底盘出力使能'),
            V('g_robot_status.protection_current_reasons', '当前保护原因位图'),
            V('g_robot_status.protection_latched_reasons', '历史锁存原因位图')]),
        G('第1步 双Pitch人工联合零位', command + [
            V('g_arm_status.last_result', 'Arm最近操作结果'),
            V('g_arm_status.faults', 'Arm总体故障'),
            V('g_arm_status.enabled_mask', '机械臂使能掩码')] +
          joint_fields(1, ('position_deg', 'calibrated', 'enabled')) +
          joint_fields(2, ('position_deg', 'calibrated', 'enabled'))),
        G('第2步 0档简易抬肩抬肘', command + [
            V('g_robot_status.mode', '模式 0安全1行驶2夹取3待机'),
            V('g_robot_status.active', '机械臂协调开控'),
            V('g_robot_status.wait_neutral', '等待摇杆回中'),
            V('g_arm_status.enabled_mask', '机械臂使能掩码'),
            V('g_robot_status.remote_axis[0]', '左竖 肩'),
            V('g_robot_status.remote_axis[2]', '右竖 肘')] +
          joint_fields(1, ('position_deg', 'target_deg', 'effort')) +
          joint_fields(2, ('position_deg', 'target_deg', 'effort')),
          [('肩目标/反馈/输出', ['g_arm_status.joint[1].target_deg',
                                'g_arm_status.joint[1].position_deg',
                                'g_arm_status.joint[1].effort']),
           ('肘目标/反馈/输出', ['g_arm_status.joint[2].target_deg',
                                'g_arm_status.joint[2].position_deg',
                                'g_arm_status.joint[2].effort'])]),
        G('第3步 少量改PI与限流', ARM_TUNE +
          joint_fields(1, ('position_deg', 'effort')) +
          joint_fields(2, ('position_deg', 'effort')) + [
              V('g_arm_status.enabled_mask', '机械臂使能掩码'),
              V('g_arm_status.last_result', 'Arm最近操作结果')]),
        G('第4步 左下待机保持', select + [
            V('g_robot_status.mode', '模式 0安全1行驶2夹取3待机'),
            V('g_robot_status.active', '机械臂协调开控'),
            V('g_arm_status.enabled_mask', '机械臂使能掩码')] +
          joint_fields(1, ('position_deg', 'target_deg', 'effort')) +
          joint_fields(2, ('position_deg', 'target_deg', 'effort')) +
          wheel_fields(0, ('measured_rpm', 'current_ma')) +
          wheel_fields(1, ('measured_rpm', 'current_ma')),
          [('肩目标/反馈/输出', ['g_arm_status.joint[1].target_deg',
                                'g_arm_status.joint[1].position_deg',
                                'g_arm_status.joint[1].effort']),
           ('肘目标/反馈/输出', ['g_arm_status.joint[2].target_deg',
                                'g_arm_status.joint[2].position_deg',
                                'g_arm_status.joint[2].effort']),
           ('左轮目标/实际/电流', ['g_chassis_status.target_rpm[0]',
                                  'g_chassis_status.measured_rpm[0]',
                                  'g_chassis_status.current_ma[0]']),
           ('右轮目标/实际/电流', ['g_chassis_status.target_rpm[1]',
                                  'g_chassis_status.measured_rpm[1]',
                                  'g_chassis_status.current_ma[1]'])]),
        G('排查1 不使能不抬起或下垂', [
            V('g_arm_status.faults', 'Arm总体故障'),
            V('g_arm_status.last_result', 'Arm最近操作结果'),
            V('g_arm_status.configuration_valid', '机械臂配置合法'),
            V('g_arm_status.enabled_mask', '机械臂使能掩码'),
            V('g_robot_status.protection_current_reasons', '当前保护原因位图'),
            V('g_robot_status.protection_latched_reasons', '历史锁存原因位图'),
            V('can_tx_error_count[0]', 'CAN1发送错误累计'),
            V('g_arm_status.tx_errors[0]', 'Arm发送错误累计'),
            V('g_arm_status.rx_dropped', 'Arm接收丢帧累计')] +
          joint_fields(1, ('online', 'calibrated', 'faults', 'position_deg',
                           'target_deg', 'effort')) +
          joint_fields(2, ('online', 'calibrated', 'faults', 'position_deg',
                           'target_deg', 'effort')) +
          joint_fields(3, ('online', 'faults')) +
          joint_fields(4, ('online', 'faults')))]
    attach_scope_specs(items)
    return items


def bind(items, symbols):
    for group in items:
        for spec in group['variables'] + group.get('scope_variables', []):
            address,width,treat = symbols.resolve(spec['expr'])
            if not 0x20000000 <= address < 0x20020000:
                raise ValueError('只允许绑定片内RAM观测/请求: '+spec['expr'])
            spec.update(last_addr=address,byte_size=width,treat_as=treat)
            if spec['writable'] and not spec['expr'].startswith(('robot_tune_mode','tune_',
                    'g_chassis_debug_request.','g_arm_tune_request.','g_robot_debug_command','g_ins_debug_command')):
                raise ValueError('不可写符号: '+spec['expr'])


def create(path, title, items, backup, root_expressions=()):
    elf_path = REPO / 'build' / 'Debug' / 'dx_assignment5_freertos.elf'
    if not elf_path.is_file():
        raise FileNotFoundError(f'先编译单一 Debug 整车固件，没有找到 {elf_path}')
    sym = Symbols(elf_path)
    try:
        bind(items,sym)
        if backup and path.exists():
            shutil.copy2(path,path.with_suffix('.pmpx.bak'))
        result = generate(path,title=title+'｜单固件Debug',elf_path=elf_path,groups=items,
                          template_path=HERE/'样式模板.xml', root_expressions=root_expressions)
        # 回读生成文件的表达式/缓存地址/宽度/权限，不能留下旧ELF地址。
        import xml.etree.ElementTree as ET
        for obj in ET.parse(path).findall('./objects/object[@type="CVariable"]'):
            get=lambda n:obj.find(f'member[@name="{n}"]').text
            address,width,treat=sym.resolve(get('name'))
            assert int(get('address'),0)==address
            assert int(get('last_addr'),0)==address and int(get('byte_size'))==width and int(get('treat_as'))==treat
        result.update(file=path.name,sha256=hashlib.sha256(elf_path.read_bytes()).hexdigest())
        return result
    finally:
        sym.stream.close()


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-backup',action='store_true')
    args=parser.parse_args()
    tune,diagnostic=groups()
    feedback_diag = variables('g_yaw_feedback_diag', [
        ('window_valid','统计窗口有效',''),('window_seq','窗口更新序号',''),
        ('window_ms','同窗时间','ms'),('window_frames','同窗反馈帧数',''),
        ('encoder_speed_deg_s','编码器角差平均速度','deg/s'),
        ('rpm_mean_deg_s','CAN rpm同窗平均速度','deg/s'),
        ('rpm_min_deg_s','窗口CAN最小速度','deg/s'),('rpm_max_deg_s','窗口CAN最大速度','deg/s'),
        ('zero_rpm_fraction','窗口零rpm比例',''),('raw_encoder','原始编码器',''),
        ('raw_rpm','原始有符号rpm','rpm'),('raw_effort','原始电机反馈输出','raw'),
        ('parsed_speed_deg_s','当前解析速度','deg/s'),('parsed_position_deg','当前解析位置','deg'),
        ('frames','已接收Yaw帧数',''),('invalid_frames','无效帧数',''),
        ('gap_resets','接收间隔超限次数',''),('max_gap_ms','最大接收间隔','ms'),
        ('velocity_kp','控制器当前速度P',''),('velocity_ki','控制器当前速度I',''),
        ('position_kp','控制器当前位置P',''),('updated_ms','最近接收时间','ms'),
        ('can_id','反馈CAN ID',''),('bus','CAN总线',''),('dlc','报文长度','')]) + [
        V(f'g_yaw_feedback_diag.raw_data[{i}]',f'原始CAN字节{i}') for i in range(8)]
    feedback_diag += variables('g_arm_status.joint[0]', [
        ('can_velocity_deg_s','CAN原始rpm速度','deg/s'),
        ('encoder_velocity_deg_s','编码器40ms速度','deg/s'),
        ('velocity_deg_s','当前闭环采用速度','deg/s'),
        ('velocity_target_deg_s','闭环目标速度','deg/s'),
        ('velocity_window_ms','编码器测速窗口','ms')])
    feedback_diag += [V('g_arm_stack_free_bytes','Arm历史最小剩余栈','byte'), V('g_stack_overflow_latched','任务栈溢出锁存 0正常')]
    tune.append(G('Yaw反馈诊断（只读500ms同窗）', feedback_diag))
    diagnostic.append(G('Yaw反馈诊断（只读500ms同窗）', feedback_diag))
    yaw_pid_fields = variables('g_yaw_feedback_diag', [
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
        ('yaw_spd_output','Yaw速度PID输出','raw')])
    yaw_pid_fields += [V('g_arm_status.yaw_friction_ff_applied','Yaw实际摩擦前馈','raw')]
    tune.append(G('Yaw新PID（只读）', yaw_pid_fields))
    diagnostic.append(G('Yaw新PID（只读）', yaw_pid_fields))
    records=[create(OUTPUT/'底盘调参.pmpx','底盘与Yaw分批调参',tune,not args.no_backup,
                    root_expressions=[spec['expr'] for spec in CH_MOTION]),
             create(OUTPUT/'底盘排障.pmpx','底盘与Yaw排障',diagnostic,not args.no_backup),
             create(OUTPUT/'机械臂简易调参与排障.pmpx','机械臂抬起与保持',arm_groups(),not args.no_backup)]
    (HERE/'绑定校验.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
    for record in records:
        print(f"已生成 {record['file']}：{record['groups']} 个中文分组，{record['variables']} 个变量，{record['scopes']} 个示波器；{Path(record['elf']).parent.name}")
    print('三份工程绑定同一份 Debug 整车固件；按批次分组，不需要换固件。本工具不烧录、不连接、不写目标。')


if __name__=='__main__':
    try:
        main()
    except Exception as error:
        print('生成失败：'+str(error),file=sys.stderr)
        raise SystemExit(1)
