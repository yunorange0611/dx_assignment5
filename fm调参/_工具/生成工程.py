"""按当前 ELF 生成三份 FreeMASTER 工程；不烧录、不连接、不向目标写值。"""
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
    return dict(expr=expr, label=('★ ' if write else '') + label, unit=unit,
                writable=write, min=low, max=high, description=expr + '；' + (note or ('参数先写齐，request_seq最后递增。' if write else '只读观察，不能用它控制电机。')))


def variables(prefix, names):
    return [V(prefix + '.' + name, label, unit) for name, label, unit in names]


COMMON = [V('uwTick', '系统毫秒计数', 'ms'), V('g_dr16_status.received_ms', '遥控最近接收时刻', 'ms'),
          V('g_robot_status.updated_ms', '整车协调发布时间', 'ms'),
          V('g_robot_status.stop_epoch', '全局关控次数'),
          V('g_robot_status.remote_age_ms', '遥控接收年龄', 'ms'),
          V('g_robot_status.wait_neutral', '等待回中'),
          V('g_robot_status.protection_disabled', '机械臂保护失能'),
          V('g_robot_status.remote_online', '遥控在线'), V('g_robot_status.mode', '模式 0安全1行驶2夹取3待机'),
          V('g_robot_status.active', '机械臂协调开控'), V('g_arm_status.enabled_mask', '机械臂使能掩码'),
          V('g_robot_status.protection_current_reasons', '当前保护原因位图'),
          V('g_robot_status.protection_latched_reasons', '历史锁存原因位图'),
          V('g_robot_debug_command', '整车命令 1停2校零3清错4请求开控', write=True, low=0, high=4)]
CH_COMMON = variables('g_chassis_status', [('updated_ms','底盘发布时间','ms'),('ready','底盘任务就绪',''),
            ('configuration_valid','底盘配置合法',''),('hardware_confirmed','底盘实测确认',''),
            ('faults','底盘故障位图',''),('enabled','底盘出力使能',''),('bench_active','轮组脉冲运行',''),
            ('wheels_stationary','驱动轮静止确认','')])
REMOTE = variables('g_dr16_status', [('valid','接收帧有效',''),('consecutive_frames','连续有效帧数',''),
            ('good_frames','有效帧累计',''),('rejected_frames','拒收累计',''),('uart_errors','串口错误累计',''),('restart_errors','DMA重启错误累计','')]) + [V(f'g_dr16_status.frame.channel[{i}]', f'遥控通道{i} 中位约0') for i in range(5)] + [V(f'g_dr16_status.frame.switches[{i}]', f'拨杆原码{i} 上1中3下2') for i in range(2)]
CH_REQUEST = [V('g_chassis_debug_request.mode','轮测试模式 0停1清错2电流3速度',write=True,low=0,high=3),
              V('g_chassis_debug_request.wheel','测试轮 0左ID1 1右ID2',write=True,low=0,high=1),
              V('g_chassis_debug_request.value','轮测试值 电流mA或转子rpm',write=True,low=-300,high=300),
              V('g_chassis_debug_request.duration_ms','脉冲持续时间','ms',True,1,500),
              V('g_chassis_debug_request.suspended_confirm','悬空确认 填0x53555350',write=True),
              V('g_chassis_debug_request.request_seq','轮测试提交序号 最后加1',write=True)] + variables('g_chassis_debug_status', [('consumed_seq','已消费轮测试序号',''),('result','轮测试结果 见排障PDF',''),('until_ms','轮脉冲截止时刻','ms')])
CH_TUNE = [V('g_chassis_tune_request.velocity_kp','轮速P申请','mA/rpm',True,0.001,1000),
           V('g_chassis_tune_request.velocity_ki','轮速I申请','mA/(rpm*s)',True,0,1000),
           V('g_chassis_tune_request.max_current_ma','轮组运行限流申请','mA',True,1,1000),
           V('g_chassis_tune_request.request_seq','轮参数提交序号 最后加1',write=True)] + variables('g_chassis_tune_status', [('consumed_seq','已消费轮参数序号',''),('active_seq','已生效轮参数序号',''),('result','轮参数结果 0待机1成功2非法3忙4未就绪',''),('velocity_kp','生效轮速P','mA/rpm'),('velocity_ki','生效轮速I','mA/(rpm*s)'),('max_current_ma','生效运行限流','mA')])


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
        ('drive_position_deg','电机驱动角','deg'),('drive_target_deg','电机驱动目标','deg'),
        ('drive_velocity_deg_s','电机驱动速度','deg/s'),('velocity_target_deg_s','闭环目标速度','deg/s'),
        ('effort','控制输出','mA' if i in (0,3,4) else 'Nm')]]


ARM_OTHER_HEALTH = [V(f'g_arm_status.joint[{i}].{name}', label, unit)
                    for i in (3, 4) for name, label, unit in [
                        ('online', '辅助轴在线', ''), ('calibrated', '辅助轴已校零', ''),
                        ('faults', '辅助轴故障', ''), ('drive_velocity_deg_s', '辅助轴驱动速度', 'deg/s')]]


ARM_TUNE = [V('g_arm_tune_request.joint','调参轴 0Yaw1肩2肘3Roll4爪',write=True,low=0,high=4),
            V('g_arm_tune_request.position_kp','位置P申请','1/s',True,0.001,1000),
            V('g_arm_tune_request.velocity_kp','速度P申请',write=True,low=0,high=1000000),
            V('g_arm_tune_request.velocity_ki','速度I申请',write=True,low=0,high=1000000),
            V('g_arm_tune_request.effort_limit','运行输出上限申请 DM为Nm DJI为mA',write=True,low=0),
            V('g_arm_tune_request.request_seq','关节参数提交序号 最后加1',write=True)] + variables('g_arm_tune_status', [('consumed_seq','已消费关节参数序号',''),('active_seq','已生效关节参数序号',''),('result','关节参数结果 engineer::Result',''),('active_joint','最近生效轴',''),('position_kp','生效位置P','1/s'),('velocity_kp','生效速度P',''),('velocity_ki','生效速度I',''),('effort_limit','生效输出上限 注意轴单位','')])
YAW_LIMIT = [V('g_yaw_limit_request.half_range_deg','Yaw软限位半幅申请','deg',True,2.001,90),
             V('g_yaw_limit_request.request_seq','限位提交序号 最后加1',write=True)] + variables('g_yaw_limit_status', [('consumed_seq','限位已消费序号',''),('active_seq','限位生效序号',''),('result','限位结果 engineer::Result',''),('ready','在线限位就绪',''),('active_half_range_deg','生效半幅','deg'),('min_deg','生效最小角','deg'),('max_deg','生效最大角','deg'),('max_speed_deg_s','限位基础速度上限','deg/s')])
YAW_REQUEST = [V('g_yaw_debug_request.mode','Yaw速度测试 0停止1短脉冲',write=True,low=0,high=1),
               V('g_yaw_debug_request.speed_deg_s','Yaw速度测试目标','deg/s',True,-5,5),
               V('g_yaw_debug_request.duration_ms','Yaw脉冲时间','ms',True,1,500),
               V('g_yaw_debug_request.suspended_confirm','Yaw支撑确认 填0x59415753',write=True),
               V('g_yaw_debug_request.request_seq','Yaw测试提交序号 最后加1',write=True)] + variables('g_yaw_debug_status', [('consumed_seq','Yaw测试已消费序号',''),('result','Yaw测试结果 0停1接受2非法3未就绪4撤销',''),('active','Yaw速度测试运行',''),('speed_deg_s','接受的Yaw测试速度','deg/s')])
POWER = variables('g_chassis_status', [('power_frames','功率帧累计',''),('power_received_ms','功率最近接收时刻','ms'),('power_w','功率计反馈','W'),('energy_j','缓冲能量','J'),('power_configuration_valid','功率配置合法',''),('power_confirmed','功率实测确认',''),('power_phase','功率停车阶段',''),('power_reason','功率停车原因',''),('power_session_allowed','能量允许运行',''),('power_stop_requested','功率请求受控停车',''),('power_inhibited','功率禁止重启','')])
IMU = variables('g_ins_status', [('updated_ms','IMU发布时间','ms'),('received_ms','IMU采样时刻','ms'),('frames','IMU采样累计',''),('ready','IMU任务就绪',''),('confirmed','安装与阈值已确认',''),('calibrated','偏置校准完成',''),('valid','IMU有效',''),('faults','IMU故障位图',''),('tilt_deg','倾斜角','deg'),('tipped','翻车判定',''),('calibrating','正在静态标定',''),('driver_errors','驱动错误累计','')]) + [V('g_ins_debug_command','请求IMU静态标定 写1',write=True,low=0,high=1), V('g_vehicle_health.healthy','整车健康'), V('g_vehicle_health.reasons','整车不放行原因位图')]


def G(title, data, curves=()):
    dedup = list({v['expr']:v for v in data}.values())
    return dict(title=title, variables=dedup, scopes=[dict(title=t,exprs=e,period=.01) for t,e in curves])


def wheel_scope(i):
    return ('左轮转子速度与最终电流' if i == 0 else '右轮转子速度与最终电流',
            [f'g_chassis_status.{n}[{i}]' for n in ('target_rpm','measured_rpm','current_ma')])


def arm_scope(i):
    return ('Yaw目标与反馈' if i == 0 else ('肩关节跟踪与输出' if i == 1 else '肘关节跟踪与输出'),
            [f'g_arm_status.joint[{i}].{n}' for n in ('target_deg','reference_deg','position_deg','effort')])


def groups(profile):
    safety = COMMON + [V('g_arm_status.updated_ms', '机械臂发布时间', 'ms')]
    if profile == 'YawBenchDebug':
        safety += YAW_REQUEST + joint(0)
    else:
        safety += CH_COMMON + [V(f'g_chassis_status.current_ma[{i}]', '轮组提交电流', 'mA') for i in range(2)]
    base = G('第0批 安全常驻｜' + profile, safety)
    if profile == 'ChassisBenchDebug':
        tune = [base, G('第1批 遥控与两轮反馈｜不出力',COMMON+CH_COMMON+REMOTE+wheel(0)+wheel(1)),
                G('第2批 左轮ID1方向｜mode=2',COMMON+CH_COMMON+CH_REQUEST+wheel(0),[wheel_scope(0)]),
                G('第3批 右轮ID2方向｜mode=2',COMMON+CH_COMMON+CH_REQUEST+wheel(1),[wheel_scope(1)]),
                G('第4批 两轮速度PI｜mode=3',COMMON+CH_COMMON+CH_TUNE+CH_REQUEST+wheel(0)+wheel(1),[wheel_scope(0),wheel_scope(1)]),
                G('第5批 抬臂后待机保持｜左下才允许轮脉冲',COMMON+CH_COMMON+CH_REQUEST+joint(1)+joint(2)+ARM_OTHER_HEALTH,[arm_scope(1),arm_scope(2)]),
                G('第6批 下一阶段Yaw｜先切换分组工具选择Yaw',COMMON)]
    elif profile == 'YawBenchDebug':
        tune = [base,G('第6批 Yaw线束中位与软限位｜先关控',COMMON+YAW_LIMIT+joint(0)),
                G('第7批 Yaw速度PI｜mode=1小幅短脉冲',COMMON+ARM_TUNE+YAW_LIMIT+YAW_REQUEST+joint(0),
                  [('Yaw目标速度与反馈及电流',['g_yaw_debug_request.speed_deg_s','g_arm_status.joint[0].velocity_target_deg_s','g_arm_status.joint[0].velocity_deg_s','g_arm_status.joint[0].effort'])]),
                G('第8批 位置P需整车模式｜结束后切Vehicle',COMMON+ARM_TUNE+joint(0),[arm_scope(0)])]
    else:
        tune = [base,G('第8批 IMU标定与真实整车许可',COMMON+CH_COMMON+IMU+POWER),
                G('第9批 Yaw位置P与小角度跟踪',COMMON+ARM_TUNE+YAW_LIMIT+joint(0),[arm_scope(0)]),
                G('第10批 慢速落地与差速｜快速档未接通',COMMON+CH_COMMON+CH_TUNE+wheel(0)+wheel(1)+[V('g_chassis_status.v_m_s','整车轮速估计纵向速度','m/s'),V('g_chassis_status.omega_rad_s','整车轮速估计转向速度','rad/s'),V('g_robot_status.fast_requested','快速档请求'),V('g_robot_status.drive_gear','许可档位 0慢1快')],[wheel_scope(0),wheel_scope(1)]),
                G('第11批 功率停车与故障恢复',COMMON+CH_COMMON+POWER+CH_REQUEST+IMU,
                  [('功率与缓冲',['g_chassis_status.power_w','g_chassis_status.energy_j'])]),
                G('第12批 固化参数与重启复验',COMMON+CH_TUNE+ARM_TUNE+YAW_LIMIT)]
    diagnostics = [G('排查0 系统心跳与开控原因｜'+profile,COMMON),G('排查1 遥控不在线或回中失败',COMMON+REMOTE),
                   G('排查2 CAN1与机械臂反馈',COMMON+[V('can_tx_error_count[0]','CAN1发送错误累计'),V('g_arm_status.tx_errors[0]','Arm发送错误累计'),V('g_arm_status.rx_dropped','Arm接收丢帧累计')]+joint(0 if profile=='YawBenchDebug' else 1))]
    if profile != 'YawBenchDebug':
        diagnostics += [G('排查3 轮请求拒绝与抬臂保持门槛',COMMON+CH_COMMON+CH_REQUEST+CH_TUNE+joint(1)+joint(2)),
                        G('排查4 左右轮不转反转抖动',COMMON+CH_COMMON+wheel(0)+wheel(1)+[V('can_tx_error_count[1]','CAN2发送错误累计'),V('g_chassis_status.rx_dropped','CAN2接收丢帧累计')],[wheel_scope(0),wheel_scope(1)]),
                        G('排查5 功率计反馈与停车阶段',COMMON+CH_COMMON+POWER)]
    if profile != 'ChassisBenchDebug':
        diagnostics += [G('排查6 Yaw零位限位与在线参数',COMMON+YAW_LIMIT+ARM_TUNE+joint(0),[arm_scope(0)])]
    if profile == 'YawBenchDebug':
        diagnostics += [G('排查7 Yaw短脉冲不接受',COMMON+YAW_LIMIT+YAW_REQUEST+joint(0))]
    if profile.startswith('Vehicle'):
        diagnostics += [G('排查8 IMU翻车及整车不放行',COMMON+CH_COMMON+IMU)]
    return tune, diagnostics


def arm_groups():
    return [G('第0步 关控与连接｜ChassisBenchDebug',COMMON+[V('g_arm_status.updated_ms','机械臂发布时间','ms'),V('g_arm_status.ready','机械臂任务就绪')]),
            G('第1步 双Pitch人工联合零位｜命令2',COMMON+[V('g_arm_status.last_result','Arm最近操作结果'),V('g_arm_status.configuration_valid','Arm配置合法')]+joint(1)+joint(2)+ARM_OTHER_HEALTH),
            G('第2步 简易抬肩抬肘｜左中 遥控操作',COMMON+joint(1)+joint(2),[arm_scope(1),arm_scope(2)]),
            G('第3步 少量改PI与限流｜先左上关控',COMMON+ARM_TUNE+joint(1)+joint(2)),
            G('第4步 左下待机保持｜配合底盘短脉冲',COMMON+CH_COMMON+joint(1)+joint(2)+ARM_OTHER_HEALTH,[arm_scope(1),arm_scope(2)]),
            G('排查1 不使能不抬起或下垂',COMMON+ARM_TUNE+[V('g_arm_status.last_result','Arm最近操作结果'),V('g_arm_status.faults','Arm总体故障'),V('can_tx_error_count[0]','CAN1发送错误累计')]+joint(1)+joint(2))]


def bind(items, symbols):
    for group in items:
        for spec in group['variables']:
            address,width,treat = symbols.resolve(spec['expr'])
            if not 0x20000000 <= address < 0x20020000:
                raise ValueError('只允许绑定片内RAM观测/请求: '+spec['expr'])
            spec.update(last_addr=address,byte_size=width,treat_as=treat)
            if spec['writable'] and not spec['expr'].startswith(('g_chassis_debug_request.','g_chassis_tune_request.','g_yaw_limit_request.','g_yaw_debug_request.','g_arm_tune_request.','g_robot_debug_command','g_ins_debug_command')):
                raise ValueError('不可写符号: '+spec['expr'])


def create(path, profile, title, items, backup):
    elf_path = REPO / 'build' / profile / 'dx_assignment5_freertos.elf'
    if not elf_path.is_file():
        raise FileNotFoundError(f'先编译 {profile}，没有找到 {elf_path}')
    sym = Symbols(elf_path)
    try:
        bind(items,sym)
        if backup and path.exists():
            shutil.copy2(path,path.with_suffix('.pmpx.bak'))
        result = generate(path,title=title+'｜'+profile,elf_path=elf_path,groups=items,
                          template_path=HERE/'样式模板.xml')
        # 回读生成文件的表达式/缓存地址/宽度/权限，不能留下旧ELF地址。
        import xml.etree.ElementTree as ET
        for obj in ET.parse(path).findall('./objects/object[@type="CVariable"]'):
            get=lambda n:obj.find(f'member[@name="{n}"]').text
            address,width,treat=sym.resolve(get('address'))
            assert int(get('last_addr'),0)==address and int(get('byte_size'))==width and int(get('treat_as'))==treat
        result.update(file=path.name,sha256=hashlib.sha256(elf_path.read_bytes()).hexdigest())
        return result
    finally:
        sym.stream.close()


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',choices=['ChassisBenchDebug','YawBenchDebug','VehicleDebug'],default='ChassisBenchDebug')
    parser.add_argument('--no-backup',action='store_true')
    args=parser.parse_args()
    tune,diagnostic=groups(args.profile)
    records=[create(OUTPUT/'底盘调参.pmpx',args.profile,'底盘与Yaw分批调参',tune,not args.no_backup),
             create(OUTPUT/'底盘排障.pmpx',args.profile,'底盘与Yaw排障',diagnostic,not args.no_backup),
             create(OUTPUT/'机械臂简易调参与排障.pmpx','ChassisBenchDebug','机械臂抬起与保持',arm_groups(),not args.no_backup)]
    (HERE/'绑定校验.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
    for record in records:
        print(f"已生成 {record['file']}：{record['groups']} 个中文分组，{record['variables']} 个变量，{record['scopes']} 个示波器；{Path(record['elf']).parent.name}")
    print('只更新本地工程文件。固件需你自行选择匹配版本；机械臂简易工程固定ChassisBenchDebug，YawBench时不要连接该工程。')


if __name__=='__main__':
    try:
        main()
    except Exception as error:
        print('生成失败：'+str(error),file=sys.stderr)
        raise SystemExit(1)
