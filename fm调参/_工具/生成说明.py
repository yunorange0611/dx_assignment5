"""生成三份上车操作说明；内容对应单 Debug 固件和 fm调参 三个工程。"""
from pathlib import Path
import html
import pymupdf as fitz

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = fitz.Archive('C:/Windows/Fonts')
CSS = '''
@font-face {font-family:cn;src:url(simhei.ttf);}
@font-face {font-family:mono;src:url(consola.ttf);}
body {font-family:cn,sans-serif;font-size:10.3pt;line-height:1.45;color:#192e3b;}
h1 {font-size:19pt;line-height:1.22;color:#124e70;margin:0 0 12pt;}
h2 {font-size:12.5pt;color:#12628a;margin:12pt 0 6pt;}
p {margin:5pt 0 7pt;} li {margin-bottom:5pt;}
ol,ul {padding-left:20pt;margin:6pt 0 9pt;}
table {width:100%;border-collapse:collapse;margin:8pt 0;font-size:8.8pt;line-height:1.34;}
th {background:#e1eef5;text-align:left;}
td,th {border:0.5pt solid #a8bdca;padding:5pt;vertical-align:top;}
code {font-family:mono,cn;font-size:8.5pt;color:#125473;overflow-wrap:anywhere;}
.note {background:#eff5f8;padding:9pt;border-left:3pt solid #18739c;}
.stop {background:#fff1e8;padding:9pt;border-left:3pt solid #b65d28;}
.check {background:#eef7ee;padding:8pt;border-left:3pt solid #2e7d32;}
.small {font-size:9pt;color:#415b69;} .tag {font-size:9pt;color:#12628a;}
'''


def v(text):
    return '<code>' + html.escape(text) + '</code>'


def table(headers, rows):
    return '<table><tr>' + ''.join('<th>' + h + '</th>' for h in headers) + '</tr>' + ''.join(
        '<tr>' + ''.join('<td>' + str(c) + '</td>' for c in row) + '</tr>' for row in rows) + '</table>'


def ol(*items):
    return '<ol>' + ''.join('<li>' + item + '</li>' for item in items) + '</ol>'


def ul(*items):
    return '<ul>' + ''.join('<li>' + item + '</li>' for item in items) + '</ul>'


def note(text):
    return '<p class="note">' + text + '</p>'


def stop(text):
    return '<p class="stop">' + text + '</p>'


def check(text):
    return '<p class="check"><b>通过标准：</b>' + text + '</p>'


def page(title, body):
    return title, '<h1>' + title + '</h1>' + body


def setting(group, aim):
    return ('<p><b>打开 Group：</b>' + group + '</p>'
            '<p class="note"><b>本批只做：</b>' + aim + '</p>')


def pdf(filename, pages):
    doc = fitz.open()
    toc = []
    for title, content in pages:
        story = fitz.Story('<html><body>' + content + '</body></html>',
                           user_css=CSS, archive=ARCHIVE)
        part = story.write_with_links(
            lambda number, filled: (fitz.Rect(0, 0, 595, 842), fitz.Rect(42, 42, 553, 787), None))
        toc.append([1, title, len(doc) + 1])
        doc.insert_pdf(part)
        part.close()
    doc.set_toc(toc)
    for i, p in enumerate(doc):
        p.draw_line((42, 801), (553, 801), color=(.7, .78, .82), width=.5)
        p.insert_text((42, 817), 'RM / ' + filename.split('_')[0] + ' / ' + str(i + 1) +
                      ' of ' + str(len(doc)), fontsize=8, color=(.25, .35, .4))
    doc.set_metadata(dict(title=filename.removesuffix('.pdf'), author='工程车电控协作',
                          subject='2026-11-13：单Debug整车固件；0档独立底盘直驱；直接写调参目标；不换固件'))
    doc.subset_fonts()
    path = ROOT / filename
    data = doc.tobytes(garbage=4, deflate=True)
    # PDF 查看器常会占用目标文件；先写同目录临时文件，再原子替换。
    import os
    import tempfile
    handle, temp_name = tempfile.mkstemp(dir=path.parent, prefix=path.stem + '.', suffix='.tmp')
    os.close(handle)
    temporary = Path(temp_name)
    try:
        temporary.write_bytes(data)
        os.replace(temporary, path)
    except PermissionError:
        temporary.unlink(missing_ok=True)
        preview_dir = ROOT.parent / 'build' / 'fm_pdf_preview'
        preview_dir.mkdir(parents=True, exist_ok=True)
        preview = preview_dir / filename
        preview.write_bytes(data)
        print(filename + '：目标被查看器占用；已生成预览 ' + str(preview))
        print('关闭查看器后重新运行本脚本即可原位覆盖。')
        doc.close()
        return
    print(filename + ': ' + str(len(doc)) + ' pages')
    doc.close()


STEPS = [
    page('先看这一页', '''
<p class="note"><b>只烧一份 Debug 整车固件。</b>底盘、Yaw、机械臂共用同一份固件；三份 FreeMASTER 工程只是不同的观察页。烧录后加载 <code>build/Debug/dx_assignment5_freertos.elf</code>，板内程序必须和 ELF 同次构建。</p>
''' + table(['工程', '用途'], [
        ('底盘调参.pmpx', '按第0～12批逐批调底盘和Yaw。'),
        ('底盘排障.pmpx', '出现异常、需要看保护位图/计数器时用。'),
        ('机械臂简易调参与排障.pmpx', '抬臂、左下保持、简单的机械臂排障。')]) + '''
<h2>四条原则</h2>
''' + ol(
        '<b>先看再给：</b>先确认心跳、反馈和在线，再写目标。',
        '<b>先小后大：</b>先小目标确认方向和停止，再进闭环、再加大。',
        '<b>异常先停：</b>目标写0或切回0档；全局关控用 <code>g_robot_debug_command=1</code>。',
        '<b>在线值只在RAM：</b>FreeMASTER 改的参数重启就丢，确认后写回源码。') + '''
<p class="stop"><b>左上是行驶，不是关控；左下是待机保持，不是撤力。</b>机械臂没有承托时不要做关控、校零或大电流测试。</p>
<p class="note"><b>当前0档是独立底盘直驱：</b>左拨杆上、遥控在线、五通道回中保持300ms后，左竖/右横直接驱动底盘；右拨杆位置不参与解锁。左横在解锁后还可直控Yaw（需Yaw已在线/校零/使能；命令5只使能Yaw、6只失能Yaw），使用线束软限位，不用正常行驶姿态包络；不要求机械臂 active、IMU 或功率。上电、关控、翻车、失联、离开上档、功率锁存都会撤销解锁，重新运动要重做回中。改回 <code>kChassisIndependent=false</code> 才恢复整车互锁路径。</p>
'''),
    page('路线表：按第0～12批走', '''
''' + table(['批次 / Group', '档位', '本批目标', '做完得到什么'], [
        ('第0批 安全常驻', '0', '不接动力，测命令1、遥控失联、零输出。', '关控和失联路径可信。'),
        ('第1批 遥控与两轮反馈', '0', '认清通道、拨杆、轮子ID和向前正方向。', '输入映射和轮反馈对得上。'),
        ('第2批 左轮ID1电流方向', '1', '左轮小电流，右轮不出力。', '左轮方向正确，500ms能停。'),
        ('第3批 右轮ID2电流方向', '2', '右轮重复正反方向。', '右轮方向正确。'),
        ('第4批 两轮速度PI', '3 / 4', '先P后I，左3档、右4档。', '两轮速度闭环可用。'),
        ('第5批 0档抬臂左下保持', '0', '正常0档抬臂，待机保持，再测轮。', '抬起的机械臂在调轮时不下垂。'),
        ('第6批 Yaw线束中位与软限位', '5 / 6', '填写固定零点或人工中位校零，申请小半幅。', 'Yaw有可信的零位和范围。'),
        ('第7批 Yaw速度内环PI', '5', '小角速度正负跟踪，验证边界减速。', 'Yaw速度环稳定。'),
        ('第8批 IMU功率与真实整车许可', '0', '确认真实IMU、功率和互锁。', '正常0档有真实放行条件。'),
        ('第9批 Yaw位置双环', '6', '小角度位置目标，调位置P。', 'Yaw位置环可用。'),
        ('第10批 正常遥控慢速落地', '0', '悬空确认后小步落地。', '方向、松杆停车、转向正确。'),
        ('第11批 停止清错与故障恢复', '0 / 退出', '分别测退出、关控、清错、功率停车。', '故障不续跑，停车距离可接受。'),
        ('第12批 固化参数与重启复验', '0', '写回源码，重烧后复验。', '重启后行为一致。')]) + '''
<p>第5批要用正常遥控抬臂。<b>第8批的真实条件还没做好，就先跳过第5批</b>，继续独立调轮和Yaw。不要为了提前抬臂伪造IMU、功率或互锁许可。</p>
<p class="note">当前 <code>kChassisIndependent=true</code>：0档底盘直驱不要求第8批的整车条件；第8批只影响正常整车/机械臂路径。底盘方向、回中解锁、松杆停车和功率锁存仍要逐项验证。Yaw直控另见“独立底盘Yaw直控”页；命令5/6只使能/失能Yaw。</p>
''' + table(['总线 / 对象', '配置'], [
        ('CAN2 左/右 M3508', 'ID1/ID2，反馈0x201/0x202，指令0x200第一/二槽。'),
        ('CAN1 Yaw GM6020', '电压模式，ID1，反馈0x205，指令0x1FF第一槽。'),
        ('CAN1 机械臂', '两台DM、roll、夹爪；CAN1只有一个发送owner。')]) + '''
<p class="small">不要新增第二个Yaw驱动或第二个0x1FF发送者。</p>
'''),
    page('连接一次，之后只选批次', '''
<h2>连接</h2>
''' + ol(
        '两轮悬空或落地条件明确，机械臂可靠支撑；先烧录 Debug 整车固件。',
        '打开对应的 .pmpx，加载同次构建的 <code>build/Debug/dx_assignment5_freertos.elf</code>。',
        'Start Communication；<code>uwTick</code> 必须持续增加。不增加先查固件、ELF 和连接。',
        '重新编译后刷新符号并重开工程，不要继续用旧 ELF 的地址。') + '''
<h2>FreeMASTER 常用操作</h2>
''' + table(['动作', '效果'], [
        ('点左侧 Group', '只换页面，不会改变固件状态。'),
        ('双击变量 Value 并回车', '才真正写入变量。只读变量不要写。'),
        ('打开示波器', '先开图再给短目标，否则可能错过500ms过程。'),
        ('切换工程', '先 Stop Communication，一次只让一份工程通信。')]) + '''
<p class="note">调参页只放本批要写和要看的量。时间戳、计数器、保护位图和生效参数在“底盘排障.pmpx”里查；正常调环页不重复堆这些量。</p>
'''),
    page('每次调参的通用顺序', '''
<p>所有批次都按这个顺序，区别只是对象和单位。</p>
''' + ol(
        '<b>停：</b>本批目标写0；机构有支撑并停稳。异常先写 <code>g_robot_debug_command=1</code>。',
        '<b>选对象：</b>写 <code>robot_tune_mode</code>。切档会自动清旧目标。',
        '<b>写参数：</b>改 P/I、限流或半幅；没有单独的 enable 变量。',
        '<b>开始：</b>电流/速度档直接写非零目标，位置档写目标角；看 <code>ready=1</code>、<code>blocked=0</code>。目标保持0时未开始是正常的。',
        '<b>停：</b>目标写0；或 <code>robot_tune_mode=0</code> 退出。') + '''
<p class="note">当前代码没有 <code>tune_enable</code>，也不再要求“左下右中、5通道回中300ms、停稳”才进入调参。但0档独立底盘直驱仍要求回中300ms解锁；机械臂必须支撑、轮子必须悬空或落地条件明确，这是安全要求。</p>
<p class="stop"><b>电流档非零最多500ms；速度档和位置档没有自动结束。</b>速度/位置测试必须自己写0或退出。故障、失联后不会自动续跑，恢复后要重新选档并重新写目标。</p>
'''),
    page('第0批 安全常驻', setting('第0批 安全常驻', '不接电机动力，验证命令1、遥控失联和零输出。') + '''
<h2>步骤</h2>
''' + ol(
        '电机动力断开；两轮悬空，机械臂支撑。<code>robot_tune_mode=0</code>，电流/速度目标0。',
        '记下 <code>g_robot_status.stop_epoch</code>，写 <code>g_robot_debug_command=1</code>；命令应自动回0。',
        '<code>stop_epoch</code> 加1；<code>protection_latched_reasons</code> 含16这一位；<code>enabled_mask=0</code>；<code>g_chassis_status.enabled=0</code>；两轮 <code>current_ma=0</code>。',
        '开遥控：<code>good_frames</code> 增长、<code>valid=1</code>。关遥控：<code>good_frames</code> 停止、<code>remote_age_ms</code> 增大、<code>remote_online=0</code>、保护含4；所有轴失能、电流0。',
        '重开遥控：在线恢复，但<b>不能自动出力</b>。') + check(
        '命令被取走、关控次数加1、失联被识别、恢复后不自动跑。') + '''
''' + table(['现象', '先查'], [
        ('心跳不更新', '固件/ELF是否对应、任务是否还在跑。'),
        ('命令1不归零', '控制任务是否在消费命令；不要反复写。'),
        ('关遥控后 good_frames 还在涨', '接收机是否在重复发旧合法帧；停止后续运动测试。')]) + '''
<p class="small">没有电机动力时出现CAN/反馈超时是正常的；不要为了清故障关闭保护。</p>
'''),
    page('第1批 遥控与两轮反馈', setting('第1批 遥控与两轮反馈', '认清每个实物输入和两轮反馈，不出力。') + '''
<h2>遥控</h2>
''' + ol(
        '<code>robot_tune_mode=0</code>、目标0、命令1；遥控检查可先不给电机动力。',
        '一次只动一个摇杆：左竖、左横、右竖、右横；记录 <code>frame.channel[i]</code> 和 <code>remote_axis[i]</code> 的符号，然后回中。',
        '拨轮也回中；五个通道都在 ±30 内。',
        '左/右拨杆各拨三段；原码上1、中3、下2；记录 <code>remote_switch</code>。') + '''
''' + table(['当前暂定', '实物确认'], [
        ('左竖 ch3，左横 ch2', '右竖 ch1，右横 ch0。'),
        ('左拨杆 switches[0]', '右拨杆 switches[1]。'),
        ('摇杆中位约0，满杆约±660', '处理后统一到 [-1,1]，死区约6%。'),
        ('左拨杆：上行驶、中夹取、下待机', '左上不是关控。')]) + '''
<h2>两轮</h2>
''' + ol(
        '给轮子动力，仍悬空；<code>feedback_ms</code> 刷新，静止时 <code>measured_rpm≈0</code>。',
        '手转左轮向前：<code>measured_rpm[0]</code> 应为正；右轮 <code>[1]</code> 为正。',
        '反了改 <code>Chassis/chassis_config.h</code> 的 <code>direction</code>，重烧后复验。') + check(
        '每个输入只影响对应轴；向前反馈为正；命令1下两轮电流为0。') + '''
<p class="stop">全局停止用命令1，不是左上拨杆。左下是待机保持，不是撤力。</p>
'''),
    page('第2批 左轮ID1电流方向', setting('第2批 左轮ID1电流方向', '只让左轮出一个小电流，确认方向和500ms截止。') + '''
<h2>写入顺序</h2>
''' + table(['变量', '本次示例'], [
        (v('tune_wheel_current_limit_ma'), '100mA；目标最大±10000mA，输出限流受源码 max_current_ma 约束。'),
        (v('robot_tune_mode'), '1。切档会清旧目标。'),
        (v('tune_current_ma'), '+50mA；非零即开始本次测试，<b>最后写这一项</b>。')]) + '''
<h2>步骤</h2>
''' + ol(
        '两轮悬空；先 <code>robot_tune_mode=0</code>、目标0，等轮停。',
        '写限流100、mode1；确认目标0；此时 ready=0 是正常的（还没给非零目标）。',
        '最后写 +50；非零电流开始500ms计时；看 <code>current_ma[0]</code> 为正、<code>current_ma[1]=0</code>、<code>measured_rpm[0]</code> 方向与实物向前一致。',
        '500ms 到期自动归零并锁禁；要停就先把 <code>tune_current_ma</code> 写0。',
        '反方向：目标0 → mode0 → 等停；再 mode1、写 −50。') + check(
        '只有左轮出力、方向正确、500ms能停、mode0能退出。') + '''
''' + table(['现象', '处理'], [
        ('小电流下不动', '可能静摩擦；先核对反馈、方向和协议，再小步加。'),
        ('电流一给就飞/方向反', '立即命令1；查ID、direction和电调模式，不要进速度闭环。'),
        ('写了非零目标但 ready=0', '看 reason/blocked；本次没放行，不要继续加电流。')]) + '''
<p class="small"><code>current_ma</code> 是最终入队的提交值，不是电机执行确认。</p>
'''),
    page('第3批 右轮ID2电流方向', setting('第3批 右轮ID2电流方向', '用同样的小电流确认右轮，不是让两轮一起转。') + '''
''' + ol(
        '左轮测试结束：目标0、mode0，等两轮停稳。',
        '限流保留100；<code>robot_tune_mode=2</code>，确认目标0。',
        '最后写 <code>tune_current_ma=+50</code>；非零电流开始500ms计时。',
        '应只有 <code>current_ma[1]</code> 非零，<code>current_ma[0]=0</code>。',
        '看 <code>measured_rpm[1]</code> 与实物向前一致；500ms后归零。',
        '反方向：目标0、mode0、等停；再 mode2、写 −50。') + check(
        '右轮单独正反方向正确；左轮始终零输出；超时能停。') + '''
<p class="note">功率未确认时独立底盘0档不经过功率环，只受 max_current_ma 和速度斜坡约束；这不代表可以落地行驶。调轮仍应使用小电流、悬空条件。</p>
'''),
    page('第4批 两轮速度PI', setting('第4批 两轮速度PI', '先P后I；先左轮3档，再右轮4档。') + '''
<h2>起步参数</h2>
''' + table(['变量', '本次示例'], [
        (v('tune_wheel_kp'), '1.0 mA/rpm；源码占位，不是最终答案。'),
        (v('tune_wheel_ki'), '0，先关积分。'),
        (v('tune_wheel_current_limit_ma'), '100mA；目标最大±10000mA，输出限流受源码 max_current_ma 约束。'),
        (v('robot_tune_mode'), '3（左）或4（右）。'),
        (v('tune_speed_rpm'), '+50，<b>3508转子rpm</b>。')]) + '''
<h2>步骤</h2>
''' + ol(
        '两轮悬空；停目标；mode3；写 +50；看 <code>ready=1</code>、<code>blocked=0</code>。',
        '看 <code>target_rpm</code>、<code>measured_rpm</code>、<code>current_ma</code>：方向对、实际跟随、右轮电流0。',
        '目标0，等停稳；小步改P；再写 +50 重复。',
        'P稳定后再少量加I；出现振荡/回摆先减I。',
        '右轮切 mode4，重复同样流程。') + check(
        '±50～±100稳定、写0能停、另一轮不出力；记录P/I/限流和曲线。') + '''
<p class="stop">速度档是持续控制，没有500ms自动结束。必须把 <code>tune_speed_rpm</code> 写0或回mode0；关闭FreeMASTER不会停。</p>
''' + table(['现象', '本次只改什么'], [
        ('方向对、电流未顶限、跟得慢', 'P小步加，例如1→1.2。'),
        ('电流高频跳/啸叫', '先减P，查反馈和机械松动，不加I。'),
        ('电流顶限还跟不上', '查摩擦、负载和测试限流；加P/I不会增加允许电流。'),
        ('稳定但有固定静差', 'I从0少量加，例如0.05。'),
        ('回零反复过冲', '先减I。')]) + '''
<p class="small">本批只验证悬空小速度，不代表落地负载、功率制动或坡道停车。</p>
'''),
    page('第5批 0档抬臂左下保持', setting('第5批 0档抬臂左下保持', '正常0档抬臂，左下保持，再测轮；不自动使能其他轴。') + '''
<p class="stop"><b>先决条件：</b>真实IMU、功率、互锁、五轴在线且已校零、Yaw人工中位和在线半幅、实物配置确认。没做好就先跳过本批，继续独立调轮/Yaw；不要伪造许可。</p>
<h2>抬起</h2>
''' + ol(
        '<code>robot_tune_mode=0</code>、目标0；两轮悬空，机械臂支撑。',
        '正常模式：底盘停稳、四摇杆和右拨杆回中；确认 <code>wait_neutral=0</code>；写 <code>g_robot_debug_command=4</code>；<code>active=1</code>、<code>enabled_mask=31</code>。',
        '左竖轻推抬肩，回中；右竖轻推抬肘，回中。每次只动一个输入。',
        '看肩/肘 <code>target_deg</code>、<code>position_deg</code>、<code>effort</code>；参考轨迹可以比目标慢。',
        '到安全姿态，左拨杆下待机；确认 <code>mode=3</code>、<code>active=1</code>、<code>enabled_mask=31</code>，角度保持。') + '''
<h2>测轮</h2>
''' + ol(
        '用第2～4批；已使能机械臂继续保持，未使能轴不会自动开。',
        '退出调轮：目标0、mode0；机械臂继续保持。不要用命令1代替日常退出。') + check(
        '抬臂平稳、待机几秒不下垂、切轮调参时机械臂不掉。') + '''
<p class="stop">两DM当前各0.3Nm只是启动上限，不保证抬得起实物。effort顶限还下垂时先承托，与队友核对负载/力矩，不要猛加I。命令1会撤掉保持。</p>
'''),
    page('第6批 Yaw线束中位与软限位', setting('第6批 Yaw线束中位与软限位', '填写固定单圈零点或人工找中位，申请一个已确认的小半幅。') + '''
<h2>固定零点（推荐）</h2>
''' + ol(
        '支撑机械臂；<code>robot_tune_mode=0</code>、目标0；命令1；确认 <code>enabled_mask=0</code>。',
        '失能状态下把车头和Yaw对正；读 <code>g_arm_status.joint[0].raw_encoder</code>（0..8191），不要凭空填0。',
        '把该整数写入 <code>Arm/yaw_config.h</code> 的 <code>kYawZeroEncoder</code>，重编Debug并烧录，刷新FM符号。',
        '首次反馈后 <code>fixed_zero_encoder</code> 显示该值、<code>calibrated=1</code>；命令2不会覆盖固定Yaw零点。') + '''
<h2>人工校零（kYawZeroEncoder=-1时）</h2>
''' + ol(
        '手转Yaw很小角度，看 <code>joint[0].position_deg</code> 的方向；找自然、未预拧的中位，在底盘和Yaw外圈做标记，停稳。',
        '写 <code>robot_tune_mode=5</code> 或6；写命令2（只校Yaw）；看 <code>last_result=0</code>、<code>joint[0].calibrated=1</code>、<code>position_deg≈0</code>。') + '''
<h2>半幅</h2>
''' + ol(
        '直接写 <code>tune_yaw_half_range_deg</code>（按线束实测，例如10）；看 <code>g_yaw_limit_status.result=0</code>、<code>active_half_range_deg</code>、<code>min/max</code>；实际角、目标、参考都要留2°余量。',
        '被拒绝：改好条件后重新写一个<b>不同的</b>已确认值；拒绝不会排队。') + check(
        '零点可重复、半幅生效、边界值正确。') + '''
<p class="note"><b>固定零点写在源码，重启/重烧后仍有效；人工校零只存RAM。</b>mode0的命令2校全部五轴（固定Yaw跳过），mode5/6的命令2只校Yaw。无滑环，不能转整圈找零。</p>
<p class="small">软件接受半幅2°～90°只是防误填边界，不是线束能承受90°的证明。</p>
'''),
    page('第7批 Yaw速度内环PI', setting('第7批 Yaw速度内环PI', '先P后I，验证小角速度和边界减速。') + '''
<h2>起步参数</h2>
''' + table(['变量', '本次示例'], [
        (v('tune_yaw_velocity_kp'), '20；源码占位，不是最终答案。'),
        (v('tune_yaw_velocity_ki'), '0，先关积分。'),
        (v('tune_yaw_voltage_limit_raw'), '25000 raw；Yaw调参电压原码上限25000。'),
        (v('robot_tune_mode'), '5。'),
        (v('tune_yaw_speed_deg_s'), '+1°/s，<b>最后写</b>；非零即开始。')]) + '''
<h2>步骤</h2>
''' + ol(
        '两轮悬空、机械臂支撑；停目标；mode5；写 +1；看 <code>ready=1</code>、<code>blocked=0</code>。',
        '看 <code>velocity_target_deg_s</code>、<code>velocity_deg_s</code>、<code>effort</code>；短时后写0，等停，再写 −1。',
        '改P/I：目标0、等停、改一项；再写 ±1 重复。',
        '最后用很小速度靠近软限位，看速度目标提前减小；不要撞限位。',
        '两轮 <code>current_ma</code> 应为0。') + check(
        '正负跟随、写0能停、边界前减速、底盘不出力。') + '''
<p class="stop">5档速度是持续命令，没有500ms自动结束。写了非零就必须自己写0；不要关掉FreeMASTER等它停。</p>
''' + table(['现象', '本次只改什么'], [
        ('方向对、未顶限、跟得慢', 'P小步加，例如20→24。'),
        ('输出快跳/啸叫', '先减P，查反馈连续性和机构松动。'),
        ('顶限还跟不上', '查摩擦、负载和电压输出上限，不能硬加P。'),
        ('有重复静差', 'I少量加，例如0.2；不同时改P。'),
        ('回零反摆', '先减I。')]) + '''
'''),
    page('独立底盘Yaw直控（比赛）', setting('0档底盘直驱｜左横Yaw', '解锁后左横直接控制Yaw；不经过正常行驶姿态包络。') + '''
<p class="note">前提：<code>kChassisIndependent=true</code>、<code>robot_tune_mode=0</code>、左拨杆上、遥控在线、五通道回中保持300ms，<code>independent_armed=1</code>；右拨杆位置不参与解锁。Yaw 必须在线、已校零、无故障。</p>
<h2>使能</h2>
''' + ol(
        '支撑机械臂，确认Yaw线束中位和已生效半幅；<code>g_yaw_limit_status.ready=1</code>。',
        '写 <code>g_robot_debug_command=5</code>：只使能Yaw，不动其他关节；确认 <code>joint[0].enabled=1</code>。',
        '若 <code>homing=1</code>：固定零点轴先回零，等 <code>position_deg≈0</code>、<code>homing=0</code>；回零期间左横不推进目标。',
        '需要停Yaw：写命令6只失能Yaw；命令1会全局关控。') + '''
<h2>直控</h2>
''' + ol(
        '左拨杆上、五通道回中，等 <code>independent_armed=1</code>、<code>yaw_direct_active=1</code>、<code>homing=0</code>；右拨杆位置不参与解锁。',
        '左横轻推，<code>yaw_direct_target_deg</code> 从当前角按限位速度积分；看 <code>joint[0].position_deg</code>、<code>velocity_deg_s</code>、<code>effort</code>。',
        '松开左横保持当前目标，不回中；目标只能在 <code>g_yaw_limit_status.min_deg/max_deg</code> 内。',
        '离开上档、关控、翻车、失联、功率停车都会撤销直控，Yaw按统一策略失能（通常不主动回中）；恢复要重新回中解锁并使能Yaw。') + check(
        '只动Yaw、其他关节不动；松开保持；边界前减速；关控/离档立即撤销。') + '''
<p class="stop">这条路径使用线束软限位，不是正常行驶姿态包络，也不是碰撞保护；先确认整臂和线束不会干涉。</p>
<p class="small">Yaw未使能时底盘照常驱动，Yaw不动；不要把“底盘在动”当作Yaw已受控。</p>
'''),
    page('第8批 IMU功率与真实整车许可', setting('第8批 IMU功率与真实整车许可', '确认真实IMU、功率和整车许可，不驱动。') + '''
<h2>IMU</h2>
''' + ol(
        '整车静置、所有轴失能、机械臂支撑；看 <code>g_ins_status</code> 时间戳、<code>valid=1</code>、<code>calibrated=1</code>、<code>faults=0</code>、<code>tilt_deg≈0</code>。',
        '需要重标定：写 <code>g_ins_debug_command=1</code>；标定期间不要晃车，等 <code>calibrated=1</code>。',
        '有支撑时缓慢改变姿态，确认 <code>tipped</code> 进入/退出；恢复后不会自动使能。') + '''
<h2>功率与整车</h2>
''' + ol(
        '功率计：<code>power_frames</code> 增长、<code>power_received_ms</code> 刷新、<code>power_w/energy_j</code> 合理。',
        '实测帧周期和低能量/超时阈值后，再把源码 <code>ChassisPowerConfig.confirmed</code> 改真并重编；<code>power_confirmed</code> 是只读状态。',
        '看 <code>g_vehicle_health.reasons=0</code>；非0按表查真实条件，不要写只读许可。') + check(
        'IMU有效且已校准、功率数据新鲜、整车 reasons=0、所有轴失能。') + '''
<p class="note">当前 <code>kChassisIndependent=true</code> 时，0档底盘直驱不使用本页的整车互锁/功率许可；本页用于改回正常整车路径前的条件核对和机械臂开控。IMU 翻车仍会否决独立底盘。</p>
''' + table(['整车 reasons 位', '优先检查'], [
        ('1', '姿态/配置未确认，Yaw在线范围等。'),
        ('2 / 4', '底盘/机械臂快照超时。'),
        ('8 / 16', 'IMU无效/翻车。'),
        ('32', '轮故障或轮反馈不新鲜。'),
        ('64 / 128', '功率/能量禁止，或机械臂故障。')]) + '''
<p class="small">默认 <code>confirmed=false</code> 是有意保留的实测门槛；不是在FreeMASTER把许可写1就能放行。</p>
'''),
    page('第9批 Yaw位置双环', setting('第9批 Yaw位置双环', '速度环稳定后，再调位置P；小角度开始。') + '''
<h2>起步参数</h2>
''' + table(['变量', '本次示例'], [
        (v('tune_yaw_position_kp'), '2；先用第7批已稳定的速度环。'),
        (v('tune_yaw_voltage_limit_raw'), '25000 raw。'),
        (v('robot_tune_mode'), '6。'),
        (v('tune_yaw_target_deg'), '+1°，<b>最后写</b>。')]) + '''
<h2>步骤</h2>
''' + ol(
        '两轮悬空、机械臂支撑；停目标；mode6；选档后先保持在当前角，看 <code>ready=1</code>、<code>blocked=0</code>。',
        'mode6会把 <code>tune_yaw_target_deg</code> 对齐当前角，不是自动回0；确认没有跳变。',
        '写 +1；看 <code>target_deg</code>、<code>reference_deg</code>、<code>position_deg</code>、<code>velocity_target_deg_s</code>、<code>effort</code>。',
        '写回当前角/0，等稳；再写 −1。',
        '改位置P：目标靠近当前、停稳、改一项；再给小目标。') + check(
        '平稳接近、不过冲、目标始终在生效半幅内。') + '''
<p class="note">位置档会持续保持目标，没有自动到期；退出用mode0。目标靠边被约束是正常限位，不要靠扩大半幅解决。</p>
'''),
    page('第10批 正常遥控慢速落地', setting('第10批 正常遥控慢速落地', '先悬空确认正常链，再小步落地；不测快速档。') + '''
<h2>恢复正常控制</h2>
<p class="note">当前 <code>kChassisIndependent=true</code>：0档底盘直驱不需要命令4或机械臂 active；左拨杆上、遥控在线、五通道回中保持300ms后，左竖/右横直接驱动；右拨杆位置不参与解锁。左横在解锁后还可直控Yaw（命令5/6只使能/失能Yaw），使用线束软限位。下面第1～2步仅在改回正常整车路径后执行；仍先悬空确认方向和松杆停车，再落地。</p>
''' + ol(
        '所有调参目标0、<code>robot_tune_mode=0</code>；机械臂保持，两轮停稳。',
        '第8批真实条件就绪；左中夹取、右中，摇杆回中，<code>wait_neutral=0</code>，写命令4；确认 <code>active=1</code>、<code>enabled_mask=31</code>。',
        '机械臂移到允许行驶的收拢姿态；左上行驶、右中；确认真实互锁到位。') + '''
<h2>悬空确认</h2>
''' + ol(
        '左竖：两轮规范速度同号；松杆回0。',
        '右横：差速转向，右推应顺时针；左横独立调Yaw，不要混淆。') + '''
<h2>落地</h2>
''' + ol(
        '空载、平地、留出停车距离；用已验证的保守限流和慢速上限。',
        '小前进→松杆→测停止距离；再小后退。',
        '小转弯、原地转向；倒车时右推仍应顺时针。',
        '行驶切夹取/待机，必须先受控停车再放行机械臂动作。') + check(
        '方向对、松杆能停、转向对、不进入快速档。') + '''
<p class="note">当前底盘只有慢速参数；<code>fast_requested</code> 不会让车变快。左上不是关控。</p>
<p class="small"><code>wheels_stationary=1</code> 只表示轮速低，不能证明车身没有滑动。</p>
'''),
    page('第11批 停止清错与故障恢复', setting('第11批 停止清错与故障恢复', '分别验证退出、全局关控、清错和功率停车。') + '''
''' + table(['试验', '应该看到'], [
        ('轮调参中目标0/mode0', '轮停；已使能机械臂仍保持。'),
        ('6档退出', '原已使能轴保持；只由调参新增的Yaw使能被撤销。'),
        ('低输出时命令1', '所有轴失能、轮电流0；机械臂先支撑。'),
        ('关遥控/反馈丢失', '取消出力、不自动续跑；恢复后重新选档/写目标。'),
        ('功率受限停车', '按低能量/反馈超时停车；记录距离。')]) + '''
<h2>清错</h2>
''' + ol(
        'Arm/Yaw：承托、停机、排除原因后写命令3。',
        '底盘/功率：<code>g_chassis_debug_request.mode=1</code>，最后把 <code>request_seq</code> 加1；看结果。',
        '清错不自动使能；反馈丢失可能让零位失效，要重新安全校零。') + '''
<h2>功率停车</h2>
''' + ol(
        '在正常0档、保守低速、可控环境里验证低能量和功率反馈超时。',
        '记录停车距离和电流；推进限流不能把反向制动也削没。',
        '断线后不承诺闭环刹停；零电流也不等于机械锁止。') + check(
        '退出/关控行为清楚、故障不续跑、停车距离满足现场要求。') + '''
<p class="stop">清错不是开控。清完要按正常0档步骤重新请求；不要用命令1代替日常退出。</p>
'''),
    page('第12批 固化参数与重启复验', setting('第12批 固化参数与重启复验', '把实测值写回源码，重烧后从安全步骤复验。') + '''
''' + table(['记录什么', '写回哪里'], [
        ('左右ID、方向、轮径/轮距、总比', 'Chassis/chassis_config.h'),
        ('轮速P/I、正常限流、速度/斜坡', 'Chassis/chassis_config.h'),
        ('Yaw方向、位置P、速度P/I、上限', 'Arm/yaw_config.h'),
        ('Yaw线束半幅/制动余量', 'Yaw安全配置 + 每次上电在线申请'),
        ('IMU、功率、可行驶姿态', 'Imu/Power/Vehicle配置'),
        ('四轴关节参数', 'Arm/arm_config.cpp（队友核对）')]) + '''
<h2>复验</h2>
''' + ol(
        '停目标、mode0；承托机械臂；命令1；断动力。',
        '写回源码，重编译Debug并烧录；刷新FreeMASTER符号，重开工程。',
        '重做心跳、命令1、遥控失联；核对 <code>kYawZeroEncoder</code> 或重新校零、申请Yaw半幅、等IMU标定。',
        '先低输出悬空复验，再正常慢速落地；确认重启没有旧目标自动续跑。') + check(
        '重启后行为一致、参数来自源码、没有自动续跑。') + '''
<p class="note">FreeMASTER在线修改只在RAM，保存.pmpx不等于写Flash。记录源码版本、生效参数、负载、方向、曲线和停车距离。</p>
''')]


FAULTS = [
    page('出问题先按这张表停下来', '''
<p class="note">先判断：是要保留机械臂保持，还是必须全局关控。不要靠乱加P或限流试运气。</p>
''' + table(['你现在遇到什么', '先做', '打开 Group'], [
        ('轮子异常运动/方向反', '目标0或mode0；危险时命令1，先承托机械臂。', '排查0、3、4'),
        ('不想调轮了但机械臂还要保持', '目标0、mode0；不要用命令1代替退出。', '排查0'),
        ('Yaw异常/线束受拉', '立即命令1并承托；检查中位和实际角。', '排查6、7'),
        ('所有变量不更新', '先停运动/动力；核对固件和ELF，再查任务。', '排查0'),
        ('ready=0、blocked=1', '看 reason；排除原因后重新选档/写目标。', '排查0、3、7'),
        ('机械臂0档不能开控', '查真实整车条件；不能借调参档伪造放行。独立底盘直驱不看这些条件。', '排查5、8')]) + '''
<p class="stop">左上是行驶，不是关控；左下是保持，不是撤力。命令1会撤掉机械臂保持。关掉FreeMASTER通信不会停止速度档。</p>
<p class="small"><code>current_ma</code> 是提交值，<code>effort</code> 是控制输出，都不是电机执行确认。时间戳和反馈年龄必须持续更新。</p>
'''),
    page('遥控完全收不到：先查接收链', '''
<p>打开“底盘排障.pmpx”的“排查1 遥控不在线或回中失败”。电机动力可隔离，mode0、目标0，程序持续运行。</p>
''' + table(['按顺序看', '怎么判断'], [
        ('uwTick', '必须增加。停住先查固件/ELF/任务，不要先怀疑接线。'),
        ('start_calls', '正常至少1；为0说明接收初始化没执行。'),
        ('service_ms', '应随uwTick更新；停住说明接收维护所在任务没跑。'),
        ('last_start_result / rx_running', '0成功、1错误、2忙、3超时；rx_running=1只说明UART+DMA已启动。'),
        ('rx_events / rx_bytes', '持续增加说明回调收到字节；都为0继续看DMA。'),
        ('dma_remaining', '初始72。一直是72：查接收机上电、对频、PC11接线和共地；减少但回调0：查中断。'),
        ('last_size / rejected_frames', '合法长度18/36/54/72；其他长度或内容非法会拒收。'),
        ('uart_errors / last_uart_error', '位：1校验、2噪声、4帧格式、8溢出、16DMA。查100000波特率、9B+EVEN、接线。'),
        ('good_frames', '持续增加后再看原始通道和映射。')]) + '''
<p class="small">软件修复不等于实机接收已恢复；把上面的计数一起记录再定位。</p>
'''),
    page('排查0：心跳、调参原因和保护', '''
<h2>先看时间戳</h2>
<p><code>uwTick</code>、<code>g_robot_status.updated_ms</code>、<code>g_chassis_status.updated_ms</code>、<code>g_arm_status.updated_ms</code> 必须更新。静态mode0时调参快照不一定持续产生，不能只靠它判任务死活。</p>
<h2>g_robot_tune_status.reason</h2>
''' + table(['值', '含义', '先做什么'], [
        ('0', '正常/无调参', '看mode是否已退出。'),
        ('1', '调参就绪', '核对ready=1、blocked=0。'),
        ('2', '未就绪', '查所选对象反馈/配置/故障；Yaw档还需遥控mapping_confirmed，轮档不要求。'),
        ('3', '关控/严重中断', '先承托，排除失联/故障；回0再选。'),
        ('4', '模式/目标非法', '查mode 0～6、有限数值、本档范围。'),
        ('5', '电流超时', '非零电流到500ms；目标0、mode0、停稳后再选。'),
        ('6', '保留', '当前代码不再用它表示“未停稳”。'),
        ('7', '目标为0/未开始', '选档后目标仍为0；写非零目标才开始。')]) + '''
<h2>保护和清错</h2>
''' + table(['位', '含义'], [
        ('1 / 2', '启动保护 / 配置未确认或非法'),
        ('4 / 8', '遥控失联 / 互锁过期'),
        ('16 / 32', '显式全局关控 / 控制周期异常'),
        ('64 / 128', '机械臂故障 / 开控被拒'),
        ('256 / 512', '底盘故障 / IMU故障'),
        ('1024 / 2048 / 4096', '倾覆 / 整车状态 / 功率保护')]) + '''
<p><code>protection_current_reasons</code> 是当前原因，<code>protection_latched_reasons</code> 是历史锁存；位值可相加。历史非零不代表现在还在。</p>
<p><b>清错：</b>Arm/Yaw 用命令3；底盘用 <code>g_chassis_debug_request.mode=1</code>，最后 <code>request_seq+1</code>。清错不使能。</p>
'''),
    page('排查2/4：CAN 与两轮不转', '''
<h2>按顺序查</h2>
''' + ol(
        '<b>供电和身份：</b>CAN2左ID1、右ID2，反馈0x201/0x202，指令0x200第一/二槽；CAN1同号ID不冲突。',
        '<b>反馈新鲜：</b><code>feedback_frames</code> 增长、<code>feedback_ms</code> 刷新；手转确认左右下标没串。',
        '<b>总线：</b><code>can_tx_error_count[1]</code>、<code>rx_dropped</code> 是否增加；查波特率、接线、终端电阻和供电。',
        '<b>许可：</b>调参看 <code>ready=1</code>、<code>blocked=0</code>、<code>tune_blocked=0</code>、<code>faults=0</code>。',
        '<b>最后看PI：</b>目标、实际、电流同图比较；方向错先停，不加P。') + '''
''' + table(['现象', '处理'], [
        ('顶层ready=1但底盘不出力', '底盘有本地否决；查tune_blocked、tune_result、faults和功率。'),
        ('非所选轮有电流', '停止，核对mode、ID槽位、符号和是否有第二个发送者。'),
        ('有电流但很小不动', '可能摩擦，也可能协议/供电；先看实物反馈再小步加。'),
        ('电流一给就飞/负反馈', '立即关控；核对direction和目标/反馈符号，禁止进闭环。'),
        ('measured_rpm很大但车慢', '这是3508转子rpm；总比约19，不是轮胎rpm。')]) + '''
<p class="small">CAN发送异常会撤输出并清旧帧；物理断线不能保证控制帧送达，零输出也不等于机械抱死。</p>
'''),
    page('排查3：调轮参数与机械臂保持', '''
<h2>参数看实际生效值</h2>
<p><code>tune_wheel_kp/ki/current_limit_ma</code> 是输入；在 mode1～4 中合法值直接生效，不要求目标归零或停稳。看 <code>g_chassis_status.tune_result</code> 和实际 <code>g_chassis_tune_status.velocity_kp/ki/max_current_ma</code>。</p>
''' + table(['结果', '含义'], [
        ('0', '待机/未进入轮组调参。'),
        ('1', '已生效；成功也要核对实际值。'),
        ('2', '参数非法：P>0、I>=0、限流>0且不超过源码 max_current_ma。'),
        ('3', '忙：当前周期在切换/提交，等下一周期再看。'),
        ('4', '反馈/配置/源时间/本地健康未就绪。')]) + '''
<h2>机械臂不应因调轮自动使能</h2>
''' + ol(
        '调轮前 <code>enabled_mask=0</code>；切1～4后仍应是0。',
        '原本已抬起并保持的轴继续保持；保持轴必须反馈新鲜、已校零、无故障。',
        '轮子停但机械臂突然软：查是否写了命令1、遥控失联、关节掉线或保持失败。',
        '日常退出用目标0、mode0，不要用命令1代替退出。') + '''
<p class="small">两DM各0.3Nm只是起步上限；抬不起先承托，核对负载/机构/力矩，不靠增大I硬顶。</p>
'''),
    page('排查5/8：功率与整车为什么不放行', '''
<h2>功率阶段</h2>
''' + table(['power_phase', '含义'], [
        ('0 / 1 / 2', '未放行 / 就绪 / 行驶'),
        ('3', '低能量受控停车'),
        ('4', '功率反馈失联受控停车'),
        ('5', '禁行锁存；需恢复条件、停稳并清错')]) + '''
<p><code>power_reason</code>：0无、1未确认、2配置非法、3没有功率计、4低能量、5反馈丢失、6停车超时、7已停、8外部关控中断。</p>
<h2>整车 reasons</h2>
''' + table(['位', '优先检查'], [
        ('1', '姿态/配置未确认，Yaw在线范围'),
        ('2 / 4', '底盘/机械臂快照超时'),
        ('8 / 16', 'IMU无效/翻车'),
        ('32', '轮故障或轮反馈不新鲜'),
        ('64 / 128', '功率/能量禁止，或机械臂故障')]) + '''
<p class="small">位值可相加，例如72=64+8。不能把许可写1解除；任务持续根据真实反馈生产互锁。功率默认未确认时独立底盘0档不经过功率环；调轮仍应小电流悬空。</p>
'''),
    page('排查6/7：Yaw 不动、限位被拒', '''
<h2>四个硬条件</h2>
''' + ol(
        'CAN1 GM6020电压模式反馈持续。',
        '真实线束中位已人工校零。',
        '<code>g_yaw_limit_status.ready=1</code> 且半幅真实生效。',
        '选5/6、核对参数；写非零目标前先看 ready/blocked。') + '''
<h2>结果码</h2>
<p>适用于 <code>g_yaw_limit_status.result</code>、<code>g_robot_tune_status.yaw_parameter_result</code>、<code>g_arm_status.last_result</code>；0才是成功。</p>
''' + table(['值', '含义/处理'], [
        ('0', '成功；同时核对active_half_range和实际P/I/上限。'),
        ('1 / 2', '参数非法 / 配置未确认。'),
        ('3', '忙：限位要全轴失能；参数先把速度写0或目标对齐当前角，再改。'),
        ('4 / 5', '不在线 / 没校零。'),
        ('6 / 7', '故障锁存 / 仍在动。'),
        ('8', '新范围排除了实际角、目标或参考；先安全回中。')]) + '''
<p><b>靠边变慢</b>是正常制动，不要加P或半幅突破。<b>目标写了没动</b>：先查切档是否清了目标，再看ready/blocked和实际参数。</p>
<p><b>独立底盘左横直控Yaw：</b>先确认 <code>independent_armed=1</code>、<code>yaw_direct_active=1</code>、<code>joint[0].enabled=1</code>；只使能/失能用命令5/6，命令1全局关控。Yaw未使能时底盘照常、Yaw不动。</p>
<p><b>固定零点：</b>失能对正车头，读 <code>joint[0].raw_encoder</code> 填入 <code>kYawZeroEncoder</code> 后重编；首次反馈 <code>calibrated=1</code>，命令2不覆盖。固定轴首次开控 <code>homing=1</code> 先回零，回零完成前不接受位置/遥控目标。</p>
<p class="small">mode0命令2校五轴，mode5/6只校Yaw；不要为消报警在任意姿态重新认零。</p>
'''),
    page('保护记录与清错：当前、历史分开看', '''
<p><code>g_robot_status.protection_current_reasons</code> 是当前原因；<code>protection_latched_reasons</code> 是历史锁存。位值可相加，历史非零不等于现在所有原因都存在。</p>
<p><b>底盘 faults：</b>1配置、2反馈超时、4传输、8周期、16命令超时、32温度、64能量。复合值按位拆开。</p>
<h2>两条清错路径</h2>
''' + ol(
        '<b>Arm/Yaw：</b>承托、停机、排除原因后写命令3。',
        '<b>底盘/功率：</b><code>g_chassis_debug_request.mode=1</code>，最后 <code>request_seq+1</code>。') + '''
<p class="stop">清错不是开控。反馈丢失可能让零位失效，必须重新安全校零；调参锁禁要重新选档/写目标；正常遥控还要命令4。</p>
<h2>找人帮查时带上</h2>
<p>源码/Debug ELF版本、当前批次、<code>robot_tune_mode</code>、reason/ready/blocked、所写目标和实际参数、本地faults、时间戳、曲线、实物现象。</p>
<p class="small">逻辑测试和本说明不代表实车通过；方向、承重、撤力、线束限位、功率停车距离都要低输出实测。</p>
''')]


ARM = [
    page('机械臂只做一件事：慢抬后保持', '''
<p class="note">目的：把机械臂移到已确认安全的位置，为底盘调轮留空间。用同一份 Debug 固件：<b>mode0抬起 → 左下保持 → 切1～4调轮 → mode0退出仍保持</b>。</p>
<p>打开“机械臂简易调参与排障.pmpx”，和另外两份工程共用同一个 ELF。一次只开一份工程通信。</p>
<h2>开始前必须满足</h2>
<p>真实IMU/功率/互锁、五轴在线且已校零、Yaw人工中位和在线半幅、实物配置确认。<b>第8批没做好就先不抬臂</b>，继续独立悬空调轮/Yaw。</p>
''' + table(['joint', '轴', '记住'], [
        ('0', 'Yaw', '无滑环；真实线束中位和小范围。'),
        ('1', '肩，大pitch DM', '左竖；与肘联合零位。'),
        ('2', '肘，小pitch DM', '右竖；驱动角与肩有关联。'),
        ('3 / 4', 'roll / 夹爪 M2006', '也要校零；本操作先不主动控制。')]) + '''
<p class="stop">两DM各0.3Nm只是启动上限，不保证抬得起实物。先支架承托；异常先接住再命令1。左下是保持，命令1是撤力，不能混用。</p>
'''),
    page('第0/1步：承托，摆好联合零位', setting('第0步 关控与连接 / 第1步 双Pitch人工联合零位', '只校零，不开控、不抬起。') + '''
''' + ol(
        '停轮、mode0；整臂可靠承托；命令1；确认 <code>enabled_mask=0</code>。',
        '<code>uwTick</code>、<code>g_arm_status.updated_ms</code> 更新；五轴 <code>online=1</code>、反馈新鲜。',
        '请队友把两个pitch摆到约定的<b>联合参考姿态</b>；roll对参考，夹爪对安全张开参考；Yaw对线束中位标记。',
        '全部静止后写 <code>g_robot_debug_command=2</code>。<b>mode0的命令2校全部五轴</b>，不能只摆两个DM。',
        '<code>last_result=0</code>、五轴 <code>calibrated=1</code>；随后按第6批申请Yaw已测半幅。') + check(
        '五轴都校零成功，肩/肘角与联合参考一致。') + '''
<p class="note">两pitch关系：<code>q肩=a1</code>、<code>q肘=a2−a1</code>；不能把第二台DM在任意姿态独立认零。</p>
''' + table(['结果', '先查'], [
        ('3', '仍有轴使能；先承托、全部关控。'),
        ('4 / 5', '反馈未在线 / 零位未建立。'),
        ('7', '仍在动，等静止。'),
        ('校后calibrated又变0', '反馈连续性丢失；修反馈并重新安全校零。')]) + '''
<p class="small">mode5/6的命令2只校Yaw，不用于这里。重启后零位要重新建立。</p>
'''),
    page('第2步：0档开控，只慢抬肩和肘', setting('第2步 0档简易抬肩抬肘', '每次只动一个输入，先不碰其他轴。') + '''
''' + ol(
        '两轮悬空、整臂有承托；正常整车健康已通过。mode0，左中夹取、右中；摇杆回中，确认 <code>wait_neutral=0</code>。',
        '写 <code>g_robot_debug_command=4</code>；命令被取走，<code>active=1</code>、<code>enabled_mask=31</code> 才允许推杆。',
        '只轻推左竖，肩动很小角度就回中；实际方向要和目标一致。',
        '肩正常后，只轻推右竖调肘，同样小动即回中；另一个输入保持不动。',
        '逐步移到队友确认不会干涉、不会碰地、能承重保持的姿态；所有输入回中。') + check(
        '肩/肘平稳跟随、方向正确、没有持续下垂或抖动。') + '''
<p class="note">看肩/肘 <code>target_deg</code>、<code>position_deg</code>、<code>effort</code>（Nm）。参考轨迹可以比目标慢；effort一直顶限还下垂，先承托，不要继续堆目标或猛加I。</p>
<p class="stop">左上是行驶，不能用它关控。紧急时命令1会撤掉保持，先有机械支撑。</p>
'''),
    page('第4步：左下保持，再切轮子档', setting('第4步 左下待机保持', '保留已经使能的机构保持，不额外使能未开轴。') + '''
''' + ol(
        '抬臂结束，所有输入回中；左下待机、右中。',
        '看 <code>g_robot_status.mode=3</code>、<code>active=1</code>、<code>enabled_mask=31</code>；几秒内角度不持续下垂。',
        '两轮可靠悬空且实际静止。',
        '停止机械臂工程通信，打开底盘调参第5批；选mode1～4，确认目标0和参数；写非零目标，ready后继续。',
        '原已使能机械臂保持；未使能轴不因切档自动开。退出调轮先目标0，再mode0。') + check(
        '调轮期间机械臂稳定保持，轮子只按所选对象出力。') + '''
<h2>要再调机械臂姿态</h2>
<p>mode0停轮；左中夹取；摇杆回中，正常条件就绪后命令4恢复遥控，再轻推肩/肘。留在非零轮档推杆只会取消调参，不会自动变成抬臂。</p>
<h2>收工</h2>
<p><b>停轮/mode0 → 等停稳 → 支架接住机械臂 → 命令1 → 确认 enabled_mask=0 → 断动力。</b>关FreeMASTER通信不是停止；mode0退出也不是撤力。</p>
<p class="stop">保持不稳：先承托关控，解决机械臂问题再调轮。保持轴掉线/故障可能触发全局撤力。</p>
'''),
    page('第3步可选：少量调参和简单排障', setting('第3步 少量改PI与限流', '能稳定抬起保持就先不改，只处理必要参数。') + '''
<p>沿用队友的关节参数申请，不再增加一套机械臂调参输出模式。先停轮、承托整臂、全局关控，全部轴失能且静止，再完整填写下表。</p>
''' + table(['变量', '以肩为例；肘另选并填自己的值'], [
        (v('g_arm_tune_request.joint'), '1=肩，2=肘。'),
        (v('g_arm_tune_request.position_kp'), '例如保留源码2，必须是正数。'),
        (v('g_arm_tune_request.velocity_kp'), '例如源码起点0.03。'),
        (v('g_arm_tune_request.velocity_ki'), '先0；稳定后必要时少量加。'),
        (v('g_arm_tune_request.effort_limit'), '本次核定的Nm上限，不超过源码允许值；当前启动上限0.3Nm。'),
        (v('g_arm_tune_request.request_seq'), '其他项填齐，最后读当前值再加1。')]) + '''
<p><code>g_arm_tune_status.result=0</code> 才成功；<code>active_joint</code>、<code>active_seq</code> 和实际参数要对上本次。失败申请也消耗序号；修好条件后用新序号，不会迟到生效。参数成功不自动开控。</p>
''' + table(['现象', '先怎么处理'], [
        ('不使能/不抬起', '查正常整车健康、五轴在线/校零、回中和命令4。'),
        ('目标变、实际不动', '查使能、反馈、电机状态和限流，不只加位置P。'),
        ('effort顶限仍下垂', '承托；和队友核对负载/机构/力矩，不猛加I。'),
        ('方向反/抖动', '立即承托关控；查映射、方向和反馈，回保守参数。')]) + '''
<p class="small">只做能安全抬起并保持的最小调试；满载性能和更大力矩另行核定。</p>
''')]


if __name__ == '__main__':
    import sys
    sys.stdout.reconfigure(encoding='utf-8')
    pdf('00_分批调参步骤.pdf', STEPS)
    pdf('01_排障与变量说明.pdf', FAULTS)
    pdf('02_机械臂抬起与保持.pdf', ARM)
