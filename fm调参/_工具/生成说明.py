"""按 project4 的分批操作卡，原位更新单固件、运行时切档的三份 PDF。"""
from pathlib import Path
import html
import pymupdf as fitz

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = fitz.Archive('C:/Windows/Fonts')
CSS = '''
@font-face {font-family:cn;src:url(simhei.ttf);}
@font-face {font-family:mono;src:url(consola.ttf);}
body {font-family:cn,sans-serif;font-size:10.3pt;line-height:1.4;color:#192e3b;}
h1 {font-size:19pt;line-height:1.22;color:#124e70;margin:0 0 12pt;}
h2 {font-size:12.5pt;color:#12628a;margin:12pt 0 6pt;}
p {margin:5pt 0 7pt;} li {margin-bottom:5pt;}
ol,ul {padding-left:20pt;margin:6pt 0 9pt;}
table {width:100%;border-collapse:collapse;margin:8pt 0;font-size:8.8pt;line-height:1.32;}
th {background:#e1eef5;text-align:left;}
td,th {border:0.5pt solid #a8bdca;padding:5pt;vertical-align:top;}
code {font-family:mono,cn;font-size:8.5pt;color:#125473;overflow-wrap:anywhere;}
.note {background:#eff5f8;padding:9pt;border-left:3pt solid #18739c;}
.stop {background:#fff1e8;padding:9pt;border-left:3pt solid #b65d28;}
.small {font-size:9pt;color:#415b69;} .tag {font-size:9pt;color:#12628a;}
'''


def v(text):
    return '<code>' + html.escape(text) + '</code>'


def table(headers, rows):
    return '<table><tr>'+''.join('<th>'+h+'</th>' for h in headers)+'</tr>'+''.join('<tr>'+''.join('<td>'+str(c)+'</td>' for c in row)+'</tr>' for row in rows)+'</table>'


def page(title, body):
    return title, '<h1>'+title+'</h1>'+body


def setting(group, mode, aim):
    return '<p class="tag">固件始终是 Debug 整车固件，不换固件。</p><p><b>打开 Group：</b>'+group+'</p><p><b>robot_tune_mode：</b>'+str(mode)+'</p><p class="note"><b>本页只做：</b>'+aim+'</p>'


def pdf(filename, pages):
    doc = fitz.open()
    toc = []
    for title, content in pages:
        story = fitz.Story('<html><body>'+content+'</body></html>', user_css=CSS, archive=ARCHIVE)
        part = story.write_with_links(lambda number, filled: (fitz.Rect(0,0,595,842),fitz.Rect(42,42,553,787),None))
        toc.append([1,title,len(doc)+1])
        doc.insert_pdf(part)
        part.close()
    doc.set_toc(toc)
    for i, p in enumerate(doc):
        p.draw_line((42,801),(553,801),color=(.7,.78,.82),width=.5)
        p.insert_text((42,817),'RM / '+filename.split('_')[0]+' / '+str(i+1)+' of '+str(len(doc)),fontsize=8,color=(.25,.35,.4))
    doc.set_metadata(dict(title=filename.removesuffix('.pdf'),author='工程车电控协作',subject='2026-11-11：单Debug整车固件；robot_tune_mode运行时切档；按批次操作'))
    doc.subset_fonts()
    path = ROOT / filename
    data = doc.tobytes(garbage=4,deflate=True)
    if path.exists():
        with path.open('r+b') as target:
            target.write(data)
            target.truncate()
    else:
        path.write_bytes(data)
    print(filename+': '+str(len(doc))+' pages')
    doc.close()


STEPS = [
page('先看这一页：只用一份固件', '''
<p>这次改成和 project4 一样的操作方式：<b>烧一次 Debug 整车固件，在 FreeMASTER 改 robot_tune_mode 选择调谁，再用 tune_enable 决定本次是否允许输出。</b>左、右轮与 Yaw 调试不再要求更换三个固件。遥控器仍负责行驶／夹取／待机。</p>
<p class="note">每批只看四件事：<b>点哪组 → 填什么 → 看什么 → 怎样算通过。</b>先完成第0、1批，再做第2批；不要一上车就试闭环或落地。</p>
'''+table(['robot_tune_mode','本档实际控制什么','其他机构'],[
('0','正常遥控；左上行驶、左中夹取、左下待机。','按正常整车保护与遥控模式工作。'),
('1 / 2','1左轮电流；2右轮电流。一次非零最多500ms。','另一轮零输出；已使能机械臂保持。'),
('3 / 4','3左轮速度PI；4右轮速度PI。速度持续到改零／退出。','另一轮零输出；已使能机械臂保持。'),
('5','Yaw速度PI，跳过位置外环。','底盘零输出；其他已使能轴保持。'),
('6','Yaw位置P＋速度PI双环。','底盘零输出；其他已使能轴保持。')])+'''
<p><b>tune_enable默认0；切档也自动清0。</b>0只停止当前测试，其他已使能轴继续保持；1通过健康、静止、回中检查后才放行。未使能的其他机械臂轴不会因为切调参档自动使能。Yaw若由调参单独使能，停止／退出时撤销这次新增使能；原本就已使能的Yaw改为当前位置保持。</p>
<p class="stop"><b>退出调参：</b>robot_tune_mode=0，保留已有机械臂保持。<br><b>全局关控：</b>g_robot_debug_command=1，撤掉各轴出力。关控不能代替日常退出，承托机械臂后再关控。</p>
<p class="small">本文起步数字是小步试验示例，未声称已完成实车方向、负载、限位或停车距离验收。需要阅读的内容直接覆盖原三份PDF。</p>'''),
page('路线表：按第0～12批走', '''
'''+table(['批次／Group','档位','本批做完得到什么'],[
('第0批 安全常驻','0','不接动力，验证命令1、遥控失联和软件零输出。'),
('第1批 遥控与两轮反馈','0','认清通道、拨杆、轮子、ID和向前正方向。'),
('第2批 左轮ID1电流方向','1','小电流500ms内左轮动作，右轮不出力。'),
('第3批 右轮ID2电流方向','2','右轮重复正反方向检查。'),
('第4批 两轮速度PI','3、4','先P后I，分别获得稳定的轮速闭环。'),
('第5批 0档抬臂左下保持','0→1～4','正常整车条件就绪后抬臂，保持时调轮。'),
('第6批 Yaw线束中位与软限位','5或6，先全部关控','人工中位校零，在线申请一个已确认的小半幅。'),
('第7批 Yaw速度内环PI','5','小角速度正负跟踪，验证边界制动。'),
('第8批 IMU功率与真实整车许可','0','补齐正常整车真实保护与姿态许可。'),
('第9批 Yaw位置双环','6','小角度目标，速度内环稳定后再调位置P。'),
('第10批 正常遥控慢速落地','0','慢速前后、转向、松杆停车与夹取切换。'),
('第11批 停止清错与故障恢复','0／原档退出','确认故障不续跑，实测功率限制下停车距离。'),
('第12批 固化参数与重启复验','0','写回源码，重烧后重新校零、申请半幅并复验。')])+'''
<p>第5批要用正常遥控抬臂，<b>如果第8批的整车条件尚未做好，就先跳过第5批</b>，继续独立调轮和Yaw。不要为了提前抬臂伪造IMU、功率或互锁许可。</p>
<p>底盘两台M3508：CAN2左ID1、右ID2，反馈0x201/0x202，0x200第一／二槽。Yaw：CAN1 GM6020电流模式ID1，反馈0x205，0x1FE第一槽。CAN1还有队友机械臂，不能新增第二个Yaw驱动或发送者。</p>'''),
page('连接一次，之后只切mode', '''
<h2>1．烧录和打开工程</h2>
<ol><li>两轮架空、整臂可靠承托，先断开电机动力。编译并烧录本工程的<b>Debug</b>固件。</li>
<li>停止FreeMASTER通信，关闭三份.pmpx。运行本文件夹的“切换分组.cmd”；它现在只刷新<b>同一Debug ELF</b>，不再选1／2／3固件。</li>
<li>打开“底盘调参.pmpx”。Project → Options → MAP Files应指向本工程 build/Debug/dx_assignment5_freertos.elf，并与板中固件一致。</li>
<li>沿用原有连接设置，Start Communication；uwTick应持续增加。刚重编译就刷新符号，再重新打开工程。</li></ol>
<h2>2．Group是操作页，mode才是切档</h2>
<p>点击左侧Group只改变你看到的变量与曲线，<b>不会切调参档</b>。双击变量Value填数字并回车，才会写变量。例如左轮电流：robot_tune_mode写1只是选择，tune_enable仍是0；准备好后明确写1才允许本次输出。</p>
<p>变量列表用完整源码名；中文说明在Group、Description和本文。状态变量只看、不写。示波器先打开再做短电流测试，否则可能错过500ms过程。</p>
<h2>3．三份工程使用同一个ELF</h2>
<p>日常操作用“底盘调参.pmpx”；异常换“底盘排障.pmpx”；简单抬臂用“机械臂简易调参与排障.pmpx”。一次只让一份工程通信，切换前停止上一份。</p>
<p class="note">切档不需要重新烧录。只有修改源码、确认项或固化参数后才需要重编译、烧录、刷新符号。DM改ID工具仍是独立维护工具，不属于这里的运动调参。</p>'''),
page('每次切档都照这六步', '''
<h2>进入1～6档</h2>
<ol><li><b>先停：</b>tune_enable=0，停止当前测试，等待实际静止；其他原已使能轴保持。Yaw只由本次测试使能时会撤Yaw力，先承托。退出时不用命令1。</li>
<li><b>回0：</b>robot_tune_mode=0。故障、超时或未就绪锁禁后尤其要先回0，不能从1跳2试图绕过去。</li>
<li><b>选档并填参数：</b>robot_tune_mode写本次档位，tune_enable自动清0；电流、轮速、Yaw速度目标清0，Yaw位置对齐当前角。此时改P／I／输出上限，核对目标和实际参数；不是等ready才填参数。</li>
<li><b>摆遥控：</b>左下、右中，所有5个通道（含拨轮）均在±30内，保持至少300ms。所调机构与已有保持轴均停稳。</li>
<li><b>明确开启：</b>确认目标0（位置档为当前角），tune_enable写1；等待g_robot_tune_status.ready=1、blocked=0，<b>最后才填小运动目标</b>。刚开启会先清目标／准备一周期。</li>
<li><b>随时能停：</b>tune_enable写0停止当前测试；速度写0也可闭环减速；mode0退出。突发异常用全局命令1。离开左下右中、偏杆、失联或反馈异常会取消调参。</li></ol>
<h2>本次被拒绝，不会等条件好后突然启动</h2>
<p>tune_enable=0时ready=0、reason=7正常，表示仅选择、尚未开启；可以安全准备参数。明确写1仍被拒绝／blocked=1时，先mode0，排除原因，重新选择并明确开启。不能通过enable0→1清除已有故障锁禁。</p>
<h2>调完恢复正常遥控</h2>
<p><b>tune_enable=0 → mode=0 → 轮子静止，已有机械臂继续保持 → 选择左中夹取或左上行驶 → 所有操作输入回中至少300ms → 确认正常整车保护全部就绪 → 写g_robot_debug_command=4。</b>只回0不会立即放行摇杆；在左下待机写命令4也不会预存开控请求。</p>
<p class="stop">关控命令1会撤掉机械臂保持；关闭FreeMASTER通信不会自动停机。速度档3／4／5没有500ms自动结束，必须亲自改零或退出。</p>'''),
page('第0批 A：不接动力，只测关控', setting('第0批 安全常驻｜单固件Debug',0,'先验证软件收到全局关控，完全不发运动目标。')+'''
<h2>准备</h2>
<ol><li>电机动力隔离，只给C板和接收机供电；两轮架空，机械臂可靠承托。</li><li>robot_tune_mode保持0、tune_enable保持0；不写命令2或4，不改硬件确认项。</li><li>看uwTick、g_robot_status.updated_ms、g_chassis_status.updated_ms持续变化。</li></ol>
<h2>按这四步做</h2>
<ol><li>记下g_robot_status.stop_epoch，例如现在为0。</li><li>g_robot_debug_command写1。命令应自动被取走回0。</li><li>stop_epoch应增加；g_robot_status.protection_latched_reasons应包含16这一位，不要求整个数值恰好等于16。</li><li>g_arm_status.enabled_mask=0；g_chassis_status.enabled=0；current_ma[0]和[1]都为0。</li></ol>
<p><b>通过：</b>控制心跳更新、命令被取走、关控次数增加、所有出力许可和两轮提交电流为0。</p>
<p><b>未通过：</b>时间戳冻结先查固件／通信；命令不归零查控制任务。先停在本批，不继续开控。</p>
<p class="note">没有电机动力时，CAN缺应答、反馈超时等故障可以出现。此时不为了清掉故障而关闭保护。本批只是软件路径检查，尚不能证明运动中电机真正撤力。</p>'''),
page('第0批 B：不接动力，测遥控失联', setting('第1批 遥控与两轮反馈，再回第0批',0,'确认失联被发现，重新在线也不自动续跑。')+'''
<ol><li>打开遥控器，看g_dr16_status.good_frames持续增加，valid=1，consecutive_frames至少3。</li><li>记下good_frames，关闭遥控器；它应停止增长，g_robot_status.remote_age_ms增大。</li><li>超过约100ms并经任务处理后，remote_online应为0；保护原因包含4这一位。</li><li>再次确认所有轴失能、两轮提交电流0。</li><li>重新打开遥控器。在线可以恢复，<b>所有轴不能只因重新在线就自动出力</b>。</li></ol>
<h2>遇到问题</h2>
'''+table(['现象','先做什么'],[
('始终remote_online=0','检查接收机、接线、连续有效帧和当前固件；不直接改在线状态。'),
('关遥控后good_frames仍增加','核对接收机是否还在发旧合法帧；停止后续运动测试。'),
('当前原因里没一直显示16','命令1可只持续一个控制周期；看历史锁存和stop_epoch。')])+'''
<p><b>通过后记下来：</b>命令1会被取走、stop_epoch增加、关遥控后在线变0、恢复后不自动续跑。</p>
<p>后面完成小电流方向验证后，再补做一次带动力的关控／失联试验：先仅让单轮低速转，确认命令1立即撤除提交输出。机械臂先承托，不能拿带载坠落验证关控。</p>'''),
page('第1批 A：认清遥控，不调PID', setting('第1批 遥控与两轮反馈｜不出力',0,'把实物每一个输入与源码通道对应起来。')+'''
<h2>看这些变量</h2>
<p>三份工程都有独立Group“<b>遥控映射检查｜原始与处理后</b>”。本批电机动力可保持隔离；mode0、tune_enable=0、命令1关控，也能看处理后的映射，不必开电机。</p>
<p>同页对比g_dr16_status.frame.channel[0]～[4]／frame.switches[0]／[1]与g_robot_status.remote_axis[0]～[3]／remote_switch[0]／[1]。它们全部只读，不是在线改映射的入口。</p>
<ol><li>只动左竖，记录哪一个channel变化、向上是正还是负，回中。</li><li>分别只动左横、右竖、右横，每次只动一个；记录后回中。</li><li>检查拨轮channel[4]也能回中，所有通道均在±30内。</li><li>分别只拨左、右三段开关，确认switches索引。原码上1、中3、下2。</li></ol>
'''+table(['当前源码暂定','应由实物确认'],[
('左竖channel[3]；左横[2]','右竖channel[1]；右横[0]。'),
('左拨杆switches[0]；右拨杆[1]','正常整车：左上行驶、左中夹取、左下待机。'),
('摇杆中位约0、满杆约±660','调参要求含拨轮共5通道均在±30内。')])+'''
<p><b>先完成这页原始输入记录，再看下一页处理后对比。</b>本批保持关控，不把命令4当成看映射的前提。</p>'''),
page('第1批 A2：原始值与处理后对比', setting('遥控映射检查｜原始与处理后',0,'关控时也能核对实际代码中的映射。')+'''
<h2>原始值和处理后值怎样对应</h2>
'''+table(['原始输入（当前配置）','处理后只读变量','正常观察'],[
('channel[3] 左竖','g_robot_status.remote_axis[0]','上推应正，下推应负。'),
('channel[2] 左横','g_robot_status.remote_axis[1]','右推应正，左推应负。'),
('channel[1] 右竖','g_robot_status.remote_axis[2]','上推应正，下推应负。'),
('channel[0] 右横','g_robot_status.remote_axis[3]','右推应正，左推应负。'),
('switches[0] 左拨','g_robot_status.remote_switch[0]','上1／中3／下2。'),
('switches[1] 右拨','g_robot_status.remote_switch[1]','上1／中3／下2。')])+'''
<p>当前四轴方向均+1，都是<b>待实物确认的配置</b>。原始约±660被归一化为处理后±1，约6%死区内变0，再对死区外重新缩放，因此小输入不必与原码严格等比例。每次只动一杆，只有相应处理后轴变化才算映射正确。</p>
<p>同时看valid、good_frames、received_ms、remote_age_ms和remote_online：帧数／接收时间需更新、年龄小、在线为1。处理后数值只是上一帧的映射，不用静止不变的数字证明接收机仍在线。</p>
<p>不一致时，修改2User_Driver/Remote/operator_config.cpp里的通道索引、开关索引和方向；实物核对完才把mapping_confirmed设true，重编译烧录并复查。</p>
<p class="stop">本固件左上是行驶，<b>不是关控</b>。全局关控用g_robot_debug_command=1或已核定的失联保护；左下是保持，不能当撤力使用。</p>
<p><b>通过：</b>你能说清楚每个实物输入对应哪个变量，拨轮回中可靠，遥控关闭后帧停止。</p>'''),
page('第1批 B：手转两轮，核对方向', setting('第1批 遥控与两轮反馈｜不出力',0,'只看反馈，不申请运动。')+'''
<ol><li>先断电接好底盘动力，再上电；两轮继续架空、机械臂有支撑。命令1确保全局失能。</li><li>两轮feedback_frames持续增长、feedback_ms更新；静止时measured_rpm接近0。</li><li>从驾驶视角认左、右：手转左轮，只应主要影响measured_rpm[0]；右轮对应[1]。</li><li>把左轮往“车向前走”的方向手转：轮胎最下面一点相对车体向后走。规范后的measured_rpm应为正。右轮重复。</li><li>若某侧符号反，修改Chassis/chassis_config.h的direction对应项（+1／−1），重新烧录后复验。</li></ol>
'''+table(['下标／变量','含义'],[
('[0]','左轮 CAN2 ID1，反馈0x201。'),('[1]','右轮 CAN2 ID2，反馈0x202。'),
('measured_rpm','已按direction规范的<b>电机转子rpm</b>。不是轮胎rpm。'),
('gear_ratio=19','3508内部减速比计入总比；外部直装1:1并不等于电机转子到轮胎总比1。')])+'''
<h2>进入第2批前，只确认本批实际核对过的项目</h2>
<p>chassis_config.h：CAN ID、方向、轮径0.145m、轮距0.330m、总减速比等核对后，hardware_confirmed才设true。遥控mapping_confirmed也需按上一页实测。其他IMU、功率、姿态项尚未测就保留未确认。</p>
<p><b>通过：</b>两轮身份不串、向前反馈为正、数据持续更新；没有运动申请时两路current_ma均0。</p>'''),
page('第2批：左轮小电流方向', setting('第2批 左轮ID1电流方向｜robot_tune_mode=1',1,'左轮很小的正／反向电流试验；右轮始终零输出。')+'''
<h2>先准备</h2>
<p>两轮可靠悬空。tune_enable=0、mode先回0；左下右中、5通道回中至少300ms，轮子停稳。打开“左轮转子速度与最终电流”示波器。机械臂未使能可继续失能。</p>
'''+table(['按顺序写','本次示例'],[
(v('tune_wheel_current_limit_ma'),'100mA，小测试限流；硬上限300mA。'),
(v('robot_tune_mode'),'1。切档清enable和目标，先确认电流目标0。'),
(v('tune_enable'),'参数正确、停稳、回中后写1；等ready=1、blocked=0。'),
(v('tune_current_ma'),'+50mA。<b>最后写这一项</b>。')])+'''
<h2>写完看什么</h2>
<ol><li>g_robot_tune_status.mode=1、ready=1；g_chassis_status.tune_mode=1，tune_blocked=0。</li><li>current_ma[0]为小正值（量化后可能略有差别），current_ma[1]=0。</li><li>若轮子已动，measured_rpm[0]应为正，实物向前方向应一致。不一致立即命令1。</li><li>非零电流最长500ms，到期ready变0、blocked变1、reason=5，提交电流归0。</li></ol>
<p>50mA可能不足以克服摩擦。先确认输出／反馈和实物，不直接拉满；若需增大，每次小步，保持≤300mA。</p>
<h2>下一次怎么重新测</h2>
<p><b>tune_enable=0 → tune_current_ma=0 → mode0 → 等轮子静止、全通道回中300ms → 重新选1 → tune_enable=1 → ready=1 → 电流写−50。</b>只改正负或大小不能续500ms时间。</p>
<p><b>通过：</b>正负方向一致；超时和mode回0都能停止，右轮始终零输出。current_ma只是最终接受入队的提交值，仍须看实物响应。</p>'''),
page('第3批：右轮重复方向试验', setting('第3批 右轮ID2电流方向｜robot_tune_mode=2',2,'用同样的小电流确认右轮；不是让两轮一起转。')+'''
<ol><li>左轮试验结束，tune_enable=0、电流目标0、mode0，等待两轮实际停稳。</li><li>左下右中、5通道回中300ms。打开右轮示波器。</li><li>tune_wheel_current_limit_ma保留100mA。mode写2，确认目标0、enable仍0；再写tune_enable=1，等ready=1且blocked=0。</li><li>tune_current_ma最后写+50；应只有current_ma[1]非零，current_ma[0]=0。</li><li>看measured_rpm[1]与实物向前符号一致。最长500ms到期后归零。</li><li>enable0、回mode0并停稳，再明确选2、enable1、ready后写−50，验证反方向。</li></ol>
<h2>这三个现象必须分开看</h2>
'''+table(['现象','处理'],[
('enable0且ready0、reason7','只选档尚未开启是正常的；准备好并回中停稳后明确enable1。'),
('enable1后ready仍0或blocked1','本次没有放行。查原因，不继续加电流；先回0再重新选档。'),
('提交电流有了，但小电流下不动','可能静摩擦；先核对协议、实际反馈、方向，再小步试。'),
('电流一给，速度迅速上升或方向错误','立即关控，核对ID、方向和电调模式，不进入速度闭环。')])+'''
<p>功率配置尚未确认时，仅允许这种受≤300mA限制的独立悬空调轮；不意味着正常mode0可以绕过功率。功率一旦已确认，其反馈／能量保护在调轮时也生效。</p>
<p><b>通过：</b>左右两轮各自正反测试完成；选哪一轮就只有那一轮出力；停止与超时可重复验证。</p>'''),
page('第4批 A：左轮速度环，先只用P', setting('第4批 两轮速度PI｜左3档 右4档',3,'从小转子rpm调速度PI；先关闭I。')+'''
<h2>enable保持0，先填写本次参数</h2>
'''+table(['变量','起步示例／含义'],[
(v('tune_wheel_kp'),'1.0，单位mA/rpm；源码占位值，不是最终答案。'),
(v('tune_wheel_ki'),'0，先关积分。'),
(v('tune_wheel_current_limit_ma'),'100mA，先保守；调轮硬上限300mA。'),
(v('robot_tune_mode'),'enable0、回mode0、停稳后写3；选档后可改参数。'),
(v('tune_enable'),'确认参数正确、速度目标0，回中300ms后写1。'),
(v('tune_speed_rpm'),'ready=1后写+50，<b>3508转子rpm</b>。')])+'''
<h2>示波器看“目标、实际、电流”</h2>
<ol><li>先目标0，看是否有持续异响／抖动；没有再给+50。</li><li>target_rpm[0]与measured_rpm[0]同号，实际平稳靠近目标；右轮电流0。</li><li>目标写0，实际应回到零附近。然后尝试−50，仍只调左轮。</li><li>参数需要修改时，先目标0减速并停稳，再enable0，改一项，核对生效值；enable1、ready后重复同样小目标。</li></ol>
<p><b>P怎样改：</b>方向正确、未顶电流但跟得慢时，P可小幅增加，例如1→1.2；出现高频抖动／啸叫先停，减P并查机械松动和反馈。不要照抄project4数值，它的量纲和电机不同。</p>
<p class="stop">3／4速度档是持续控制，<b>没有500ms自动结束</b>。关闭FreeMASTER不会停止；必须把tune_speed_rpm写0或mode回0。</p>'''),
page('第4批 B：少量I，再测右轮', setting('第4批；对比两条轮速示波器','先3，再4','先获得稳定P，再消除可重复的静差。')+'''
<h2>先判断是不是该加I</h2>
'''+table(['曲线','本次处理'],[
('电流正负高频跳、轮子抖','先减P；查反馈／松动，不加I。'),
('电流已顶限，实际跟不上','先查摩擦／负载／测试上限；增大P或I不会增加允许电流。'),
('方向对、曲线稳定、还有固定速度差','把I从0小步增加，例如0.05；每次先目标0停稳再改。'),
('松杆回零后反复冲过零、回摆','I可能过大；先减I。')])+'''
<h2>把相同流程交给右轮</h2>
<ol><li>tune_speed_rpm=0、tune_enable=0、mode0，实际静止；左下右中并回中300ms。</li><li>mode=4，确认目标0，再tune_enable=1、ready=1后写+50；应只有右轮出力。</li><li>验证0、−50；再在已验证的小范围内试±100等目标。FreeMASTER限制|目标|≤300转子rpm。</li><li>同一组轮速P／I用于左右轮。两边差异明显时先查机械与方向，不能靠一边参数掩盖硬件问题。</li></ol>
<h2>达标和记录</h2>
<p>正负目标均不发散、不持续抖；改零能收敛，另一轮不出力；记录P、I、测试限流和曲线。g_chassis_tune_status.result=1表示参数成功；同时看实际生效P／I和g_chassis_status.tune_result。</p>
<p>本批只验收悬空小速度，不能证明落地负载、功率制动或坡道停车。调轮最大300mA也不是正常行驶上限。</p>'''),
page('第5批：先抬臂保持，再调轮', setting('第5批 0档抬臂左下保持｜再选1到4档测轮','0→1～4','不换固件，让已抬起的机械臂继续保持。')+'''
<h2>若正常整车条件未就绪，先跳过本批</h2>
<p>抬臂走mode0正常整车路径，要求真实IMU、功率、五轴反馈／零位、Yaw软限位和互锁。独立轮测试通过不代表可以正常开控。第8批未完成时，先继续第6／7批，之后再回来。</p>
<h2>抬起：只在0档正常夹取</h2>
<ol><li>mode=0，全部目标归零，等待轮子静止；整臂有支撑。</li><li>按“02_机械臂抬起与保持.pdf”准备五轴联合零位和正常整车条件。</li><li>左中、右中，操作输入回中至少300ms，wait_neutral=0后明确写命令4；active=1、enabled_mask=31才继续。</li><li>轻推左竖抬肩，回中；再轻推右竖抬肘，回中。每次只动一个输入，不盲碰roll／夹爪。</li><li>到达已确认安全的姿态，切左下待机。所有输入回中，确认各轴稳定保持几秒，无持续下垂。</li></ol>
<h2>测轮：保持稳定后才切档</h2>
<p>左下右中、全部5通道回中300ms、两轮悬空且已静止；mode选1～4，参数正确、目标0后写tune_enable=1，ready=1后给已验证的小目标。其他已使能轴继续保持，未使能轴不会自动使能。</p>
<p>偏杆、离开左下右中、关节掉线或失联会取消调参；全局关控会连机械臂保持一起撤掉。<b>退出调轮只用目标0、mode0。</b></p>
<p class="stop">两DM当前各0.3Nm只是源码启动上限，不保证抬得起实物。顶限仍下垂时先承托，与队友核对机构／负载／力矩需求，不猛加I。</p>'''),
page('第6批 A：Yaw人工线束中位', setting('第6批 Yaw线束中位与软限位｜先关控','5或6；校零时全部失能','只认中位和方向，不给Yaw运动目标。')+'''
<h2>先做机械和源码确认</h2>
<p>Yaw是GM6020电流模式、ID1、约1:1，不能用旧电压模式驱动。Arm/yaw_config.h的direction、kYawSafetyConfirmed及安全范围，Arm/arm_config.cpp的hardware_confirmed，遥控mapping_confirmed，必须按实物核对。</p>
<ol><li>mode回0，所有测试目标0；承托整臂，命令1，确认enabled_mask=0。</li><li>Yaw在线、反馈连续。慢手转很小角度看joint[0].position_deg和velocity_deg_s方向；不要多圈转。</li><li>整理线束，找自然、未预拧的中位，在底盘与Yaw外圈做标记。对准标记并等静止。</li><li>tune_enable保持0、robot_tune_mode写5或6；只是选择，不给运动目标；<b>保持所有轴失能</b>。</li><li>g_robot_debug_command写2。在5／6档它只申请Yaw校零；看g_arm_status.last_result=0、joint[0].calibrated=1、position_deg接近0。</li></ol>
<p class="note"><b>mode5／6的命令2只校Yaw；mode0的命令2校全部五轴。</b>不要在mode0只摆Yaw就按2。校零本身会请求停机，不能一边保持负载一边重校。</p>
<p>本次非零档会因校零请求停机而锁禁，不急着清：先完成下一页半幅，随后回0再明确选5。关控不会自动建立线束中位。</p>
<p class="stop">无滑环，不能转一圈找零。重启、重烧或连续角丢失后，重新整理线束、对中位标记，再人工校零。</p>'''),
page('第6批 B：直接写一个小半幅', setting('第6批；仍保持全部轴失能','5或6，目标不动','确认真实生效范围，才允许Yaw闭环。')+'''
<h2>只改一个变量，不填限位申请表</h2>
<p>以人工线束中位为0。若实际确认左右各10°均安全，<b>tune_yaw_half_range_deg写10</b>，申请区间就是−10°～+10°。10只是解释半幅的例子，不是没看线束就照填。</p>
<ol><li>tune_enable=0，整臂有支撑，命令1后确认enabled_mask=0；Yaw在线、已在真实中位校零、实际静止。</li><li>修改tune_yaw_half_range_deg，直接回车。不要再写旧g_yaw_limit_request或request_seq。</li><li>看g_yaw_limit_status.result=0、ready=1；active_half_range_deg等于所填值，min_deg／max_deg正确。</li><li>还要保证实际角、目标和参考均在新范围内，留出当前2°制动余量。</li></ol>
'''+table(['result','先处理什么'],[
('2 / 3','配置未确认／有轴仍使能；先确认真实条件并全部关控。'),
('4 / 5 / 6','掉线／未校零／故障锁存。先排除原因，必要时安全重校零。'),
('7 / 8','还在动／范围排除了实际角、目标或参考。')])+'''
<p><b>被拒绝的值只处理一次，不会等条件变好后补执行。</b>条件修好后需要重新编辑成另一个已确认安全的值来再次申请；仅把同一个数字再回车不算值变化。不能为了重试填未经核对的大范围。</p>
<p>软件接受半幅&gt;2°且≤90°，这是防误填边界，不是线束可承受90°的证明。参数、零位和在线范围都只在RAM，重启重建。</p>
<p><b>通过：</b>真实中位明确，小范围结果成功、边界值正确。随后enable0、mode回0，停稳、回中300ms，再重新选5；先核对参数，enable1、ready1后才给小目标。</p>'''),
page('第7批 A：Yaw速度内环，只用P', setting('第7批 Yaw速度内环PI｜robot_tune_mode=5',5,'先验证小角速度，位置外环不参与。')+'''
<h2>先给小上限和参数</h2>
'''+table(['变量','起步示例'],[
(v('tune_yaw_position_kp'),'2，保留合法正值；5档不拿它做位置闭环。'),
(v('tune_yaw_velocity_kp'),'20，源码占位起点，不是实测答案。'),
(v('tune_yaw_velocity_ki'),'0，先关闭积分。'),
(v('tune_yaw_current_limit_ma'),'100mA；Yaw调参硬上限150mA。'),
(v('robot_tune_mode'),'enable0、回mode0、停稳后写5；先填写并核对参数。'),
(v('tune_enable'),'目标0，左下右中且回中300ms后写1，等ready=1。'),
(v('tune_yaw_speed_deg_s'),'ready=1后最后写+1，单位度／秒。')])+'''
<h2>先开示波器再给速度</h2>
<ol><li>看g_robot_tune_status.yaw_parameter_result=0，生效P／I／电流与本次一致；joint[0].enabled=1。</li><li>实际velocity_deg_s与目标同号；velocity_target_deg_s可能因边界制动比输入更小；effort是提交电流mA。</li><li>只观察很短时间就把tune_yaw_speed_deg_s写0，等待静止，再写−1，随后又写0。</li><li>底盘current_ma[0]／[1]都应0。其他原已使能轴保持，未使能不新增。</li></ol>
<p class="stop">5档速度是持续命令，<b>没有500ms自动结束</b>。一旦写+1就必须主动写0，不关FreeMASTER等它停；全程不要靠近线束边界。</p>
<p><b>通过：</b>正负方向和编码器反馈一致，小速度可跟踪，改零／退出／关控都能停止。</p>'''),
page('第7批 B：调稳速度PI和边界制动', setting('第7批；观察Yaw目标速度与反馈及电流',5,'先中间小范围调P，再少量I，最后验证限位减速。')+'''
<h2>每次改参数先停目标</h2>
<p>tune_yaw_speed_deg_s=0，等实际静止；tune_enable=0后修改一项P／I或电流上限，确认yaw_parameter_result=0及生效值；再enable1、ready后重复同样±1°/s。直接修改无需seq。Yaw若只由测试新增使能，enable0会撤销该Yaw使能，先承托；原已使能Yaw则当前位置保持。</p>
'''+table(['曲线／实物','这次只改什么'],[
('方向正确、电流未顶限、跟得慢','P小步增加，例如20→24；测试目标保持不变。'),
('电流正负快跳／啸叫','停，减P，同时查反馈连续性和机构松动。'),
('已顶本次电流上限仍跟不上','查摩擦、负载和上限；不能无条件加P。'),
('P稳定但有可重复静差','I从0少量增加，例如0.2；不同时改P。'),
('回零反摆／低频晃动','先减I；核对实际角速度和饱和。')])+'''
<h2>最后才做边界附近的低速验证</h2>
<ol><li>先确认机械不干涉、线束仍有余量。选一个比软限位更靠内的观察位置。</li><li>用很小速度慢靠近，随时准备目标0；看实际角、受限velocity_target_deg_s和effort。</li><li>向外速度应在边界前降低；到零后不能继续累计位置目标。不要用大速度去“撞”软限位。</li><li>反向回到中间应可用；方向相反或保护失效立即关控，先查符号和中位。</li></ol>
<p>接近边界变慢是保护，不能加P强行突破。实际制动还受负载和电流限制，软件范围不代替线束检查。</p>
<p><b>通过：</b>中间正负跟踪稳、速度改零可停、向边界减速可见；记录P、I、上限、半幅和曲线。</p>'''),
page('第8批 A：正常整车有哪些硬条件', setting('第8批 IMU功率与真实整车许可｜0档',0,'把正常整车条件补齐，先不开控。')+'''
<p>独立1～6档用于分机构低输出调参；<b>mode0正常遥控要求全部真实整车保护</b>，不能用调参就绪代替整车就绪。进入0档后原已使能轴保持，但正常摇杆还要显式恢复。</p>
'''+table(['文件（均在2User_Driver）','实测后才确认'],[
('Remote/operator_config.cpp','遥控通道、开关与正向；mapping_confirmed。'),
('Chassis/chassis_config.h','轮ID、方向、几何、减速比、速度PI与限流；hardware_confirmed。'),
('Arm/arm_config.cpp','四轴型号、协议量程、方向、范围、联合零位；hardware_confirmed及pitch.mapping_confirmed。'),
('Arm/yaw_config.h','方向、人工中位、线束范围和制动；kYawSafetyConfirmed。'),
('Imu/attitude_estimator.h','安装坐标和倾覆判据；installation_confirmed／thresholds_confirmed。'),
('Power/chassis_power.h','帧周期、功率、能量和停车约束；confirmed。'),
('Vehicle/vehicle_state.h','真实可行驶机械臂姿态和Yaw包络；posture_confirmed。')])+'''
<h2>最后一次烧录之后还要重新做</h2>
<p>mode0、承托、命令1，五轴全部失能；请队友按规定把两个pitch摆联合零位，roll和夹爪对参考，Yaw对线束中位。<b>mode0命令2校全部五轴</b>，结果成功且五轴calibrated=1才继续。随后重新申请已测Yaw半幅，重填在线参数，等待IMU标定。</p>
<p>不能只顾Yaw在任意机械臂姿态按mode0命令2；两DM是q肩=a1、q肘=a2−a1，必须按联合参考成对校零。</p>
<p><b>尚未确认就停在这里。</b>默认confirmed为false是有意保留的实测门槛，不是在FreeMASTER把许可写1就解决。</p>'''),
page('第8批 B：IMU与功率只读核对', setting('第8批；异常用排查5／8',0,'让姿态和功率可信，先不驱动车。')+'''
<h2>IMU</h2>
<ol><li>整车静置、所有机构失能且有支撑。g_ins_status.frames、received_ms、updated_ms持续更新。</li><li>需要重新静态标定时，g_ins_debug_command写1；等待calibrated=1。标定时不能晃车或让电机出力。</li><li>平放tilt_deg应接近0并稳定，核对实际安装轴。确认后valid=1、confirmed=1、faults=0。</li><li>全部失能、稳固支架承托时缓慢改变姿态，验证tipped进入／退出；不要摔翻实车。恢复后不能自动开控。</li></ol>
<p>当前倾覆阈值、持续时间是源码待测值；按实车正常坡度和机构姿态核定。C板在底盘，IMU用于底盘姿态／翻车，Yaw相对底盘角仍用电机编码器。</p>
<h2>功率计</h2>
<ol><li>CAN2标准帧0x31、DLC4；两个有符号小端int16，分别×0.1W和×0.1J。</li><li>power_frames增长、power_received_ms刷新；power_w、energy_j与实际工况吻合。</li><li>实测帧周期，核定超时和低缓冲阈值，确认后power_confirmed=1。功率数据新鲜、能量足够才允许运行。</li></ol>
<p>当前源码预算40W、警戒200J、停车100J、恢复250J；超时200ms、停车时限2000ms，均须验证。首版是保守反馈限幅，不能据此宣称严格保证所有瞬态功率。</p>
<h2>最终放行</h2>
<p>五轴反馈／零位、Yaw限位、两轮健康、IMU有效、功率新鲜足量、真实姿态配置都就绪；g_vehicle_health.healthy=1、reasons=0。机械臂不收拢时拒绝行驶应有效。</p>'''),
page('第9批：Yaw位置双环，小角度开始', setting('第9批 Yaw位置双环｜robot_tune_mode=6',6,'速度PI已稳，再调位置P。')+'''
<h2>准备和切档</h2>
<ol><li>两轮仍悬空、整臂有支撑，线束中位和已测半幅仍有效；速度PI使用第7批记录值。</li><li>Yaw速度目标0、tune_enable=0、mode0，等所有本次相关机构静止。左下右中、5通道回中300ms。</li><li>tune_yaw_position_kp先保留2；Yaw测试上限先100mA。mode写6，enable保持0，先核对实际参数与当前位置目标；再写tune_enable=1，等ready=1。</li><li>切6时tune_yaw_target_deg自动对齐当前角，<b>不是自动回0</b>；确认没有突然跳目标。</li></ol>
<h2>给第一次很小的角度</h2>
<p>例如当前在0°附近且范围足够，tune_yaw_target_deg先写+1°，观察稳定后写回0；再试−1°。这里是相对人工线束中位的绝对目标角，不是“每次增加1°”。目标必须在已生效软限位里。</p>
<p>看target_deg、reference_deg、position_deg：参考受速度／加速度限制，实际应平稳靠近。再看velocity_target_deg_s和effort，判断是否顶限。</p>
'''+table(['现象','处理'],[
('慢但不抖且电流未顶限','目标对齐实际并停稳，再小步增加位置P，例如2→2.4。'),
('过冲／反复摆动','减位置P；确认内环已稳定，不能先靠加I修外环。'),
('电流顶限、目标总在前面','查负载／摩擦／限流；目标领先限制应生效，不盲目加P。'),
('靠边目标被约束','正常限位；先回安全中间，不靠扩大半幅解决。')])+'''
<p>修改参数时先把目标设为实际附近、等稳定，enable0后再改；核对yaw_parameter_result=0和实际值，再enable1，ready后给新小目标。6档持续保持目标，无自动到期。调完mode0退出；如果Yaw只由调参新增使能，退出会撤销它，先承托。</p>'''),
page('第10批：0档正常遥控，先慢速落地', setting('第10批 正常遥控慢速落地｜robot_tune_mode=0',0,'先悬空确认正常链，再小步落地，不先测快速档。')+'''
<h2>恢复正常控制</h2>
<ol><li>所有调参目标0、tune_enable=0、mode0；机械臂保留保持，两轮实际静止。</li><li>第8批真实条件全部就绪；左中夹取、右中，回中300ms、wait_neutral=0，再写命令4。</li><li>确认active=1、enabled_mask=31。若拒绝，查原因后重新明确请求，不能推杆等它突然生效。</li><li>机械臂移到已确认允许行驶的收拢姿态，左上行驶，右中，回中300ms，真实许可到位。</li></ol>
<h2>仍悬空先确认映射</h2>
<p>轻推左竖：两轮规范速度同号；松杆归零。轻推右横：差速转向，右推应顺时针；左横调Yaw。不要把“右横转车”和“左横转Yaw”混淆。</p>
<h2>落地只做小目标</h2>
<ol><li>空载、平地、留出停车空间，机构和线束确认安全；用已验证的保守限流与慢速上限。</li><li>很小前进，松杆，测停止距离；再后退。先确认方向与停车，不测试满杆。</li><li>低速小转弯，再测原地转向；倒车时右推仍应顺时针。</li><li>行驶切夹取／待机，必须先受控停车再放行机械臂新动作。</li></ol>
<p>当前底盘仍只有慢速参数。右下虽可显示fast_requested，<b>不会把底盘变成已调好的快速档</b>。慢速与停车尚未通过时不提高上限。</p>
<p>“wheels_stationary=1”只表示轮速低且持续达标，不能保证车身没有滑动；首轮夹取切换在正常平地验证。</p>'''),
page('第11批：退出、关控、故障要分别测', setting('第11批 停止清错与故障恢复｜退出和关控分开',0,'确认不续跑，并补齐带动力停车验收。')+'''
'''+table(['试验','应该看到'],[
('轮速调参中目标0／mode0','轮子停止；已使能机械臂仍保持，不接受新动作。'),
('6档退出','原已使能轴保持；只由调参增加的Yaw使能被撤销，先承托。'),
('低输出悬空时命令1','所有机构按全局关控策略失能；轮提交电流0。机械臂先承托。'),
('关闭遥控／所调电机反馈丢失','取消出力，健康恢复后不能自动续跑；先回0再显式选择／恢复。'),
('调参时偏杆或离开左下右中','调参被拒绝／锁禁，不应切回摇杆突然动作。')])+'''
<h2>清错只是清错，不是开控</h2>
<p>Arm／Yaw：排除原因后g_robot_debug_command=3；底盘：g_chassis_debug_request.mode=1，最后它自己的request_seq加1。这是少数保留的维护申请，日常调轮不写这张表。故障清完不自动使能，掉失零位要重新安全校零。</p>
<h2>功率限制下也要能停</h2>
<p>在正常mode0、保守低速和可控环境中验证低能量／功率反馈超时的受控停车；记录停车距离和电流。推进限流不能无条件把反向制动也削没。故障断线后不能承诺闭环刹停，零电流也不等于机械锁止。</p>
<p>低能量恢复到阈值也不能自动续跑：等停稳、数据健康，按本地维护流程清错，再按0档正常恢复步骤明确请求控制。不在满载坡道上首次试断反馈。</p>
<p><b>通过：</b>退出和全局关控的行为清楚，所有故障恢复均需明确动作；限功率状态停车距离已实测满足现场要求。软件逻辑测试不能代替这一步。</p>'''),
page('第12批：把有效参数留在源码里', setting('第12批 固化参数与重启复验',0,'保存实测值，重烧之后从安全步骤重新验证。')+'''
'''+table(['记录什么','写回哪里'],[
('左右ID、方向、轮径／轮距、总比','Chassis/chassis_config.h。'),
('轮速P／I，正常限流，速度与斜坡','Chassis/chassis_config.h；测试限流与正常运行限流分开核定。'),
('Yaw方向、位置P、速度P／I、上限','Arm/yaw_config.h。'),
('Yaw已测线束半幅／制动余量','Yaw安全配置和启动后的在线半幅申请流程；不得把每次开机位置当中位。'),
('IMU、功率、机械臂可行驶姿态','各自Imu／Power／Vehicle配置。'),
('四轴关节参数','由队友在Arm/arm_config.cpp核对固化。')])+'''
<h2>最后执行一次完整复验</h2>
<ol><li>停目标、mode0；承托机械臂、命令1，再断动力。</li><li>把已确认的参数写回源码，编译Debug并烧录；刷新FreeMASTER符号，重新打开工程。</li><li>重新验证心跳、命令1和遥控失联；按实物基准安全校零、申请Yaw半幅，等待IMU标定。</li><li>先低输出悬空复验，再恢复正常慢速落地；确认重启没有旧调参目标自动续跑。</li></ol>
<p>FreeMASTER在线修改在RAM，重启会丢。保存.pmpx不等于把运行参数写进Flash。每次记录：源码版本、模式、目标、实际生效参数、负载、方向、曲线和停车距离。</p>
<p class="note">正常调试一直用同一Debug整车固件。1～6是运行时档位；不能靠切Group改变档位，也不用为换调参对象重烧。</p>''')]


FAULTS = [
page('出问题先按这张表停下来', '''
<p>这份与“底盘排障.pmpx”配套，仍使用同一Debug整车固件。先判断要保留机械臂保持，还是必须全局关控；再查状态，不能靠乱加P／限流试运气。</p>
'''+table(['你现在遇到什么','先做','打开Group'],[
('轮子不对／异常运动','目标0或mode0；危险时命令1，承托机械臂。','排查0、3、4。'),
('不想再调轮，但机械臂还要保持','目标0、mode0；不要用命令1代替退出。','排查0。'),
('Yaw异常／线束受拉','立即命令1，承托；检查中位与实际角。','排查6、7。'),
('所有变量不更新','先停运动／动力；核对固件与ELF，再查任务。','排查0。'),
('ready=0、blocked=1','mode0，排除原因；停稳并回中300ms后明确重选。','排查0、1、3／7。'),
('正常0档不能开控','查真实整车条件；不能借调参档伪造放行。','排查5、8。')])+'''
<p class="stop">左上在本整车固件是行驶，不是关控；左下是保持，不是撤力。全局命令1会撤掉机械臂保持。FreeMASTER断通信／关闭示波器不会自动停止速度档。</p>
<p>current_ma是已接受入队的提交量，effort是控制计算／提交量，都不是电机已执行的应答。时间戳必须持续更新，旧online=1不能当作新鲜反馈。</p>'''),
page('排查0／1：心跳、切档和回中', '''
<h2>先看更新时间，不先看PID</h2>
<p>uwTick、g_robot_status.updated_ms、g_robot_tune_status.updated_ms（调参／退出保持期间）、g_chassis_status.updated_ms必须更新。静态mode0时调参快照不一定持续产生，不能只靠它判任务死活。</p>
<h2>g_robot_tune_status.reason</h2>
'''+table(['值','含义','现在怎么做'],[
('0','正常／无调参','查看mode，是否已经退出。'),
('1','调参就绪','还要核对ready=1、blocked=0和所选owner状态。'),
('2','未就绪','查映射确认、左下右中、5通道±30、反馈、配置和故障。'),
('3','关控或严重条件中断','先承托，排除失联／故障；回0再选，不自动恢复。'),
('4','模式／目标非法','检查模式0～6、有限数值和本档范围；回0再选。'),
('5','非零电流达到500ms','本次正常保护结束；目标0、mode0、停稳后再选1／2。'),
('6','进入时没停稳','等所调对象和已使能保持轴静止，回0再重新选档。')])+'''
<p><b>reason=7：</b>输出未开启，tune_enable=0；仅选档时这正常，尚未检查／放行本次执行。准备参数、目标0、停稳回中300ms，再明确写enable1，等待ready1后给小目标。</p>
<p><b>重要：</b>切档和enable0→1都会清旧运动目标，位置对齐当前角。选档→参数→enable1→等ready1→最后小目标；不要把运动目标提前写好期待自动执行。已有锁禁不能通过enable0→1解除，从1跳2也不能清锁。</p>
<p>回中检查包括拨轮channel[4]。拨杆左下／右中必须与实物索引对应，不能只凭遥控外观猜。全部回中至少300ms后再进入非零档。</p>
<h2>单独核对原始输入和处理后映射</h2>
<p>三工程都可打开“遥控映射检查｜原始与处理后”。mode0、enable0、命令1关控即可观察，不需开控。当前配置raw channel3→remote_axis[0]左Y、2→[1]左X、1→[2]右Y、0→[3]右X，四方向均+1待实物确认；switch0→左、switch1→右，原码上1中3下2。</p>
<p>原始满杆约±660、中位0；处理后范围±1、含死区。只动一杆对比对应轴，同时检查online、received_ms、remote_age_ms、good_frames确保是新数据。映射错修改operator_config.cpp，不写只读remote_axis或快照。</p>
<p>正常0档在左中／左上回中后用命令4明确恢复；左下写4不排队，健康恢复不自动出力。</p>'''),
page('排查2／4：CAN与两轮不转', '''
<h2>按这个顺序查</h2>
<ol><li><b>供电和身份：</b>CAN2左ID1、右ID2，反馈0x201/0x202，0x200第一／二槽；CAN1的同号ID不冲突。</li><li><b>新鲜反馈：</b>feedback_frames增长、feedback_ms刷新；只看online旧值不够。手转确认左右下标没有串。</li><li><b>总线：</b>can_tx_error_count[1]、rx_dropped是否继续增加；查波特率、接线、终端电阻与供电。</li><li><b>许可：</b>robot_tune_status.ready=1且blocked=0；chassis tune_blocked=0、faults=0。</li><li><b>最后才看PI：</b>目标、实际、电流同图比较；方向错先停，不加P。</li></ol>
'''+table(['现象','解释与处理'],[
('顶层ready=1但底盘仍不出力','底盘有本地否决；查g_chassis_status.tune_blocked、tune_result、faults和功率。'),
('非所选轮有提交电流','停止，核对mode、ID槽位、符号和是否有其他发送者；不能继续测。'),
('电流有但很小不动','可能摩擦未克服，也可能协议／供电问题；先查实物反馈再小步。'),
('电流一给速度就飙升／负反馈反了','立即关控，核对direction和目标／反馈符号；禁止进入闭环。'),
('measured_rpm很大但车轮看起来慢','这里是3508转子rpm，总比约19；不能当轮胎rpm。')])+'''
<h2>当前提交和真正制动不是一回事</h2>
<p>CAN发送异常后会撤除输出并清旧帧；物理断线后不能承诺控制帧能送达，零输出也不等于机械抱死。带动力的停止行为仍须低输出实测。</p>'''),
page('排查3：调轮参数与机械臂保持', '''
<h2>底盘参数要看实际值</h2>
<p>tune_wheel_kp／ki／current_limit_ma是申请输入。看g_chassis_status.tune_result，以及g_chassis_tune_status.result和实际velocity_kp／ki／max_current_ma。日常调参无需wheel、duration、confirm、seq。</p>
'''+table(['轮参数结果','含义'],[
('0 / 1','尚未处理／成功。成功仍要核对实际生效值。'),
('2','参数非法、非有限数值或限流超界；P需合法正值，I非负，测试限流1～300mA。'),
('3','忙，先目标0并静止再改。'),
('4','反馈／配置／源时间或本地健康未就绪。')])+'''
<h2>机械臂不应该因为调轮就自动使能</h2>
<p>开始调轮前enabled_mask=0，切1～4后仍应0。若原已正常抬起、左下保持，调轮期间这些轴继续保持。保持轴必须反馈新鲜、已校零且无故障；出现故障／失联会触发全局安全策略。</p>
<p><b>轮子正常停、机械臂突然软了：</b>查是不是写了命令1、遥控失联、关节掉线或保持失败。日常退出应目标0、mode0；不要用全局关控退出。</p>
<p><b>mode0后摇杆没反应但臂还保持：</b>这是退出等待；正常条件全部就绪后，左中／左上回中300ms，命令4恢复。待机不自动开控。</p>
<p><b>举不起来：</b>源码两DM各0.3Nm只作起步上限，先承托，与队友核对负载和力矩，不靠增大I硬顶。若正常整车保护尚未准备好，先跳过抬臂批次。</p>'''),
page('排查5／8：正常整车为什么不放行', '''
<h2>0档检查真实条件，1～6档检查本档限制</h2>
<p>功率未确认时，只允许≤300mA的独立悬空调轮；一旦功率已确认，它的反馈和能量约束也在调轮时生效。正常0档一直要求完整真实保护。IMU安装和阈值已确认后，调参也不能绕过失效／翻车。</p>
<h2>g_chassis_status.power_phase</h2>
'''+table(['值','含义'],[
('0 / 1 / 2','未放行／就绪／行驶。'),('3','低能量受控停车。'),
('4','功率反馈失联受控停车。'),('5','禁行锁存，需恢复条件、停稳并清错，不自动续跑。')])+'''
<p>power_reason：0无、1未确认、2配置非法、3没有功率计、4低能量、5反馈丢失、6停车超时、7已停、8外部关控中断。先看power_frames、power_received_ms、energy_j，不写只读许可。</p>
<h2>g_vehicle_health.reasons位图</h2>
'''+table(['位值','优先检查'],[
('1','姿态／配置未确认，Yaw在线范围等真实配置。'),('2 / 4','底盘快照／机械臂快照超时。'),
('8 / 16','IMU无效／翻车。'),('32','轮故障或轮反馈不新鲜。'),
('64 / 128','功率／能量禁止，或机械臂故障。')])+'''
<p>位值可相加；例如72=64+8，表示功率和IMU都有问题。g_vehicle_health.healthy=1、reasons=0才算真实整车健康，机械臂不在允许收拢范围还可能拒绝行驶。</p>
<p>不能通过在FreeMASTER把许可全写1解除；任务持续根据真实反馈生产互锁。先完成第8批再恢复0档正常遥控。</p>'''),
page('排查6／7：Yaw不动、限位被拒', '''
<h2>先看四个硬条件</h2>
<p>CAN1 GM6020电流模式反馈持续；真实线束中位已人工校零；g_yaw_limit_status.ready=1且半幅真实生效；enable0选5／6并核对参数、左下右中、5通道回中300ms，明确enable1、ready1后给小目标。</p>
<p>若Yaw是本次调参新增使能，退出会撤销它；其他原本失能轴不应被顺带使能。检查joint[0].enabled及enabled_mask，不能只看整车active。</p>
<h2>限位结果／Yaw参数结果用这一张</h2>
<p>适用于g_yaw_limit_status.result、g_robot_tune_status.yaw_parameter_result，以及g_arm_status.last_result；本表0才是成功。</p>
'''+table(['值','含义／处理'],[
('0','成功，同时核对active_half_range和实际生效P／I／上限。'),
('1 / 2','参数非法／配置未确认。'),('3','忙：限位必须所有轴失能；参数先目标0、静止、enable0再改。'),
('4 / 5','不在线／没校零，先修连续反馈并安全重校。'),
('6 / 7','故障锁存／仍在动；排除原因、停稳。'),
('8','新范围排除了实际角、目标或参考；先安全回中。')])+'''
<p>半幅直接写tune_yaw_half_range_deg；被拒绝后不迟到生效，要在安全条件恢复后重新编辑一个已核对的值。不要再填旧限位序号。</p>
<p>mode5／6命令2只校Yaw；mode0命令2校五轴，必须摆好全部联合参考且失能。不能为消报警在任意姿态重新认零。</p>
<p><b>靠边变慢：</b>看实际position_deg和受限velocity_target_deg_s，可能是正常制动；不要用增大P或半幅突破。<b>目标写了却没动：</b>先查切档或enable上升是否清了目标，再查enable／ready／blocked和实际生效参数；ready1后再写小目标。</p>'''),
page('保护记录与清错：当前、历史分开看', '''
<p>g_robot_status.protection_current_reasons是当前原因；protection_latched_reasons是历史锁存。位图可相加。历史非零不等于现在所有原因仍存在。</p>
'''+table(['位值','含义'],[
('1 / 2','启动保护／配置未确认或非法。'),('4 / 8','遥控失联／互锁过期或无效。'),
('16 / 32','显式全局关控／控制任务周期异常。'),('64 / 128','机械臂故障／开控被拒。'),
('256 / 512','底盘故障／IMU故障。'),('1024 / 2048 / 4096','倾覆／整车状态异常／功率保护。')])+'''
<h2>底盘本地faults</h2>
<p>g_chassis_status.faults：1配置、2反馈超时、4传输、8周期、16命令超时、32温度、64能量。复合值按位拆开，先恢复真实故障，再维护清错。</p>
<h2>只记住两条清错路径</h2>
<p><b>Arm／Yaw：</b>承托、停机，排除原因后命令3；<b>底盘／功率：</b>本地维护mode=1，最后维护request_seq加1。不是把robot_tune_mode写1清错；robot_tune_mode=1是左轮电流测试。</p>
<p>清错不使能、反馈丢失可能让calibrated清0，必须按真实标记安全重校。调参锁禁还要先回0再明确选档；正常遥控还需正常条件与命令4。</p>
<h2>找别人帮查时带上这几项</h2>
<p>本次源码／Debug ELF、当前批次、robot_tune_mode与reason／ready／blocked、所写目标与实际参数、本地faults、时间戳、曲线和实物现象。不要只发“没反应”。</p>
<p class="small">本说明与源码逻辑核对不代表实车通过。方向、承重保持、撤力后行为、线束软限位与功率停车距离都需实机低输出验证。</p>''')]


ARM = [
page('机械臂只做一件事：慢抬后保持', '''
<p class="note">目的只是把机械臂移到已确认安全的位置，为底盘调轮留空间。用同一Debug整车固件，<b>mode0抬起 → 左下保持 → 切1～4调轮 → mode0退出仍保持</b>。</p>
<p>打开“机械臂简易调参与排障.pmpx”，与其余两份工程共用同一个Debug ELF。停止上一份通信再打开这一份，切换Group不切档。独立“遥控映射检查｜原始与处理后”可在mode0、enable0、命令1关控时观测，无需开电机。</p>
<p>当前配置channel3→remote_axis[0]左Y、2→[1]左X、1→[2]右Y、0→[3]右X，方向均+1待实物确认；switch0左／switch1右，上1中3下2。原始满杆约±660、中位0；处理后±1含死区。对比时同时看online、good_frames和接收时间，保证是新鲜数据。</p>
<h2>开始前必须满足</h2>
<p>这是0档正常整车操作，需要真实IMU／功率／互锁、五轴在线校零、Yaw人工中位和在线软限位、实物配置确认。<b>若第8批尚未做好，先不抬臂，继续独立悬空调轮／Yaw。</b>不能用假许可临时放行。</p>
'''+table(['joint下标','轴','只需要记住'],[
('0','Yaw','无滑环，真实线束中位与小范围。'),('1','肩，大pitch DM','左竖；与肘联合零位。'),
('2','肘，小pitch DM','右竖；自身驱动角与肩有关联。'),('3 / 4','roll／夹爪M2006','也要在线校零；本操作先不主动控制。')])+'''
<p class="stop">两DM各0.3Nm的源码启动上限不保证举得起实物。先用支架承托，任何异常先接住再全局关控。左下是保持，命令1是撤力，两者不能混用。</p>'''),
page('第0、1步：承托，摆好联合零位', setting('第0步关控与连接，再到第1步双Pitch联合零位',0,'只校零，不开控、不抬起。')+'''
<ol><li>停轮、mode0；整臂可靠承托，命令1，确认enabled_mask=0。</li><li>uwTick、g_arm_status.updated_ms更新，五轴online=1、反馈新鲜，无需猜某个旧online值。</li><li>请队友把两个pitch摆到约定的<b>联合参考姿态</b>；roll对参考，夹爪对安全张开参考；Yaw对真实线束中位标记。</li><li>全部实际静止，再写g_robot_debug_command=2。<b>mode0的命令2校全部五轴</b>，不能只摆两个DM。</li><li>last_result=0、五轴calibrated=1，肩／肘角与规定参考一致；随后按底盘说明第6批直接申请Yaw已测半幅。</li></ol>
<p>两个pitch的关系是q肩=a1、q肘=a2−a1，不能把第二台DM在任意姿态独立认零。手动联合姿态由队友确认，你不用现场重新做逆解。</p>
'''+table(['结果','先查'],[
('3','仍有轴使能；先承托、全部关控。'),('4 / 5','反馈未在线／零位未建立。'),
('7','仍在动，等静止。'),('校后calibrated又变0','反馈连续性丢失；修反馈并重新安全校零。')])+'''
<p>mode5／6的命令2只校Yaw，不用于本页五轴联合校零。重启后零位需重新建立，不把开机当前位置自动当零。</p>'''),
page('第2步：0档开控，只慢抬肩和肘', setting('第2步 0档左中简易抬肩抬肘｜遥控操作',0,'每次只动一个输入，先不碰其他轴。')+'''
<ol><li>两轮仍悬空，整臂有承托；正常整车健康已通过。mode0，左中夹取、右中，操作输入回中至少300ms。</li><li>确认wait_neutral=0，再写g_robot_debug_command=4。命令被取走，active=1、enabled_mask=31，才允许推杆。</li><li>只轻推左竖，肩移动很小角度立即回中。实际方向与目标应一致；不一致承托并命令1，先查方向。</li><li>肩正常后，只轻推右竖调肘，同样小动即回中。保持另一个输入不动。</li><li>逐步移到队友确认不会干涉、不会碰地、能承重保持的姿态。所有输入回中。</li></ol>
<h2>只看两条曲线即可</h2>
<p>肩与肘的target_deg／position_deg应平稳接近，effort单位Nm；参考轨迹可以比直接目标慢。effort一直顶限且仍下垂时先承托，不能继续推杆堆积目标或猛加I。</p>
<p><b>没开控：</b>先查真实健康、五轴在线／校零、Yaw限位、回中和当前保护。拒绝的开控不自动补执行，条件修好再明确写命令4。</p>
<p class="stop">正常固件左上是行驶，不能用它关控。紧急时命令1会撤掉保持，先有机械支撑。不要在机械臂尚不能稳定托住时开始轮测试。</p>'''),
page('第4步：左下保持，再切轮子档', setting('第4步 左下待机保持｜同固件切1到4档测轮','0→1～4','保留已经使能的机构保持，不额外使能未开轴。')+'''
<ol><li>抬臂结束，所有输入回中，左下待机、右中。</li><li>看g_robot_status.mode=3、active=1，enabled_mask仍31；几秒内角度不持续下垂、无明显抖动。</li><li>5通道含拨轮均在±30，持续300ms；两轮实际静止且继续可靠悬空。</li><li>停止机械臂工程通信，打开底盘调参第5批。选择mode1～4，确认目标0和参数正确，再写tune_enable=1；等ready=1，最后给已验证的小电流／轮速目标。</li><li>原已使能的机械臂保持；未使能轴不因切档自动开启。退出调轮先目标0，再mode0，机械臂继续保持。</li></ol>
<h2>要再调机械臂姿态</h2>
<p>mode0，停轮；左中夹取，所有输入回中300ms，确认正常整车条件后写命令4恢复遥控，再轻推肩／肘。留在非零轮档推杆只会取消调参，不应自动转成抬臂。</p>
<h2>收工</h2>
<p><b>停轮／mode0 → 等实际静止 → 支架接住机械臂 → 命令1全局关控 → 确认enabled_mask=0 → 断动力。</b>关闭FreeMASTER通信不是停止；mode0退出也不是撤力。</p>
<p><b>保持不稳：</b>先承托关控，解决机械臂问题再调轮。保持轴反馈掉线或故障可能触发全局撤力，不能靠底盘调参档绕过。</p>'''),
page('第3步可选：少量调参和简单排障', setting('第3步 少量改PI与限流｜承托后全局关控',0,'能稳定抬起保持就先不改，只处理必要参数。')+'''
<p>这部分沿用队友关节参数申请，不再增加一套机械臂调参输出模式。先停轮、承托整臂、全局关控，全部轴失能且静止，再完整填写下表。</p>
'''+table(['变量','以肩为例；肘需另选并填自己正确值'],[
(v('g_arm_tune_request.joint'),'1=肩，2=肘。'),
(v('g_arm_tune_request.position_kp'),'例如保留源码2，合法正值。'),
(v('g_arm_tune_request.velocity_kp'),'例如源码起点0.03。'),
(v('g_arm_tune_request.velocity_ki'),'先0，稳定后必要时少量I。'),
(v('g_arm_tune_request.effort_limit'),'本次核定Nm上限，不超过源码允许值；当前启动上限0.3Nm。'),
(v('g_arm_tune_request.request_seq'),'其他项全部填齐，最后读当前数值再加1。')])+'''
<p>g_arm_tune_status.result=0才成功；active_joint、active_seq和实际参数必须对应本次。失败申请也消耗序号，修好条件后用新序号，不会迟到生效。参数成功不自动开控，按第2步重新恢复。</p>
'''+table(['现象','先怎么处理'],[
('不使能／不抬起','查正常整车健康、五轴在线与校零、回中和命令4。'),
('目标变、实际不动','查使能、反馈、电机状态和限流，不只加位置P。'),
('effort顶限仍下垂','承托，与队友核对负载／机构／力矩；不猛加I。'),
('方向反／抖动','立即承托关控；查映射、方向与反馈，回保守参数。')])+'''
<p class="small">只做能安全抬起并保持的最小调试；满载性能和更大力矩由双方另行核定。</p>''')]


if __name__ == '__main__':
    import sys
    sys.stdout.reconfigure(encoding='utf-8')
    pdf('00_分批调参步骤.pdf', STEPS)
    pdf('01_排障与变量说明.pdf', FAULTS)
    pdf('02_机械臂抬起与保持.pdf', ARM)
