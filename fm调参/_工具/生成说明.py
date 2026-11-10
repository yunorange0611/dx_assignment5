"""按 project4 的操作卡方式生成分批调参、排障和简易抬臂 PDF。"""
from pathlib import Path
import html
import pymupdf as fitz

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = fitz.Archive('C:/Windows/Fonts')
CSS = '''
@font-face {font-family:cn;src:url(simhei.ttf);}
@font-face {font-family:mono;src:url(consola.ttf);}
body {font-family:cn,sans-serif;font-size:10.5pt;line-height:1.42;color:#192e3b;}
h1 {font-size:20pt;line-height:1.22;color:#124e70;margin:0 0 12pt;}
h2 {font-size:13pt;color:#12628a;margin:12pt 0 6pt;}
h3 {font-size:11pt;margin:9pt 0 4pt;}
p {margin:5pt 0 7pt;} li {margin-bottom:5pt;}
ol,ul {padding-left:20pt;margin:6pt 0 9pt;}
table {width:100%;border-collapse:collapse;margin:8pt 0;font-size:9pt;line-height:1.35;}
th {background:#e1eef5;text-align:left;}
td,th {border:0.5pt solid #a8bdca;padding:5pt;vertical-align:top;}
code {font-family:mono,cn;font-size:8.7pt;color:#125473;overflow-wrap:anywhere;}
pre {font-family:mono,cn;font-size:8.5pt;line-height:1.4;padding:8pt;background:#f0f5f8;white-space:pre-wrap;}
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


def setting(firmware, group, aim):
    return '<p class="tag">固件：'+firmware+'</p><p><b>打开的 Group：</b>'+group+'</p><p class="note"><b>本页只做：</b>'+aim+'</p>'


def code(lines):
    return '<pre>'+html.escape(lines.strip())+'</pre>'


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
    doc.set_metadata(dict(title=filename.removesuffix('.pdf'),author='工程车电控协作',subject='2026-11-11：按批次操作；变量使用源码名；与当前FreeMASTER分组配套'))
    doc.subset_fonts()
    path = ROOT / filename
    # Write the complete PDF without removing the existing file first. Some PDF
    # viewers allow file writes but keep a handle that prevents delete/replace.
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
page('先按这张路线走', '''
<p>这份说明按 project4 的方式重写：<b>每批先告诉你点哪一组，再告诉你填什么、看什么、没通过怎么办。</b>按顺序做，做完一批再看下一批，不必一次读完。</p>
<p class="note"><b>你现在准备上车：先只做第0批和第1批。</b>第0批不接电机动力；第1批只看反馈、不发运动请求。两批通过后，才做左轮短脉冲。</p>
'''+table(['批次','本批结束时，你应该得到什么'],[
('第0批','软件能收到关控命令；遥控失联能被发现。'),
('第1批','知道哪个通道、拨杆、反馈对应哪个实物；两轮正方向明确。'),
('第2、3批','左轮、右轮分别完成很短的正反向电流测试。'),
('第4批','一组保守的轮速 PI，左右轮都能稳定跟随。'),
('第5批（可稍后做）','机械臂能慢抬、左下保持，保持时可以调轮。'),
('第6、7批','Yaw线束中位、小范围限位、速度 PI。'),
('第8、9批','真实整车保护就绪，再调 Yaw 位置 P。'),
('第10、11批','慢速落地、停车、故障与功率停车验收。'),
('第12批','把参数写回源码，重启再验证。')])+'''
<h2>本车和以前的车不同在哪里</h2>
<p>底盘只有左右两轮，左 ID1、右 ID2，走 CAN2。Yaw 是 CAN1 的 GM6020 ID1。这里没有旧车的跟随、小陀螺和四轮运动学，旧车 PID 数值也不能直接照抄。</p>
<p>文中的起步数字是<b>用于小步试验的示例</b>，不是已经实测好的参数。上一步没有通过，就不要靠增大电流或 P 值往下推进。</p>
<p class="small">更新：2026-11-11。需要阅读的内容都在 PDF；_工具 内是生成脚本，不需要先读。</p>'''),
page('开始前：打开对的固件和文件', '''
<h2>1．先选本批固件</h2>
'''+table(['你现在做什么','编译、烧录哪一个','切换分组.cmd选项'],[
('第0～5批：底盘悬空、抬臂保持','ChassisBenchDebug','1'),
('第6～7批：只测Yaw速度','YawBenchDebug','2'),
('第8～12批：整车、Yaw位置、落地','VehicleDebug','3')])+'''
<p><b>Debug 不是底盘调参固件。</b>普通 Debug 只有四轴机械臂，不启动底盘。点击 FreeMASTER 的 Group 也不会给板子切固件。</p>
<h2>2．按顺序连接</h2>
<ol><li>断开电机动力，支撑机械臂、架稳车身；自行编译并烧录本批固件。</li>
<li>停止 FreeMASTER 通信，关闭这三份工程。运行本文件夹的“切换分组.cmd”，选择上表的数字。</li>
<li>打开“底盘调参.pmpx”。Project → Options → MAP Files 中应指向<b>这次所烧固件对应的 ELF</b>。</li>
<li>沿用你原有的通信设置，Start Communication。先看 uwTick 是否持续增加。</li></ol>
<p>切换工具只更新本地分组和符号地址，不会编译、烧录或让电机动。每次重新编译或换固件后，都重新运行一次工具，并重新打开工程。</p>
<h2>3．只打开一份工程通信</h2>
<p>底盘操作用“底盘调参”；查异常换“底盘排障”；第5批可以换“机械臂简易调参与排障”。切换前停止上一份通信。机械臂简易文件始终配 ChassisBenchDebug。</p>
<p class="stop">电机一旦获电，就要有随手能切断动力的方式。机械臂撤力可能下落，先承托。不要通过打断点暂停控制任务来测试关控。</p>'''),
page('先学会：填写、提交和停止', '''
<h2>变量名现在与源码一致</h2>
<p>例如 g_chassis_debug_request.mode 就是列表中完整的变量名。<b>双击它的 Value 格，输入数字，回车。</b>本文每批会明确列出“要写的变量”；状态变量只看，不改。</p>
<h2>request_seq 就相当于“提交一次”</h2>
<p>先把一张申请表的其他字段填完，<b>最后再改它自己的 request_seq</b>。先看现在是多少，再加1：现在是0就写1，现在是7就写8。不要每次都写1，也不要连续刷新序号。</p>
'''+table(['这张表','它的提交序号','用来干什么'],[
('g_chassis_debug_request','request_seq','停止、清底盘故障、电流或轮速短测试。'),
('g_chassis_tune_request','request_seq','修改轮速 PI 和运行电流上限。'),
('g_yaw_limit_request','request_seq','申请Yaw允许的左右半幅。'),
('g_yaw_debug_request','request_seq','Yaw速度短测试。'),
('g_arm_tune_request','request_seq','修改Yaw或机械臂关节参数。')])+'''
<p>这些序号<b>各管各的</b>，不用相互相等。consumed_seq 跟上说明“程序看过了”，成功还要看 result 和 active_seq。被拒绝也算看过；条件修好后，需要新的序号，不会自己补执行。</p>
<h2>全局命令不用 request_seq</h2>
'''+table(['g_robot_debug_command填什么','含义'],[
('1','全局关控；程序取走后自动回0。'),('2','按当前固件人工校零，范围可能是四轴、仅Yaw或全部五轴。'),
('3','清机械臂/Yaw故障；不清底盘故障。'),('4','请求机械臂/整车开控；条件不满足会拒绝。')])+'''
<p class="stop">关控不是关闭 FreeMASTER。ChassisBench 的左上只关机械臂，仍可接受轮测试；Vehicle 的左上是行驶。后文不把“左上”统称为整车急停。</p>'''),
page('第0批 A：先不接动力，只测关控', setting('ChassisBenchDebug','第0批 安全常驻','确认软件收到关控命令，本页不让任何电机出力。')+'''
<h2>准备</h2>
<ol><li>断电接线，隔离底盘、Yaw和机械臂的动力供电。只给C板、遥控接收机供电。</li><li>两轮可靠悬空，机械臂承托。硬件/映射确认仍保持未确认状态。</li><li>不写命令2或4，不提交底盘 mode=2/3，也不填写运动确认码。</li></ol>
<h2>先检查这些观察量</h2>
'''+table(['变量','应看到什么'],[
(v('uwTick'),'持续增加。'),(v('g_robot_status.updated_ms'),'持续增加，不能停在旧数值。'),
(v('g_chassis_status.updated_ms'),'持续增加。'),(v('g_arm_status.enabled_mask'),'0，机械臂失能。'),
(v('g_chassis_status.enabled')+' / '+v('g_chassis_status.bench_active'),'都是0。'),
(v('g_chassis_status.current_ma[0]')+' / '+v('g_chassis_status.current_ma[1]'),'都是0。')])+'''
<h2>实际操作</h2>
<ol><li>记下 g_robot_status.stop_epoch 的当前值，例如是0。</li><li>把 g_robot_debug_command 写成1，然后等待下一次刷新。</li><li>命令应自动回0；stop_epoch 应增加，例如0变1。</li><li>g_robot_status.protection_latched_reasons 应包含16这一位。比如原来是3，触发后可能变19；不要求整个数值等于16。</li><li>再次检查机械臂使能、底盘使能和两路电流请求，仍全部为0。</li></ol>
<p><b>通过：</b>时间戳在更新，命令被取走，关控次数增加，关控原因留下记录。</p>
<p><b>不通过：</b>时间戳冻结先查通信/固件；命令不回0查控制任务。不要继续开控尝试。</p>'''),
page('第0批 B：仍不接动力，测遥控失联', setting('ChassisBenchDebug','第1批 遥控与两轮反馈 → 第0批 安全常驻','确认关闭遥控器后，程序能发现失联。')+'''
<h2>按这五步做</h2>
<ol><li>打开遥控器。看 g_dr16_status.good_frames 持续增加，g_robot_status.remote_online=1。</li>
<li>记一下当前 good_frames，再关闭遥控器。</li><li>good_frames 应停止增加，g_robot_status.remote_age_ms 应不断增大。</li>
<li>超过约100ms并经过任务处理后，remote_online 应变0；保护原因应包含4这一位。</li>
<li>重新打开遥控器：在线应恢复，但机械臂、底盘不应因为“重新在线”自动出力。</li></ol>
<h2>这几种现象怎么判断</h2>
'''+table(['现象','现在怎么做'],[
('一直 remote_online=0','先看 good_frames、valid、consecutive_frames；查接收机接线和匹配固件。'),
('关遥控后 good_frames 仍增长','停止后续运动测试，核对接收机是否重复发送合法旧帧；不能仅凭看似在线就放行。'),
('当前保护原因没一直显示16','正常：关控请求可能只出现一个控制周期。第0批A看锁存原因和stop_epoch。'),
('CAN或电机反馈报错','本阶段未给电机供电，缺反馈或没有应答节点导致ACK错误是预期；不为消掉报错关闭保护。')])+'''
<h2>本批完成后在纸上记三项</h2>
<p>①命令1是否被取走；②stop_epoch是否增加；③关遥控是否变离线。</p>
<p class="note">这里验证的是“输入 → 软件保护 → 零输出请求”。还没有验证“运动中的电机真正撤力”。后面逐轮悬空测试通过后，再单独补做带动力的关控验收。</p>
<p>不要为了本批通过，把一串 confirmed 改成 true。第0批结束，仍不需要电机运动。</p>'''),
page('第1批 A：认清遥控每一个输入', setting('ChassisBenchDebug','第1批 遥控与两轮反馈｜不出力','把实物摇杆、拨杆与变量一一对应，不调PID。')+'''
<h2>要看的变量</h2>
<p>g_dr16_status.frame.channel[0] 到 [4]；g_dr16_status.frame.switches[0]、[1]；good_frames、valid、consecutive_frames。</p>
<h2>一步一步核对</h2>
<ol><li>遥控器打开，所有摇杆回中。good_frames增长，valid=1，consecutive_frames至少3。</li>
<li>只动左竖，记录哪个 channel 变化、向上是正还是负；回中。</li><li>只动左横，再只动右竖、右横；每次仅动一个输入并记录。</li>
<li>检查第5通道（拨轮）。它也必须回中。悬空测轮要求<b>全部5通道绝对值不超过30</b>。</li>
<li>只拨左边三段开关，确认究竟是 switches[0] 还是[1]变化；右边同样检查。</li></ol>
'''+table(['实物动作','正常原码 / 记录'],[
('摇杆中位','约0；满杆通常约±660。'),('拨杆上 / 中 / 下','分别为1 / 3 / 2。'),
('左竖 / 左横','本车配置暂定channel[3] / channel[2]，实物不符就据实修改。'),
('右竖 / 右横','暂定channel[1] / channel[0]。'),('左拨杆 / 右拨杆','暂定switches[0] / switches[1]；必须亲自确认。')])+'''
<h2>发现不一致怎么办</h2>
<p>在 2User_Driver/Remote/operator_config.cpp 修改通道索引、左右拨杆索引和正向。核对完成后才把 mapping_confirmed 设 true；重新编译、烧录、刷新 FreeMASTER 地址，再复查。</p>
<p><b>通过：</b>你不用猜，就知道每个杆对应哪个变量；全部回中满足±30，关闭遥控后帧停止。</p>'''),
page('第1批 B：认清左右轮反馈与方向', setting('ChassisBenchDebug','第1批 遥控与两轮反馈｜不出力','只用手转轮子，确定ID、反馈归属与向前正方向。')+'''
<h2>准备与操作</h2>
<ol><li>先断电，恢复底盘电调动力接线，再上电。两轮继续悬空，机械臂仍有承托；不提交运动请求。</li><li>两轮 feedback_frames 都应增长，feedback_ms 不断更新，measured_rpm 静止时接近0。</li>
<li>从车的行驶视角确认左、右。手转左轮，主要只有 measured_rpm[0]变化；右轮则是[1]。</li>
<li>手把左轮往“让车前进”的方向转：轮胎最下面一点相对车身向后走。观察 measured_rpm[0]符号。右轮重复。</li>
<li>规范后的两侧向前转都应为正。若某侧为负，修改 chassis_config.h 的 direction 对应项，从+1改−1或反过来；重新编译后手转复验。</li></ol>
'''+table(['变量下标','对应实物'],[
('[0]','左轮，CAN2 ID1，反馈0x201。'),('[1]','右轮，CAN2 ID2，反馈0x202。'),
('measured_rpm','已经过direction规范的转子rpm；不是车轮rpm。'),('feedback_frames / feedback_ms','前者累计帧数，后者最近反馈时刻。')])+'''
<h2>准备第2批前，源码只确认这两处</h2>
<p>① Chassis/chassis_config.h：ID、接线、反馈正向、轮径0.145m、轮距0.330m、总比19等核对后，hardware_confirmed=true。外部直装1:1，仍有3508内部减速比。</p>
<p>② Remote/operator_config.cpp：本页与上一页实测完成，mapping_confirmed=true。</p>
<p class="note">先只试轮，不必把机械臂 hardware_confirmed、pitch.mapping_confirmed 一起打开；左上失能路径允许机械臂尚未校零。功率计保护不参与悬空短脉冲，整车阶段再验收。</p>
<p><b>通过：</b>左1右2与反馈一致，两侧“向前”为正。没通过，不接速度闭环。</p>'''),
page('第2批 A：让左轮只动一小下', setting('ChassisBenchDebug','第2批 左轮ID1方向｜mode=2','只测左轮电流方向，暂时不调PI。')+'''
<h2>先把车和遥控摆好</h2>
<p>两轮悬空；机械臂承托且 enabled_mask=0；左拨杆上、右拨杆中、5通道都回中。两轮反馈正常，底盘 faults=0，等 wheels_stationary=1。</p>
<h2>按表从上往下填写</h2>
'''+table(['要写的完整变量','首次填写'],[
(v('g_chassis_debug_request.mode'),'2，电流测试。'),(v('g_chassis_debug_request.wheel'),'0，左轮。'),
(v('g_chassis_debug_request.value'),'50，单位mA。'),(v('g_chassis_debug_request.duration_ms'),'100，单位ms，只动0.1秒。'),
(v('g_chassis_debug_request.suspended_confirm'),'1398100816，即0x53555350。'),
(v('g_chassis_debug_request.request_seq'),'<b>最后</b>读取当前值，再加1。')])+'''
<p>悬空确认码只是你亲自确认“轮子已经架空”的声明，不是程序检测到了悬空。</p>
<h2>提交前先开示波器</h2>
<p>打开本组“左轮转子速度与最终电流”，开始记录。再按表提交一次。观察实物，不要把手伸进轮子。</p>
<p>100ms测试很短，Watch可能错过运行瞬间。consumed_seq 跟上说明看过请求；结束 result=8通常表示正常到期。</p>
<p class="stop">50mA可能不足以克服摩擦，轮子未必明显动。没有动作时先看下一页判断，不要直接改成300，更不要落地试。</p>'''),
page('第2批 B：看结果、反向、停止', setting('ChassisBenchDebug','仍在第2批；必要时切“底盘排障”的排查3/4','完成左轮正反向记录，并学会停止和清底盘故障。')+'''
<h2>正向脉冲后看什么</h2>
'''+table(['看什么','怎样判断'],[
('g_chassis_debug_status.result','1接受；8正常到期；4参数非法；5未就绪；7条件变化而取消。'),
('g_chassis_status.current_ma[0]','应在测试期间出现正的提交电流。50mA经编码量化可能约48.8mA。'),
('g_chassis_status.current_ma[1]','非选中右轮应为0。'),
('左轮实物与 measured_rpm[0]','应向前转、反馈为正。电流模式target_rpm为0是正常的。')])+'''
<p>电流显示是“程序接受发送的值”，不是电机实测电流。请求被拒绝时先查门槛；请求确实接受、无异常但克服不了摩擦时，可在静止后每次小幅增加，例如50→100mA，仍用100ms单次测试。未验证的范围不直接拉满。</p>
<h2>再做反向</h2>
<ol><li>等待左轮实际停止、wheels_stationary=1。</li><li>把value改成刚才已验证能小幅动作的同等幅值取负：例如正向100mA才动，就试−100；50已能动则试−50。其余仍为mode=2、wheel=0、duration_ms=100、确认码不变。</li><li>最后将同一张表的 request_seq 再加1；应反向转、measured_rpm为负。</li></ol>
<h2>随时停止怎么填</h2>
'''+code('g_chassis_debug_request.mode = 0\ng_chassis_debug_request.request_seq = 当前值 + 1')+'''
<p>也可以 g_robot_debug_command=1，取消当前脉冲；它同时撤机械臂保持，先承托。停止通信不是停止命令。命令1只取消当前轮测试，不永久锁住独立调轮入口；下一次测试仍要明确提交新序号。</p>
<h2>底盘 faults 非0怎么重来</h2>
<p>先排除接线/反馈/遥控原因，等轮静止；左上、右中、5通道中位。mode改1，最后request_seq加1。result=3后，确认底盘faults=0，再发新的测试。命令3只清机械臂，不清底盘。</p>
<p><b>通过：</b>左轮正反向正确，右轮零输出；到期撤力，无故障。带动力关控验收应在方向已验证、可立即切断动力时另做；无出力检查不能替代它。</p>'''),
page('第3批：右轮照表单独验证', setting('ChassisBenchDebug','第3批 右轮ID2方向｜mode=2','只换成右轮，重复正反向；不同时测试两轮。')+'''
<h2>仍保持原准备条件</h2>
<p>两轮悬空，左上、右中、5通道回中，机械臂失能，两轮在线、faults=0、wheels_stationary=1。</p>
'''+table(['要写的变量','第一次：正向','第二次：反向'],[
(v('g_chassis_debug_request.mode'),'2','2'),(v('g_chassis_debug_request.wheel'),'1，右轮','1'),
(v('g_chassis_debug_request.value'),'+50mA','−50mA'),(v('g_chassis_debug_request.duration_ms'),'100','100'),
(v('g_chassis_debug_request.suspended_confirm'),'1398100816','1398100816'),
(v('g_chassis_debug_request.request_seq'),'最后加1','实际静止后再加1')])+'''
<h2>每次都对照这三件事</h2>
<ol><li>current_ma[1]是本次提交电流，current_ma[0]保持0。</li><li>正脉冲让右轮向前、measured_rpm[1]为正；负脉冲反过来。</li><li>测试到期电流回0；实物允许因惯性滑转，等停稳再继续。</li></ol>
<p>50mA不一定克服右轮摩擦；本轮应独立确认小动作幅值，再用同等幅值正负测试，不直接照搬左轮阈值。若动了左轮，先停，回第1批查ID/接线；若方向反，停机后查direction，改源码、重编译，再从手转反馈检查开始。不要靠改PID补救方向。</p>
<h2>本批记录</h2>
'''+table(['记录项目','填写'],[
('左轮 direction / 最小可见动作电流','____________________________'),
('右轮 direction / 最小可见动作电流','____________________________'),
('左右反馈符号、到期撤力','通过 / 未通过：________________')])+'''
<p><b>通过：</b>左右轮都能分别正反向，选中关系和符号明确。接下来才允许速度PI。</p>'''),
page('第4批 A：先给轮速环一组小参数', setting('ChassisBenchDebug','第4批 两轮速度PI｜mode=3','先只用速度P，I暂时为0；申请参数不会让轮子转。')+'''
<h2>先停轮，再填写参数表</h2>
<ol><li>轮测试mode=0并提交新序号；等两轮静止。</li><li>确认底盘 enabled=0、bench_active=0、反馈在更新。</li><li>按下表完整填三个参数，最后提交参数表自己的序号。</li></ol>
'''+table(['要写的变量','保守起步示例'],[
(v('g_chassis_tune_request.velocity_kp'),'1.0，先沿用源码P起步。'),
(v('g_chassis_tune_request.velocity_ki'),'0，先不用积分。'),
(v('g_chassis_tune_request.max_current_ma'),'例如100mA，前提是第2/3批已经验证这个电流水平；否则取已验证的保守上限。'),
(v('g_chassis_tune_request.request_seq'),'最后加1；注意不是轮测试的request_seq。')])+'''
<h2>怎样确认真的生效</h2>
<p>看 g_chassis_tune_status：</p>
'''+table(['字段','通过标准'],[
('consumed_seq','等于你刚提交的参数序号。'),('result','1，才表示轮参数成功。'),
('active_seq','等于刚提交的序号。'),('velocity_kp / velocity_ki / max_current_ma','与这次填写一致。')])+'''
<p>result=2参数非法；3仍在运行，先停；4未就绪，查反馈、静止和通信。若底盘faults非0，先按第2批B用mode=1清本地故障，再申请参数。拒绝后不会自行生效，修好再加新序号。</p>
<p class="note">本车左右轮共用一组PI，但两边都要测。在线参数重启会丢失。源码启动上限是1000mA；悬空速度测试还受300mA硬上限约束，不能把1000当作这一步可用的测试输出。</p>'''),
page('第4批 B：先测左轮，再测右轮', setting('ChassisBenchDebug','第4批及其左右轮示波器','用小速度短测试观察P，不做长时间持续运转。')+'''
<h2>注意单位</h2>
<p>这里的50rpm是<b>3508转子每分钟50转</b>。按总比19，车轮约2.63rpm；不是轮胎每分钟50转。</p>
'''+table(['g_chassis_debug_request字段','这次填什么'],[
('mode','3，轮速测试。'),('wheel','0，先左轮。'),('value','+50，单位转子rpm。'),
('duration_ms','200，即0.2秒。'),('suspended_confirm','1398100816。'),('request_seq','其他填完后最后加1。')])+'''
<h2>每一次测试都这样做</h2>
<ol><li>两轮悬空；左上、右中，5通道回中；故障为0、轮子静止。</li><li>先打开左轮示波器，再提交上述请求。</li><li>看 target_rpm[0] 与 measured_rpm[0] 是否同号、实际是否向目标靠近；电流单独看，不能当速度曲线。</li><li>等停稳，value换−50，再提交新序号。</li><li>左右方向都测完，把wheel换1，重复+50、−50；同时打开右轮示波器。</li></ol>
<h2>曲线怎么看</h2>
'''+table(['现象','先做什么'],[
('目标和实际方向相反','立刻停，回去检查方向；不要增加P。'),
('几乎没动，电流一直顶上限','先查摩擦、输出上限与机构，不能据此断言P太小。'),
('跟得慢，但输出没顶限、方向正确','停稳后小幅增加P，例如1.0→1.2，再完整提交参数表。'),
('抖动、啸叫，电流正负快速摆动','先停，减P并检查反馈；不要再继续加。')])+'''
<p><b>当前目标：</b>先得到一组不抖、方向正确的P。200ms短测试不要求达到未经实测的固定精度。</p>'''),
page('第4批 C：确有静差，再加一点I', setting('ChassisBenchDebug','第4批 两轮速度PI','P已经稳定后，才判断是否需要积分。')+'''
<h2>先判断是不是该加I</h2>
<p>只有方向正确、反馈正常、P稳定，且重复测试总是稍低于目标、输出又没一直顶限，才考虑少量I。短测试太短、摩擦未克服、输出已饱和，都不是直接加I的理由。</p>
<h2>操作顺序</h2>
<ol><li>mode=0提交，等轮实际静止。</li><li>保留已验证P和电流上限，只把velocity_ki从0改一个很小的值，例如0.05。</li><li>三个参数都填成这次正确值；最后g_chassis_tune_request.request_seq加1。</li><li>确认result=1、active_seq和三个生效值正确。</li><li>重新用同样+50/−50rpm、同样200ms测左右轮，对比曲线。一次只改变一个参数。</li></ol>
<p>本车I按实际时间积分，单位为mA/(rpm·s)。project4旧PID的ki数值不能直接迁移。源码0.5只是占位，不能当成已经调好的答案。</p>
<h2>本批达标</h2>
<ul><li>两边正负速度都跟随同方向，无持续振荡或明显啸叫。</li><li>不是一直靠顶电流上限才动；到期、停止或条件丢失后可撤销。</li><li>保存本次P、I、电流上限和左右曲线。长时间稳态与落地负载，留到第10批再补验。</li></ul>
<p>单次短测试最长500ms；不要通过持续刷request_seq绕成连续运行。清错不是续跑，下一次测试仍要明确提交。</p>
<h2>下一步怎么选</h2>
<p>需要机械臂举起再调轮，做第5批；暂时不需要，可让机械臂继续承托失能，直接去第6批调Yaw。先保存轮参数，因为换固件后在线参数会丢失。</p>'''),
page('第5批：抬起机械臂，再调轮', setting('ChassisBenchDebug','机械臂简易调参与排障.pmpx；随后回底盘第5批','让机械臂慢抬、保持；不是重新整定整条机械臂。')+'''
<p>本批详细操作单独写在 <b>02_机械臂抬起与保持.pdf</b>，按“准备 → 校零 → 慢抬 → 保持 → 测轮 → 收工”执行。</p>
<h2>你先记住这四种状态</h2>
'''+table(['拨杆/状态','机械臂做什么','此时能否测轮'],[
('左上，enabled_mask=0','失能，需要支撑。','可走独立悬空短测试。'),
('左中，active=1','遥控慢抬/夹取。','不允许同时测轮。'),
('左下，mode=3、active=1、mask=30','四轴闭环保持；30就是0x1e。','四轴全部健康静止后才可短测。'),
('全局命令1或遥控失联','撤掉保持。','当前轮测试也取消，先承托。')])+'''
<h2>进入保持后怎么测轮</h2>
<ol><li>确认机械臂已稳定保持几秒，无下垂或抖动；右中、5通道回中。</li><li>停止机械臂那份工程通信，再打开底盘调参第5批。</li><li>两轮仍必须悬空。按第2～4批同样的方法提交单轮短脉冲。</li><li>切回左中、偏杆、任何关节反馈失效或保持不稳，当前轮测试应取消。</li></ol>
<p>只看两台DM不够，roll和夹爪也必须在线、校零、无故障、实际静止。本批不使用伪造的“允许行驶”变量。</p>
<p class="stop">两DM当前各0.3Nm启动上限不保证能举起实物负载。输出顶限仍下垂时，先承托并和机械臂队友处理；不猛加I，也不在线突破上限。</p>
<p><b>通过：</b>能慢抬、左下保持；保持静止时能测轮，离开保持或故障时取消。收工先停轮、接住机械臂，再关控。</p>'''),
page('第6批 A：切Yaw固件，找线束中位', setting('YawBenchDebug','第6批 Yaw线束中位与软限位｜先关控','只认清Yaw反馈和人工零位，先不转动测试。')+'''
<h2>先换固件，不是只点Group</h2>
<ol><li>停轮、承托机械臂，隔离动力后烧录YawBenchDebug。</li><li>切换分组工具选2，重新打开底盘调参。其余四轴保持失能，底盘不启动。</li><li>确认CAN1 GM6020 ID1；反馈0x205、电流命令0x1FE第一槽。不能套用旧电压模式驱动。</li></ol>
<h2>本批源码确认入口</h2>
<p>Arm/yaw_config.h：方向、1:1传动、线束允许区间和安全参数；kYawSafetyConfirmed。Arm/arm_config.cpp：hardware_confirmed。Remote/operator_config.cpp：mapping_confirmed。实物核对后分别填写，不要求四轴pitch映射或IMU/功率放行。</p>
<h2>找中位和校零</h2>
<ol><li>写g_robot_debug_command=1；确认joint[0].enabled=0，整臂有支撑。</li><li>看joint[0].online=1，反馈连续。慢慢手转小角度，看角度/速度变化方向；不要多圈转。</li><li>整理线束，让它处于自然、不预拧的位置；在底盘与Yaw外圈各做中位标记。</li><li>对准标记，等实物静止，再写g_robot_debug_command=2。</li><li>检查g_arm_status.last_result=0、joint[0].calibrated=1；此时position_deg应在零附近。</li></ol>
<p>YawBench的命令2只校Yaw。校零前的角度不能当作线束绝对位置；开机位置也不会自动成为安全中位。</p>
<p class="stop">无滑环，不能靠“转一圈找零”。断电或丢失多圈位置后，要重新整理线束、对中位标记，再校零。</p>'''),
page('第6批 B：申请一个小的左右范围', setting('YawBenchDebug','仍在第6批','确认半幅真实生效，才允许下一批Yaw测试。')+'''
<h2>半幅是什么意思</h2>
<p>以人工线束中位为0。如果实物已经确认左右各10°安全，half_range_deg填10，生效区间就是−10°到+10°。<b>这里的10是演示，不是让你没看线束就照填。</b></p>
<h2>按顺序填写</h2>
'''+table(['要写的变量','填法'],[
(v('g_yaw_limit_request.half_range_deg'),'填写刚刚实物确认安全的半幅，例如10。'),
(v('g_yaw_limit_request.request_seq'),'最后读当前值并加1。')])+'''
<p>申请时所有关节失能、Yaw在线校零且静止、无故障。实际角、目标和参考都要处于新范围内，并留出2°制动余量。</p>
<h2>填写后必须看这四项</h2>
'''+table(['观察变量','若申请10°，应看到'],[
('g_yaw_limit_status.result','0，限位成功。'),('g_yaw_limit_status.ready','1。'),
('g_yaw_limit_status.active_seq','等于本次限位申请序号。'),
('active_half_range_deg / min_deg / max_deg','10 / −10 / +10。')])+'''
<h2>没成功怎么处理</h2>
<p>2=源码配置未确认；4=不在线；5=没校零；7=还在动；8=新范围排除了当前角/目标/参考。先停机排除对应原因，再提交新序号。</p>
<p>当前软件接受半幅&gt;2°且≤90°，这是防错边界，不是推荐把线束放到90°。先只测中间的小范围；暂时不要故意撞边界。</p>
<p><b>通过：</b>中位标记明确，零位有效，小半幅已生效。换固件或重启后，这些RAM状态需要按流程重新建立。</p>'''),
page('第7批 A：Yaw先只调速度P', setting('YawBenchDebug','第7批 Yaw速度PI｜mode=1小幅短脉冲','不让位置外环参与，只检查速度内环。')+'''
<h2>先停Yaw，完整申请参数</h2>
<p>写全局命令1，等Yaw实际静止；第6批的校零与限位仍有效。以下表格是起步示例：</p>
'''+table(['要写的变量','填写'],[
(v('g_arm_tune_request.joint'),'0，Yaw。'),(v('g_arm_tune_request.position_kp'),'2，先保留；本固件不验收位置P。'),
(v('g_arm_tune_request.velocity_kp'),'20，源码占位P起步。'),(v('g_arm_tune_request.velocity_ki'),'0，先关积分。'),
(v('g_arm_tune_request.effort_limit'),'例如100mA，须是本次确认可用的保守上限；不能超过150mA测试硬限。'),
(v('g_arm_tune_request.request_seq'),'最后加1。')])+'''
<h2>这次成功是 result=0</h2>
<p>看g_arm_tune_status.result=0、active_joint=0、active_seq等于本次序号，并核对四个生效参数。<b>不要套用底盘参数“1成功”的规则。</b>虽然本固件跳过位置外环，参数表仍要求position_kp&gt;0，保留2，不填0。</p>
<p>如果result=3，先停所有关节；4查在线；5重做安全校零；6排除故障后命令3清Arm故障；7等静止。条件恢复后重新提交，不自动补执行。</p>
<h2>本固件运行时应该看哪个使能</h2>
<p>YawBench 的 g_robot_status.active 固定为0、mode固定为安全态，<b>它们不能证明Yaw没出力</b>。下一页看g_yaw_debug_status.active、joint[0].enabled、速度与effort。</p>
<p>测试最终电流取源码上限、150mA测试上限、在线effort_limit三者中的最小值。把源码300mA当作这里实际测试上限是错误的。</p>'''),
page('第7批 B：Yaw正负各测一小下', setting('YawBenchDebug','第7批的“Yaw目标速度与反馈及电流”示波器','先正1°/s，再负1°/s；不连续转，不靠近线束边界。')+'''
<h2>准备后按表提交</h2>
<p>整臂可靠承托，左上、右中，5通道回中；Yaw位于小范围中部，在线、校零、无故障并已静止。先开示波器。</p>
'''+table(['要写的变量','第一次填写'],[
(v('g_yaw_debug_request.mode'),'1，短时间速度测试。'),
(v('g_yaw_debug_request.speed_deg_s'),'+1，单位度/秒。'),
(v('g_yaw_debug_request.duration_ms'),'200，即0.2秒。'),
(v('g_yaw_debug_request.suspended_confirm'),'1497454419，即0x59415753。'),
(v('g_yaw_debug_request.request_seq'),'最后加1。')])+'''
<h2>提交后按这个顺序看</h2>
<ol><li>consumed_seq跟上；测试时active=1、joint[0].enabled=1，结束后撤力。</li><li>目标速度与实际速度同号，实际向目标靠近；joint[0].effort是提交电流mA。</li><li>等实际静止后，speed_deg_s换−1，其余保持，最后再加新序号。</li></ol>
<p>只走0.2秒，理论运动约0.2°，实物可能不明显。先看曲线和反馈，不直接把速度拉满。硬限|速度|≤5°/s、时间≤500ms。</p>
<h2>停止与结果</h2>
<p>g_yaw_debug_request.mode=0后，再将它自己的request_seq加1；全局命令1也可停。result：0停、1接受、2非法、3未就绪/忙、4取消或到期。正常到期看到4不等于电机故障。</p>
<p><b>不对就停：</b>实物与反馈反向、突然扫转、线束受拉、啸叫。先断开出力，检查方向/线束/反馈，不靠增大P压过去。</p>'''),
page('第7批 C：把速度内环调稳', setting('YawBenchDebug','仍在第7批','每次只改一个参数，先稳定P，再考虑少量I。')+'''
'''+table(['曲线或实物','本次处理'],[
('正负跟踪都平稳、没有抖动','先保存当前P，保持小目标重复验证。'),
('跟得慢但电流未顶限，方向已正确','停机后小幅增加velocity_kp，例如20→24；完整提交参数表，再测同样目标。'),
('电流正负快速跳、机械啸叫','先停，减P；同时查反馈连续性和机构松动。'),
('电流顶100/150mA，实际跟不上','先查摩擦、负载和测试限流。不是无限加P的依据。'),
('P稳定，仍有可重复静差','停机后把velocity_ki从0小幅加一点，例如0.2，再比较。')])+'''
<h2>每次改参数的完整循环</h2>
<p>停脉冲 → 等静止、全部关节失能 → 填joint与四个正确参数 → 参数request_seq加1 → 看result=0和active_seq → 再提交速度短测试。</p>
<p>不要只换joint或只改一个字段就提交一张含旧值的表。你可以一次只调整一个参数，但其他参数也必须保持本次正确值。</p>
<h2>接近边界变慢是保护</h2>
<p>程序按<b>实际角度</b>提前限制向外速度；目标也有限制。接近边界时目标速度可能被压低，这时不要加P强行突破。速度PI在中间小范围调；整臂撤力后的行为还须双方实测。</p>
<h2>本批达标与记录</h2>
<p>正负方向一致，短测试可取消，没有持续抖动；记录速度P、I、电流上限、半幅和曲线。不要在此固件验收位置P。</p>
<p class="note">下一步切Vehicle，在线参数、校零和半幅都会重置。先把本批结果记下来。第8批的整车保护没通过，不做第9批位置测试，也不落地。</p>'''),
page('第8批 A：整车准备清单，先不开控', setting('VehicleDebug','第8批 IMU标定与真实整车许可','补齐真实整车条件，不把许可变量硬改成1。')+'''
<h2>换固件后先做什么</h2>
<ol><li>停机、承托、断开动力后烧录VehicleDebug；切换分组工具选3，重新打开底盘调参。</li><li>两轮仍悬空，整臂可靠承托。写命令1，确认所有输出失能。</li><li>重新核对五轴反馈。命令2现在会校<b>全部五轴</b>：必须与队友把两pitch、roll、夹爪和Yaw一起摆好规定零位，不能只顾Yaw随手按。</li><li>安全联合零位、反馈静止健康时校零；重新申请已测Yaw半幅，重填已记录的轮和Yaw参数。</li></ol>
<h2>源码确认表：测一项，确认一项</h2>
'''+table(['源码文件（均在2User_Driver）','必须实测的内容'],[
('Chassis/chassis_config.h','左右ID、方向、几何、速度PI/限流；hardware_confirmed。'),
('Remote/operator_config.cpp','遥控通道、左右拨杆和正向；mapping_confirmed。'),
('Arm/arm_config.cpp','队友四轴方向、量程、联合零位和范围；hardware_confirmed及pitch映射。'),
('Arm/yaw_config.h','线束中位、范围、制动余量和Yaw方向；kYawSafetyConfirmed。'),
('Imu/attitude_estimator.h','安装坐标和翻车进入/退出阈值，两个confirmed。'),
('Power/chassis_power.h','真实功率、超时、缓冲阈值与制动约束；confirmed。'),
('Vehicle/vehicle_state.h','机械臂哪些收拢角度允许行驶；posture_confirmed。')])+'''
<p><b>不要一次全部改true。</b>没测过的保留未确认，本阶段不开控、用反馈完成核对。它们不是FreeMASTER里的“强行解锁开关”。</p>
<p>本页只做准备，IMU和功率具体看后两页。</p>'''),
page('第8批 B：IMU静态标定与翻车识别', setting('VehicleDebug','第8批 IMU标定与真实整车许可','确认底盘姿态可信；Yaw角度仍来自电机编码器。')+'''
<h2>静置标定</h2>
<ol><li>机械臂、底盘、Yaw全部失能且有支撑，车放平稳不晃动。</li><li>frames、received_ms、updated_ms持续更新；驱动有反馈后等待静态标定完成。</li><li>需要重做时，将g_ins_debug_command写1。保持静止几秒，看calibrating、calibrated。</li><li>calibrated=1后，平放时tilt_deg应接近0且稳定。安装轴映射由实际板到车体方向核对，不能只凭“看起来接近0”确认。</li><li>安装与阈值确认后重编译，再核对confirmed=1、valid=1、faults=0。</li></ol>
<h2>不出力测试倾斜判据</h2>
<p>保持所有电机失能、机构承托。用稳固支架缓慢改变车体姿态，观察tilt_deg变化；按经过确认的阈值验证tipped进入/退出。不要把整车真的摔翻来试。</p>
'''+table(['当前源码占位值','含义'],[
('trip_tilt_deg=55，trip_ms=150','超过进入角度持续一段时间后判倾覆。'),
('release_tilt_deg=40，release_ms=500','恢复到退出角度内持续一段时间后，倾覆状态可解除。'),
('静态标定500样本、至少1500ms','不是写1立刻成功；标定时晃动可能重来或失败。')])+'''
<p>这些数值需结合实车正常坡度与机构姿态确认，不是比赛推荐标准。姿态恢复也不能自动恢复电机使能。</p>
<h2>没通过怎么办</h2>
<p>frames不增：查BMI088驱动/接线；calibrated不成：先查仍有出力、晃动与快照失效；角度方向不符：核对安装轴映射。不要用调Yaw PID修IMU问题。</p>
<p><b>通过：</b>新数据持续到来、平放稳定、倾斜识别合理；确认后valid=1且没有IMU故障。</p>'''),
page('第8批 C：功率反馈与整车放行', setting('VehicleDebug','第8批；异常时“底盘排障”排查5/8','确认真实功率数据和机械臂姿态许可，不运动。')+'''
<h2>功率计先只读</h2>
<ol><li>CAN2功率帧0x31、DLC4。看power_frames持续增长、power_received_ms刷新。</li><li>核对power_w和energy_j与功率计实际显示/工况一致。协议是有符号小端int16×0.1W/J。</li><li>实测帧周期，确认timeout_ms留有合理余量；检查低缓冲停车阈值和制动电流。</li><li>确认这些约束后才将功率配置confirmed设true，编译、烧录、刷新地址并重新核对。</li></ol>
'''+table(['当前源码占位','用途'],[
('budget_w=40W','首版保守预算，不代表严格瞬态功率保证。'),
('warning_energy_j=200J','开始降速预留。'),('stop_energy_j=100J','请求停车。'),
('restart_energy_j=250J','清功率禁行所需的恢复能量。'),
('timeout_ms=200；stopping_timeout_ms=2000','功率断链判定与受控停车时限。')])+'''
<h2>机械臂姿态许可由实物产生</h2>
<p>与队友确定允许行驶的肩/肘/roll收拢角度及Yaw区间，再填写vehicle_state.h。当前15°/15°/20°只是占位。未收拢时应拒绝行驶；不要在FreeMASTER写假许可。</p>
<h2>最后一次烧录后，再做最终检查</h2>
<p>本批若改确认项后重新编译烧录，RAM中的零位、限位和在线参数会重置。最后一次烧录完成后，回第8批A第3～4步，重做五轴联合校零、Yaw半幅和实测参数申请，并重新完成IMU静态标定，再检查下列状态。</p>
<p>五轴在线校零、Yaw限位ready=1；轮反馈与配置健康；IMU valid=1、tipped=0；功率确认且数据新鲜、能量足够；g_vehicle_health.healthy=1、reasons=0，当前保护原因已排除。</p>
<p class="note">若尚未满足，不跳到位置P或落地。查“01_排障与变量说明.pdf”。历史锁存原因不一定立即清零；是否仍有问题先看current_reasons，清错和开控分开做。</p>'''),
page('第9批：Yaw位置P，小角度慢跟踪', setting('VehicleDebug','第9批 Yaw位置P与小角度跟踪','固定使用夹取模式，避免把转向杆与Yaw杆混淆。')+'''
<h2>前提</h2>
<p>第8批通过；两轮悬空，机械臂可靠承托，已验证Yaw速度PI、线束中位和小半幅。确认范围内没有机构干涉。</p>
<h2>按顺序做</h2>
<ol><li>写命令1，所有关节失能，等静止。</li><li>g_arm_tune_request填joint=0；position_kp先2；velocity_kp/ki填第7批实测值；effort_limit填已验证上限。最后该表request_seq加1，确认result=0、active_seq正确。</li>
<li><b>左拨杆中（夹取）、右拨杆中</b>，所有杆回中等待约300ms，观察wait_neutral=0后，再写g_robot_debug_command=4。</li><li>成功时active=1、enabled_mask=31（0x1f）。没成功先查原因，不能一直刷命令4。</li>
<li>只轻点<b>右横</b>，使Yaw目标改变约1～2°就回中；其余杆保持不动。反向重复小角度。</li></ol>
<h2>曲线与判断</h2>
'''+table(['变量/现象','怎样判断'],[
('joint[0].target_deg / reference_deg / position_deg','指令、限速后的参考、实际角；看实际是否平稳追上。'),
('跟得慢，速度内环和输出都正常','关控后小幅增加position_kp，例如2→2.4，再完整提交参数。'),
('冲过头、回弹、来回摇','先停；减位置P，也检查速度环是否足够稳定。'),
('目标被限幅或速度被压低','先查软限位/目标领先，不用增大P对抗。')])+'''
<p><b>通过：</b>小角度正负跟踪稳定，回中后目标不再增长，实际停稳；逐步靠近已确认的小边界时能限制向外运动。</p>
<p class="stop">Vehicle左上是行驶，不是关控。使用命令1或已验证的遥控失联撤力；承托整臂后再操作。</p>'''),
page('第10批：第一次慢速落地', setting('VehicleDebug','第10批 慢速落地与差速｜快速档未接通','只验收前后、转向和停车，先不追速度。')+'''
<h2>落地前</h2>
<p>前面各批通过；带动力关控已在悬空条件下验证；场地清空且能立即切断动力。机械臂处于已确认可行驶的收拢姿态、Yaw靠中位。最初可在源码进一步降低速度上限，再编译验证。</p>
<p>当前源码慢速上限0.25m/s、转向0.6rad/s，加速度0.3m/s²、减速度0.5m/s²，均需实测。没有接通快速档，不通过拨快档开始试车。</p>
<h2>固定顺序，每次只做一个动作</h2>
<ol><li>停稳、回中，左上选择行驶，右中；满足回中和真实许可后显式请求开控。</li><li>左竖轻推很小一段，短距离前进，回中，观察停车。正前进时两侧measured_rpm为正。</li><li>完全停稳后，左竖小幅反向，短距离后退，再回中。</li><li>左竖回中，只轻推右横，小幅原地转向；右推应使车顺时针。</li><li>都正常后才测试缓慢边走边转；倒车时转向符号不翻转。</li></ol>
<h2>记录与处理</h2>
'''+table(['现象','处理'],[
('直行明显跑偏','先查两轮方向、轮径/摩擦与速度跟踪，不先靠改转向输入补偿。'),
('松杆滑很远','停，检查速度目标回0、减速斜坡和输出能力；降低速度，再调停车。'),
('轮速跟不上且电流一直限幅','按负载检查输出能力/机构，不能简单加I。'),
('切夹取后机械臂不立刻响应','先等停车、回中与模式切换条件，不强行绕过。')])+'''
<p><b>通过：</b>正反、转向、松杆停车、模式转换都可重复。两轮低速确认只是“驱动轮静止”，不能保证抱死滑移或坡道车身不动。</p>'''),
page('第11批 A：故障撤力与重新来', setting('VehicleDebug','第11批 功率停车与故障恢复','低速、可立即切断动力的条件下，逐项验证故障响应。')+'''
<h2>先在悬空与可靠承托下测</h2>
<ol><li>用已经验证的最小运动，提交全局命令1。观察当前输出撤销、stop_epoch增加，机构由支撑接住。</li><li>重新恢复健康并明确开控后，测关闭遥控器。应离线、输出撤销；重开遥控不自动续跑。</li><li>反馈超时、发送异常等测试先停机准备接线或由有经验成员做可控故障注入；不带电随手拔电源/CAN制造异常。</li></ol>
<p>故障发生时，检查提交电流与使能，同时看实物。CAN断链后不能承诺闭环刹停，电机侧超时与实际撤力行为必须验证。</p>
<h2>恢复固定按这六步</h2>
<ol><li>先停机、承托机构，命令1；排除遥控、接线、反馈、姿态等实际原因。</li><li>若Arm/Yaw有故障，写命令3。若校零丢失，重新摆好联合零位再命令2，不能靠清错恢复零位。</li><li>若底盘或功率锁存：g_chassis_debug_request.mode=1，最后该表request_seq加1。</li><li>确认底盘faults=0、power_inhibited解除、当前保护原因排除。功率清错还要求新鲜数据且energy_j达到重启阈值。</li><li>重新申请需要的Yaw半幅，确认所有实际参数和许可。</li><li>选择夹取/行驶模式，回中满足条件后，显式命令4；清错本身不会开控。</li></ol>
<p class="stop">Vehicle中底盘mode=0或1会同时请求全局关控，机械臂保持也撤掉。不要把它当“只停轮但保持举臂”的按钮。</p>
<p><b>通过：</b>故障恢复后不会自动执行旧运动；每次重新运动都来自新的明确请求。</p>'''),
page('第11批 B：低缓冲和断功率反馈时停车', setting('VehicleDebug','第11批及功率/缓冲示波器','验证功率限制时仍能在实际可接受距离内停下。')+'''
<h2>按由轻到重的顺序测</h2>
<ol><li>正常能量、小速度直行，松杆停车。量停车距离，记起始速度和最低缓冲。</li><li>在可控条件下进入预警区，观察速度/推进输出变小，再松杆，重新测距离。</li><li>靠近已验证低能量阈值时，应进入请求停车阶段，拒绝新的推进。先小速度验证，不能故意把缓冲耗到断电。</li><li>功率反馈失联测试先停机设置，由电控成员可控屏蔽功率帧；别把整条CAN2断开当作“仅功率失联”，那会同时丢轮反馈，策略不同。</li></ol>
'''+table(['观察变量','应该怎样变化'],[
('power_phase','3低能量停车；4功率反馈失联停车；之后通常5锁存禁行。'),
('power_stop_requested','停车阶段为1，目标应减为0。'),
('power_session_allowed','受控停车过程中可仍为1，这是保留闭环制动所需，不等于继续推进。'),
('power_inhibited','停车后锁存为1；数据或缓冲恢复不会自行续跑。')])+'''
<h2>区分两类故障</h2>
<p><b>只有功率计失联、轮反馈健康：</b>有时限受控停车，保留经过限制的制动电流。<b>轮反馈/CAN本身故障：</b>撤除输出，不再承诺闭环制动。</p>
<p>源码制动上限当前500mA，停车时限2000ms，都是待测值。停车距离过长，先降低速度、增加预留；不能因为“限功率”就接受失去制动后滑很远。</p>
<p><b>通过：</b>正常、预警/低缓冲和仅功率断链下都记录停车距离、最低能量与电源电压，结果可接受。满载、坡道及严格瞬态功率留待后续独立验收。</p>'''),
page('第12批：保存参数，重启重验', setting('按要验收的阶段选择对应固件','第12批 固化参数与重启复验','让在线调出的结果真正进入源码，不依赖这次RAM状态。')+'''
'''+table(['要保存的结果','写回源码位置'],[
('轮方向、轮径轮距/总比、轮PI/限流','Chassis/chassis_config.h。'),
('慢速、转向速度、加减速','同上；快速档另做，不在这里解锁。'),
('Yaw位置P、速度PI、电流上限','Arm/yaw_config.h。'),
('线束范围、制动余量/减速度、目标领先','Arm/yaw_config.h；在线半幅仍需要启动后明确申请。'),
('机械臂肩肘参数','Arm/arm_config.cpp，与机械臂队友共同维护。'),
('功率/停车阈值、IMU、收拢姿态','Power、Imu、Vehicle对应配置文件。')])+'''
<h2>重启后按顺序复验</h2>
<ol><li>停机、承托、编译、烧录，刷新对应FreeMASTER地址。</li><li>核对在线生效参数已经等于这次写回源码的数值。</li><li>重新整理线束、对齐联合零位，按当前固件范围校零；重新申请Yaw半幅。</li><li>完成IMU标定、确认遥控/反馈/功率新鲜，先悬空小动作，再落地慢速。</li><li>重新验收关控、失联、正负方向、保持与停车；不是只看“能动”就结束。</li></ol>
<h2>每批只留这张小记录</h2>
'''+table(['记录项','填写'],[
('日期 / 固件 / 本地Git提交','_______________________________'),
('本次参数 / 半幅 / 方向','_______________________________'),
('曲线文件 / 实际现象','_______________________________'),
('通过还是停在哪一步','_______________________________')])+'''
<p class="note">本说明依据源码和逻辑核对编写，没有代替你的实物验收。在线数值掉电会丢；零位不自动保存；输出为零也不是机械抱闸。</p>''')]


FAULTS = [
page('先按现象找位置', '''
<p>配合“底盘排障.pmpx”。先看数据有没有更新，再看请求有没有被看见，再看拒绝原因。<b>没有收到/接受指令时，不先调PID。</b></p>
'''+table(['现象','先打开哪组','先查什么'],[
('所有数值冻结/明显错位','排查0','uwTick、updated_ms、固件与ELF是否一致。'),
('遥控不在线/回中失败','排查1','good_frames、remote_age_ms、5通道、拨杆索引。'),
('轮子写了不动','排查3','debug consumed_seq/result、底盘faults、确认/反馈/静止/拨杆。'),
('某轮反转、啸叫或不跟踪','排查4','轮归属与direction，再看目标/实际rpm与最终电流。'),
('抬臂保持后仍不能测轮','排查3','mode=3、active=1、mask=30；四轴都在线校零静止。'),
('Yaw半幅/参数申请失败','Yaw下排查6/7','校零、全部失能、result、active_seq、范围是否排除了当前位置。'),
('Vehicle不能开控/突然停','Vehicle下排查5/8','功率状态、IMU、整车reasons与当前保护位图。')])+'''
<h2>先停止，再排查</h2>
<p>全局关控：g_robot_debug_command=1。承托机械臂，因为它也撤保持。ChassisBench只停轮：mode=0加新的轮测试序号；YawBench只停Yaw：Yaw mode=0加新序号。</p>
<p class="stop">Vehicle中的底盘mode=0或1会全局关控。左上在Vehicle是行驶。关闭FreeMASTER通信不是急停。</p>
<h2>不要混淆这些量</h2>
<p>current_ma和joint.effort是程序提交/计算的控制量，不是电机已经执行的证明。转子rpm不是车轮rpm。时间戳需要持续变化；旧快照上的online=1不算新鲜反馈。</p>'''),
page('轮测试结果：查这一张', '''
<h2>g_chassis_debug_status.result</h2>
'''+table(['值','说明','处理'],[
('0','初始','尚未处理新请求。'),('1','已接受','观察实物和曲线，有限时间后结束。'),
('2','已停止','mode=0处理完成，不自动续跑。'),('3','已请求清错','再查g_chassis_status.faults是否为0。'),
('4','参数非法','检查mode、wheel、value、duration_ms和悬空确认码。'),
('5','未就绪','依次查下面的门槛。'),('6','忙','先停止，等实际静止，再用新序号请求。'),
('7','条件丢失而取消','查偏杆/拨杆、显式关控、反馈、总线和任务周期。'),('8','正常到期','有限脉冲结束，不必仅因8清故障。')])+'''
<h2>result=5时，按这个顺序查</h2>
<ol><li>固件是否ChassisBenchDebug；底盘和遥控实测确认是否已按实物填写。</li><li>左上、右中且Arm失能，或左下真实四轴保持；5通道全部在±30内。</li><li>两轮都有新反馈，温度&lt;70℃；实际静止，建议等wheels_stationary=1。</li><li>底盘faults是否为0，CAN2错误计数是否仍在增长。</li></ol>
<p>纯轮悬空不要求机械臂在线校零，也不依赖功率计放行。不能因为Arm保护原因未清就把所有确认强行打开。</p>
<h2>没看到result=1，不一定没测到</h2>
<p>Watch约100ms刷新，可能错过100/200ms测试。看consumed_seq、结束结果和事先打开的示波器。只改value、不更新request_seq，不会重新提交。</p>'''),
page('参数、Yaw限位：成功值不一样', '''
<h2>轮参数 g_chassis_tune_status.result</h2>
'''+table(['值','解释'],[
('0 / 1','初始 / 参数已生效。成功还看active_seq及实际P/I/限流。'),
('2','数值非法或电流超过源码启动上限。'),('3','底盘仍使能/测试未停，先停止。'),
('4','反馈不健康、不静止、源数据超时或通信异常。')])+'''
<h2>关节参数与Yaw半幅：result=0才成功</h2>
<p>适用于g_arm_tune_status.result、g_yaw_limit_status.result；不同接口只会返回表中部分结果码，必须带着变量名看：</p>
'''+table(['值','解释与下一步'],[
('0','成功，再看active_seq和实际生效值。'),('1','参数非法；完整填写表格。'),
('2','配置未确认；核对实物与源码确认入口。'),('3','忙；停止全部关节出力。'),
('4','反馈不在线；先恢复连续反馈。'),('5','没校零；整理线束/联合零位后重做。'),
('6','故障锁存；排除原因后命令3清Arm故障。'),('7','仍在运动；等实际静止。'),
('8','越范围；新限位排除了实际角、目标或参考，先安全回中。')])+'''
<h2>Yaw短测试 g_yaw_debug_status.result</h2>
<p>0停止；1接受；2非法；3未就绪或忙；4取消或到期。到期的4不是上面参数接口的“不在线”。</p>
<p><b>牢记：</b>每个结果码都要带着变量名看。失败申请也消费序号，修好条件后用新的序号再提交。</p>'''),
page('保护与故障：当前和历史分开看', '''
<p>g_robot_status.protection_current_reasons 是当前仍存在的原因；protection_latched_reasons 是历史锁存。位图可以叠加，例如20=16+4，表示关控请求与遥控失联。</p>
'''+table(['位值','含义','优先检查'],[
('1','启动保护','正常初始锁存，不能据此判断一直有故障。'),('2','配置未确认/不合法','各固件对应的实测确认入口。'),
('4','遥控失联','good_frames、源时间、接收机。'),('8','互锁超时/无效','Vehicle真实状态更新，不写假许可。'),
('16','显式关控','命令1等；单周期可能看不到当前值，查stop_epoch。'),('32','任务周期异常','任务是否阻塞/饿死，不能只提高优先级。'),
('64 / 128','Arm故障 / 开控被拒','关节faults、在线校零、回中、配置。'),
('256 / 512','底盘故障 / IMU故障','本地faults / IMU数据与标定。'),
('1024 / 2048 / 4096','倾覆 / 整车状态 / 功率','姿态、真实互锁、功率阶段。')])+'''
<h2>底盘本地 g_chassis_status.faults</h2>
<p>1配置、2反馈超时、4发送/接收传输、8周期、16命令超时、32温度、64能量。若为6，就是2+4；先恢复反馈与总线。</p>
<h2>清错分两条</h2>
<p><b>机械臂/Yaw：</b>命令3。<b>底盘/功率：</b>底盘debug mode=1加新序号。两者都不自动开控，也不会自动恢复丢失的零位。</p>
<p>ChassisBench全局STOP会取消当前脉冲，但不是永久锁死独立测试入口；随后健康的新请求仍可测试。要只测关控，就保持动力隔离并不提交运动请求。</p>'''),
page('功率状态与常见实物问题', '''
<h2>g_chassis_status.power_phase</h2>
'''+table(['值','阶段'],[
('0','未放行/失能。'),('1','就绪。'),('2','行驶。'),('3','低能量受控停车。'),('4','功率反馈失联受控停车。'),('5','锁存禁行。')])+'''
<p>power_reason：0无原因、1未确认、2配置非法、3无功率计、4低能量、5功率反馈丢失、6停车超时、7已停车、8被外部关控中断。</p>
'''+table(['现象','处理'],[
('反馈帧数不增长','查供电、ID、接线和波特率；不能先加PID。'),
('50mA不动但请求接受','可能摩擦未克服；保留短时间、小步验证，不直接拉满。'),
('电流一直顶限仍下垂/跟不上','查负载、机构和力矩上限；不猛加I。'),
('Yaw向边界走时变慢','保护可能在正常生效；查实际角、半幅和制动余量。'),
('Yaw/机械臂calibrated掉成0','反馈连续性丢失；先停、支撑并安全重校，不仅清错。'),
('换固件后参数变回去','在线值在RAM中；重填实测值，最后按第12批写回源码。'),
('乱码或保存后分组异常','先备份；_工具/修复中文乱码.py仅用于修编码。变量名仍应为源码名。')])+'''
<h2>给别人排查时，带上这五项</h2>
<p>固件配置名；在哪一批；完整写入变量和值；consumed_seq/result/active_seq；实物现象或曲线。不要只说“没反应”。</p>
<p class="small">软件核对不代替方向、保持、撤力和停车距离实测。重新编译后刷新ELF绑定，变量显示名保持源码名。</p>''')]


ARM = [
page('机械臂只做：慢抬，再保持', '''
<p class="note">你的目的只是让机械臂举起来，给底盘调轮留空间。这里只走一条简单路线：<b>停轮 → 支撑 → 校零 → 慢抬 → 左下保持 → 测轮 → 收工。</b></p>
<p><b>固件固定：ChassisBenchDebug。</b>打开“机械臂简易调参与排障.pmpx”。不要拿它连接YawBench或Vehicle固件。</p>
<h2>第0步：准备</h2>
<ol><li>先在底盘工程mode=0加新序号，确认两轮停止；随后停止上一份FreeMASTER通信。</li><li>打开机械臂简易工程，先看uwTick和g_arm_status.updated_ms持续更新。</li><li>机械臂用支架承托，左上、右中，写g_robot_debug_command=1，确认enabled_mask=0。</li><li>让队友确认DM量程、两个pitch对应关系、方向、联合零位和安全范围。Arm hardware_confirmed及pitch.mapping_confirmed只能实测后填写。</li></ol>
<h2>四轴编号，先认清</h2>
'''+table(['joint下标','轴','注意'],[
('1','肩，大pitch DM','左竖输入。'),('2','肘，小pitch DM','右竖输入；驱动角与肩有关联。'),
('3','roll M2006','也必须在线校零，暂不主动操作。'),('4','夹爪M2006','也必须在线校零，暂不主动操作。')])+'''
<p>此固件不发送Yaw，关节0不参与。四轴使能掩码30就是0x1e；不要把它当作某个角度。</p>
<p class="stop">两台DM当前启动上限各0.3Nm，只是待测起步值，不保证托得住负载。支撑必须能接住，不能靠你先把PID加大来保证不掉。</p>'''),
page('第1步：摆零位，再校零', setting('ChassisBenchDebug','第1步 双Pitch人工联合零位｜命令2','把机械臂四轴校零，不开控。')+'''
<h2>先确认在线</h2>
<p>四轴joint[1]、[2]、[3]、[4]都需要online=1，反馈新鲜。不是只有两台DM在线就够。</p>
<h2>按顺序做</h2>
<ol><li>保持左上失能并承托机构。若有Arm故障，排除原因后写命令3，再检查faults。</li><li>请队友按约定摆好两个pitch的<b>联合参考姿态</b>，同时对齐roll参考和夹爪安全张开参考。</li><li>等所有轴实际静止，再写g_robot_debug_command=2。</li><li>看g_arm_status.last_result=0；四轴calibrated都为1。</li><li>检查肩/肘角与实物约定一致；不要在任意姿态重置零位来消掉报错。</li></ol>
<h2>第二台DM为什么不能独立乱定零</h2>
<p>本车第一驱动角a1=肩角；第二驱动角a2=肩角+肘相对角。它们成对建立零位。你只要按队友规定的联合姿态摆好，不需要自己计算逆解。</p>
<h2>没通过就停在这里</h2>
'''+table(['现象','先查'],[
('last_result=4','有轴不在线，查四轴全部反馈。'),('last_result=7','还在动，承托并等静止。'),
('last_result=3','仍有轴使能；先确认全部失能，再重新校零。'),('校零后又calibrated=0','反馈间断使位置不可信，先修反馈，再重做安全校零。')])+'''
<p><b>通过：</b>四轴在线、静止、无故障，校零结果成功。下一步才开控慢抬。</p>'''),
page('第2步：慢抬；第4步：左下保持', setting('ChassisBenchDebug','先“第2步 简易抬肩抬肘”，再“第4步 左下待机保持”','只用两个pitch把机械臂移到安全姿态。')+'''
<h2>慢抬</h2>
<ol><li>左上、右中，所有摇杆回中。健康连接下从左上切左中，保持回中约300ms。</li><li>确认g_robot_status.active=1、enabled_mask=30。没开控就查原因，别推杆等它突然动。</li><li>只轻推左竖，肩小幅移动后马上回中；方向或动作不符就承托关控。</li><li>肩正常后，轻推右竖，调整肘。第一次每次只动一个输入，不碰roll/夹爪输入。</li><li>慢慢到达已经确认不干涉、不会碰地的姿态，所有输入回中。</li></ol>
<h2>保持</h2>
<ol><li>从左中切左下。此时是保持，<b>不是关控</b>。</li><li>看mode=3、active=1、enabled_mask=30，实际关节速度接近0，角度保持几秒不持续下垂或抖动。</li><li>若明显下垂或抖动，立刻承托，回左上撤力，先解决机械臂问题，不测轮。</li></ol>
<h2>保持稳了，才回底盘工程</h2>
<p>停止这份通信，打开底盘调参第5批。左下、右中、5通道回中，四轴均在线校零静止且无故障，才可沿用已验证的单轮短脉冲。两轮仍须悬空。</p>
<p>左中正在抬臂时不能同时调轮。切回左中、偏杆或保持丢失，轮测试应取消。</p>
<h2>收工</h2>
<p><b>先停轮并等静止 → 用支撑接住机械臂 → 左上/命令1关控 → 确认失能 → 再断动力。</b></p>'''),
page('第3步（可选）：只做少量参数调整', setting('ChassisBenchDebug','第3步 少量改PI与限流｜先左上关控','只有需要时再调；能稳定抬起保持，就先不改。')+'''
<p>先停轮、承托、左上关控，确认所有轴失能。下面以肩为例，肘须单独选joint=2并填它自己的正确参数。</p>
'''+table(['要写的变量','肩的源码起步示例'],[
(v('g_arm_tune_request.joint'),'1；肘为2。'),(v('g_arm_tune_request.position_kp'),'2。'),
(v('g_arm_tune_request.velocity_kp'),'0.03。'),(v('g_arm_tune_request.velocity_ki'),'首次可先0；源码占位0.005不是实测答案。'),
(v('g_arm_tune_request.effort_limit'),'不超过已确认的保守值；当前启动上限0.3Nm。'),
(v('g_arm_tune_request.request_seq'),'最后加1。')])+'''
<p>结果必须是g_arm_tune_status.result=0、active_joint为这次选轴、active_seq对应本次序号；再核对生效参数。成功后不会自动开控，重新左上→左中、回中后小动作验证。</p>
<h2>简单排障</h2>
'''+table(['现象','先怎么做'],[
('active=0或mask=0','查四轴在线/校零、确认配置、当前保护与回中。'),
('effort顶0.3Nm仍下垂','承托，和队友核对负载/机构/力矩需求；不猛加I。'),
('目标在变而实物不动','查使能、电机状态、反馈和限流，不只改位置P。'),
('方向不对/明显抖动','立即承托关控，检查方向和映射，退回保守参数。'),
('左下保持但轮请求拒绝','四轴都要检查，尤其roll/夹爪在线校零和速度；不是只看两个pitch。')])+'''
<p class="note">先满足“能安全抬起并保持”。更大负载、更大力矩、完整机械臂性能由双方另行调试，不在这张简易操作卡里扩展。</p>''')]


if __name__ == '__main__':
    import sys
    sys.stdout.reconfigure(encoding='utf-8')
    pdf('00_分批调参步骤.pdf', STEPS)
    pdf('01_排障与变量说明.pdf', FAULTS)
    pdf('02_机械臂抬起与保持.pdf', ARM)
