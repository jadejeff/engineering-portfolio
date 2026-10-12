"""Requirement 1 successful vision algorithm, wrapped for multimode dispatch.
The recognition/matching algorithm is preserved from 1success(3).py.
"""
import sensor, image, time, math

# ---------- 模式 ----------
MODE = "AUTO"                     # "AUTO" / "CALIB" / "DETECT" / "RECORD" / "MATCH"

# ---------- 半区方向 ----------
LANDSCAPE = True                  # True=A4横放(297x210, 左右半区); False=竖放(210x297, 上下半区)
RECORD_RIGHT = False              # 录制模式检测半区：False=左半区（目标拼合位所在侧）
MATCH_RIGHT  = True               # 匹配模式检测半区：True=右半区（碎片随机摆放侧）
DETECT_RIGHT = True               # DETECT 模式检测半区

# ---------- 标定参数（Part1） ----------
RECT_THRESH_CALIB = 500           # 标定阶段 find_rects 阈值（高阈值防误检）
STABLE_CNT = 5                    # 连续稳定帧数才标定

# ---------- 检测参数（Part2/Part3 鲁棒版） ----------
# 主阈值用于正常识别；当不足4块时自动启用更宽松的备用阈值。
WHITE_THRESH         = [(38, 100, -30, 30, -10, 40)]
WHITE_THRESH_RELAXED = [(28, 100, -40, 40, -20, 50)]
# 只用于标准库录制的分离阈值。当前实机画面中，相邻白片之间的阴影桥
# 在L=38时会连成一块；L=42可断开该桥，同时保留四块总计约98%的前景面积。
# MATCH仍使用上面两套成功版阈值，不改变比赛识别和坐标输出。
WHITE_THRESH_RECORD_SEPARATE = [(42, 100, -30, 30, -10, 40)]

RECT_THRESH    = 200              # 运行阶段 A4 检测阈值
PIXELS_THRESHOLD = 30
AREA_THRESHOLD   = 30
MIN_PIECE_PX     = 120            # 原来300会把最小/细长碎片误删
RELAXED_MIN_PX   = 70
MAX_PIECE_PX     = 12000
MAX_BBOX_AREA    = 25000
MIN_DENSITY      = 0.07           # 三角形/细长块的外接框密度可能很低
MAX_ELONG        = 8.0
MERGE = False
MARGIN = 3
CENTER_DIST_THRESH = 10
LONG_MIN = 8
USE_LINE_SIGNATURE = False        # 完整轮廓匹配已取代线段签名，关闭可明显提速
MAX_MATCH_CANDIDATES = 6          # 备用阈值出现噪声时只保留较大的候选

# ---- 录制标准库时的A4纸边缘抗干扰 ----
# 录制位置由我们自己摆放，可以避开纸张外边缘；比赛MATCH阶段不使用此裁边，
# 防止随机摆放的真实碎片靠近右半区边缘时被误删。
RECORD_ROI_MARGIN_PX = 9          # 录制ROI从A4外边缘向内缩，中心分界线不缩
RECORD_EDGE_GUARD_MM = 7.0        # 候选轮廓距A4外边缘不足此距离时视为纸边干扰
RECORD_MIN_AREA_MM2 = 220.0       # 录制阶段过滤很小的边缘亮斑；当前最小碎片约600mm²
RECORD_FAILOPEN_GUARD_PX = 2      # 物理过滤异常时，候选仍须避开录制ROI外侧边界

# ---- 四块同时录制 ----
# 四块必须在左半区最终拼合位置摆好，并且二值图中能够分成4个独立连通区域。
RECORD_ALL_AT_ONCE = True
RECORD_STABLE_FRAMES = 5          # 连续5组完整识别稳定后一次性写入标准库
RECORD_POS_TOL_MM = 3.0           # 相邻完整帧同一碎片质心允许变化
RECORD_RESTART_MOVE_MM = 10.0     # 超过该位移认为重新摆放，清空录制历史
RECORD_MAX_CANDIDATES = 6         # 纸边过滤后仍有少量噪声时，从较大候选中选4块
RECORD_ORDER_BY_X = True          # 第一帧按物理X从小到大确定slot 0~3

# ---- 录制阶段面积稳定校验 ----
# 同一碎片在连续录制帧中的面积必须稳定，避免反光、断裂或粘连被写入标准库。
RECORD_AREA_TOL_RATIO = 0.18       # 正常稳定帧允许最大18%面积波动
RECORD_AREA_RESTART_RATIO = 0.35   # 超过35%认为识别对象发生变化，重新计数
RECORD_AREA_ALIGN_WEIGHT_MM = 25.0 # 多帧对齐时，面积差折算成位置代价

# ---- 匹配阶段面积身份校验 ----
# 面积先负责确认“是哪一块”，完整轮廓再负责角度和面积接近时的消歧。
MATCH_AREA_WEIGHT = 8.0            # 面积误差在总匹配代价中的权重
MATCH_AREA_MAX_PAIR_ERROR = 0.40   # 任意匹配对面积误差超过40%则整帧拒绝
MATCH_AREA_MAX_AVG_ERROR = 0.28    # 四块平均面积误差超过28%则整帧拒绝
MATCH_AREA_CONFIDENCE_GAP = 0.12   # 面积最优方案领先第二方案足够多时直接锁定

# 完整二值轮廓的极坐标描述子；继续用于身份匹配，旋转角改由有方向主轴计算。
RADIAL_BINS = 72
RADIAL_OFFSET = 9
MAX_CONTOUR_SAMPLES = 180
DESCRIPTOR_MIN_QUALITY = 0.20

# ---- 多边形结构消歧（复用要求2（1）前端成功版参数）----
# 径向描述子之后追加结构字段，不移动旧径向字段，避免改变既有角度计算索引。
STRUCT_OFFSET = RADIAL_OFFSET + RADIAL_BINS
STRUCT_VALID_INDEX = STRUCT_OFFSET
STRUCT_VERTEX_INDEX = STRUCT_OFFSET + 1
STRUCT_EDGE_OFFSET = STRUCT_OFFSET + 2
STRUCT_EDGE_COUNT = 5
STRUCT_SIGNATURE_LENGTH = STRUCT_EDGE_OFFSET + STRUCT_EDGE_COUNT

STRUCT_BOUNDARY_ROI_PAD_PX = 4
STRUCT_CONTOUR_SAMPLE_MAX = 240
STRUCT_RDP_START_MM = 0.8
STRUCT_RDP_MAX_MM = 8.0
STRUCT_RDP_STEP_MM = 0.4
STRUCT_MAX_VERTICES = 5
STRUCT_MIN_VERTEX_TURN_DEG = 12.0
STRUCT_COLLINEAR_DISTANCE_MM = 1.6
STRUCT_MIN_EDGE_MM = 12.0

# 结构只作为面积+径向轮廓的第三层通用证据，不按碎片或slot写死。
STRUCT_VERTEX_WEIGHT = 4.0
STRUCT_EDGE_PROFILE_WEIGHT = 2.0
MATCH_COMBINED_CONFIDENCE_GAP = MATCH_AREA_CONFIDENCE_GAP

NEW_SLOT_MIN_DIST = 20

SKIP_FRAMES = 2
VERBOSE = False
# 要求1诊断日志：比赛默认关闭。只向OpenMV IDE/USB终端打印，
# 不增加或改变发往STM32的任务字段。
REQ1_DIAGNOSTICS = False
DRAW_ANGLE = True
ANGLE_QUANTIZE = False
STALE_FRAMES = 30
MATCH_CACHE_FRAMES = 20            # 偶发一帧只检出3块时，短时间沿用上一组完整结果

# 面积分配准备锁定身份时，若有效的完整轮廓给出另一套全局分配，
# 本帧视为证据冲突并等待下一帧，不在两种身份之间猜测。
MATCH_REQUIRE_EVIDENCE_AGREEMENT = True

# ---- 移除遮挡后的光线稳定等待 ----
LIGHT_SETTLE_MS = 2000              # 首次检测到4块后等待2秒，再开始匹配和角度采样

# ---- 通用180°分支校正（仅在完整轮廓证据足够明确时启用） ----
# 不按三角形、面积或slot写死。对主轴给出的两个候选角 d 和 d+180°，
# 直接比较它们与目标完整轮廓的误差；只有两者差异足够明显时才改分支。
GENERIC_BRANCH_SCORE_ENABLE = True
BRANCH_MIN_CONTRAST = 0.18         # 两个候选误差的相对差异至少18%才允许翻转
NEAR_180_BRANCH_GUARD_DEG = 150.0  # 0/180分支区域证据不足时等待下一帧
BRANCH_SYMMETRY_RATIO_MAX = 0.12   # 模板自身近180°对称时，两分支物理等价

# ---- 主轴异常时的完整径向姿态回退 ----
# 正常姿态仍使用成功版的物理主轴角；只有主轴两分支都明显偏离完整径向
# 全局解，并且径向峰唯一、拟合充分、形状自身不具旋转对称性时才回退。
# 这用于修复斜边平行A4长边时，像素主轴经透视映射后偶发落入错误轴族。
RADIAL_POSE_MIN_QUALITY = 0.50
RADIAL_POSE_AXIS_GAP_DEG = 10.0
RADIAL_POSE_MIN_IMPROVEMENT = 0.35
RADIAL_POSE_MIN_UNIQUENESS = 0.30
RADIAL_POSE_MAX_FIT_RATIO = 0.55
RADIAL_POSE_MIN_SELF_RATIO = 0.30
RADIAL_POSE_EXCLUDE_BINS = 3

# ---- 角度稳定（完整轮廓给出有方向角，按360°周期投票）----
ANG_WIN        = 7
ANG_MIN_VOTES  = 4
ANG_TOL        = 10
ANG_JUMP_RESET = 45

# 身份稳定与角度稳定物理隔离。每个slot必须连续对应同一取料位置，
# 达到独立票数后才允许发送；不能再由“角度已经稳定”替代身份稳定。
IDENTITY_WIN = 7
IDENTITY_MIN_VOTES = 4
IDENTITY_POS_TOL_MM = 15.0

# ---- 有方向主轴角：二阶矩主轴 + 质心相对外接圆圆心偏移 ----
# 不调用 image.get_enclosing_circle() 等固件中不存在的接口；
# 外接圆直接由当前代码已经能读取的 contour.points() 在 Python 中计算。
DIRECTION_OFFSET_MIN_PX = 0.8       # 质心偏移小于该值时，180°方向不可靠
ENCIRCLE_MAX_POINTS     = 24        # 外接圆最多使用的轮廓采样点数

# ---- 旋转校准：与STM32代码对应 ----
# STM32每次抓取前都把旋转轴回到0，放置时把收到的ang作为绝对目标角；
# 因此这里发送“当前碎片旋转到目标轮廓所需的相对角”，范围[-180,180]。
ANG_SIGN   = 1                     # STM32正向若与视觉正角相反，改为-1
ANG_OFFSET = 0
SLOT_ANGLE_OFFSET = [0, 0, 0, 0]  # 暂不补偿slot1的16°，后续实测再填

# ---------- 二值化可视化（调试用） ----------
SHOW_BINARY_MASK = False          # 比赛关闭；调阈值时再改True，避免每帧复制图像拖慢速度
BINARY_PREVIEW_SCALE = 4          # 预览图相对ROI的缩小倍数（越大画面越小）

# ---- 串口 ----
UART_ENABLE   = True
UART_ID       = 3
UART_BAUD     = 115200

# STM32状态帧：
# $READY# / $ACK# / $WAIT_START# / $STARTED# / $RUN,n# /
# $DONE,n# / $ALL_DONE# / $ERR,...#
UART_RX_BUFFER_MAX = 192
CLEAR_STABLE_FRAMES = 4           # ALL_DONE后右半区连续少于4块，才允许下一轮
# ------------------------------

if LANDSCAPE:
    A4_W = 297.0
    A4_H = 210.0
else:
    A4_W = 210.0
    A4_H = 297.0

# ---------- UART由多模式main.py传入 ----------
uart_rx_buffer = ""

# ---------- STM32状态帧非阻塞接收 ----------

def poll_mcu_frames(uart_obj):
    """
    非阻塞读取STM32发来的ASCII帧，返回去掉$和#后的消息列表。
    能处理一次收到多帧、半帧以及噪声字节。
    """
    global uart_rx_buffer

    if uart_obj is None:
        return []

    try:
        count = uart_obj.any()
    except Exception:
        count = 0

    if not count:
        return []

    try:
        raw = uart_obj.read(count)
    except Exception as e:
        if VERBOSE:
            print("UART RX err:", e)
        return []

    if raw is None:
        return []

    try:
        # OpenMV不同固件对decode(errors=...)支持不完全，使用最兼容写法。
        if isinstance(raw, (bytes, bytearray)):
            try:
                chunk = raw.decode()
            except Exception:
                chunk = "".join(chr(b) for b in raw)
        else:
            chunk = str(raw)
    except Exception:
        chunk = ""

    if not chunk:
        return []

    uart_rx_buffer += chunk

    # 防止长期噪声导致字符串无限增长。
    if len(uart_rx_buffer) > UART_RX_BUFFER_MAX:
        dollar = uart_rx_buffer.rfind("$")
        if dollar >= 0:
            uart_rx_buffer = uart_rx_buffer[dollar:]
        else:
            uart_rx_buffer = ""

    messages = []

    while True:
        start = uart_rx_buffer.find("$")
        if start < 0:
            uart_rx_buffer = ""
            break

        if start > 0:
            uart_rx_buffer = uart_rx_buffer[start:]

        end = uart_rx_buffer.find("#", 1)
        if end < 0:
            break

        payload = uart_rx_buffer[1:end].strip()
        uart_rx_buffer = uart_rx_buffer[end + 1:]

        if payload:
            messages.append(payload)

    return messages

# ================= Part1：标定 =================
def gauss_solve_8(A, b):
    n = 8
    M = [A[i][:] + [b[i]] for i in range(n)]
    for col in range(n):
        pivot = col
        max_abs = abs(M[col][col])
        for row in range(col + 1, n):
            v = abs(M[row][col])
            if v > max_abs:
                max_abs = v
                pivot = row
        if max_abs < 1e-12:
            return None
        if pivot != col:
            M[col], M[pivot] = M[pivot], M[col]
        piv = M[col][col]
        for j in range(col, n + 1):
            M[col][j] /= piv
        for row in range(n):
            if row == col:
                continue
            factor = M[row][col]
            if abs(factor) > 0:
                for j in range(col, n + 1):
                    M[row][j] -= factor * M[col][j]
    return [M[i][n] for i in range(n)]

def solve_homography(src, dst):
    """4点解单应矩阵（像素->物理mm），返回9参数。"""
    A = [[0.0] * 8 for _ in range(8)]
    B = [0.0] * 8
    for i in range(4):
        x, y = src[i]
        X, Y = dst[i]
        A[2*i]   = [x, y, 1, 0, 0, 0, -X*x, -X*y]
        A[2*i+1] = [0, 0, 0, x, y, 1, -Y*x, -Y*y]
        B[2*i]   = X
        B[2*i+1] = Y
    h = gauss_solve_8(A, B)
    if h is None:
        return None
    return [h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1.0]

def save_calib(mm):
    try:
        with open("/flash/calib.txt", "w") as f:
            f.write("".join("%f\n" % v for v in mm))
        print("calib saved OK (%d params)" % len(mm))
        return True
    except Exception as e:
        print("save calib failed:", e)
        return False

def load_calib():
    try:
        with open("/flash/calib.txt", "r") as f:
            lines = [x for x in f.read().strip().split("\n") if x.strip() != ""]
            values = [float(x) for x in lines]

            # 兼容旧6参数仿射和当前9参数单应矩阵；残缺文件不能进入运行阶段。
            # 旧逻辑会让错误长度一直传到px_to_mm，并在录制边缘过滤中把每块
            # 候选静默删除，最终只看到“碎片数=0”。
            if len(values) not in (6, 9):
                print(
                    "calib.txt参数数目无效：%d（应为6或9），自动重新标定" %
                    len(values)
                )
                return None

            for value in values:
                if value != value or abs(value) > 1e9:
                    print("calib.txt含无效数值，自动重新标定")
                    return None

            return values
    except Exception:
        return None

def px_to_mm(m, x, y):
    if len(m) == 6:
        a, b, c, d, e, f = m
        return (a*x + b*y + c, d*x + e*y + f)
    h11, h12, h13, h21, h22, h23, h31, h32, h33 = m
    w = h31*x + h32*y + h33
    if abs(w) < 1e-9:
        w = 1e-9
    return ((h11*x + h12*y + h13) / w, (h21*x + h22*y + h23) / w)

def local_scale_at(m, x, y):
    if len(m) == 6:
        return math.sqrt(abs(m[0]*m[4] - m[1]*m[3]))
    X0, Y0 = px_to_mm(m, x, y)
    X1, Y1 = px_to_mm(m, x + 1, y)
    X2, Y2 = px_to_mm(m, x, y + 1)
    sx = math.sqrt((X1-X0)**2 + (Y1-Y0)**2)
    sy = math.sqrt((X2-X0)**2 + (Y2-Y0)**2)
    if sx <= 0 or sy <= 0:
        return 1.6
    return math.sqrt(sx * sy)

# ================= A4 检测（共用） =================
def select_a4_rect(rects, img):
    frame_area = img.width() * img.height()
    candidates = []
    for r in rects:
        area = r.w() * r.h()
        mx = max(r.w(), r.h())
        mn = min(r.w(), r.h())
        ar = mn / mx if mx > 0 else 0
        if 0.03 * frame_area < area < 0.95 * frame_area and 0.3 < ar < 1.0:
            candidates.append((area, r))
    return max(candidates, key=lambda x: x[0])[1] if candidates else None

def detect_a4_corners(img, thresh):
    try:
        rects = img.find_rects(threshold=thresh)
        if not rects:
            return None
        r = select_a4_rect(rects, img)
        if r is None:
            return None
        corners = r.corners()
        if not isinstance(corners, (list, tuple)) or len(corners) != 4:
            return None
        return corners
    except KeyboardInterrupt:
        raise
    except BaseException as e:
        print("WARN: detect_a4_corners failed:", e)
        return None

def classify_corners(corners):
    """图像上方=物理下方(BL/BR)，图像下方=物理上方(TL/TR)。"""
    ys = [p[1] for p in corners]
    y_thresh = (min(ys) + max(ys)) / 2
    top = [p for p in corners if p[1] <= y_thresh]
    bot = [p for p in corners if p[1] > y_thresh]
    top.sort(key=lambda p: p[0])
    bot.sort(key=lambda p: p[0])
    return {"BL": top[0], "BR": top[-1], "TL": bot[0], "TR": bot[-1]}

def half_roi(lab, img, right=True, outer_margin_px=0):
    """
    返回当前半区的矩形ROI。

    outer_margin_px 只从A4纸的外边缘向内缩，不裁掉左右半区的中心分界线。
    录制阶段用它排除A4白边、胶带和边缘反光；MATCH阶段保持0，避免漏掉
    随机摆在右半区边缘附近的真实碎片。
    """
    xs = [lab[c][0] for c in ("TL", "TR", "BL", "BR")]
    ys = [lab[c][1] for c in ("TL", "TR", "BL", "BR")]

    paper_min_x = max(0, int(min(xs)))
    paper_max_x = min(img.width() - 1, int(max(xs)))
    paper_min_y = max(0, int(min(ys)))
    paper_max_y = min(img.height() - 1, int(max(ys)))

    min_x = paper_min_x
    max_x = paper_max_x
    min_y = paper_min_y
    max_y = paper_max_y

    margin = max(0, int(outer_margin_px))

    if LANDSCAPE:
        mid_x = int(sum(xs) / 4)

        # 上、下都是A4外边缘，均向内缩。
        min_y += margin
        max_y -= margin

        if right:
            # 右半区：保留中线，右侧外边缘向内缩。
            min_x = mid_x
            max_x -= margin
        else:
            # 左半区：左侧外边缘向内缩，保留中线。
            min_x += margin
            max_x = mid_x
    else:
        mid_y = int(sum(ys) / 4)

        # 左、右都是A4外边缘。
        min_x += margin
        max_x -= margin

        if right:  # 竖放时 right 复用为 upper
            min_y = mid_y
            max_y -= margin
        else:
            min_y += margin
            max_y = mid_y

    min_x = max(0, min(min_x, img.width() - 2))
    min_y = max(0, min(min_y, img.height() - 2))
    max_x = max(min_x + 1, min(max_x, img.width() - 1))
    max_y = max(min_y + 1, min(max_y, img.height() - 1))

    return (min_x, min_y, max_x - min_x, max_y - min_y)

# ================= Part2：碎片检测（共用） =================
def safe_stat(img, rect, thresholds=None):
    x, y, w, h = rect
    x = max(0, min(x, img.width() - 1))
    y = max(0, min(y, img.height() - 1))
    w = max(1, min(w, img.width() - x))
    h = max(1, min(h, img.height() - y))
    try:
        return img.get_statistics(thresholds, roi=(x, y, w, h))
    except Exception:
        return None

def rect_dist(r1, r2):
    x1, y1, w1, h1 = r1
    x2, y2, w2, h2 = r2
    if x1 + w1 < x2:
        dx = x2 - (x1 + w1)
    elif x2 + w2 < x1:
        dx = x1 - (x2 + w2)
    else:
        dx = 0
    if y1 + h1 < y2:
        dy = y2 - (y1 + h1)
    elif y2 + h2 < y1:
        dy = y1 - (y2 + h2)
    else:
        dy = 0
    return math.sqrt(dx * dx + dy * dy)

def cluster_blobs(blobs):
    if not blobs:
        return []
    items = [(b.cx(), b.cy(), b.rect(), b.pixels(), b) for b in blobs]
    used = [False] * len(items)
    groups = []
    for i in range(len(items)):
        if used[i]:
            continue
        cx, cy, rect, area, b0 = items[i]
        xs = [rect[0], rect[0] + rect[2]]
        ys = [rect[1], rect[1] + rect[3]]
        group_blobs = [b0]
        used[i] = True
        changed = True
        while changed:
            changed = False
            cur_rect = (min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys))
            for j in range(len(items)):
                if used[j]:
                    continue
                cx2, cy2, r2, _, bj = items[j]
                d = rect_dist(cur_rect, r2)
                dc = math.sqrt((cx2 - cx)**2 + (cy2 - cy)**2)
                if d < 1.0 and dc < CENTER_DIST_THRESH:
                    used[j] = True
                    group_blobs.append(bj)
                    xs += [r2[0], r2[0] + r2[2]]
                    ys += [r2[1], r2[1] + r2[3]]
                    cx = (cx * area + cx2 * items[j][3]) / (area + items[j][3])
                    cy = (cy * area + cy2 * items[j][3]) / (area + items[j][3])
                    area += items[j][3]
                    changed = True
        r = (min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys))
        groups.append((r, group_blobs))
    return groups

def _detect_pieces_once(img, roi, thresholds, min_pixels):
    """
    find_blobs 已经直接在阈值二值掩膜上寻找连通区域。
    不再对整个外接矩形求平均LAB：三角形/细长块的框内黑背景很多，
    旧逻辑会把真实白色碎片错误过滤掉。
    """
    try:
        blobs = img.find_blobs(
            thresholds,
            roi=roi,
            x_stride=1,
            y_stride=1,
            pixels_threshold=max(20, min_pixels // 2),
            area_threshold=max(20, min_pixels // 2),
            merge=MERGE,
            margin=MARGIN
        )
    except Exception as e:
        if VERBOSE:
            print("find_blobs err:", e)
        return []

    filtered = []
    for b in blobs:
        if b.pixels() < min_pixels:
            continue
        filtered.append(b)

    return filter_pieces(cluster_blobs(filtered), roi)

def _piece_pixel_count(piece):
    return sum(b.pixels() for b in piece[1])

def detect_pieces(img, roi):
    """
    两级阈值检测：
      1. 主阈值；
      2. 主阈值不足4块时，自动启用宽松阈值。
    MATCH阶段若出现少量噪声，按真实前景像素数保留较大的候选。
    """
    primary = _detect_pieces_once(img, roi, WHITE_THRESH, MIN_PIECE_PX)

    if len(primary) >= 4:
        if REQ1_DIAGNOSTICS:
            print(
                "REQ1DBG threshold=PRIMARY candidates=%d" % len(primary)
            )
        primary.sort(key=_piece_pixel_count, reverse=True)
        return primary[:MAX_MATCH_CANDIDATES]

    relaxed = _detect_pieces_once(img, roi, WHITE_THRESH_RELAXED, RELAXED_MIN_PX)

    # 取候选数更多的一组，避免把两套阈值的同一碎片重复加入。
    pieces = relaxed if len(relaxed) > len(primary) else primary
    if REQ1_DIAGNOSTICS:
        print(
            "REQ1DBG threshold=%s primary=%d relaxed=%d selected=%d" %
            (
                "RELAXED" if len(relaxed) > len(primary) else "PRIMARY",
                len(primary), len(relaxed), len(pieces)
            )
        )
    pieces.sort(key=_piece_pixel_count, reverse=True)

    if VERBOSE and len(primary) != len(relaxed):
        print("detect fallback primary=%d relaxed=%d" %
              (len(primary), len(relaxed)))

    return pieces[:MAX_MATCH_CANDIDATES]

def _piece_mm_bounds(piece, m):
    """把候选外接范围的四角映射到A4毫米坐标，返回物理边界。"""
    r, _ = piece
    bx, by, bw, bh = r
    x2 = bx + max(0, bw - 1)
    y2 = by + max(0, bh - 1)

    pts_mm = [
        px_to_mm(m, bx, by),
        px_to_mm(m, x2, by),
        px_to_mm(m, bx, y2),
        px_to_mm(m, x2, y2)
    ]

    xs = [p[0] for p in pts_mm]
    ys = [p[1] for p in pts_mm]
    return min(xs), max(xs), min(ys), max(ys)

def _piece_area_mm2_approx(piece, m):
    """按二值前景像素数和局部比例估算面积，用于快速过滤微小亮斑。"""
    r, blist = piece
    cx, cy = binary_shape_centroid(blist, r)
    scale = local_scale_at(m, cx, cy)
    total_px = sum(b.pixels() for b in blist)
    return total_px * scale * scale

def filter_record_edge_artifacts(pieces, m, record_right):
    """
    仅用于RECORD阶段：
      1. 删除贴着A4纸外边缘的白边/反光候选；
      2. 删除物理面积很小的亮斑；
      3. 不把半区中心分界线当作外边缘，因此靠近中线的真实目标仍可录制。
    """
    kept = []

    for piece in pieces:
        try:
            min_x, max_x, min_y, max_y = _piece_mm_bounds(piece, m)
            area_mm2 = _piece_area_mm2_approx(piece, m)
        except Exception as e:
            if VERBOSE:
                print("record edge filter err:", e)
            continue

        if area_mm2 < RECORD_MIN_AREA_MM2:
            if VERBOSE:
                print("  record noise dropped: area=%.0fmm2" % area_mm2)
            continue

        near_outer_edge = False

        if LANDSCAPE:
            # 上下边始终是A4外边缘。
            if min_y < RECORD_EDGE_GUARD_MM:
                near_outer_edge = True
            if max_y > A4_H - RECORD_EDGE_GUARD_MM:
                near_outer_edge = True

            # 左半区只检查左纸边；右半区只检查右纸边。中线不检查。
            if record_right:
                if max_x > A4_W - RECORD_EDGE_GUARD_MM:
                    near_outer_edge = True
            else:
                if min_x < RECORD_EDGE_GUARD_MM:
                    near_outer_edge = True
        else:
            # 竖放时左右均为外边缘。
            if min_x < RECORD_EDGE_GUARD_MM:
                near_outer_edge = True
            if max_x > A4_W - RECORD_EDGE_GUARD_MM:
                near_outer_edge = True

            if record_right:
                if max_y > A4_H - RECORD_EDGE_GUARD_MM:
                    near_outer_edge = True
            else:
                if min_y < RECORD_EDGE_GUARD_MM:
                    near_outer_edge = True

        if near_outer_edge:
            if VERBOSE:
                print(
                    "  A4 edge artifact dropped: "
                    "x=(%.1f,%.1f) y=(%.1f,%.1f) area=%.0f" %
                    (min_x, max_x, min_y, max_y, area_mm2)
                )
            continue

        kept.append(piece)

    return kept


def _record_piece_clear_of_outer_roi_edges(piece, roi, record_right):
    """录制物理过滤失效时，用当前帧ROI边界做安全兜底检查。"""
    r, _ = piece
    bx, by, bw, bh = r
    rx, ry, rw, rh = roi
    guard = RECORD_FAILOPEN_GUARD_PX

    left = bx
    right = bx + max(0, bw - 1)
    top = by
    bottom = by + max(0, bh - 1)
    roi_left = rx
    roi_right = rx + max(0, rw - 1)
    roi_top = ry
    roi_bottom = ry + max(0, rh - 1)

    if LANDSCAPE:
        # 上下始终是A4外边；半区中心侧允许靠近。
        if top <= roi_top + guard or bottom >= roi_bottom - guard:
            return False
        if record_right:
            return right < roi_right - guard
        return left > roi_left + guard

    # 竖放时左右始终是A4外边；半区中心侧允许靠近。
    if left <= roi_left + guard or right >= roi_right - guard:
        return False
    if record_right:
        return bottom < roi_bottom - guard
    return top > roi_top + guard


def detect_record_pieces(img, roi, m, record_right):
    """
    录制专用检测包装：
      1. 保留成功版主/宽松阈值检测；
      2. 不足4块时才尝试L=42分离阈值；
      3. 物理坐标边缘过滤异常地把完整4块删掉时，仅在4块都远离
         当前帧ROI外侧边界的条件下恢复原候选。

    返回(pieces, info)，info只用于串口日志，不参与录制或匹配。
    """
    standard = detect_pieces(img, roi)
    separated = []
    separate_tried = False
    source = standard
    path = "STANDARD"

    if len(standard) < 4:
        separate_tried = True
        separated = _detect_pieces_once(
            img,
            roi,
            WHITE_THRESH_RECORD_SEPARATE,
            MIN_PIECE_PX
        )

        # 只有恰好得到4块时才采用，避免把过分割或反光候选带入标准库。
        if len(separated) == 4:
            source = separated
            path = "RECORD_L42"

    edge_filtered = filter_record_edge_artifacts(
        source,
        m,
        record_right
    )
    pieces = edge_filtered

    if (
        len(source) == 4 and
        len(edge_filtered) < 4 and
        all(
            _record_piece_clear_of_outer_roi_edges(
                piece,
                roi,
                record_right
            )
            for piece in source
        )
    ):
        # ROI已经从A4外边缘内缩9px；这里防止旧/异常标定矩阵把画面中央
        # 的四个真实候选全部当成纸边删除。后续仍须通过5帧面积、结构稳定检查。
        pieces = source
        path += "+ROI_SAFE_FALLBACK"

    info = (
        len(standard),
        len(separated) if separate_tried else -1,
        len(edge_filtered),
        path
    )
    return pieces, info

def draw_binary_preview(img, roi):
    """把ROI区域的二值化结果缩放到画面左上角显示，方便调试阈值。"""
    if not SHOW_BINARY_MASK:
        return
    try:
        rx, ry, rw, rh = roi
        pw = max(rw // BINARY_PREVIEW_SCALE, 20)
        ph = max(rh // BINARY_PREVIEW_SCALE, 20)
        # copy=True 避免修改原图；roi 只处理当前半区
        mask = img.binary(WHITE_THRESH, roi=roi, copy=True)
        # 裁剪出ROI区域再缩小
        small = mask.copy((rx, ry, rw, rh)).resize(pw, ph)
        img.draw_image(small, 0, 0)
        img.draw_rectangle(0, 0, pw, ph, color=(255, 0, 0), thickness=1)
        del mask, small
    except Exception as e:
        if VERBOSE:
            print("binary preview err:", e)

def filter_pieces(pieces, roi):
    roi_w, roi_h = roi[2], roi[3]
    kept = []

    for (r, blist) in pieces:
        bx, by, bw, bh = r
        total_px = sum(b.pixels() for b in blist)
        bbox_area = bw * bh
        density = total_px / bbox_area if bbox_area > 0 else 0.0

        if total_px <= 0:
            continue
        if total_px > MAX_PIECE_PX or bbox_area > MAX_BBOX_AREA:
            continue

        # 白色纸边/反光条通常又长又稀；真实三角形则允许较低密度。
        if density < MIN_DENSITY:
            if VERBOSE:
                print("  low density dropped px=%d bbox=%d density=%.2f" %
                      (total_px, bbox_area, density))
            continue

        if min(bw, bh) <= 0:
            continue

        elong = float(max(bw, bh)) / min(bw, bh)
        if elong > MAX_ELONG:
            if VERBOSE:
                print("  bad elong dropped w=%d h=%d elong=%.1f" %
                      (bw, bh, elong))
            continue

        # 跨越大半个检测半区的对象更可能是A4边缘或背景反光。
        if bw > roi_w * 0.72 or bh > roi_h * 0.72:
            continue

        kept.append((r, blist))

    return kept

# ---------- 位姿与形状签名 ----------
def binary_shape_centroid(blist, rect):
    """
    按二值化后属于碎片的全部像素计算真实形状质心。

    单个 blob 的 cx/cy 本身就是匹配像素的平均位置；
    若同一碎片因反光或阈值断裂被分成多个 blob，则按像素数加权合并，
    避免使用外接矩形中心。
    """
    if blist:
        total_px = sum(b.pixels() for b in blist)
        if total_px > 0:
            cx = sum(b.cx() * b.pixels() for b in blist) / total_px
            cy = sum(b.cy() * b.pixels() for b in blist) / total_px
            return cx, cy

    x, y, w, h = rect
    return x + w / 2.0, y + h / 2.0

def get_main_contour(img, rect, blist, contour_thresholds=None):
    """
    在碎片ROI内提取真实二值轮廓。
    主阈值失败时再尝试宽松阈值，保证小块/细长块也能得到轮廓。
    """
    if not blist:
        return None

    if contour_thresholds is None:
        contour_thresholds = (WHITE_THRESH, WHITE_THRESH_RELAXED)

    for thresholds in contour_thresholds:
        try:
            cs = img.find_contours(thresholds, roi=rect)
            if cs:
                return max(cs, key=lambda c: c.area())
        except Exception as e:
            if VERBOSE:
                print("contour err:", e)

    return None


# 以下固件兼容边界追踪与多边形简化流程取自要求2（1）前端成功版；
# 要求1继续使用自己的A4毫米映射，不改变质心、角度或径向轮廓。
def _structure_pixel_brightness(value):
    if isinstance(value, (tuple, list)):
        if len(value) >= 3:
            return int((value[0] + value[1] + value[2]) // 3)
        return int(value[0]) if value else 0
    if value is None:
        return 0
    value = int(value)
    if value <= 255:
        return value
    red = ((value >> 11) & 0x1f) * 255 // 31
    green = ((value >> 5) & 0x3f) * 255 // 63
    blue = (value & 0x1f) * 255 // 31
    return (red + green + blue) // 3


def _structure_otsu_threshold(histogram, total):
    weighted_total = 0
    for index in range(256):
        weighted_total += index * histogram[index]
    background_weight = 0
    background_sum = 0
    best_threshold = 127
    best_variance = -1.0
    for threshold in range(256):
        count = histogram[threshold]
        background_weight += count
        background_sum += threshold * count
        if background_weight <= 0:
            continue
        foreground_weight = total - background_weight
        if foreground_weight <= 0:
            break
        background_mean = background_sum / background_weight
        foreground_mean = (
            (weighted_total - background_sum) / foreground_weight
        )
        difference = background_mean - foreground_mean
        variance = (
            background_weight * foreground_weight *
            difference * difference
        )
        if variance > best_variance:
            best_variance = variance
            best_threshold = threshold
    return best_threshold


def _structure_trace_boundary(mask, seed_x, seed_y):
    height = len(mask)
    width = len(mask[0]) if height else 0
    if width <= 0:
        return []
    seed_x = max(0, min(width - 1, int(seed_x)))
    seed_y = max(0, min(height - 1, int(seed_y)))
    nearest = (seed_x, seed_y) if mask[seed_y][seed_x] else None
    if nearest is None:
        nearest_distance = 1e30
        for y in range(height):
            row = mask[y]
            for x in range(width):
                if row[x]:
                    distance = (x - seed_x) ** 2 + (y - seed_y) ** 2
                    if distance < nearest_distance:
                        nearest = (x, y)
                        nearest_distance = distance
    if nearest is None:
        return []
    x, y = nearest
    while x > 0 and mask[y][x - 1]:
        x -= 1
    directions = [
        (1, 0), (1, 1), (0, 1), (-1, 1),
        (-1, 0), (-1, -1), (0, -1), (1, -1)
    ]
    start = (x, y)
    current = start
    backtrack = (x - 1, y)
    first_next = None
    points = [start]
    for _ in range(width * height * 4):
        back_direction = (
            backtrack[0] - current[0],
            backtrack[1] - current[1]
        )
        try:
            back_index = directions.index(back_direction)
        except ValueError:
            back_index = 4
        next_point = None
        for step in range(1, 9):
            direction_index = (back_index + step) % 8
            dx, dy = directions[direction_index]
            nx, ny = current[0] + dx, current[1] + dy
            if (
                0 <= nx < width and
                0 <= ny < height and
                mask[ny][nx]
            ):
                previous_index = (direction_index - 1) % 8
                pdx, pdy = directions[previous_index]
                backtrack = (current[0] + pdx, current[1] + pdy)
                next_point = (nx, ny)
                break
        if next_point is None:
            return points
        if first_next is None:
            first_next = next_point
        elif current == start and next_point == first_next:
            if points and points[-1] == start:
                points.pop()
            break
        points.append(next_point)
        current = next_point
    return points


def _structure_adaptive_boundary(frame, rectangle, seed):
    x, y, width, height = rectangle
    x0 = max(0, x - STRUCT_BOUNDARY_ROI_PAD_PX)
    y0 = max(0, y - STRUCT_BOUNDARY_ROI_PAD_PX)
    x1 = min(frame.width(), x + width + STRUCT_BOUNDARY_ROI_PAD_PX)
    y1 = min(frame.height(), y + height + STRUCT_BOUNDARY_ROI_PAD_PX)
    width = max(1, x1 - x0)
    height = max(1, y1 - y0)
    brightness = [bytearray(width) for _ in range(height)]
    histogram = [0] * 256
    minimum = 255
    maximum = 0
    for local_y in range(height):
        row = brightness[local_y]
        for local_x in range(width):
            value = _structure_pixel_brightness(
                frame.get_pixel(x0 + local_x, y0 + local_y)
            )
            row[local_x] = value
            histogram[value] += 1
            if value < minimum:
                minimum = value
            if value > maximum:
                maximum = value
    threshold = _structure_otsu_threshold(
        histogram,
        width * height
    )
    mask = [bytearray(width) for _ in range(height)]
    foreground_count = 0
    for local_y in range(height):
        source_row = brightness[local_y]
        mask_row = mask[local_y]
        for local_x in range(width):
            if source_row[local_x] > threshold:
                mask_row[local_x] = 1
                foreground_count += 1
    del brightness
    del histogram
    points = _structure_trace_boundary(
        mask,
        int(round(seed[0] - x0)),
        int(round(seed[1] - y0))
    )
    del mask
    return (
        [(point[0] + x0, point[1] + y0) for point in points],
        threshold,
        minimum,
        maximum,
        foreground_count
    )


def _structure_point_line_distance(point, start, end):
    vx = end[0] - start[0]
    vy = end[1] - start[1]
    denominator = vx * vx + vy * vy
    if denominator <= 1e-12:
        dx = point[0] - start[0]
        dy = point[1] - start[1]
        return math.sqrt(dx * dx + dy * dy)
    numerator = abs(
        vy * point[0] - vx * point[1] +
        end[0] * start[1] - end[1] * start[0]
    )
    return numerator / math.sqrt(denominator)


def _structure_rdp_open(points, epsilon):
    if len(points) <= 2:
        return points[:]

    maximum = -1.0
    split = 0
    for index in range(1, len(points) - 1):
        distance = _structure_point_line_distance(
            points[index], points[0], points[-1]
        )
        if distance > maximum:
            maximum = distance
            split = index

    if maximum > epsilon:
        left = _structure_rdp_open(points[:split + 1], epsilon)
        right = _structure_rdp_open(points[split:], epsilon)
        return left[:-1] + right
    return [points[0], points[-1]]


def _structure_vertex_turn_degrees(previous, current, following):
    first_x = current[0] - previous[0]
    first_y = current[1] - previous[1]
    second_x = following[0] - current[0]
    second_y = following[1] - current[1]
    first_length = math.sqrt(first_x * first_x + first_y * first_y)
    second_length = math.sqrt(second_x * second_x + second_y * second_y)
    if first_length <= 1e-9 or second_length <= 1e-9:
        return 0.0
    cosine = (
        (first_x * second_x + first_y * second_y) /
        (first_length * second_length)
    )
    cosine = max(-1.0, min(1.0, cosine))
    return math.degrees(math.acos(cosine))


def _structure_simplify_closed(points, epsilon):
    if len(points) < 3:
        return []

    first = points[0]
    farthest = 1
    maximum = 0.0
    for index in range(1, len(points)):
        dx = points[index][0] - first[0]
        dy = points[index][1] - first[1]
        distance = dx * dx + dy * dy
        if distance > maximum:
            maximum = distance
            farthest = index

    first_chain = _structure_rdp_open(
        points[:farthest + 1], epsilon
    )
    second_chain = _structure_rdp_open(
        points[farthest:] + [points[0]], epsilon
    )
    polygon = first_chain[:-1] + second_chain[:-1]

    while len(polygon) > 3:
        weakest_index = None
        weakest_rank = None
        count = len(polygon)

        for index in range(count):
            previous = polygon[(index - 1) % count]
            current = polygon[index]
            following = polygon[(index + 1) % count]
            edge1 = math.sqrt(
                (current[0] - previous[0]) ** 2 +
                (current[1] - previous[1]) ** 2
            )
            edge2 = math.sqrt(
                (following[0] - current[0]) ** 2 +
                (following[1] - current[1]) ** 2
            )
            line_distance = _structure_point_line_distance(
                current, previous, following
            )
            turn_degrees = _structure_vertex_turn_degrees(
                previous, current, following
            )
            removable = (
                min(edge1, edge2) < STRUCT_MIN_EDGE_MM or
                line_distance < max(
                    STRUCT_COLLINEAR_DISTANCE_MM,
                    epsilon * 0.55
                ) or
                turn_degrees < STRUCT_MIN_VERTEX_TURN_DEG
            )

            if removable:
                rank = (turn_degrees, line_distance, min(edge1, edge2))
                if weakest_rank is None or rank < weakest_rank:
                    weakest_rank = rank
                    weakest_index = index

        if weakest_index is None:
            break
        del polygon[weakest_index]

    return polygon


def _structure_polygon_vertices(contour_pixels, m):
    if not contour_pixels or len(contour_pixels) < 6:
        return [], 0.0

    step = max(1, len(contour_pixels) // STRUCT_CONTOUR_SAMPLE_MAX)
    points_mm = [
        px_to_mm(m, contour_pixels[index][0], contour_pixels[index][1])
        for index in range(0, len(contour_pixels), step)
    ]

    epsilon = STRUCT_RDP_START_MM
    best = []
    best_epsilon = epsilon
    while epsilon <= STRUCT_RDP_MAX_MM + 1e-6:
        polygon = _structure_simplify_closed(points_mm, epsilon)
        if len(polygon) >= 3:
            best = polygon
            best_epsilon = epsilon
        if 3 <= len(polygon) <= STRUCT_MAX_VERTICES:
            return polygon, epsilon
        epsilon += STRUCT_RDP_STEP_MM

    return best, best_epsilon


def _canonical_edge_profile(polygon):
    """生成与起点、顺逆时针和整体尺度无关的排序边长比例。"""
    count = len(polygon)
    if count < 3 or count > STRUCT_EDGE_COUNT:
        return [0.0] * STRUCT_EDGE_COUNT

    lengths = []
    for index in range(count):
        x1, y1 = polygon[index]
        x2, y2 = polygon[(index + 1) % count]
        dx = x2 - x1
        dy = y2 - y1
        lengths.append(math.sqrt(dx * dx + dy * dy))

    perimeter = sum(lengths)
    if perimeter <= 1e-6:
        return [0.0] * STRUCT_EDGE_COUNT

    canonical = sorted(
        [value / perimeter for value in lengths],
        reverse=True
    )
    while len(canonical) < STRUCT_EDGE_COUNT:
        canonical.append(0.0)
    return canonical


def build_polygon_structure_from_points(contour_points, m):
    """把成功版边界点链转换为结构签名。"""
    polygon, epsilon = _structure_polygon_vertices(contour_points, m)
    if not (3 <= len(polygon) <= STRUCT_MAX_VERTICES):
        # 保留实际简化出的顶点数，便于录制失败时定位轮廓是缺失还是多出毛刺。
        # valid仍为0，因此该值不参与标准库平均、身份匹配或角度计算。
        return 0.0, len(polygon), [0.0] * STRUCT_EDGE_COUNT, epsilon

    profile = _canonical_edge_profile(polygon)
    if max(profile) <= 0.0:
        return 0.0, 0, [0.0] * STRUCT_EDGE_COUNT, epsilon

    return 1.0, len(polygon), profile, epsilon


def build_polygon_structure(contour, m):
    """兼容旧调用：从OpenMV contour对象生成结构签名。"""
    if contour is None:
        return 0.0, 0, [0.0] * STRUCT_EDGE_COUNT, 0.0

    try:
        contour_points = contour.points()
    except Exception:
        contour_points = None

    return build_polygon_structure_from_points(contour_points, m)

def _fill_radial_gaps(values):
    """循环插值填补极坐标描述子中的空角度格。"""
    n = len(values)
    nonzero = [i for i, v in enumerate(values) if v > 0]

    if len(nonzero) < 3:
        return values

    out = values[:]

    for i in range(n):
        if out[i] > 0:
            continue

        left_i = None
        right_i = None
        left_d = 0
        right_d = 0

        for d in range(1, n):
            j = (i - d) % n
            if values[j] > 0:
                left_i = j
                left_d = d
                break

        for d in range(1, n):
            j = (i + d) % n
            if values[j] > 0:
                right_i = j
                right_d = d
                break

        if left_i is not None and right_i is not None:
            total = left_d + right_d
            out[i] = (values[left_i] * right_d +
                      values[right_i] * left_d) / total
        elif left_i is not None:
            out[i] = values[left_i]
        elif right_i is not None:
            out[i] = values[right_i]

    return out

def build_radial_descriptor_from_points(pts, m, cx, cy):
    """从有序边界点生成要求1原有的72格径向描述子。"""
    if not pts or len(pts) < 6:
        return [0.0] * RADIAL_BINS, 0.0

    center_x_mm, center_y_mm = px_to_mm(m, cx, cy)
    desc = [0.0] * RADIAL_BINS

    step = max(1, len(pts) // MAX_CONTOUR_SAMPLES)
    used = 0

    for idx in range(0, len(pts), step):
        px, py = pts[idx]
        x_mm, y_mm = px_to_mm(m, px, py)
        dx = x_mm - center_x_mm
        dy = y_mm - center_y_mm
        radius = math.sqrt(dx * dx + dy * dy)

        if radius <= 1e-6:
            continue

        theta = math.atan2(dy, dx)
        if theta < 0:
            theta += 2.0 * math.pi

        bin_i = int(theta * RADIAL_BINS / (2.0 * math.pi) + 0.5) % RADIAL_BINS

        if radius > desc[bin_i]:
            desc[bin_i] = radius

        used += 1

    nonzero = sum(1 for v in desc if v > 0)
    quality = float(nonzero) / RADIAL_BINS

    if used < 6 or nonzero < 3:
        return [0.0] * RADIAL_BINS, 0.0

    desc = _fill_radial_gaps(desc)

    # 轻微圆周平滑，降低单像素轮廓抖动。
    smooth = [0.0] * RADIAL_BINS
    for i in range(RADIAL_BINS):
        smooth[i] = (
            desc[(i - 1) % RADIAL_BINS] +
            2.0 * desc[i] +
            desc[(i + 1) % RADIAL_BINS]
        ) / 4.0

    max_r = max(smooth)
    if max_r <= 1e-6:
        return [0.0] * RADIAL_BINS, 0.0

    return [v / max_r for v in smooth], quality


def build_radial_descriptor(contour, m, cx, cy):
    """
    把完整二值轮廓映射到A4毫米坐标，以质心为原点建立360°径向描述子。
    描述子随碎片旋转做循环移位，因此可直接求完整[-180,180]旋转角，
    不再把theta和theta+180误认为同一方向。
    """
    if contour is None:
        return [0.0] * RADIAL_BINS, 0.0

    try:
        pts = contour.points()
    except Exception:
        pts = None

    return build_radial_descriptor_from_points(pts, m, cx, cy)

def _circle_contains(circle, point):
    """判断点是否位于圆内；circle=(cx,cy,r)。"""
    if circle is None:
        return False
    dx = point[0] - circle[0]
    dy = point[1] - circle[1]
    return dx * dx + dy * dy <= circle[2] * circle[2] + 1e-5


def _diameter_circle(a, b):
    """以两点连线为直径的圆。"""
    cx = (a[0] + b[0]) * 0.5
    cy = (a[1] + b[1]) * 0.5
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return (cx, cy, math.sqrt(dx * dx + dy * dy) * 0.5)


def _circle_from_three(a, b, c):
    """三点外接圆；近共线时退化为能包住三点的最小直径圆。"""
    ax, ay = a
    bx, by = b
    cx, cy = c

    d = 2.0 * (
        ax * (by - cy) +
        bx * (cy - ay) +
        cx * (ay - by)
    )

    if abs(d) < 1e-7:
        candidates = (
            _diameter_circle(a, b),
            _diameter_circle(a, c),
            _diameter_circle(b, c)
        )
        best = None
        for circle in candidates:
            if (
                _circle_contains(circle, a) and
                _circle_contains(circle, b) and
                _circle_contains(circle, c)
            ):
                if best is None or circle[2] < best[2]:
                    best = circle
        return best if best is not None else candidates[0]

    a2 = ax * ax + ay * ay
    b2 = bx * bx + by * by
    c2 = cx * cx + cy * cy

    ux = (
        a2 * (by - cy) +
        b2 * (cy - ay) +
        c2 * (ay - by)
    ) / d

    uy = (
        a2 * (cx - bx) +
        b2 * (ax - cx) +
        c2 * (bx - ax)
    ) / d

    dx = ux - ax
    dy = uy - ay
    return (ux, uy, math.sqrt(dx * dx + dy * dy))


def _sample_contour_points(contour):
    """
    从真实轮廓均匀采样，控制最小外接圆计算量。
    这里只使用本工程中已经验证可运行的 contour.points()。
    """
    if contour is None:
        return []

    try:
        raw = contour.points()
    except Exception:
        return []

    if not raw:
        return []

    n = len(raw)
    if n <= ENCIRCLE_MAX_POINTS:
        return [(float(p[0]), float(p[1])) for p in raw]

    step = float(n) / ENCIRCLE_MAX_POINTS
    points = []
    pos = 0.0

    for _ in range(ENCIRCLE_MAX_POINTS):
        idx = int(pos)
        if idx >= n:
            idx = n - 1
        p = raw[idx]
        points.append((float(p[0]), float(p[1])))
        pos += step

    return points


def _minimum_enclosing_circle(points):
    """
    对少量轮廓采样点求最小外接圆。

    使用确定性的增量三层算法，点数被限制为24，因此适合OpenMV。
    不依赖当前固件缺失的 image.get_enclosing_circle()。
    """
    if not points:
        return None

    circle = None

    for i in range(len(points)):
        p = points[i]

        if circle is not None and _circle_contains(circle, p):
            continue

        circle = (p[0], p[1], 0.0)

        for j in range(i):
            q = points[j]

            if _circle_contains(circle, q):
                continue

            circle = _diameter_circle(p, q)

            for k in range(j):
                r = points[k]

                if _circle_contains(circle, r):
                    continue

                circle = _circle_from_three(p, q, r)

    return circle


def _fallback_circle_from_rect(rect):
    """轮廓不可用时，仅以外接矩形构造备用圆。"""
    x, y, w, h = rect
    cx = x + w * 0.5
    cy = y + h * 0.5
    radius = math.sqrt(w * w + h * h) * 0.5
    return (cx, cy, radius)


def rough_contour_angle(contour, main_blob, rect, m, shape_cx, shape_cy):
    """
    得到0~360°有方向主轴角。

    1. 用二阶矩主轴 major_axis_line() 得到只有180°含义的轴；
    2. 用真实轮廓的最小外接圆圆心到碎片质心的偏移确定“重端”；
    3. 在主轴的两个相反方向中，选择更靠近“重端”的方向；
    4. 把所选方向映射到A4毫米坐标，得到最终绝对朝向角。

    这里只更换角度算法，不改检测、标定、面积身份、轮廓签名和串口流程。
    """
    x, y, w, h = rect
    elong = 1.0

    # 保留原代码的长宽比计算，用于形状签名。
    if contour is not None:
        try:
            corners = contour.min_corners()
            side_lens = []

            for j in range(len(corners)):
                x1, y1 = corners[j]
                x2, y2 = corners[(j + 1) % len(corners)]
                dx = x2 - x1
                dy = y2 - y1
                side_lens.append(math.sqrt(dx * dx + dy * dy))

            side_lens.sort(reverse=True)
            if len(side_lens) >= 2 and side_lens[1] > 0:
                elong = side_lens[0] / side_lens[1]
        except Exception:
            pass

    # 二阶矩主轴向量。major_axis_line() 在原工程中已经使用过，
    # 因此不引入任何新的固件API。
    axis_dx = 1.0
    axis_dy = 0.0

    if main_blob is not None:
        try:
            x1, y1, x2, y2 = main_blob.major_axis_line()
            axis_dx = float(x2 - x1)
            axis_dy = float(y2 - y1)
        except Exception:
            try:
                theta = main_blob.rotation()
                axis_dx = math.cos(theta)
                axis_dy = math.sin(theta)
            except Exception:
                axis_dx = 1.0 if w >= h else 0.0
                axis_dy = 0.0 if w >= h else 1.0

    axis_norm = math.sqrt(axis_dx * axis_dx + axis_dy * axis_dy)
    if axis_norm <= 1e-7:
        axis_dx = 1.0 if w >= h else 0.0
        axis_dy = 0.0 if w >= h else 1.0
        axis_norm = 1.0

    axis_dx /= axis_norm
    axis_dy /= axis_norm

    # 由轮廓采样点自行计算最小外接圆，避免调用固件不支持的接口。
    circle = _minimum_enclosing_circle(
        _sample_contour_points(contour)
    )

    if circle is None:
        circle = _fallback_circle_from_rect(rect)

    circle_x, circle_y, _ = circle

    # 浮点形状质心相对最小外接圆圆心的偏移。
    head_dx = shape_cx - circle_x
    head_dy = shape_cy - circle_y
    offset = math.sqrt(head_dx * head_dx + head_dy * head_dy)

    if offset >= DIRECTION_OFFSET_MIN_PX:
        # 点积小于0，说明当前主轴方向背离“重端”，翻转180°。
        if axis_dx * head_dx + axis_dy * head_dy < 0:
            axis_dx = -axis_dx
            axis_dy = -axis_dy
    else:
        # 近中心对称：该方法无法可靠判定头尾。
        # 为防止每帧随机翻转，固定选择图像坐标中的同一半轴。
        if axis_dx < 0 or (abs(axis_dx) < 1e-6 and axis_dy < 0):
            axis_dx = -axis_dx
            axis_dy = -axis_dy

    # 将选定的像素方向映射到物理毫米坐标。
    mx0, my0 = px_to_mm(m, shape_cx, shape_cy)
    mx1, my1 = px_to_mm(
        m,
        shape_cx + axis_dx * 20.0,
        shape_cy + axis_dy * 20.0
    )

    angle = math.degrees(math.atan2(my1 - my0, mx1 - mx0)) % 360.0

    # 轮廓异常时保持原有矩形长宽比回退。
    if elong <= 0:
        elong = float(w) / h if h > 0 else 1.0
        if elong < 1.0:
            elong = 1.0 / elong

    return angle, elong

def piece_pose_and_signature(
    img,
    rect,
    blist,
    m,
    contour_thresholds=None
):
    x, y, w, h = rect
    main_blob = max(blist, key=lambda b: b.pixels()) if blist else None
    contour = get_main_contour(
        img,
        rect,
        blist,
        contour_thresholds
    )

    # 所有位置坐标统一使用二值前景真实质心。
    cx, cy = binary_shape_centroid(blist, rect)
    mx, my = px_to_mm(m, cx, cy)

    # 有方向绝对角：二阶矩主轴 + 质心相对真实最小外接圆圆心偏移。
    angle, elong = rough_contour_angle(
        contour, main_blob, rect, m, cx, cy
    )

    if USE_LINE_SIGNATURE:
        try:
            segs = img.find_line_segments(
                roi=rect,
                merge_distance=6,
                max_theta_diff=12
            )
        except Exception:
            segs = []
    else:
        segs = []

    long_segs = [s for s in segs if s.length() > LONG_MIN]

    if ANGLE_QUANTIZE:
        a = angle
        if a > 180:
            a -= 360
        angle = round(a / 90.0) * 90.0
        angle %= 360

    scale = local_scale_at(m, cx, cy)
    if scale <= 0:
        scale = 1.6

    area_px = sum(b.pixels() for b in blist) if blist else (w * h)

    if long_segs:
        peri_px = sum(s.length() for s in long_segs)
    else:
        peri_px = 2 * (w + h)

    area_mm2 = area_px * scale * scale
    peri_mm = peri_px * scale
    compact = (
        4.0 * math.pi * area_px / (peri_px * peri_px)
        if peri_px > 0 else 0.0
    )
    n_edges = len(long_segs)

    side_lengths = sorted(
        [s.length() for s in long_segs],
        reverse=True
    )[:4]

    while len(side_lengths) < 4:
        side_lengths.append(0.0)

    radial, radial_quality = build_radial_descriptor(
        contour, m, cx, cy
    )
    radial_source = "OPENMV_CONTOUR"

    # 结构边界仍按原来的位置和方法提取；当固件的contour.points()返回
    # 空点链时，只把它用作同一72格径向描述子的输入，不改变粗角算法。
    structure_boundary = None
    structure_threshold = -1
    try:
        (
            structure_boundary,
            structure_threshold,
            _structure_minimum,
            _structure_maximum,
            _structure_foreground
        ) = _structure_adaptive_boundary(
            img,
            rect,
            (cx, cy)
        )
    except KeyboardInterrupt:
        raise
    except BaseException as e:
        if VERBOSE:
            print("adaptive structure boundary err:", e)

    # 若OpenMV contour.points()为空（本次实机模板和当前四块的radialQ均为0），
    # 用同一条真实物理边界补回原要求1的72格径向描述子。
    if (
        radial_quality < DESCRIPTOR_MIN_QUALITY and
        structure_boundary and
        len(structure_boundary) >= 6
    ):
        fallback_radial, fallback_radial_quality = (
            build_radial_descriptor_from_points(
                structure_boundary,
                m,
                cx,
                cy
            )
        )

        if fallback_radial_quality > radial_quality:
            radial = fallback_radial
            radial_quality = fallback_radial_quality
            radial_source = "ADAPTIVE_BOUNDARY"

    if structure_boundary and len(structure_boundary) >= 6:
        (
            structure_valid,
            structure_vertices,
            structure_edges,
            _structure_epsilon
        ) = build_polygon_structure_from_points(
            structure_boundary,
            m
        )
    else:
        # 仅在成功版边界提取因固件/内存异常不可用时保留原contour回退。
        (
            structure_valid,
            structure_vertices,
            structure_edges,
            _structure_epsilon
        ) = build_polygon_structure(contour, m)

    if REQ1_DIAGNOSTICS:
        print(
            "REQ1DBG structure center=(%.1f,%.1f) boundaryPoints=%d "
            "otsu=%d epsilon=%.1f valid=%s vertices=%d "
            "radial=%s q=%.3f" %
            (
                cx,
                cy,
                len(structure_boundary) if structure_boundary else 0,
                structure_threshold,
                _structure_epsilon,
                "YES" if structure_valid > 0.5 else "NO",
                structure_vertices,
                radial_source,
                radial_quality
            )
        )
    del structure_boundary

    base_sig = (
        round(area_mm2, 1),
        round(peri_mm, 1),
        round(compact, 5),
        round(elong, 3),
        round(n_edges, 1),
        round(side_lengths[0] * scale, 1),
        round(side_lengths[1] * scale, 1),
        round(side_lengths[2] * scale, 1),
        round(radial_quality, 4)
    )

    structure_sig = (
        round(structure_valid, 1),
        round(structure_vertices, 1)
    ) + tuple(round(v, 5) for v in structure_edges)

    sig = (
        base_sig +
        tuple(round(v, 5) for v in radial) +
        structure_sig
    )

    return (
        round(mx, 1),
        round(my, 1),
        round(angle % 360, 1),
        sig
    )

def diagnostic_piece_metrics(img, rect, blist, m):
    """计算只用于日志的真实轮廓指标，不参与检测、匹配或角度输出。"""
    x, y, w, h = rect
    area_px = sum(b.pixels() for b in blist) if blist else 0
    cx, cy = binary_shape_centroid(blist, rect)
    scale = local_scale_at(m, cx, cy)
    contour = get_main_contour(img, rect, blist)

    points = []
    if contour is not None:
        try:
            points = contour.points() or []
        except Exception:
            points = []

    perimeter_px = 0.0
    if len(points) >= 2:
        for i in range(len(points)):
            x1, y1 = points[i]
            x2, y2 = points[(i + 1) % len(points)]
            dx = x2 - x1
            dy = y2 - y1
            perimeter_px += math.sqrt(dx * dx + dy * dy)

    compactness = 0.0
    if perimeter_px > 1e-6:
        compactness = 4.0 * math.pi * area_px / (
            perimeter_px * perimeter_px
        )

    min_corner_count = 0
    if contour is not None:
        try:
            min_corner_count = len(contour.min_corners())
        except Exception:
            min_corner_count = 0

    aspect = 0.0
    if min(w, h) > 0:
        aspect = float(max(w, h)) / min(w, h)

    return {
        "cx": cx,
        "cy": cy,
        "area_px": area_px,
        "perimeter_mm": perimeter_px * scale,
        "aspect": aspect,
        "compactness": compactness,
        "contour_points": len(points),
        "min_corners": min_corner_count
    }

# ================= Part3：标准库 / 匹配 / 串口 =================
def save_demo(slots, shapes):
    try:
        with open("/flash/slots.txt", "w") as f:
            for (x, y, a) in slots:
                f.write("%.2f %.2f %.2f\n" % (x, y, a))
        with open("/flash/shapes.txt", "w") as f:
            for sig in shapes:
                f.write(" ".join("%.6g" % v for v in sig) + "\n")
        print("saved slots & shapes OK")
        print_demo_summary(slots, shapes)
    except Exception as e:
        print("save demo failed:", e)

def load_demo():
    try:
        with open("/flash/slots.txt", "r") as f:
            slots = []
            for line in f:
                line = line.strip()
                if not line:
                    continue
                xs = line.split()
                if len(xs) >= 3:
                    slots.append((float(xs[0]), float(xs[1]), float(xs[2])))
        with open("/flash/shapes.txt", "r") as f:
            shapes = []
            for line in f:
                line = line.strip()
                if not line:
                    continue
                vs = [float(v) for v in line.split()]
                if vs:
                    shapes.append(tuple(vs))
        if not slots or not shapes:
            return None
        return slots, shapes
    except Exception:
        return None

def print_demo_summary(slots, shapes):
    print("========== 已录制标准库 ==========")
    for i, (s, sig) in enumerate(zip(slots, shapes)):
        if len(sig) >= 8:
            # 新版形状签名在前8项基础特征后还附带完整径向轮廓描述子，
            # 因此这里只读取前8项，不能直接解包整个sig。
            area_mm2, peri_mm, compact, elong, n_edges, sd0, sd1, sd2 = sig[:8]
        else:
            area_mm2, peri_mm, compact, elong, n_edges = (list(sig) + [0]*5)[:5]
            sd0 = 0
        print("落点[%d] 位置(%.1f,%.1f) 角度%.1f°" % (i, s[0], s[1], s[2]))
        print(
            "        形状: 面积=%.0fmm^2 周长=%.0fmm 长宽比=%.2f "
            "边数≈%.0f 最长边=%.0fmm 径向Q=%.2f" %
            (
                area_mm2,
                peri_mm,
                elong,
                n_edges,
                sd0,
                sig[8] if len(sig) > 8 else 0.0
            )
        )
        if _has_structure(sig):
            print(
                "        结构: 顶点=%d 归一化边=%s" %
                (
                    int(round(sig[STRUCT_VERTEX_INDEX])),
                    [
                        round(sig[STRUCT_EDGE_OFFSET + k], 3)
                        for k in range(STRUCT_EDGE_COUNT)
                    ]
                )
            )
    print("==================================")

def print_template_diagnostics(slots, shapes):
    """输出当前/flash标准库区分度；只在REQ1_DIAGNOSTICS开启时调用。"""
    if not REQ1_DIAGNOSTICS:
        return

    print("========== REQ1 TEMPLATE DIAGNOSTICS ==========")
    for i, (slot, sig) in enumerate(zip(slots, shapes)):
        print(
            "REQ1DBG template slot=%d place=(%.1f,%.1f) targetAngle=%.1f "
            "area=%.1f perimeterFeature=%.1f compactFeature=%.5f "
            "elong=%.3f lineEdges=%.0f radialQ=%.3f "
            "vertices=%d structureOK=%s symmetry180=%.3f len=%d" %
            (
                i, slot[0], slot[1], slot[2],
                sig[0] if len(sig) > 0 else 0.0,
                sig[1] if len(sig) > 1 else 0.0,
                sig[2] if len(sig) > 2 else 0.0,
                sig[3] if len(sig) > 3 else 0.0,
                sig[4] if len(sig) > 4 else 0.0,
                sig[8] if len(sig) > 8 else 0.0,
                int(round(sig[STRUCT_VERTEX_INDEX]))
                if _has_structure(sig) else 0,
                "YES" if _has_structure(sig) else "NO",
                descriptor_180_symmetry_ratio(sig),
                len(sig)
            )
        )

    for i in range(len(shapes)):
        for j in range(i + 1, len(shapes)):
            area_err = area_relative_error(shapes[i][0], shapes[j][0])
            shape_score, _, shape_ok = descriptor_best_rotation(
                shapes[i], shapes[j]
            )
            print(
                "REQ1DBG templatePair %d-%d areaSeparation=%.3f "
                "contourDistance=%.4f contourOK=%s structureDistance=%.4f "
                "structureOK=%s" %
                (
                    i, j, area_err, shape_score,
                    "YES" if shape_ok else "NO",
                    structure_distance(shapes[i], shapes[j])[0],
                    "YES" if structure_distance(shapes[i], shapes[j])[1]
                    else "NO"
                )
            )
    print("================================================")

def norm_delta(d):
    """有方向角归一化到[-180,180]。"""
    d %= 360
    if d > 180:
        d -= 360
    return int(round(d))

def angle_diff(a, b):
    """两个有方向角之间的最小差值，周期360°。"""
    d = abs(a - b) % 360
    return 360 - d if d > 180 else d

def _has_radial_data(sig):
    return (
        sig is not None and
        len(sig) >= RADIAL_OFFSET + RADIAL_BINS
    )

def _has_radial(sig):
    return (
        _has_radial_data(sig) and
        sig[8] >= DESCRIPTOR_MIN_QUALITY
    )


def _has_structure(sig):
    return (
        sig is not None and
        len(sig) >= STRUCT_SIGNATURE_LENGTH and
        sig[STRUCT_VALID_INDEX] >= 0.5 and
        3 <= int(round(sig[STRUCT_VERTEX_INDEX])) <= STRUCT_MAX_VERTICES
    )


def structure_distance(current_sig, target_sig):
    """比较通用顶点数量和循环规范化边长比例。"""
    if not _has_structure(current_sig) or not _has_structure(target_sig):
        return 1e9, False

    current_vertices = int(round(current_sig[STRUCT_VERTEX_INDEX]))
    target_vertices = int(round(target_sig[STRUCT_VERTEX_INDEX]))
    vertex_error = abs(current_vertices - target_vertices)

    edge_error = 0.0
    for index in range(STRUCT_EDGE_COUNT):
        edge_error += abs(
            current_sig[STRUCT_EDGE_OFFSET + index] -
            target_sig[STRUCT_EDGE_OFFSET + index]
        )

    return (
        STRUCT_VERTEX_WEIGHT * vertex_error +
        STRUCT_EDGE_PROFILE_WEIGHT * edge_error,
        True
    )


def descriptor_180_symmetry_ratio(sig):
    """模板180°自相似误差相对于自身径向起伏的比例。"""
    if not _has_radial(sig):
        return 1e9

    radial = sig[RADIAL_OFFSET:RADIAL_OFFSET + RADIAL_BINS]
    mean_value = sum(radial) / RADIAL_BINS
    energy = sum(
        (value - mean_value) * (value - mean_value)
        for value in radial
    ) / RADIAL_BINS
    error_180 = descriptor_error_at_rotation(sig, sig, 180.0)

    if error_180 is None:
        return 1e9
    if energy <= 1e-9:
        return 0.0 if error_180 <= 1e-9 else 1e9
    return error_180 / energy

def area_relative_error(area_a, area_b):
    """
    两个面积的对称相对误差，范围0~1：
      0   表示完全一致；
      越大表示面积身份越不相符。
    """
    a = max(abs(float(area_a)), 1.0)
    b = max(abs(float(area_b)), 1.0)
    return abs(a - b) / max(a, b)

def descriptor_best_rotation(current_sig, target_sig):
    """
    比较当前碎片和目标模板的完整二值轮廓。

    对每个候选旋转角，循环移位当前极坐标轮廓，并与目标轮廓比较。
    返回：
      score  : 越小越匹配
      d_angle: 当前碎片应旋转到目标姿态的物理角度[-180,180]
      ok     : 描述子是否有效
    """
    if not _has_radial(current_sig) or not _has_radial(target_sig):
        return 1e9, 0, False

    cur = current_sig[RADIAL_OFFSET:RADIAL_OFFSET + RADIAL_BINS]
    tgt = target_sig[RADIAL_OFFSET:RADIAL_OFFSET + RADIAL_BINS]

    best_score = 1e18
    best_shift = 0

    for shift in range(RADIAL_BINS):
        err = 0.0

        # 当前轮廓旋转shift后，目标角度格k对应当前原来的k-shift。
        for k in range(RADIAL_BINS):
            diff = tgt[k] - cur[(k - shift) % RADIAL_BINS]
            err += diff * diff

        err /= RADIAL_BINS

        if err < best_score:
            best_score = err
            best_shift = shift

    d_angle = best_shift * 360.0 / RADIAL_BINS
    if d_angle > 180.0:
        d_angle -= 360.0

    # 此处只评价完整轮廓与长宽比；面积身份由match_all中的独立层负责。
    elong_penalty = abs(current_sig[3] - target_sig[3])

    total_score = (
        best_score * 12.0 +
        elong_penalty * 0.25
    )

    return total_score, d_angle, True

def descriptor_error_at_rotation(current_sig, target_sig, d_angle):
    """
    计算“当前轮廓旋转 d_angle 后”与目标轮廓的均方误差。

    与 descriptor_best_rotation() 不同，这里不在0~360°内全局搜索，
    只评价外部给定的角度。这样可专门比较主轴的两个候选分支：
      candidate_a = d
      candidate_b = d + 180°

    使用线性插值读取循环径向描述子，避免5°分箱边界导致误判。
    """
    if not _has_radial(current_sig) or not _has_radial(target_sig):
        return None

    cur = current_sig[RADIAL_OFFSET:RADIAL_OFFSET + RADIAL_BINS]
    tgt = target_sig[RADIAL_OFFSET:RADIAL_OFFSET + RADIAL_BINS]

    shift = float(d_angle) * RADIAL_BINS / 360.0
    err = 0.0

    for k in range(RADIAL_BINS):
        src_pos = k - shift
        base = int(math.floor(src_pos))
        frac = src_pos - base
        i0 = base % RADIAL_BINS
        i1 = (i0 + 1) % RADIAL_BINS
        cur_v = cur[i0] * (1.0 - frac) + cur[i1] * frac
        diff = tgt[k] - cur_v
        err += diff * diff

    return err / RADIAL_BINS


def descriptor_global_rotation_evidence(current_sig, target_sig):
    """
    评价72格径向轮廓的完整360°旋转峰。

    返回：
      angle       : 当前碎片旋到模板的细化角度；
      best_error  : 最优轮廓均方误差；
      uniqueness  : 排除最优峰附近后，相对第二盆地的区分度；
      fit_ratio   : 最优误差相对于两轮廓径向起伏的比例；
      self_ratio  : 当前/模板自身非零旋转的最小误差比例；
      ok          : 描述子是否可评价。

    此函数只在物理主轴与全局径向角相差较大时调用，不进入正常帧热路径。
    """
    if not _has_radial(current_sig) or not _has_radial(target_sig):
        return 0.0, 1e9, 0.0, 1e9, 0.0, False

    cur = current_sig[RADIAL_OFFSET:RADIAL_OFFSET + RADIAL_BINS]
    tgt = target_sig[RADIAL_OFFSET:RADIAL_OFFSET + RADIAL_BINS]
    errors = [0.0] * RADIAL_BINS
    best_shift = 0
    best_error = 1e18

    for shift in range(RADIAL_BINS):
        err = 0.0
        for k in range(RADIAL_BINS):
            diff = tgt[k] - cur[(k - shift) % RADIAL_BINS]
            err += diff * diff
        err /= RADIAL_BINS
        errors[shift] = err
        if err < best_error:
            best_error = err
            best_shift = shift

    second_error = 1e18
    for shift in range(RADIAL_BINS):
        circular_bins = abs(shift - best_shift)
        circular_bins = min(circular_bins, RADIAL_BINS - circular_bins)
        if circular_bins <= RADIAL_POSE_EXCLUDE_BINS:
            continue
        if errors[shift] < second_error:
            second_error = errors[shift]

    if second_error >= 1e17:
        uniqueness = 0.0
    else:
        uniqueness = (
            (second_error - best_error) /
            max(second_error, 1e-9)
        )

    cur_mean = sum(cur) / RADIAL_BINS
    tgt_mean = sum(tgt) / RADIAL_BINS
    cur_energy = sum(
        (value - cur_mean) * (value - cur_mean)
        for value in cur
    ) / RADIAL_BINS
    tgt_energy = sum(
        (value - tgt_mean) * (value - tgt_mean)
        for value in tgt
    ) / RADIAL_BINS
    mean_energy = (cur_energy + tgt_energy) * 0.5
    fit_ratio = best_error / max(mean_energy, 1e-9)

    self_ratio = 1e9
    for radial, energy in ((cur, cur_energy), (tgt, tgt_energy)):
        self_error = 1e18
        for shift in range(
            RADIAL_POSE_EXCLUDE_BINS + 1,
            RADIAL_BINS - RADIAL_POSE_EXCLUDE_BINS
        ):
            err = 0.0
            for k in range(RADIAL_BINS):
                diff = radial[k] - radial[(k - shift) % RADIAL_BINS]
                err += diff * diff
            err /= RADIAL_BINS
            if err < self_error:
                self_error = err
        ratio = self_error / max(energy, 1e-9)
        if ratio < self_ratio:
            self_ratio = ratio

    # 用最优格左右各一格的误差作亚格细化；最多移动半格，避免跨峰。
    prev_error = errors[(best_shift - 1) % RADIAL_BINS]
    next_error = errors[(best_shift + 1) % RADIAL_BINS]
    denominator = prev_error - 2.0 * best_error + next_error
    fraction = 0.0
    if denominator > 1e-12:
        fraction = 0.5 * (prev_error - next_error) / denominator
        fraction = max(-0.5, min(0.5, fraction))

    angle = (
        best_shift + fraction
    ) * 360.0 / RADIAL_BINS
    angle %= 360.0
    if angle > 180.0:
        angle -= 360.0

    refined_error = descriptor_error_at_rotation(
        current_sig,
        target_sig,
        angle
    )
    if refined_error is not None and refined_error <= best_error:
        best_error = refined_error
    else:
        angle = best_shift * 360.0 / RADIAL_BINS
        if angle > 180.0:
            angle -= 360.0

    return (
        angle,
        best_error,
        uniqueness,
        fit_ratio,
        self_ratio,
        True
    )


def choose_axis_branch_by_contour(current_sig, target_sig, visual_angle):
    """
    在 visual_angle 与 visual_angle+180°之间做通用选择。

    返回：
      chosen_angle : 选择后的相对旋转角
      confident    : 完整轮廓是否足以区分两个180°分支
      contrast     : 两个候选误差的相对差异，越大越可信
    """
    candidate_a = float(visual_angle)
    candidate_b = (candidate_a + 360.0) % 360.0 - 180.0

    err_a = descriptor_error_at_rotation(
        current_sig, target_sig, candidate_a
    )
    err_b = descriptor_error_at_rotation(
        current_sig, target_sig, candidate_b
    )

    if err_a is None or err_b is None:
        return candidate_a, False, 0.0

    worse = max(err_a, err_b, 1e-9)
    contrast = abs(err_a - err_b) / worse

    # 轮廓接近180°对称时，两个误差非常接近。此时不允许描述子
    # 强行改写主轴结果，避免上一版出现“两个正确、两个被翻反”。
    if contrast < BRANCH_MIN_CONTRAST:
        return candidate_a, False, contrast

    if err_b < err_a:
        return candidate_b, True, contrast

    return candidate_a, True, contrast

def fallback_shape_distance(a, b):
    """旧标准库或轮廓描述子失效时的基础形状距离。"""
    d = 0.0
    d += abs(a[0] - b[0]) / max(abs(b[0]), 100.0)
    d += abs(a[1] - b[1]) / 600.0
    d += abs(a[2] - b[2]) * 3.0
    d += abs(a[3] - b[3]) * 8.0
    d += abs(a[4] - b[4]) * 0.5

    for i in range(3):
        d += abs(a[5+i] - b[5+i]) / 200.0

    return d

def push_angle(hist, slot_i, ang):
    h = hist.setdefault(slot_i, [])
    h.append(ang)

    if len(h) > ANG_WIN:
        h.pop(0)

def stable_angle(hist, slot_i):
    """
    对完整轮廓得到的有方向旋转角进行360°周期聚类，
    防止+179°/-179°在边界处被错误判为相差358°。
    """
    h = hist.get(slot_i, [])

    if len(h) < ANG_MIN_VOTES:
        return None

    best_cluster = []

    for cand in h:
        cluster = [a for a in h if angle_diff(a, cand) <= ANG_TOL]

        if len(cluster) > len(best_cluster):
            best_cluster = cluster

    if len(best_cluster) < ANG_MIN_VOTES:
        return None

    ref = best_cluster[0]
    aligned = []

    for a in best_cluster:
        d = (a - ref) % 360

        if d > 180:
            d -= 360

        aligned.append(ref + d)

    aligned.sort()
    return norm_delta(aligned[len(aligned) // 2])

def push_identity_observation(hist, slot_i, px, py, structure_count):
    """记录某slot的取料位置和离散结构，返回是否发生身份跳变。"""
    h = hist.setdefault(slot_i, [])
    jumped = False

    if h:
        dx = px - h[-1][0]
        dy = py - h[-1][1]
        if (
            math.sqrt(dx * dx + dy * dy) > IDENTITY_POS_TOL_MM or
            structure_count != h[-1][2]
        ):
            h = []
            hist[slot_i] = h
            jumped = True

    h.append((px, py, structure_count))
    if len(h) > IDENTITY_WIN:
        h.pop(0)

    return jumped

def stable_identity_votes(hist, slot_i, px, py, structure_count):
    """统计同一物理位置且顶点结构一致的身份票数。"""
    h = hist.get(slot_i, [])
    votes = 0

    for hx, hy, h_structure_count in h:
        dx = px - hx
        dy = py - hy
        if (
            h_structure_count == structure_count and
            math.sqrt(dx * dx + dy * dy) <= IDENTITY_POS_TOL_MM
        ):
            votes += 1

    return votes

def send_to_mcu(uart_obj, results):
    if uart_obj is None:
        return False

    parts = []

    for (slot_i, px, py, sx, sy, ang) in results:
        parts.append(
            "%d,%.1f,%.1f,%.1f,%.1f,%d" %
            (slot_i, px, py, sx, sy, ang)
        )

    frame = "$" + ";".join(parts) + "#\n"

    try:
        uart_obj.write(frame)
        print("UART发送:", frame.strip())
        return True
    except Exception as e:
        print("WARN: uart write failed:", e)
        return False

def _assign_optimal(dmat):
    best = [None, 1e18]

    def rec(k, used, chosen, cost):
        if k == 4:
            if cost < best[1]:
                best[1] = cost
                best[0] = chosen[:]
            return

        for j in range(4):
            if j in used:
                continue

            used.add(j)
            chosen.append(j)
            rec(k + 1, used, chosen, cost + dmat[k][j])
            chosen.pop()
            used.discard(j)

    rec(0, set(), [], 0)
    return best[0]

def _assign_best_two(dmat):
    """返回4对4分配中的最优、次优方案及其总代价。"""
    best_assign = None
    best_cost = 1e18
    second_assign = None
    second_cost = 1e18

    def rec(k, used, chosen, cost):
        nonlocal best_assign, best_cost, second_assign, second_cost

        if k == 4:
            candidate = chosen[:]

            if cost < best_cost:
                second_assign = best_assign
                second_cost = best_cost
                best_assign = candidate
                best_cost = cost
            elif candidate != best_assign and cost < second_cost:
                second_assign = candidate
                second_cost = cost

            return

        for j in range(4):
            if j in used:
                continue

            used.add(j)
            chosen.append(j)
            rec(k + 1, used, chosen, cost + dmat[k][j])
            chosen.pop()
            used.discard(j)

    rec(0, set(), [], 0.0)
    return best_assign, best_cost, second_assign, second_cost

def match_all(picks, templates, slots):
    """
    三层身份匹配：

    第一层：比较当前碎片面积与四个标准模板面积，得到全局4对4面积分配。
    第二层：完整径向轮廓参与面积接近碎片的消歧。
    第三层：已验证的多边形顶点数量和边长比例进一步区分结构。

    当面积最优分配明显优于第二名时，直接采用面积分配，防止轮廓噪声把
    两块碎片身份交换；当面积本身接近时，再由面积+完整轮廓联合决定。
    """
    global last_area_match_info

    last_area_match_info = {
        "mode": "INVALID",
        "gap": 0.0,
        "combined_gap": 0.0,
        "structure_gap": 0.0,
        "avg_error": 1.0,
        "pair_errors": []
    }

    if len(templates) != 4 or len(picks) != 4 or len(slots) != 4:
        return []

    # 多边形结构是B方案的增强证据，不是要求1成功版标准库的必需字段。
    # 只有当前4块和4个模板都具备有效结构时才整组启用；否则整组回退到
    # 成功版的“面积+径向轮廓”身份路径，不能把缺失结构当作零代价参与比较。
    structure_enabled = (
        all(_has_structure(pick[3]) for pick in picks) and
        all(_has_structure(template) for template in templates)
    )

    if (
        not all(_has_radial(pick[3]) for pick in picks) or
        not all(_has_radial(template) for template in templates)
    ):
        last_area_match_info["mode"] = "RADIAL_INVALID"
        return []

    shape_mat = [[0.0] * 4 for _ in range(4)]
    area_mat = [[0.0] * 4 for _ in range(4)]
    structure_mat = [[0.0] * 4 for _ in range(4)]
    combined_mat = [[0.0] * 4 for _ in range(4)]
    amat = [[0.0] * 4 for _ in range(4)]
    radial_angle_mat = [[0.0] * 4 for _ in range(4)]
    valid_mat = [[False] * 4 for _ in range(4)]
    structure_valid_mat = [[False] * 4 for _ in range(4)]

    for i in range(4):
        current_sig = picks[i][3]
        current_area = current_sig[0]

        for j in range(4):
            target_sig = templates[j]
            target_area = target_sig[0]

            # 完整轮廓仍只负责身份匹配和消歧；
            # 旋转角统一由“目标有方向角 - 当前有方向角”得到。
            shape_score, radial_angle, ok = descriptor_best_rotation(
                current_sig,
                target_sig
            )

            if not ok:
                shape_score = fallback_shape_distance(
                    current_sig,
                    target_sig
                )

            # 目标摆放角 sa 与当前碎片角 pa 都是0~360°有方向角。
            # 这里得到真正需要旋转的相对角，归一化到[-180,180]。
            d_angle = (
                slots[j][2] -
                picks[i][2] +
                180.0
            ) % 360.0 - 180.0

            area_error = area_relative_error(
                current_area,
                target_area
            )

            if structure_enabled:
                structure_score, structure_ok = structure_distance(
                    current_sig,
                    target_sig
                )
            else:
                structure_score = 0.0
                structure_ok = False

            # 超出单对面积容差的候选加入巨大代价，基本禁止该身份组合。
            hard_penalty = (
                1000.0
                if area_error > MATCH_AREA_MAX_PAIR_ERROR
                else 0.0
            )

            shape_mat[i][j] = shape_score
            area_mat[i][j] = area_error
            structure_mat[i][j] = structure_score
            combined_mat[i][j] = (
                shape_score +
                MATCH_AREA_WEIGHT * area_error +
                structure_score +
                hard_penalty
            )
            amat[i][j] = d_angle
            radial_angle_mat[i][j] = radial_angle
            valid_mat[i][j] = ok
            structure_valid_mat[i][j] = structure_ok

    (
        area_assign,
        area_best_cost,
        area_second_assign,
        area_second_cost
    ) = _assign_best_two(area_mat)

    if structure_enabled:
        (
            structure_assign,
            structure_best_cost,
            structure_second_assign,
            structure_second_cost
        ) = _assign_best_two(structure_mat)
    else:
        structure_assign = None
        structure_best_cost = 0.0
        structure_second_assign = None
        structure_second_cost = 0.0

    (
        combined_assign,
        combined_best_cost,
        combined_second_assign,
        combined_second_cost
    ) = _assign_best_two(combined_mat)

    if (
        area_assign is None or
        (structure_enabled and structure_assign is None) or
        combined_assign is None
    ):
        return []

    if area_second_assign is None:
        area_gap = 1e9
    else:
        area_gap = area_second_cost - area_best_cost

    if combined_second_assign is None:
        combined_gap = 1e9
    else:
        combined_gap = combined_second_cost - combined_best_cost

    if not structure_enabled:
        structure_gap = 0.0
    elif structure_second_assign is None:
        structure_gap = 1e9
    else:
        structure_gap = structure_second_cost - structure_best_cost

    if REQ1_DIAGNOSTICS:
        print(
            "REQ1DBG assignment area best=%s cost=%.4f second=%s "
            "cost=%.4f gap=%.4f" %
            (
                area_assign, area_best_cost,
                area_second_assign, area_second_cost,
                area_gap
            )
        )
        if structure_enabled:
            print(
                "REQ1DBG assignment structure best=%s cost=%.4f second=%s "
                "cost=%.4f gap=%.4f" %
                (
                    structure_assign, structure_best_cost,
                    structure_second_assign, structure_second_cost,
                    structure_gap
                )
            )
        else:
            print(
                "REQ1DBG structure disabled: fallback to successful "
                "AREA+CONTOUR matching"
            )
        print(
            "REQ1DBG assignment combined best=%s cost=%.4f second=%s "
            "cost=%.4f gap=%.4f" %
            (
                combined_assign, combined_best_cost,
                combined_second_assign, combined_second_cost,
                combined_gap
            )
        )

        for i in range(4):
            ranked = sorted(range(4), key=lambda j: combined_mat[i][j])
            best_slot = ranked[0]
            second_slot = ranked[1]
            print(
                "REQ1DBG candidate=%d bestSlot=%d secondSlot=%d rowGap=%.4f" %
                (
                    i,
                    best_slot,
                    second_slot,
                    combined_mat[i][second_slot] - combined_mat[i][best_slot]
                )
            )
            for j in range(4):
                print(
                    "REQ1DBG cost candidate=%d slot=%d area=%.4f "
                    "contour=%.4f structure=%.4f combined=%.4f "
                    "radialOK=%s structureOK=%s" %
                    (
                        i, j,
                        area_mat[i][j],
                        shape_mat[i][j],
                        structure_mat[i][j],
                        combined_mat[i][j],
                        "YES" if valid_mat[i][j] else "NO",
                        "YES" if structure_valid_mat[i][j] else "NO"
                    )
                )

    area_pair_errors = [
        area_mat[i][area_assign[i]]
        for i in range(4)
    ]

    area_assignment_valid = (
        max(area_pair_errors) <= MATCH_AREA_MAX_PAIR_ERROR and
        sum(area_pair_errors) / 4.0 <= MATCH_AREA_MAX_AVG_ERROR
    )

    # 面积映射足够明确时准备锁定面积身份；否则使用面积+轮廓联合结果。
    area_lock_requested = (
        area_assignment_valid and
        area_gap >= MATCH_AREA_CONFIDENCE_GAP
    )

    # 最优和第二优全局身份过近时不猜测；等待下一帧形成稳定、清晰的证据。
    if (
        structure_enabled and
        combined_gap < MATCH_COMBINED_CONFIDENCE_GAP
    ):
        last_area_match_info = {
            "mode": "LOW_CONFIDENCE",
            "gap": area_gap,
            "combined_gap": combined_gap,
            "structure_gap": structure_gap,
            "avg_error": sum(area_pair_errors) / 4.0,
            "pair_errors": area_pair_errors
        }
        return []

    # 当前四个面积候选都具备有效完整轮廓时，两层证据必须对全局身份达成一致。
    # 冲突通常来自阈值断裂/反光造成的面积次序短暂反转；本帧等待重采，
    # 不让面积层静默覆盖已经存在的轮廓反证。
    contour_evidence_complete = all(
        valid_mat[i][area_assign[i]] and
        valid_mat[i][combined_assign[i]]
        for i in range(4)
    )

    if (
        area_lock_requested and
        MATCH_REQUIRE_EVIDENCE_AGREEMENT and
        contour_evidence_complete and
        area_assign != combined_assign
    ):
        last_area_match_info = {
            "mode": "EVIDENCE_CONFLICT",
            "gap": area_gap,
            "combined_gap": combined_gap,
            "structure_gap": structure_gap,
            "avg_error": sum(area_pair_errors) / 4.0,
            "pair_errors": area_pair_errors
        }
        return []

    if area_lock_requested:
        assign = area_assign
        match_mode = "AREA_LOCK"
    else:
        assign = combined_assign
        match_mode = (
            "AREA+CONTOUR+STRUCTURE"
            if structure_enabled else
            "AREA+CONTOUR"
        )

    pair_errors = [
        area_mat[i][assign[i]]
        for i in range(4)
    ]
    avg_area_error = sum(pair_errors) / 4.0
    max_area_error = max(pair_errors)

    # 最终发送前的硬保护：面积层不通过，整组结果作废，不允许发给STM32。
    if (
        max_area_error > MATCH_AREA_MAX_PAIR_ERROR or
        avg_area_error > MATCH_AREA_MAX_AVG_ERROR
    ):
        last_area_match_info = {
            "mode": "AREA_REJECT",
            "gap": area_gap,
            "combined_gap": combined_gap,
            "structure_gap": structure_gap,
            "avg_error": avg_area_error,
            "pair_errors": pair_errors
        }
        return []

    last_area_match_info = {
        "mode": match_mode,
        "gap": area_gap,
        "combined_gap": combined_gap,
        "structure_gap": structure_gap,
        "avg_error": avg_area_error,
        "pair_errors": pair_errors,
        "radial_pose_slots": []
    }

    matched = []

    for i, j in enumerate(assign):
        px, py, pa, sig = picks[i]
        sx, sy, sa = slots[j]

        visual_angle = amat[i][j]

        if REQ1_DIAGNOSTICS:
            print(
                "REQ1DBG pose candidate=%d slot=%d currentAngle=%.1f "
                "targetAngle=%.1f rawRelative=%.1f" %
                (i, j, pa, sa, visual_angle)
            )

        # 正常路径仍只比较当前主轴角和它的180°对向分支。
        # 若像素主轴经透视映射后落入错误轴族，才允许唯一且强拟合的
        # 360°毫米径向峰作通用回退；不按碎片、面积或slot写死。
        if GENERIC_BRANCH_SCORE_ENABLE and valid_mat[i][j]:
            candidate_a = float(amat[i][j])
            candidate_b = (
                (candidate_a + 360.0) % 360.0 - 180.0
            )
            chosen_angle, branch_confident, branch_contrast = (
                choose_axis_branch_by_contour(
                    picks[i][3],
                    templates[j],
                    visual_angle
                )
            )

            symmetry_ratio = descriptor_180_symmetry_ratio(templates[j])
            branch_equivalent = (
                symmetry_ratio <= BRANCH_SYMMETRY_RATIO_MAX
            )

            axis_angle = (
                chosen_angle if branch_confident else candidate_a
            )
            visual_angle = axis_angle
            radial_pose_used = False
            radial_pose_angle = radial_angle_mat[i][j]
            radial_pose_gap = angle_diff(axis_angle, radial_pose_angle)
            radial_pose_improvement = 0.0
            radial_pose_uniqueness = 0.0
            radial_pose_fit = 1e9
            radial_pose_self = 0.0

            # 先用零分配代价的条件短路；只有发生明显轴族冲突时，才计算
            # 全局峰的唯一性和自相关，避免影响正常帧速度。
            if (
                not branch_equivalent and
                picks[i][3][8] >= RADIAL_POSE_MIN_QUALITY and
                templates[j][8] >= RADIAL_POSE_MIN_QUALITY and
                radial_pose_gap >= RADIAL_POSE_AXIS_GAP_DEG
            ):
                (
                    radial_pose_angle,
                    radial_pose_error,
                    radial_pose_uniqueness,
                    radial_pose_fit,
                    radial_pose_self,
                    radial_pose_ok
                ) = descriptor_global_rotation_evidence(
                    picks[i][3],
                    templates[j]
                )
                axis_pose_error = descriptor_error_at_rotation(
                    picks[i][3],
                    templates[j],
                    axis_angle
                )
                radial_pose_gap = angle_diff(
                    axis_angle,
                    radial_pose_angle
                )

                if (
                    radial_pose_ok and
                    axis_pose_error is not None and
                    radial_pose_error is not None
                ):
                    radial_pose_improvement = (
                        (axis_pose_error - radial_pose_error) /
                        max(axis_pose_error, radial_pose_error, 1e-9)
                    )
                    radial_pose_used = (
                        radial_pose_gap >= RADIAL_POSE_AXIS_GAP_DEG and
                        radial_pose_error < axis_pose_error and
                        radial_pose_improvement >=
                        RADIAL_POSE_MIN_IMPROVEMENT and
                        radial_pose_uniqueness >=
                        RADIAL_POSE_MIN_UNIQUENESS and
                        radial_pose_fit <= RADIAL_POSE_MAX_FIT_RATIO and
                        radial_pose_self >= RADIAL_POSE_MIN_SELF_RATIO
                    )

            if radial_pose_used:
                visual_angle = radial_pose_angle
                last_area_match_info["radial_pose_slots"].append(j)

            # 对0/180°这一最危险分支，非对称模板必须有明确轮廓证据。
            # 完整径向峰也不可信时才整帧重采；可信全局峰可同时解决
            # 180°头尾和非180°错误轴族，不让坏主轴进入稳定投票。
            if (
                not branch_confident and
                not radial_pose_used and
                not branch_equivalent and
                max(abs(candidate_a), abs(candidate_b)) >=
                NEAR_180_BRANCH_GUARD_DEG
            ):
                last_area_match_info["mode"] = "BRANCH_AMBIGUOUS"
                last_area_match_info["branch_slot"] = j
                last_area_match_info["branch_contrast"] = branch_contrast
                last_area_match_info["symmetry_ratio"] = symmetry_ratio
                return []

            if REQ1_DIAGNOSTICS:
                err_a = descriptor_error_at_rotation(
                    picks[i][3], templates[j], candidate_a
                )
                err_b = descriptor_error_at_rotation(
                    picks[i][3], templates[j], candidate_b
                )
                print(
                    "REQ1DBG branch candidate=%d slot=%d a=%.1f errA=%s "
                    "b=%.1f errB=%s contrast=%.3f confident=%s "
                    "symmetryRatio=%.3f equivalent=%s chosen=%.1f" %
                    (
                        i, j,
                        candidate_a,
                        "%.6f" % err_a if err_a is not None else "NA",
                        candidate_b,
                        "%.6f" % err_b if err_b is not None else "NA",
                        branch_contrast,
                        "YES" if branch_confident else "NO",
                        symmetry_ratio,
                        "YES" if branch_equivalent else "NO",
                        visual_angle
                    )
                )
                print(
                    "REQ1DBG poseFusion slot=%d axis=%.1f radial=%.1f "
                    "gap=%.1f improvement=%.3f uniqueness=%.3f "
                    "fit=%.3f self=%.3f used=%s" %
                    (
                        j,
                        axis_angle,
                        radial_pose_angle,
                        radial_pose_gap,
                        radial_pose_improvement,
                        radial_pose_uniqueness,
                        radial_pose_fit,
                        radial_pose_self,
                        "YES" if radial_pose_used else "NO"
                    )
                )

            if VERBOSE:
                print(
                    "branch slot=%d contrast=%.2f confident=%s angle=%.1f" %
                    (
                        j,
                        branch_contrast,
                        "YES" if branch_confident else "NO",
                        visual_angle
                    )
                )

        d_ang = (
            ANG_SIGN * visual_angle +
            ANG_OFFSET +
            SLOT_ANGLE_OFFSET[j]
        )
        d_ang = norm_delta(d_ang)
        structure_count = (
            int(round(sig[STRUCT_VERTEX_INDEX]))
            if structure_enabled and _has_structure(sig) else
            0
        )

        matched.append((
            px, py,
            sx, sy,
            d_ang,
            j,
            combined_mat[i][j],
            valid_mat[i][j],
            structure_count
        ))

    return matched


# ================= 四块同时录制辅助 =================
def _initial_record_order(poses):
    """
    第一组完整识别确定slot顺序。
    slot只是目标模板编号，不影响形状匹配正确性；按物理X排序便于复现。
    """
    if RECORD_ORDER_BY_X:
        return sorted(poses, key=lambda p: (p[0], -p[1]))
    return sorted(poses, key=lambda p: (-p[1], p[0]))

def _align_record_poses(poses, reference):
    """
    将当前4块与上一组slot对齐。

    位置距离负责保持目标落点编号不变，面积误差用于防止检测顺序变化、
    碎片断裂或局部反光时把不同对象串到同一slot历史中。

    返回：aligned, max_distance, max_area_error
    """
    if len(poses) != 4 or len(reference) != 4:
        return None, 1e9, 1e9

    dmat = [[0.0] * 4 for _ in range(4)]
    pos_mat = [[0.0] * 4 for _ in range(4)]
    area_mat = [[0.0] * 4 for _ in range(4)]

    for i in range(4):
        for j in range(4):
            dx = poses[i][0] - reference[j][0]
            dy = poses[i][1] - reference[j][1]
            pos_distance = math.sqrt(dx * dx + dy * dy)

            area_error = area_relative_error(
                poses[i][3][0],
                reference[j][3][0]
            )

            pos_mat[i][j] = pos_distance
            area_mat[i][j] = area_error
            dmat[i][j] = (
                pos_distance +
                area_error * RECORD_AREA_ALIGN_WEIGHT_MM
            )

    assign = _assign_optimal(dmat)
    if assign is None:
        return None, 1e9, 1e9

    aligned = [None] * 4
    max_distance = 0.0
    max_area_error = 0.0

    for i, slot_i in enumerate(assign):
        aligned[slot_i] = poses[i]

        if pos_mat[i][slot_i] > max_distance:
            max_distance = pos_mat[i][slot_i]

        if area_mat[i][slot_i] > max_area_error:
            max_area_error = area_mat[i][slot_i]

    if any(p is None for p in aligned):
        return None, 1e9, 1e9

    return aligned, max_distance, max_area_error

def _circular_mean_deg(values):
    if not values:
        return 0.0

    sx = 0.0
    sy = 0.0

    for a in values:
        rad = a * math.pi / 180.0
        sx += math.cos(rad)
        sy += math.sin(rad)

    if abs(sx) < 1e-9 and abs(sy) < 1e-9:
        return values[-1] % 360.0

    return math.degrees(math.atan2(sy, sx)) % 360.0

def _average_signature(signatures):
    if not signatures:
        return None

    length = min(len(sig) for sig in signatures)
    if length <= 0:
        return None

    avg = []

    for k in range(length):
        avg.append(
            sum(sig[k] for sig in signatures) / len(signatures)
        )

    # 平均后的径向描述子重新归一化，保持模板尺度无关。
    if length >= RADIAL_OFFSET + RADIAL_BINS:
        radial = avg[RADIAL_OFFSET:RADIAL_OFFSET + RADIAL_BINS]
        max_r = max(radial)

        if max_r > 1e-6:
            for k in range(RADIAL_BINS):
                avg[RADIAL_OFFSET + k] /= max_r

    return tuple(round(v, 5) for v in avg)

def finalize_multi_record(history):
    """
    对多帧四块识别结果求平均，生成一次性标准库。
    history结构：[frame][slot] = (x, y, rough_angle, signature)
    """
    if len(history) < RECORD_STABLE_FRAMES:
        return None, None

    slots = []
    shapes = []

    # 先统一判断整套标准库是否具备可用的B方案结构证据。
    # 结构只能整库启用，不能让部分slot以“缺失=零代价”混入匹配。
    structure_counts_by_slot = []
    structure_library_enabled = True

    for structure_slot_i in range(4):
        structure_signatures = [
            frame[structure_slot_i][3] for frame in history
        ]
        structure_counts = [
            int(round(sig[STRUCT_VERTEX_INDEX]))
            for sig in structure_signatures
            if _has_structure(sig)
        ]
        structure_counts_by_slot.append(structure_counts)

        if (
            len(structure_counts) != len(structure_signatures) or
            not structure_counts or
            any(
                count != structure_counts[0]
                for count in structure_counts
            )
        ):
            structure_library_enabled = False

    for slot_i in range(4):
        samples = [frame[slot_i] for frame in history]

        x = sum(p[0] for p in samples) / len(samples)
        y = sum(p[1] for p in samples) / len(samples)
        a = _circular_mean_deg([p[2] for p in samples])

        areas = [p[3][0] for p in samples]
        mean_area = sum(areas) / len(areas)
        max_area_error = max(
            area_relative_error(v, mean_area)
            for v in areas
        )

        if max_area_error > RECORD_AREA_TOL_RATIO:
            print(
                "slot[%d]录制面积不稳定：平均%.0fmm^2 最大波动%.1f%%" %
                (slot_i, mean_area, max_area_error * 100.0)
            )
            return None, None

        sig = _average_signature([p[3] for p in samples])

        if sig is None:
            return None, None

        # 72格径向轮廓是要求1解决180°分支的必要证据。结构字段仍可选，
        # 但不能再保存本次日志中quality=0的标准库，否则只能退回无方向主轴。
        if not _has_radial(sig):
            print(
                "slot[%d]径向轮廓无效：平均质量=%.3f；"
                "等待完整边界后重新稳定" %
                (
                    slot_i,
                    sig[8] if len(sig) > 8 else 0.0
                )
            )
            return None, None

        if structure_library_enabled:
            # 顶点数是离散身份特征，不能保留浮点平均结果。
            sig_values = list(sig)
            sig_values[STRUCT_VALID_INDEX] = 1.0
            sig_values[STRUCT_VERTEX_INDEX] = float(
                structure_counts_by_slot[slot_i][0]
            )
            sig = tuple(sig_values)
        else:
            # 精确保留成功版的前81项：基础特征9项+径向描述子72项。
            # 新增结构提取失败只关闭B增强，绝不阻止slots/shapes保存。
            sig = tuple(sig[:STRUCT_OFFSET])

        slots.append((
            round(x, 2),
            round(y, 2),
            round(a, 2)
        ))
        shapes.append(sig)

    if not structure_library_enabled:
        print(
            "录制结构增强未启用：顶点计数=%s；"
            "本次仍按要求1成功版面积+径向轮廓生成标准库" %
            structure_counts_by_slot
        )

    return slots, shapes

def run(uart_obj):
    """Run Requirement 1 until STM32 selects the next test mode."""
    global uart_rx_buffer
    global last_area_match_info

    uart_rx_buffer = ""
    uart = uart_obj

    # 每次切换模式都重新初始化摄像头，避免上一算法留下曝光/缓存状态。
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(True)

    print(">>> MODE 1：运行要求1成功算法")

    m = load_calib()

    # 读已录标准库
    _rec = load_demo()

    # 与要求1成功版一致，完整径向轮廓是标准库的必要字段。
    # 多边形结构只是B方案增强：缺失时接受原81字段库并自动回退匹配。
    if _rec is not None:
        _old_shapes = list(_rec[1])
        if (
            len(_old_shapes) != 4 or
            not all(_has_radial(sh) for sh in _old_shapes)
        ):
            print(">>> 标准库径向轮廓无效，本次自动重新录制4块")
            print(">>> 旧文件不会预先删除，录制成功后才覆盖")
            _rec = None
        elif not all(_has_structure(sh) for sh in _old_shapes):
            print(">>> 标准库采用要求1成功版面积+径向轮廓格式")
            print(">>> B结构增强未启用，但不会强制重录或阻止任务运行")

    if _rec is not None:
        slots_recorded, shapes_recorded = list(_rec[0]), list(_rec[1])
        print_demo_summary(slots_recorded, shapes_recorded)
        print_template_diagnostics(slots_recorded, shapes_recorded)
    else:
        slots_recorded, shapes_recorded = [], []
    while len(slots_recorded) < 4:
        slots_recorded.append(None)
    while len(shapes_recorded) < 4:
        shapes_recorded.append(None)
    recorded_count = sum(1 for s in slots_recorded if s is not None)

    # ---------- 决定初始阶段 ----------
    def stage_after_calib():
        """标定完成后该进哪个阶段。"""
        if MODE == "DETECT":
            return "DETECT"
        if MODE == "RECORD":
            return "RECORD"
        if MODE == "MATCH":
            return "MATCH"
        return "MATCH" if recorded_count == 4 else "RECORD"

    if MODE == "CALIB" or m is None:
        stage = "CALIB"
        if m is None:
            print(">>> 未找到标定文件，先进入标定阶段：对准 A4，保持稳定")
        else:
            print(">>> 强制重新标定：对准 A4，保持稳定")
    else:
        print("calib loaded")
        stage = stage_after_calib()

    def announce(stage):
        if stage == "CALIB":
            pass
        elif stage == "DETECT":
            print(">>> 检测模式(Part2)：半区内碎片坐标/角度实时打印")
        elif stage == "RECORD":
            print(">>> 四块同时录制：请把4块按最终拼合姿态一次摆在左半区")
            print(">>> 要求二值图中能分成4个独立区域；连续稳定%d组后自动保存" %
                  RECORD_STABLE_FRAMES)
        else:
            if recorded_count != 4 and MODE == "MATCH":
                print("ERROR: 标准库不满 4 条，无法匹配；请先录制")
            print(">>> 匹配模式：请把 4 块碎片随机摆到右半区")

    print("已录制 %d / 4 块落点" % recorded_count)
    announce(stage)

    clock = time.clock()
    frame_cnt = 0
    stable = 0                        # 标定稳定帧计数
    last_good_corners = None
    last_good_frame = -9999

    ang_hist = {}
    identity_hist = {}
    last_pick_pos = {}
    last_complete_picks = None
    last_complete_frame = -9999

    # 四块同时录制历史。
    record_history = []
    record_invalid_frames = 0

    # 与STM32的循环通信状态：
    # ARMED -> WAIT_ACK -> WAIT_START -> RUNNING -> WAIT_CLEAR -> ARMED
    comm_state = "ARMED"
    match_sent = False
    clear_frame_count = 0
    mcu_ready_seen = False

    # 移除黑卡纸后，首次检测到完整4块时启动2秒等待。
    # 等待期间仍持续拍照，让自动曝光自行稳定，但不进行匹配和角度投票。
    light_wait_start_ms = None
    light_settle_done = False

    # match_all每帧写入，供日志显示面积层采用了何种分配。
    last_area_match_info = {
        "mode": "INIT",
        "gap": 0.0,
        "combined_gap": 0.0,
        "avg_error": 1.0,
        "pair_errors": []
    }

    while True:
        try:
            clock.tick()
            img = sensor.snapshot()
            frame_cnt += 1

            # 每帧先处理STM32反馈，避免运动期间因图像跳帧而漏掉ALL_DONE。
            for mcu_msg in poll_mcu_frames(uart):
                print("MCU反馈:", mcu_msg)

                # 下一次测试按键会让STM32发送新的MODE帧。
                # 返回给总调度器，由它重新载入对应的成功算法。
                if mcu_msg.startswith("MODE,"):
                    try:
                        next_mode = int(mcu_msg.split(",", 1)[1])
                    except Exception:
                        next_mode = 0

                    if next_mode in (1, 2, 3):
                        print(">>> 收到下一轮模式%d，返回总调度器" % next_mode)
                        return next_mode

                    continue

                if mcu_msg == "READY":
                    mcu_ready_seen = True

                    # STM32重新上电意味着之前保存的任务已经丢失。
                    if comm_state != "ARMED":
                        print(">>> STM32重新READY，本轮发送状态清空")
                        comm_state = "ARMED"
                        match_sent = False
                        ang_hist = {}
                        identity_hist = {}
                        last_pick_pos = {}
                        last_complete_picks = None
                        last_complete_frame = -9999
                        light_wait_start_ms = None
                        light_settle_done = False

                elif mcu_msg == "ACK":
                    comm_state = "WAIT_START"

                elif mcu_msg == "WAIT_START":
                    comm_state = "WAIT_START"

                elif mcu_msg == "STARTED":
                    comm_state = "RUNNING"

                elif mcu_msg.startswith("RUN,"):
                    comm_state = "RUNNING"

                elif mcu_msg == "ALL_DONE":
                    # 不能立刻再次发送旧布局；先等待右半区被清空。
                    comm_state = "WAIT_CLEAR"
                    match_sent = False
                    clear_frame_count = 0
                    ang_hist = {}
                    identity_hist = {}
                    last_pick_pos = {}
                    last_complete_picks = None
                    last_complete_frame = -9999
                    light_wait_start_ms = None
                    light_settle_done = False
                    print(">>> 本轮完成，等待右半区清空后进入下一轮")

                elif mcu_msg.startswith("ERR"):
                    # STM32没有接受/完成本轮任务，允许视觉重新稳定并再次发送。
                    print(">>> STM32报告错误，本轮视觉发送解锁")
                    comm_state = "ARMED"
                    match_sent = False
                    ang_hist = {}
                    identity_hist = {}
                    last_pick_pos = {}
                    last_complete_picks = None
                    last_complete_frame = -9999

            if frame_cnt % (SKIP_FRAMES + 1) != 0:
                continue

            # ============ 阶段1：标定（Part1） ============
            if stage == "CALIB":
                corners = detect_a4_corners(img, RECT_THRESH_CALIB)
                if corners:
                    lab = classify_corners(corners)
                    for name, (x, y) in lab.items():
                        img.draw_cross(x, y, color=(255, 0, 0), size=10)
                        img.draw_string(x + 5, y - 10, name, color=(255, 255, 255), scale=1.5)
                    stable += 1
                    print("detected stable=%d/%d" % (stable, STABLE_CNT))
                    if stable >= STABLE_CNT:
                        px = [lab["BL"], lab["BR"], lab["TL"], lab["TR"]]
                        mm_pts = [(0, 0), (A4_W, 0), (0, A4_H), (A4_W, A4_H)]
                        hm = solve_homography(px, mm_pts)
                        if hm is not None and save_calib(hm):
                            m = hm
                            # 立即验证 4 角误差
                            print("--- verify ---")
                            for name, expected in [("BL", (0, 0)), ("BR", (A4_W, 0)),
                                                   ("TL", (0, A4_H)), ("TR", (A4_W, A4_H))]:
                                x, y = lab[name]
                                vx, vy = px_to_mm(m, x, y)
                                ex, ey = expected
                                err = math.sqrt((vx-ex)**2 + (vy-ey)**2)
                                print("%s px=(%d,%d) -> mm=(%.1f,%.1f) err=%.1fmm" % (name, x, y, vx, vy, err))
                            stage = stage_after_calib()
                            announce(stage)
                        else:
                            print("标定求解/保存失败，继续尝试")
                            stable = 0
                else:
                    stable = 0
                    if frame_cnt % 30 == 0:
                        print("NO A4 detected; FPS=%.1f" % clock.fps())
                continue

            # ============ 阶段2：检测/录制/匹配（Part2/Part3） ============
            if m is None:
                continue
            corners = detect_a4_corners(img, RECT_THRESH)
            corner_fresh = corners is not None
            if (not corner_fresh) and last_good_corners is not None and (frame_cnt - last_good_frame) < STALE_FRAMES:
                corners = last_good_corners
            if corners:
                if corner_fresh:
                    last_good_corners = corners
                    last_good_frame = frame_cnt
                lab = classify_corners(corners)
                for name, (x, y) in lab.items():
                    img.draw_cross(x, y, color=(255, 0, 0), size=8)
                    img.draw_string(x + 5, y - 10, name, color=(255, 255, 255), scale=1)

                if stage == "RECORD":
                    use_right = RECORD_RIGHT
                elif stage == "DETECT":
                    use_right = DETECT_RIGHT
                else:
                    use_right = MATCH_RIGHT
                roi_margin = RECORD_ROI_MARGIN_PX if stage == "RECORD" else 0
                roi = half_roi(
                    lab,
                    img,
                    right=use_right,
                    outer_margin_px=roi_margin
                )
                img.draw_rectangle(roi, color=(0, 255, 255), thickness=1)

                record_detect_info = None
                record_contour_thresholds = None

                # 录制阶段额外处理相邻白片的暗桥，并防止异常/旧标定矩阵把
                # 当前ROI中央的完整4块全部当作A4边缘删除。
                if stage == "RECORD":
                    record_filter_m = m

                    # ROI来自当前帧A4角点，边缘过滤也必须使用同一帧映射。
                    # 这里只决定是否为纸边伪影；目标坐标和角度仍使用正式标定m。
                    if corner_fresh:
                        live_filter_m = solve_homography(
                            [lab["BL"], lab["BR"], lab["TL"], lab["TR"]],
                            [
                                (0, 0),
                                (A4_W, 0),
                                (0, A4_H),
                                (A4_W, A4_H)
                            ]
                        )
                        if live_filter_m is not None:
                            record_filter_m = live_filter_m

                    pieces, record_detect_info = detect_record_pieces(
                        img,
                        roi,
                        record_filter_m,
                        use_right
                    )

                    if record_detect_info[3].startswith("RECORD_L42"):
                        # 连通域既然依靠L42分开，轮廓/结构也必须优先使用同一
                        # 二值定义；否则重叠bbox内用L38会再次把两片粘回去。
                        record_contour_thresholds = (
                            WHITE_THRESH_RECORD_SEPARATE,
                        )
                else:
                    pieces = detect_pieces(img, roi)

                # 备用阈值可能多检出少量反光。
                if stage == "RECORD" and len(pieces) > 4:
                    pieces.sort(key=_piece_pixel_count, reverse=True)
                    pieces = pieces[:4]

                if stage == "MATCH" and len(pieces) > 4:
                    pieces.sort(key=_piece_pixel_count, reverse=True)
                    pieces = pieces[:4]

                # 所有参与录制、匹配和日志的形状特征，必须在画轮廓、角度线、
                # 质心十字和编号之前提取。否则这些彩色覆盖会被下一次二值轮廓
                # 搜索读回去，使RDP多边形出现额外拐点或轮廓缺口。
                clean_frame_poses = None
                clean_debug_metrics = None
                need_clean_poses = (
                    (
                        stage == "DETECT" and
                        bool(pieces) and
                        frame_cnt % 30 == 0
                    ) or
                    (
                        stage == "RECORD" and
                        len(pieces) == 4 and
                        corner_fresh
                    ) or
                    (
                        stage == "MATCH" and
                        len(pieces) == 4 and
                        comm_state == "ARMED"
                    )
                )

                if need_clean_poses:
                    clean_frame_poses = [
                        piece_pose_and_signature(
                            img,
                            r,
                            blist,
                            m,
                            record_contour_thresholds
                        )
                        for (r, blist) in pieces
                    ]

                    if REQ1_DIAGNOSTICS and stage == "MATCH":
                        clean_debug_metrics = [
                            diagnostic_piece_metrics(img, r, blist, m)
                            for (r, blist) in pieces
                        ]

                if SHOW_BINARY_MASK:
                    draw_binary_preview(img, roi)
                if frame_cnt % 30 == 0:
                    print("[%s] 碎片数=%d  FPS=%.1f" % (stage, len(pieces), clock.fps()))
                    if stage == "RECORD" and record_detect_info is not None:
                        standard_count, separate_count, edge_count, detect_path = (
                            record_detect_info
                        )
                        separate_text = (
                            "%d" % separate_count
                            if separate_count >= 0 else "未启用"
                        )
                        print(
                            "  录制检测：标准=%d L42分离=%s 物理边缘后=%d 路径=%s" %
                            (
                                standard_count,
                                separate_text,
                                edge_count,
                                detect_path
                            )
                        )

                for i, (r, blist) in enumerate(pieces):
                    # rect 只用于内部 ROI 搜索；画面上只显示碎片真实轮廓和真实质心
                    contour = get_main_contour(
                        img,
                        r,
                        blist,
                        record_contour_thresholds
                    )
                    cx_f, cy_f = binary_shape_centroid(blist, r)
                    cx = int(round(cx_f))
                    cy = int(round(cy_f))

                    if contour is not None:
                        try:
                            # 绿色：二值化后的真实形状轮廓，不画外接矩形
                            pts = contour.points()
                            if pts:
                                img.draw_polygon(pts, color=(0, 255, 0), thickness=2)

                            if DRAW_ANGLE:
                                # 紫色角度线：沿真实轮廓最小外接旋转矩形的最长边
                                mc = contour.min_corners()
                                n = len(mc)
                                best_len2 = 0
                                bx1 = by1 = bx2 = by2 = 0
                                for j in range(n):
                                    x1, y1 = mc[j]
                                    x2, y2 = mc[(j + 1) % n]
                                    l2 = (x2 - x1) ** 2 + (y2 - y1) ** 2
                                    if l2 > best_len2:
                                        best_len2 = l2
                                        bx1, by1, bx2, by2 = x1, y1, x2, y2
                                if best_len2 > 0:
                                    ux = bx2 - bx1
                                    uy = by2 - by1
                                    norm = math.sqrt(ux * ux + uy * uy)
                                    if norm > 0:
                                        ux /= norm
                                        uy /= norm
                                        length = math.sqrt(best_len2) * 0.5
                                        ex = int(cx + ux * length)
                                        ey = int(cy + uy * length)
                                        img.draw_line(cx, cy, ex, ey,
                                                      color=(255, 0, 255), thickness=2)
                        except Exception as e:
                            if VERBOSE:
                                print("draw contour err:", e)

                    # 红色十字始终画在二值真实形状质心
                    img.draw_cross(cx, cy, color=(255, 0, 0), size=7)
                    img.draw_string(cx + 5, cy - 12, "%d" % i,
                                    color=(255, 255, 0), scale=1.5)

                # ---- DETECT：Part2 功能，只打印坐标/角度 ----
                if stage == "DETECT":
                    if frame_cnt % 30 == 0 and clean_frame_poses:
                        print("detected pieces=%d" % len(pieces))
                        for i, pose in enumerate(clean_frame_poses):
                            x, y, a, sig = pose
                            print("  piece[%d] mm=(%.1f,%.1f) pa=%.1f 面积=%.0fmm^2" % (i, x, y, a, sig[0]))

                # ---- RECORD：四块同时录制标准库 ----
                elif stage == "RECORD":
                    if len(pieces) == 4 and corner_fresh:
                        record_invalid_frames = 0

                        current_poses = clean_frame_poses

                        if not record_history:
                            aligned = _initial_record_order(current_poses)
                            record_history = [aligned]
                            record_invalid_frames = 0
                            print("录制稳定 1/%d" % RECORD_STABLE_FRAMES)
                        else:
                            reference = record_history[-1]
                            (
                                aligned,
                                max_move,
                                max_area_error
                            ) = _align_record_poses(
                                current_poses,
                                reference
                            )

                            if aligned is None:
                                record_history = []
                            elif (
                                max_move > RECORD_RESTART_MOVE_MM or
                                max_area_error > RECORD_AREA_RESTART_RATIO
                            ):
                                # 位置或面积变化过大，认为碎片重新摆放/识别发生断裂。
                                aligned = _initial_record_order(current_poses)
                                record_history = [aligned]
                                print(
                                    "录制对象变化：位移%.1fmm 面积变化%.1f%%，"
                                    "重新计数 1/%d" %
                                    (
                                        max_move,
                                        max_area_error * 100.0,
                                        RECORD_STABLE_FRAMES
                                    )
                                )
                            elif (
                                max_move <= RECORD_POS_TOL_MM and
                                max_area_error <= RECORD_AREA_TOL_RATIO
                            ):
                                record_history.append(aligned)

                                if len(record_history) > RECORD_STABLE_FRAMES:
                                    record_history.pop(0)

                                print(
                                    "录制稳定 %d/%d，最大位移%.1fmm，"
                                    "最大面积变化%.1f%%" %
                                    (
                                        len(record_history),
                                        RECORD_STABLE_FRAMES,
                                        max_move,
                                        max_area_error * 100.0
                                    )
                                )
                            else:
                                # 有轻微移动但未达到重新摆放阈值，本帧不计入稳定历史。
                                if frame_cnt % 15 == 0:
                                    print(
                                        "请保持4块稳定：位移%.1fmm，面积变化%.1f%%" %
                                        (
                                            max_move,
                                            max_area_error * 100.0
                                        )
                                    )

                        if len(record_history) >= RECORD_STABLE_FRAMES:
                            new_slots, new_shapes = finalize_multi_record(
                                record_history
                            )

                            if new_slots is not None and new_shapes is not None:
                                slots_recorded = list(new_slots)
                                shapes_recorded = list(new_shapes)
                                recorded_count = 4

                                save_demo(
                                    slots_recorded,
                                    shapes_recorded
                                )

                                img.draw_string(
                                    10, 10,
                                    "4 DEMO SAVED",
                                    color=(0, 255, 0),
                                    scale=2
                                )

                                print(">>> 四块标准库一次录制完成")
                                print(">>> slot顺序按目标质心物理X从小到大")
                                record_history = []

                                if MODE == "AUTO":
                                    stage = "MATCH"
                                    comm_state = "ARMED"
                                    match_sent = False
                                    ang_hist = {}
                                    identity_hist = {}
                                    last_pick_pos = {}
                                    last_complete_picks = None
                                    last_complete_frame = -9999
                                    light_wait_start_ms = None
                                    light_settle_done = False
                                    print(
                                        ">>> 自动切换匹配模式："
                                        "请把4块随机摆到右半区"
                                    )
                            else:
                                print("标准库多帧平均失败，重新稳定")
                                record_history = []

                    else:
                        record_invalid_frames += 1

                        # 连续异常一段时间后清空旧稳定历史，防止混入前一次布局。
                        if record_invalid_frames >= 8:
                            record_history = []
                            record_invalid_frames = 0

                        if frame_cnt % 30 == 0:
                            if len(pieces) < 4:
                                print(
                                    "四块同时录制：当前只识别到%d块。"
                                    "请确认碎片之间有可见黑色分界，且未贴A4外边缘。" %
                                    len(pieces)
                                )
                            else:
                                print(
                                    "四块同时录制：当前候选%d块，"
                                    "正在排除纸边/反光干扰。" %
                                    len(pieces)
                                )

                # ---- MATCH：完整轮廓匹配 + 360°旋转角 + UART ----
                else:
                    slots4 = [s for s in slots_recorded if s is not None]
                    shapes4 = [sh for sh in shapes_recorded if sh is not None]

                    if len(slots4) != 4:
                        if frame_cnt % 30 == 0:
                            print("标准库不满4条，无法匹配")
                        continue

                    # STM32完成后，先确认右半区已经不是旧的4块布局。
                    if comm_state == "WAIT_CLEAR":
                        if len(pieces) < 4:
                            clear_frame_count += 1
                        else:
                            clear_frame_count = 0

                        img.draw_string(
                            10, 10,
                            "WAIT CLEAR %d/%d" %
                            (clear_frame_count, CLEAR_STABLE_FRAMES),
                            color=(255, 255, 0),
                            scale=1
                        )

                        if clear_frame_count >= CLEAR_STABLE_FRAMES:
                            comm_state = "ARMED"
                            clear_frame_count = 0
                            ang_hist = {}
                            identity_hist = {}
                            last_pick_pos = {}
                            last_complete_picks = None
                            last_complete_frame = -9999
                            light_wait_start_ms = None
                            light_settle_done = False
                            print(">>> 右半区已清空，可以摆放下一轮随机位置")

                        continue

                    # 任务已被STM32保存或正在执行时，不再重复做匹配和发送。
                    if comm_state in ("WAIT_ACK", "WAIT_START", "RUNNING"):
                        label = (
                            "WAIT ACK" if comm_state == "WAIT_ACK"
                            else (
                                "PRESS START" if comm_state == "WAIT_START"
                                else "MCU RUNNING"
                            )
                        )

                        img.draw_string(
                            10, 10,
                            label,
                            color=(0, 255, 0),
                            scale=2
                        )
                        continue

                    # 移除遮挡后，先等光线稳定，再开始真正采样。
                    # 触发条件只有一个：本轮第一次识别到完整4块。
                    if not light_settle_done:
                        now_ms = time.ticks_ms()

                        if light_wait_start_ms is None:
                            if len(pieces) == 4:
                                light_wait_start_ms = now_ms
                                print(
                                    ">>> 已检测到4块，等待2秒光线稳定，"
                                    "期间不采集匹配角度"
                                )
                            else:
                                img.draw_string(
                                    10, 10,
                                    "WAIT 4 PIECES",
                                    color=(255, 255, 0),
                                    scale=1
                                )
                                continue

                        elapsed_ms = time.ticks_diff(
                            now_ms,
                            light_wait_start_ms
                        )

                        if elapsed_ms < LIGHT_SETTLE_MS:
                            remain_ms = LIGHT_SETTLE_MS - elapsed_ms
                            img.draw_string(
                                10, 10,
                                "LIGHT WAIT %.1fs" % (remain_ms / 1000.0),
                                color=(255, 255, 0),
                                scale=1
                            )
                            continue

                        # 两秒结束后丢弃等待期间的所有历史，从当前帧重新开始采样。
                        light_settle_done = True
                        light_wait_start_ms = None
                        ang_hist = {}
                        identity_hist = {}
                        last_pick_pos = {}
                        last_complete_picks = None
                        last_complete_frame = -9999
                        print(">>> 光线稳定等待结束，从当前帧开始采样")

                    using_cache = False
                    picks = None

                    if len(pieces) == 4:
                        picks = clean_frame_poses
                        last_complete_picks = picks
                        last_complete_frame = frame_cnt

                    elif (
                        last_complete_picks is not None and
                        (frame_cnt - last_complete_frame) <= MATCH_CACHE_FRAMES
                    ):
                        # 目标静止时，偶发一帧只检出3块不应让整套状态失效。
                        picks = last_complete_picks
                        using_cache = True

                    if picks is None:
                        if frame_cnt % 30 == 0:
                            print(
                                "  当前检出%d块，等待自动备用阈值找到完整4块" %
                                len(pieces)
                            )
                        continue

                    if (
                        REQ1_DIAGNOSTICS and
                        not using_cache and
                        len(pieces) == 4
                    ):
                        print("REQ1DBG frame=%d candidates=4" % frame_cnt)
                        for candidate_i in range(4):
                            r, blist = pieces[candidate_i]
                            pose = picks[candidate_i]
                            sig = pose[3]
                            metrics = clean_debug_metrics[candidate_i]
                            print(
                                "REQ1DBG detect candidate=%d centroidPx=(%.1f,%.1f) "
                                "centroidMm=(%.1f,%.1f) areaPx=%d areaMm2=%.1f "
                                "perimeterMm=%.1f bboxAspect=%.3f compact=%.5f "
                                "contourPoints=%d minRectCorners=%d lineEdges=%.0f "
                                "radialQ=%.3f polygonVertices=%d "
                                "edgeProfile=%s rawAngle=%.1f" %
                                (
                                    candidate_i,
                                    metrics["cx"], metrics["cy"],
                                    pose[0], pose[1],
                                    metrics["area_px"], sig[0],
                                    metrics["perimeter_mm"],
                                    metrics["aspect"],
                                    metrics["compactness"],
                                    metrics["contour_points"],
                                    metrics["min_corners"],
                                    sig[4] if len(sig) > 4 else 0.0,
                                    sig[8] if len(sig) > 8 else 0.0,
                                    int(round(sig[STRUCT_VERTEX_INDEX]))
                                    if _has_structure(sig) else 0,
                                    [
                                        round(sig[STRUCT_EDGE_OFFSET + k], 3)
                                        for k in range(STRUCT_EDGE_COUNT)
                                    ] if _has_structure(sig) else [],
                                    pose[2]
                                )
                            )

                    matched = match_all(picks, shapes4, slots4)

                    if len(matched) != 4:
                        if frame_cnt % 30 == 0:
                            if last_area_match_info.get("mode") == "AREA_REJECT":
                                print(
                                    "  面积身份校验拒绝：平均误差%.1f%%，各对=%s" %
                                    (
                                        last_area_match_info.get(
                                            "avg_error", 1.0
                                        ) * 100.0,
                                        [
                                            round(v * 100.0, 1)
                                            for v in last_area_match_info.get(
                                                "pair_errors", []
                                            )
                                        ]
                                    )
                                )
                            elif last_area_match_info.get("mode") == "EVIDENCE_CONFLICT":
                                print(
                                    "  面积与完整轮廓身份冲突，等待下一帧："
                                    "areaGap=%.3f combinedGap=%.3f" %
                                    (
                                        last_area_match_info.get("gap", 0.0),
                                        last_area_match_info.get(
                                            "combined_gap", 0.0
                                        )
                                    )
                                )
                            elif last_area_match_info.get("mode") == "LOW_CONFIDENCE":
                                print(
                                    "  身份最优/第二优差距不足，等待下一帧："
                                    "combinedGap=%.3f structureGap=%.3f" %
                                    (
                                        last_area_match_info.get(
                                            "combined_gap", 0.0
                                        ),
                                        last_area_match_info.get(
                                            "structure_gap", 0.0
                                        )
                                    )
                                )
                            elif last_area_match_info.get("mode") == "BRANCH_AMBIGUOUS":
                                print(
                                    "  近180度分支证据不足，等待下一帧："
                                    "slot=%s contrast=%.3f symmetryRatio=%.3f" %
                                    (
                                        str(last_area_match_info.get(
                                            "branch_slot", "?"
                                        )),
                                        last_area_match_info.get(
                                            "branch_contrast", 0.0
                                        ),
                                        last_area_match_info.get(
                                            "symmetry_ratio", 1e9
                                        )
                                    )
                                )
                            elif last_area_match_info.get("mode") in (
                                "STRUCTURE_INVALID", "RADIAL_INVALID"
                            ):
                                print(
                                    "  当前轮廓特征不完整，等待重新采样：%s" %
                                    last_area_match_info.get("mode")
                                )
                            else:
                                print("  面积+完整轮廓+结构匹配失败")
                        continue

                    stable_results = []
                    all_stable = True

                    for (
                        px, py,
                        sx, sy,
                        d_ang,
                        slot_i,
                        dist,
                        contour_ok,
                        structure_count
                    ) in matched:
                        lp = last_pick_pos.get(slot_i)

                        position_jump = (
                            lp is not None and
                            math.sqrt(
                                (px - lp[0]) ** 2 +
                                (py - lp[1]) ** 2
                            ) > IDENTITY_POS_TOL_MM
                        )

                        if position_jump:
                            ang_hist[slot_i] = []
                            identity_hist[slot_i] = []

                            if REQ1_DIAGNOSTICS:
                                print(
                                    "REQ1DBG identityJump slot=%d old=(%.1f,%.1f) "
                                    "new=(%.1f,%.1f)" %
                                    (slot_i, lp[0], lp[1], px, py)
                                )

                        last_pick_pos[slot_i] = (px, py)

                        h = ang_hist.get(slot_i, [])

                        if (
                            h and
                            angle_diff(d_ang, h[-1]) > ANG_JUMP_RESET
                        ):
                            print(
                                "slot[%d] angle jump: %d -> %d, clear history" %
                                (slot_i, h[-1], d_ang)
                            )
                            ang_hist[slot_i] = []

                        # 缓存帧使用的是上一组完整识别结果，不重复灌入投票窗口。
                        if not using_cache:
                            identity_feature_jump = push_identity_observation(
                                identity_hist, slot_i, px, py, structure_count
                            )
                            if identity_feature_jump and not position_jump:
                                ang_hist[slot_i] = []
                            push_angle(ang_hist, slot_i, d_ang)

                        s_ang = stable_angle(ang_hist, slot_i)
                        identity_votes = stable_identity_votes(
                            identity_hist, slot_i, px, py, structure_count
                        )
                        identity_stable = (
                            identity_votes >= IDENTITY_MIN_VOTES
                        )

                        if s_ang is None or not identity_stable:
                            all_stable = False
                        else:
                            stable_results.append((
                                slot_i,
                                px, py,
                                sx, sy,
                                s_ang
                            ))

                        if REQ1_DIAGNOSTICS:
                            print(
                                "REQ1DBG stable slot=%d vertices=%d identityVotes=%d/%d "
                                "angleVotes=%d/%d stableAngle=%s" %
                                (
                                    slot_i,
                                    structure_count,
                                    identity_votes,
                                    IDENTITY_MIN_VOTES,
                                    len(ang_hist.get(slot_i, [])),
                                    ANG_MIN_VOTES,
                                    str(s_ang) if s_ang is not None else "NA"
                                )
                            )

                    if frame_cnt % 30 == 0 or all_stable:
                        print(
                            "--- match result [%s areaGap=%.3f combinedGap=%.3f "
                            "structureGap=%.3f avgAreaErr=%.1f%%] ---" %
                            (
                                last_area_match_info.get("mode", "?"),
                                last_area_match_info.get("gap", 0.0),
                                last_area_match_info.get(
                                    "combined_gap", 0.0
                                ),
                                last_area_match_info.get(
                                    "structure_gap", 0.0
                                ),
                                last_area_match_info.get(
                                    "avg_error", 1.0
                                ) * 100.0
                            )
                        )

                        for i, (
                            px, py,
                            sx, sy,
                            d_ang,
                            slot_i,
                            dist,
                            contour_ok,
                            structure_count
                        ) in enumerate(matched):
                            psig = picks[i][3]
                            tsig = shapes4[slot_i]
                            s_ang = stable_angle(ang_hist, slot_i)

                            tag = (
                                "稳定角=%d" % s_ang
                                if s_ang is not None
                                else "角度投票中..."
                            )
                            source = (
                                "径向全角纠偏"
                                if slot_i in last_area_match_info.get(
                                    "radial_pose_slots", []
                                )
                                else (
                                    "轮廓360"
                                    if contour_ok
                                    else "旧库主轴回退"
                                )
                            )
                            cache_tag = " CACHE" if using_cache else ""

                            print(
                                "  piece[%d] -> slot[%d] "
                                "pick=(%.1f,%.1f) place=(%.1f,%.1f) "
                                "dAng=%d vertices=%d [%s %s score=%.3f%s]" %
                                (
                                    i, slot_i,
                                    px, py,
                                    sx, sy,
                                    d_ang,
                                    structure_count,
                                    source,
                                    tag,
                                    dist,
                                    cache_tag
                                )
                            )
                            area_err = area_relative_error(
                                psig[0],
                                tsig[0]
                            )

                            print(
                                "     碎片面积=%.0f 目标面积=%.0f "
                                "面积误差=%.1f%% 轮廓质量=%.2f/%.2f" %
                                (
                                    psig[0],
                                    tsig[0],
                                    area_err * 100.0,
                                    psig[8] if len(psig) > 8 else 0,
                                    tsig[8] if len(tsig) > 8 else 0
                                )
                            )

                    if (
                        all_stable and
                        len(stable_results) == 4 and
                        not match_sent and
                        comm_state == "ARMED"
                    ):
                        stable_results.sort(key=lambda t: t[0])

                        if send_to_mcu(uart, stable_results):
                            match_sent = True
                            comm_state = "WAIT_ACK"
                            print(
                                ">>> 任务已发送，等待STM32 ACK；"
                                "收到后由串口屏按钮启动"
                            )

                        img.draw_string(
                            10, 10,
                            "TASK SENT",
                            color=(0, 255, 0),
                            scale=2
                        )
            else:
                if frame_cnt % 30 == 0:
                    print("NO A4 detected; FPS=%.1f" % clock.fps())
        except KeyboardInterrupt:
            raise
        except BaseException as e:
            print("WARN: main loop exception:", e)
