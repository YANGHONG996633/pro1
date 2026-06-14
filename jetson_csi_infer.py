"""
Jetson Nano | JetPack 4.6.1
USB 摄像头 + TensorRT .engine 推理 + 串口触发
收到 STM32 串口帧 → 抓取一帧图片 → 推理 → 回复 0x01(缺陷剔除) / 0x02(正常)

=== Jetson Nano 串口初始化 (首次使用必须执行) ===
# 1. 禁用串口控制台 (ttyTHS1 默认被 getty 占用)
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
sudo udevadm trigger

# 2. 验证串口权限
sudo usermod -a -G dialout $USER
# 注销后重新登录生效

# 3. 测试串口通信 (用跳线帽短接 TX/RX 做回环测试)
python3 -c "
import serial
ser = serial.Serial('/dev/ttyTHS1', 9600, timeout=0.5)
ser.write(b'\\xAA\\x55\\x10\\x01\\x00\\x11')
print('Sent: AA 55 10 01 00 11')
data = ser.read(6)
print('Recv:', ' '.join(f'{b:02X}' for b in data))
ser.close()
"

=== 逻辑分析仪调试检查清单 ===
# 1. 确认 UART 解码器设置为: 9600 baud, 8N1, LSB first
# 2. 探头接 STM32 PA9 (TX)，GND 接 STM32 GND
# 3. 正常应看到: AA 55 10 01 00 11 (共 6 字节，约 6.25ms)
# 4. 如果看到 0xFF / 0xFB / 0xFE 等，说明波特率设置错误或共地不良
# 5. Jetson 回复应看到: BB 01 BA (缺陷) 或 BB 02 B9 (正常)
"""

import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit  # 初始化 CUDA 上下文
import serial
import time
import threading
import queue
from typing import List, Optional, Tuple

# ── COCO 80 类名 ────────────────────────────────────────────────────────────
COCO_NAMES = [
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "dining table",
    "toilet",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush",
]

# ── 配置 ──────────────────────────────────────────────────────────────────────
ENGINE_PATH = "yolov8n_fp16.engine"  # .engine 文件路径
INPUT_W, INPUT_H = 640, 640
CONF_THRESH = 0.25
IOU_THRESH = 0.45
CAM_INDEX = 0  # USB 摄像头设备索引 (0=/dev/video0, 1=/dev/video1 ...)
CAM_W, CAM_H = 640, 480  # 摄像头分辨率 (模型输入 640x640，无需 1080p)

# ── 串口配置 ───────────────────────────────────────────────────────────────
SERIAL_PORT = "/dev/ttyTHS1"  # Jetson Nano UART1 (引脚 8/10); USB转串口用 /dev/ttyUSB0
SERIAL_BAUD = 9600
SERIAL_TIMEOUT = 0.05  # 字节读取超时 (秒)

# ── 串口帧协议 (与 STM32 一致) ──────────────────────────────────────────────
FRAME_HEADER_1 = 0xAA
FRAME_HEADER_2 = 0x55
FRAME_CMD_EXPECT = 0x10  # 期望的命令字节
REPLY_DEFECT = 0x01  # 有缺陷 → 剔除
REPLY_NORMAL = 0x02  # 无缺陷 → 放行

# 回复帧协议: 帧头 + 状态 + XOR = 3 字节
REPLY_HEADER = 0xBB
# 回复帧格式: BB STATUS (BB^STATUS)
# 例: BB 01 BA  (缺陷)   BB 02 B9 (正常)

# ── 缺陷判定：检测到列表中任一类别ID时回复 0x01 ──────────────────────────────
# 留空 = 只要检测到任意目标即判为缺陷
# 示例: DEFECT_CLASS_IDS = [0]   # 只把 "person" 当缺陷
DEFECT_CLASS_IDS = []

# ── 调试开关 ───────────────────────────────────────────────────────────────
SHOW_WINDOW = False  # 是否显示摄像头画面 (True=调试, False=生产)
DEBUG_RAW_HEX = True  # 是否打印收到的原始16进制数据（调试用）


def _hex_str(data: bytes) -> str:
    """bytes → 空格分隔的大写16进制字符串 (兼容 Python 3.6)"""
    return " ".join(f"{b:02X}" for b in data)


# ── TensorRT 引擎加载 ───────────────────────────────────────────────────────
class TRTEngine:
    def __init__(self, engine_path):
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

        # 分配主机/设备缓冲区
        self.inputs, self.outputs, self.bindings, self.stream = (
            [],
            [],
            [],
            cuda.Stream(),
        )
        self.output_shape = None  # 缓存第一个输出的 shape

        for binding in self.engine:
            size = trt.volume(self.engine.get_binding_shape(binding))
            dtype = trt.nptype(self.engine.get_binding_dtype(binding))
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            self.bindings.append(int(device_mem))
            if self.engine.binding_is_input(binding):
                self.inputs.append({"host": host_mem, "device": device_mem})
            else:
                self.outputs.append({"host": host_mem, "device": device_mem})
                if self.output_shape is None:
                    self.output_shape = self.engine.get_binding_shape(binding)

    def infer(self, img_chw: np.ndarray) -> List[np.ndarray]:
        np.copyto(self.inputs[0]["host"], img_chw.ravel())
        for inp in self.inputs:
            cuda.memcpy_htod_async(inp["device"], inp["host"], self.stream)
        self.context.execute_async_v2(self.bindings, self.stream.handle, None)
        for out in self.outputs:
            cuda.memcpy_dtoh_async(out["host"], out["device"], self.stream)
        self.stream.synchronize()
        return [out["host"] for out in self.outputs]


# ── 预处理 ────────────────────────────────────────────────────────────────────
def preprocess(frame: np.ndarray) -> Tuple[np.ndarray, float, tuple]:
    """letterbox 缩放 → CHW float32，返回 (blob, scale, pad)"""
    h0, w0 = frame.shape[:2]
    scale = min(INPUT_H / h0, INPUT_W / w0)
    nh, nw = int(round(h0 * scale)), int(round(w0 * scale))
    img = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)

    # 填充到 640x640
    dh, dw = (INPUT_H - nh) // 2, (INPUT_W - nw) // 2
    img = cv2.copyMakeBorder(
        img,
        dh,
        INPUT_H - nh - dh,
        dw,
        INPUT_W - nw - dw,
        cv2.BORDER_CONSTANT,
        value=(114, 114, 114),
    )
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    img = img.transpose(2, 0, 1)  # HWC → CHW
    img = np.ascontiguousarray(img[None])  # 加 batch 维
    return img, scale, (dw, dh)


# ── 后处理（YOLOv5 / YOLOv8 输出格式）────────────────────────────────────────
def postprocess(raw: np.ndarray, scale: float, pad: tuple, orig_h: int, orig_w: int):
    """
    raw shape: (1, 25200, 85) for YOLOv5
               (1, 84, 8400)  for YOLOv8  → 自动转置
    返回 list of (x1,y1,x2,y2,conf,cls_id)
    """
    pred = raw.squeeze()  # 去掉 batch 维

    # YOLOv8: [84, 8400] → [8400, 84]
    if pred.shape[0] < pred.shape[1]:
        pred = pred.T
        # YOLOv8 无 objectness，直接取类别最大值
        boxes = pred[:, :4]
        scores = pred[:, 4:]
        cls_ids = scores.argmax(axis=1)
        confs = scores.max(axis=1)
    else:
        # YOLOv5: [25200, 85]，第4列是 objectness
        obj_conf = pred[:, 4]
        cls_conf = pred[:, 5:].max(axis=1)
        confs = obj_conf * cls_conf
        cls_ids = pred[:, 5:].argmax(axis=1)
        boxes = pred[:, :4]

    mask = confs > CONF_THRESH
    boxes, confs, cls_ids = boxes[mask], confs[mask], cls_ids[mask]

    if len(boxes) == 0:
        return []

    # xywh → xyxy（中心格式）
    dw, dh = pad
    x1 = (boxes[:, 0] - boxes[:, 2] / 2 - dw) / scale
    y1 = (boxes[:, 1] - boxes[:, 3] / 2 - dh) / scale
    x2 = (boxes[:, 0] + boxes[:, 2] / 2 - dw) / scale
    y2 = (boxes[:, 1] + boxes[:, 3] / 2 - dh) / scale
    x1, y1 = np.clip(x1, 0, orig_w), np.clip(y1, 0, orig_h)
    x2, y2 = np.clip(x2, 0, orig_w), np.clip(y2, 0, orig_h)

    rects = np.stack([x1, y1, x2, y2], axis=1).astype(np.float32)

    # NMS（逐类）
    keep = cv2.dnn.NMSBoxes(rects.tolist(), confs.tolist(), CONF_THRESH, IOU_THRESH)
    if len(keep) == 0:
        return []
    keep = keep.flatten()
    return [(rects[i], confs[i], cls_ids[i]) for i in keep]


# ── 可视化 ────────────────────────────────────────────────────────────────────
def draw(frame, detections):
    for box, conf, cls_id in detections:
        x1, y1, x2, y2 = map(int, box)
        label = f"{COCO_NAMES[cls_id]} {conf:.2f}"
        color = (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw, y1), color, -1)
        cv2.putText(
            frame, label, (x1, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1
        )
    return frame


# ── 串口帧读取 ─────────────────────────────────────────────────────────────
def read_frame(
    ser: serial.Serial, timeout_s: float = 0.3
) -> Optional[Tuple[int, int, int]]:
    """
    阻塞读取一帧 STM32 数据: AA 55 CMD PAY0 PAY1 XOR
    返回 (cmd, payload0, payload1)，超时/校验失败返回 None
    """
    deadline = time.time() + timeout_s
    buf = bytearray()

    while time.time() < deadline:
        # 一次读尽量多的可用字节，而非逐字节读
        waiting = ser.in_waiting
        if waiting > 0:
            chunk = ser.read(min(waiting, 64))
            for b in chunk:
                buf.append(b)
                if len(buf) > 6:
                    buf.pop(0)
                if (
                    len(buf) >= 6
                    and buf[-6] == FRAME_HEADER_1
                    and buf[-5] == FRAME_HEADER_2
                ):
                    cmd = buf[-4]
                    payload0 = buf[-3]
                    payload1 = buf[-2]
                    checksum = buf[-1]
                    if checksum == (cmd ^ payload0 ^ payload1):
                        return cmd, payload0, payload1
        else:
            b = ser.read(1)  # 阻塞等待下一个字节
            if not b:
                continue
            buf.append(b[0])
            if len(buf) > 6:
                buf.pop(0)
            if (
                len(buf) >= 6
                and buf[-6] == FRAME_HEADER_1
                and buf[-5] == FRAME_HEADER_2
            ):
                cmd = buf[-4]
                payload0 = buf[-3]
                payload1 = buf[-2]
                checksum = buf[-1]
                if checksum == (cmd ^ payload0 ^ payload1):
                    return cmd, payload0, payload1

    return None


# ── 串口读取线程 ────────────────────────────────────────────────────────────
def serial_reader(
    ser: serial.Serial, frame_queue: queue.Queue, stop_event: threading.Event
):
    """
    后台线程：持续读取串口数据，将完整帧放入 queue
    避免主循环被串口阻塞
    """
    buf = bytearray()
    while not stop_event.is_set():
        try:
            waiting = ser.in_waiting
            if waiting > 0:
                chunk = ser.read(min(waiting, 64))
                if DEBUG_RAW_HEX:
                    print(f"[RAW] RX {len(chunk)}B: {_hex_str(chunk)}")
                for b in chunk:
                    buf.append(b)
                    if len(buf) > 6:
                        buf.pop(0)
                    if (
                        len(buf) >= 6
                        and buf[-6] == FRAME_HEADER_1
                        and buf[-5] == FRAME_HEADER_2
                    ):
                        cmd = buf[-4]
                        payload0 = buf[-3]
                        payload1 = buf[-2]
                        checksum = buf[-1]
                        if checksum == (cmd ^ payload0 ^ payload1):
                            frame_queue.put((cmd, payload0, payload1))
                        else:
                            print(f"[WARN] 帧校验失败: {_hex_str(buf[-6:])}")
            else:
                # 短暂阻塞等待，避免 busy-wait 吃掉 CPU
                b = ser.read(1)
                if not b:
                    continue
                if DEBUG_RAW_HEX:
                    print(f"[RAW] RX 1B: {b.hex().upper()}")
                buf.append(b[0])
                if len(buf) > 6:
                    buf.pop(0)
                if (
                    len(buf) >= 6
                    and buf[-6] == FRAME_HEADER_1
                    and buf[-5] == FRAME_HEADER_2
                ):
                    cmd = buf[-4]
                    payload0 = buf[-3]
                    payload1 = buf[-2]
                    checksum = buf[-1]
                    if checksum == (cmd ^ payload0 ^ payload1):
                        frame_queue.put((cmd, payload0, payload1))
        except serial.SerialException as e:
            print(f"[FATAL] 串口读取线程异常退出: {e}")
            break


# ── 主循环 ──────────────────────────────────────────────────────────────────
def main():
    # 1. 加载 TensorRT 引擎
    print(f"[INFO] 加载 TensorRT 引擎: {ENGINE_PATH}")
    engine = TRTEngine(ENGINE_PATH)
    print(f"[INFO] 引擎输出 shape: {engine.output_shape}")

    # 2. 打开 USB 摄像头
    print(f"[INFO] 打开 USB 摄像头 /dev/video{CAM_INDEX} ...")
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开 USB 摄像头 /dev/video{CAM_INDEX}")
    print("[INFO] 摄像头就绪")

    # 3. 打开串口
    print(f"[INFO] 打开串口 {SERIAL_PORT} @ {SERIAL_BAUD} baud...")
    try:
        ser = serial.Serial(
            SERIAL_PORT,
            SERIAL_BAUD,
            timeout=SERIAL_TIMEOUT,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
    except Exception as e:
        print(f"[FATAL] 无法打开串口 {SERIAL_PORT}: {e}")
        print("[HINT] Jetson Nano UART 默认禁用，请先执行:")
        print("       sudo systemctl stop nvgetty")
        print(f"       sudo udevadm trigger {SERIAL_PORT}")
        print("       或使用 USB转TTL: SERIAL_PORT = '/dev/ttyUSB0'")
        raise

    # 清空缓冲区
    ser.reset_input_buffer()

    # 启动串口读取线程
    frame_queue: queue.Queue = queue.Queue()
    stop_event = threading.Event()
    reader_thread = threading.Thread(
        target=serial_reader, args=(ser, frame_queue, stop_event), daemon=True
    )
    reader_thread.start()
    print(f"[INFO] 串口就绪 ({SERIAL_PORT})，等待 STM32 触发帧...")

    # 4. 主循环：等待触发 → 抓帧 → 推理 → 回复
    dbg_none_count = 0
    processing = False  # 正在处理一帧时跳过额外触发
    try:
        while True:
            try:
                # 每 50ms 检查一次队列（避免长时间阻塞）
                result = frame_queue.get(timeout=0.05)
            except queue.Empty:
                dbg_none_count += 1
                if dbg_none_count % 20 == 0:
                    print(
                        f"[DBG] 已 {dbg_none_count} 次未收到帧, in_waiting={ser.in_waiting}"
                    )
                continue

            # 收到触发帧
            dbg_none_count = 0
            cmd, _, _ = result
            if cmd != FRAME_CMD_EXPECT:
                print(f"[WARN] 收到未知命令: 0x{cmd:02X}，跳过")
                continue

            # 防抖：正在处理时收到的触发帧直接丢弃
            if processing:
                print("[SKIP] 正在处理上一帧，跳过本次触发")
                continue

            processing = True
            print("[TRIG] 收到触发帧，开始抓取图片...")

            # 直接读取一帧
            ret, frame = cap.read()
            if not ret or frame is None:
                print("[WARN] 摄像头读帧失败，跳过")
                reply_frame = bytes(
                    [REPLY_HEADER, REPLY_NORMAL, REPLY_HEADER ^ REPLY_NORMAL]
                )
                ser.write(reply_frame)
                processing = False
                continue

            orig_h, orig_w = frame.shape[:2]

            # 预处理 + 推理
            blob, scale, pad = preprocess(frame)
            raw_outputs = engine.infer(blob)
            pred = raw_outputs[0].reshape(engine.output_shape)

            dets = postprocess(pred, scale, pad, orig_h, orig_w)
            print(f"[DET] 检测到 {len(dets)} 个目标")

            # 判定缺陷
            if DEFECT_CLASS_IDS:
                has_defect = any(
                    int(cls_id) in DEFECT_CLASS_IDS for _, _, cls_id in dets
                )
            else:
                has_defect = len(dets) > 0

            reply = REPLY_DEFECT if has_defect else REPLY_NORMAL
            # 发送 3 字节回复帧: BB STATUS XOR
            reply_frame = bytes([REPLY_HEADER, reply, REPLY_HEADER ^ reply])
            ser.write(reply_frame)
            print(
                f"[REPLY] → 0x{reply:02X} ({reply_frame.hex().upper()})  {'缺陷-剔除' if has_defect else '正常-放行'}"
            )

            processing = False

            # 调试显示
            if SHOW_WINDOW:
                frame = draw(frame, dets)
                cv2.putText(
                    frame,
                    f"Reply: 0x{reply:02X} | Dets: {len(dets)}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 200, 255),
                    2,
                )
                cv2.imshow("TRT Inference", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        print("\n[INFO] 用户中断")
    finally:
        stop_event.set()
        reader_thread.join(timeout=1)
        cap.release()
        ser.close()
        cv2.destroyAllWindows()
        print("[INFO] 资源已释放")


if __name__ == "__main__":
    main()
