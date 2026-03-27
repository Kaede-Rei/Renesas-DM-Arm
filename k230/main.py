import gc
import os
import time
import ujson
import ulab.numpy as np
import image
import nncase_runtime as nn
import aicube
import wifi  # 队友写的WiFi通信库
from machine import Pin  # 新增：导入Pin用于控制激光

from libs.PipeLine import ScopedTiming
from libs.Utils import *
from media.display import *
from media.media import *
from media.sensor import *

# ==================== 全局配置 ====================
root_path = "/sdcard/mp_deployment_source/"  # root_path要以/结尾
config_path = root_path + "deploy_config.json"
image_path = "/sdcard/mp_deployment_source/test.jpg"  # 静态识别的输入图片路径
debug_mode = 1

# 【标定矩阵】：透视变换单应性矩阵 (3x3)
# 注意：这个矩阵需要你们在电脑上用 cv2.getPerspectiveTransform 算好后填入这里！
# 这里暂时填入一个单位矩阵作为占位符
TRANSFORM_MATRIX = [
                    [1.98453608, 0.00000000, -317.52577320],
                    [-0.00000000, -1.15853659, 550.81707317],
                    [-0.00000000, -0.00000000, 1.00000000]
]

# 列宽容忍度(毫米)：用于判断两株草是否在"同一列"
COLUMN_TOLERANCE_MM = 50.0

# ==================== 激光控制配置 ====================
LASER_PIN = 2  # 庐山派的Pin7引脚连接继电器的IN端
laser_pin = None  # 激光引脚对象，将在初始化时创建
laser_ready = False  # 激光就绪标志


# ==================== 核心功能函数 ====================
# 测试激光函数
#if __name__ == "__main__":
#    # 先测试激光控制
#    print(">>> 激光控制测试开始...")

#    # 初始化引脚
#    test_pin = Pin(7, Pin.OUT)

#    print("测试1: 设置低电平(0)")
#    test_pin.value(1)
#    time.sleep(2)

#    print("测试2: 设置高电平(1)")
#    test_pin.value(0)
#    time.sleep(2000)

#    print(">>> 测试完成")
#    print("观察结果：")
#    print("- 如果低电平时激光亮，高电平时灭 → 低电平触发")
#    print("- 如果高电平时激光亮，低电平时灭 → 高电平触发")

#    # 最终关闭激光
#    test_pin.value(1)  # 尝试关闭
#    time.sleep(1)

#    # 继续原来的代码...
def read_img(img_path):
    """保持原状：读取图像并转换维度"""
    img_data = image.Image(img_path)
    img_data_rgb888 = img_data.to_rgb888()
    img_hwc = img_data_rgb888.to_numpy_ref()
    shape = img_hwc.shape
    img_tmp = img_hwc.reshape((shape[0] * shape[1], shape[2]))
    img_tmp_trans = img_tmp.transpose()
    img_res = img_tmp_trans.copy()
    img_return = img_res.reshape((shape[2], shape[0], shape[1]))
    return img_return

def two_side_pad_param(input_size, output_size):
    """保持原状：图像缩放与边缘填充参数计算"""
    ratio_w = output_size[0] / input_size[0]
    ratio_h = output_size[1] / input_size[1]
    ratio = min(ratio_w, ratio_h)
    new_w = int(ratio * input_size[0])
    new_h = int(ratio * input_size[1])
    dw = (output_size[0] - new_w) / 2
    dh = (output_size[1] - new_h) / 2
    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw - 0.1))
    return top, bottom, left, right, ratio

def read_deploy_config(config_path):
    """保持原状：读取模型配置文件"""
    with open(config_path, "r") as json_file:
        try:
            config = ujson.load(json_file)
        except ValueError as e:
            print("JSON 解析错误:", e)
    return config

def pixel_to_physical(u, v, matrix):
    """
    新增功能：将像素坐标(u, v)转换为物理世界毫米坐标(X, Y)
    原理：手动实现透视变换矩阵乘法，摆脱 cv2 依赖
    """
    m11, m12, m13 = matrix[0]
    m21, m22, m23 = matrix[1]
    m31, m32, m33 = matrix[2]

    # 齐次坐标计算分母
    w = m31 * u + m32 * v + m33
    if w == 0:
        return 0.0, 0.0

    # 计算物理 X 和 Y
    physical_x = (m11 * u + m12 * v + m13) / w
    physical_y = (m21 * u + m22 * v + m23) / w

    return physical_x, physical_y

def sort_weeds(weed_list, col_tolerance):
    """
    新增功能：对杂草进行栅格化分桶排序 (先从左到右，再从上到下)
    weed_list 元素格式: {'id': id, 'conf': conf, 'x': physical_x, 'y': physical_y}
    """
    if not weed_list:
        return []

    # 1. 首先按物理 X 坐标（从左到右）进行全局粗排序
    weed_list.sort(key=lambda item: item['x'])

    columns = []
    current_column = [weed_list[0]]

    # 2. 分桶：把 X 坐标相近（在 col_tolerance 毫米内）的杂草划分为同一列
    for target in weed_list[1:]:
        if abs(target['x'] - current_column[0]['x']) <= col_tolerance:
            current_column.append(target)
        else:
            columns.append(current_column)
            current_column = [target]
    columns.append(current_column)

    # 3. 排序：对每一个分好的列，按 Y 坐标（从上到下）进行精确排序
    sorted_weeds = []
    for col in columns:
        col.sort(key=lambda item: item['y'])
        sorted_weeds.extend(col)

    return sorted_weeds

def init_laser():
    """初始化激光控制引脚"""
    global laser_pin
    try:
        laser_pin = Pin(LASER_PIN, Pin.OUT)
        laser_pin.value(0)  # 初始状态：关闭激光（根据继电器高低电平触发调整，假设低电平关闭）
        print(f">>> 激光控制引脚初始化成功 (Pin{LASER_PIN})")
        return True
    except Exception as e:
        print(f">>> 激光控制引脚初始化失败: {e}")
        return False

def control_laser(state):
    """
    控制激光开关
    state: True 打开激光, False 关闭激光
    """
    global laser_pin
    if laser_pin is None:
        print(">>> 错误: 激光引脚未初始化")
        return False

    try:
        laser_pin.value(1 if state else 0)  # 假设高电平触发继电器打开激光
        print(f">>> 激光已{'打开' if state else '关闭'}")
        return True
    except Exception as e:
        print(f">>> 激光控制失败: {e}")
        return False

def run_static_detection():
    """
    重构：单次静态识别。提取图像中的杂草并返回排序后的物理坐标列表
    """
    print(">>> 启动 K230 静态视觉解析引擎...")
    deploy_conf = read_deploy_config(config_path)
    kmodel_name = deploy_conf["kmodel_path"]
    confidence_threshold = deploy_conf["confidence_threshold"]
    nms_threshold = deploy_conf["nms_threshold"]
    model_input_size = deploy_conf["img_size"]
    num_classes = deploy_conf["num_classes"]
    nms_option = deploy_conf["nms_option"]
    model_type = deploy_conf["model_type"]
    if model_type == "AnchorBaseDet":
        anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]
    strides = [8, 16, 32]

    # 获取图像数据 (如果是接摄像头，这里替换为抓取一帧的函数)
    ai2d_input = read_img(image_path)
    frame_size = [ai2d_input.shape[2], ai2d_input.shape[1]]
    ai2d_input_tensor = nn.from_numpy(ai2d_input)
    ai2d_input_shape = ai2d_input.shape
    data = np.ones((1, 3, model_input_size[1], model_input_size[0]), dtype=np.uint8)
    ai2d_output_tensor = nn.from_numpy(data)

    # KPU及AI2D初始化
    kpu = nn.kpu()
    kpu.load_kmodel(root_path + kmodel_name)
    ai2d = nn.ai2d()
    ai2d.set_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
    top, bottom, left, right, ratio = two_side_pad_param(frame_size, model_input_size)
    ai2d.set_pad_param(True, [0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
    ai2d_builder = ai2d.build(
        [1, 3, ai2d_input_shape[1], ai2d_input_shape[2]], [1, 3, model_input_size[1], model_input_size[0]]
    )

    raw_weeds = []

    with ScopedTiming("模型单次推理", debug_mode > 0):
        ai2d_builder.run(ai2d_input_tensor, ai2d_output_tensor)
        kpu.set_input_tensor(0, ai2d_output_tensor)
        kpu.run()

        results = []
        for i in range(kpu.outputs_size()):
            out_data = kpu.get_output_tensor(i)
            results.append(out_data.to_numpy())
            del out_data

        # NMS 后处理提取坐标
        det_boxes = aicube.anchorbasedet_post_process(
            results[0], results[1], results[2],
            model_input_size, frame_size, strides, num_classes,
            confidence_threshold, nms_threshold, anchors, nms_option
        )

        if det_boxes:
            for box in det_boxes:
                # 提取目标信息
                class_id = int(box[0])
                conf = float(box[1])
                x1, y1, x2, y2 = box[2], box[3], box[4], box[5]

                # 1. 计算包围盒中心像素坐标 (u, v)
                center_u = (x1 + x2) / 2.0
                center_v = (y1 + y2) / 2.0

                # 2. 核心：将像素坐标转换为物理毫米坐标 (X, Y)
                phys_x, phys_y = pixel_to_physical(center_u, center_v, TRANSFORM_MATRIX)

                # 存入列表
                raw_weeds.append({
                    'id': class_id,
                    'conf': conf,
                    'x': phys_x,
                    'y': phys_y
                })
        else:
            print("未检测到任何杂草目标。")

    # 释放内存
    del ai2d_input_tensor
    del ai2d_output_tensor
    nn.shrink_memory_pool()
    gc.collect()

    # 3. 核心：执行栅格化分桶排序
    sorted_weeds = sort_weeds(raw_weeds, COLUMN_TOLERANCE_MM)

    print(f">>> 静态解析完成！共发现 {len(sorted_weeds)} 株杂草，已完成空间路径排序。")
    return sorted_weeds


# ==================== 主控与通信流 ====================

if __name__ == "__main__":

    # 初始化激光控制引脚
    if not init_laser():
        print(">>> 警告: 激光控制功能初始化失败，将无法控制激光")

    control_laser(False)

    # 1. [通信层]：先初始化 WiFi 实例并开启服务
    print(">>> 正在启动 WiFi 透传服务...")
    comms = wifi.WifiServer()
    comms.start()

    # 等待WiFi稳定
    time.sleep(1)
    print(">>> WiFi 服务已启动，IP: 192.168.169.1, 端口: 8080")
    print(">>> 等待瑞萨端连接...")

    # 2. [感知层]：执行单次静态解析，获取排好序的绝对物理坐标列表
    target_queue = run_static_detection()

    last_send_time = time.ticks_ms()
    current_target_index = 0
    total_targets = len(target_queue)

    # 3. 立即开始发送所有坐标（不等待mcu_ready）
    print(f">>> 开始发送 {total_targets} 个杂草坐标...")

    # 用于记录是否已发送完所有坐标
    all_sent = False
    # 用于记录是否已发送激光就绪响应
    laser_responded = False

    print(">>> 进入主循环，持续监听瑞萨消息并发送坐标...")
    while True:
        # 维持心跳，并解析 MCU 传来的状态
        frames, mcu_ready = comms.process()  # mcu_ready 参数在这里不再使用，但为了兼容保留

        # 打印调试信息（可选）
        for frame in frames:
            print("收到 MCU 反馈:", frame)

            # 检测是否收到激光就绪命令
            if isinstance(frame, (bytes, bytearray)) and frame.strip() == b"Ready to Laser":
                print(">>> 收到瑞萨端激光就绪命令，正在打开激光...")
                if control_laser(True):  # 打开激光
                    # 发送激光OK确认
                    time.sleep(2)
                    control_laser(False)

                    laser_ok_packet = b"Laser OK"
                    comms.send(laser_ok_packet)
                    print(">>> 已发送 Laser OK 确认")
                    laser_responded = True
                else:
                    print(">>> 激光打开失败，未发送确认")

        now = time.ticks_ms()

        # 发送坐标，等待 mcu_ready，控制发送间隔
        if mcu_ready and not all_sent and time.ticks_diff(now, last_send_time) >= 5000:
            last_send_time = now

            if current_target_index < total_targets:
                # 取出当前队列最前面的杂草
                weed = target_queue[current_target_index]

                # 按照队友定义的报文格式组装数据 (WEED:id,x,y,z,confidence)
                # Z轴传0.00，因为我们在平面沙盘上
                packet = f"WEED:{weed['id']},{(weed['x'] / 1000 ):.2f},{(weed['y'] / 1000):.2f},0.10,{weed['conf']:.2f}".encode()

                # 调用队友的 send 接口发送
                comms.send(packet)
                print(f"[{current_target_index+1}/{total_targets}] 发送坐标 -> {packet.decode()}")

                current_target_index += 1

            elif current_target_index == total_targets and not all_sent:
                # 当所有杂草发送完毕后，发送结束标志帧给瑞萨
                packet = "WEED:0xFF,0.00,0.00,0.00,0.00".encode()
                comms.send(packet)
                print(">>> 所有目标派发完毕，已发送结束帧 0xFF。")
                all_sent = True

        # 坐标发送完后，继续监听瑞萨的激光命令
        # 这里添加一个简单的休眠，避免CPU占用过高
        time.sleep_ms(10)
