# 帧格式说明:
#     [header] + [payload_len(2字节)] + [payload]
#     - header: 固定帧头，默认为 b'RENE:'
#     - payload_len: 负载长度，大端序无符号短整型
#     - payload: 业务数据，长度由 payload_len 指定
#
# 心跳机制:
#     当收到 payload 为 b'HEART' 的心跳帧时，服务器自动回复 b'ALIVE' 响应，
#     并将 ready 标志置为 True，表示客户端已就绪
#
# API:
# 1. __init__(ssid='K230', key='12345678', header=b'RENE:', port=8080)
#     构造函数，初始化服务器配置参数。
#     参数:
#         - ssid: Wi-Fi 热点名称
#         - key: Wi-Fi 密码
#         - header: 帧头标识，用于数据帧同步
#         - port: UDP 监听端口
#     用法: comms = WifiServer()
#
# 2. start(ip='192.168.169.1')
#     启动 Wi-Fi 热点和 UDP 服务器。
#     参数:
#         - ip: 服务器绑定的 IP 地址
#     说明:
#         该方法会创建 AP 接口并激活，等待 AP 启动成功后绑定 UDP 套接字，
#         并将套接字设置为非阻塞模式。AP 的 IP 地址可通过 ifconfig 获取
#     用法: comms.start()
#
# 3. process()
#     处理接收到的 UDP 数据，解析完整的数据帧
#     返回值:
#         - business_frames: 列表，包含解析出的所有业务数据帧（排除心跳帧）
#         - ready: 布尔值，表示客户端是否已通过心跳就绪
#     说明:
#         该方法应在主循环中周期性调用。它会从接收缓冲区中提取数据，根据帧头
#         和长度字段解析出完整帧，对心跳帧自动响应，其他帧作为业务数据返回
#         未完成的帧会保留在内部缓冲区中，等待下次调用时继续处理
#     用法: frames, mcu_ready = comms.process()
#
# 4. send(payload, target_addr=None)
#     向客户端发送数据帧。
#     参数:
#         - payload: 待发送的业务数据，字节串类型
#         - target_addr: 目标地址元组 (ip, port)，默认为 None 时使用最近通信的客户端地址
#     返回值:
#         - bool: 发送成功返回 True，失败返回 False
#     说明:
#         发送的数据会自动添加帧头和长度字段。如果尚未与任何客户端通信过且
#         target_addr 为 None，发送会失败
#     用法: comms.send(b'VISION_DATA')
#
# 使用示例:
#     import wifi
#     import time
#
#     # 创建服务器实例
#     comms = wifi.WifiServer(ssid='MyDevice', key='12345678', port=9000)
#
#     # 启动服务器
#     comms.start(ip='192.168.1.1')
#
#     last_send_time = time.ticks_ms()
#
#     while True:
#         # 处理接收数据
#         frames, mcu_ready = comms.process()
#
#         # 处理业务数据
#         for frame in frames:
#             print("Received:", frame)
#
#         # 当客户端就绪后，每 20 毫秒发送一次数据
#         now = time.ticks_ms()
#         if mcu_ready and time.ticks_diff(now, last_send_time) >= 20:
#             last_send_time = now
#             comms.send(b'VISION_DATA')
#
# 注意事项:
#     1. 该模块依赖 network 和 socket 库，适用于支持 MicroPython 的嵌入式平台
#     2. process() 方法不会阻塞，需要在主循环中频繁调用以保证数据及时处理
#     3. 心跳响应是自动完成的，无需用户代码干预
#     4. 发送数据时若 target_addr 为 None，将使用最近一次收到数据的客户端地址

import network
import socket
import struct
import time

class WifiServer:
    def __init__(self, ssid='K230', key='12345678', header=b'RENE:', port=8080):
        """
        初始化 Wi-Fi 服务器配置参数
        
        Args:
            - ssid (str): Wi-Fi 热点名称，默认为 'K230'
            - key (str): Wi-Fi 密码，默认为 '12345678'
            - header (bytes): 数据帧头标识，默认为 b'RENE:'
            - port (int): UDP 监听端口，默认为 8080
        """
        self.ssid = ssid
        self.key = key
        self.header = header
        self.port = port

        self.ap = None
        self.sock = None
        self.client_addr = None
        self.rx_buf = bytearray()

        self.ready = False

    def start(self, ip='192.168.169.1'):
        """
        启动 Wi-Fi 热点和 UDP 服务器

        Args:
            - ip (str): 服务器绑定的 IP 地址，默认为 '192.168.169.1'
        """
        self.ap = network.WLAN(network.AP_IF)
        self.ap.active(True)
        self.ap.config(ssid=self.ssid, key=self.key)
        while not self.ap.active():
            time.sleep(0.1)
        print('AP 启动成功, IP:', self.ap.ifconfig()[0])

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, self.port))
        self.sock.setblocking(False)
        print(f'UDP Server 监听端口 {self.port}')

    def process(self):
        """
        处理接收到的 UDP 数据，解析完整的数据帧

        Returns:
            - business_frames (list): 包含解析出的所有业务数据帧（排除心跳帧）
            - ready (bool): 表示客户端是否已通过心跳就绪
        """
        try:
            while True:
                data, addr = self.sock.recvfrom(1024)
                if data:
                    self.client_addr = addr
                    self.rx_buf.extend(data)
                else:
                    break
        except OSError:
            pass

        business_frames = []
        while len(self.rx_buf) >= len(self.header) + 2:
            idx = self.rx_buf.find(self.header)
            if idx == -1:
                if len(self.rx_buf) > len(self.header):
                    self.rx_buf =  self.rx_buf[-len(self.header):]
                break

            if len(self.rx_buf) < idx + len(self.header) + 2:
                break

            start = idx + len(self.header)
            payload_len = struct.unpack('>H', self.rx_buf[start:start+2])[0]

            total_frame_len = len(self.header) + 2 + payload_len
            if len(self.rx_buf) < idx + total_frame_len:
                break

            payload = self.rx_buf[start+2 : idx+total_frame_len]

            if payload == b'HEART':
                self.send(b'ALIVE')
                self.ready = True
            else:
                business_frames.append(payload)

            self.rx_buf = self.rx_buf[idx+total_frame_len:]

        return business_frames, self.ready

    def send(self, payload, target_addr=None):
        """
        向客户端发送数据帧

        Args:
            - payload (bytes): 待发送的业务数据，字节串类型
            - target_addr (tuple): 目标地址元组 (ip, port)，默认为 None 时使用最近通信的客户端地址

        Returns:
            - bool: 发送成功返回 True，否则返回 False
        """
        addr = target_addr or self.client_addr
        if not addr:
            return False

        frame = self.header + struct.pack('>H', len(payload)) + payload
        try:
            self.sock.sendto(frame, addr)
            return True
        except OSError:
            return False
