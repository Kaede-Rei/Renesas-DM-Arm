import network
import socket
import struct
import time


class FrameTCPServer:
    def __init__(self, ssid='K230', key='12345678', frame_header=b'WEED:', max_payload=512):
        """
        Args:
            ssid (str): WiFi SSID，默认 'K230'
            key (str): WiFi 密码，默认 '12345678'
            frame_header (bytes): 帧头标识，默认 b'WEED:'
            max_payload (int): 最大有效载荷长度，默认 512 字节        
        """
        self.ssid = ssid
        self.key = key

        self.frame_header = frame_header
        self.header_len = len(frame_header)
        self.max_payload = max_payload

        self.ap = None
        self.srv = None
        self.socket = None

        self.tx_buffer = bytearray()
        self.rx_buffer = bytearray()
        self.latest_frame = None

        self.last_send_time = 0
        self.send_interval = 0.02

    def start(self, ip='192.168.169.1', port=8080):
        """
        启动 WiFi AP 和 TCP 服务器

        Args:
            ip (str): 服务器 IP 地址，默认 '192.168.169.1'
            port (int): 服务器端口，默认 8080
        """
        self.ap = network.WLAN(network.AP_IF)
        self.ap.active(True)
        self.ap.config(ssid=self.ssid, key=self.key)

        while not self.ap.active():
            time.sleep(0.1)
        print('AP 已激活:', self.ap.ifconfig()[0])

        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((ip, port))
        self.srv.listen(1)
        self.srv.setblocking(None)
        print('Server 已启动')

    def is_connected(self):
        """
        检查是否有客户端连接

        Returns:
            bool: True 如果有客户端连接，否则 False
        """
        return self.socket is not None

    def process(self):
        """
        处理连接、接收数据并解析帧
        """
        self._accept()
        self._recv()
        self._parse_frames()
        self._send_flush()

    def recv(self):
        """
        接收解析后的帧数据

        Returns:
            bytes: 帧数据，如果没有可用数据则返回 None
        """
        frame = self.latest_frame
        self.latest_frame = None

        return frame

    def send(self, payload):
        """
        发送帧数据

        Args:
            payload (bytes): 要发送的有效载荷数据
        Returns:
            bool: True 如果发送成功，否则 False
        """
        if not self.socket:
            return False

        frame = self.frame_header + struct.pack('>H', len(payload)) + payload
        self.tx_buffer.extend(frame)

        return True

    def _accept(self):
        """
        接受新的客户端连接
        """
        try:
            new_sock, addr = self.srv.accept()
            
            if self.socket:
                print('检测到新连接，强制关闭旧连接:', addr)
                self._close()
            
            new_sock.setblocking(False)
            self.socket = new_sock
            print('新客户端已连接:', addr)
        except OSError as e:
            if e.args[0] not in (11, 35):
                print("Accept Error:", e)

    def _send_flush(self):
        if not self.socket or not self.tx_buffer:
            return

        now = time.ticks_ms()

        if time.ticks_diff(now, self.last_send_time) < int(self.send_interval * 1000):
            return

        try:
            sent = self.socket.send(self.tx_buffer)
            if sent > 0:
                self.tx_buffer = self.tx_buffer[sent:]
                self.last_send_time = now
        except OSError as e:
            if e.args[0] != 11:
                self._close()

    def _recv(self):
        """
        从客户端接收数据并存入接收缓冲区
        """
        if not self.socket:
            return

        try:
            data = self.socket.recv(256)
            if data:
                self.rx_buffer.extend(data)
            else:
                pass
        except OSError as e:
            if e.args[0] != 11:
                self._close()

    def _parse_frames(self):
        """
        从接收缓冲区解析完整帧并存入帧队列
        """
        buf = self.rx_buffer

        while True:
            idx = buf.find(self.frame_header)
            if idx < 0:
                if len(buf) > self.header_len:
                    del buf[:-self.header_len]
                return

            if len(buf) < idx + self.header_len + 2:
                return

            start = idx + self.header_len
            payload_len = struct.unpack('>H', buf[start:start+2])[0]

            if payload_len == 0 or payload_len > self.max_payload:
                print('[WARN] 非法长度:', payload_len)
                del buf[:start+2]
                continue

            frame_end = start + 2 + payload_len
            if len(buf) < frame_end:
                return

            payload = bytes(buf[start+2:frame_end])
            self.latest_frame = payload

            self.rx_buffer = self.rx_buffer[frame_end:]
            buf = self.rx_buffer

    def _close(self):
        """
        关闭当前连接
        """
        if self.socket:
            print('连接关闭')
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        
        self.tx_buffer = bytearray()
        self.rx_buffer = bytearray()
        self.latest_frame = None
