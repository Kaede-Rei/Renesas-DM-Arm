import network
import socket
import struct
import time

class WifiServer:
    def __init__(self, ssid='K230', key='12345678', header=b'RENE:', port=8080):
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
        addr = target_addr or self.client_addr
        if not addr:
            return False

        frame = self.header + struct.pack('>H', len(payload)) + payload
        try:
            self.sock.sendto(frame, addr)
            return True
        except OSError:
            return False

# 示例
#
# import wifi
# import time

# if __name__ == "__main__":
#     comms = wifi.WifiServer()
#     comms.start()

#     last_send_time = time.ticks_ms()

#     while True:
#         frames, mcu_ready = comms.process()
#         for f in frames:
#             print(f"[{time.time()}]收到 MCU 数据:{f}")
#             mcu_ready = True

#         now = time.ticks_ms()
#         if mcu_ready and time.ticks_diff(now, last_send_time) >= 20:
#             last_send_time = now
#             comms.send(b'VISION_DATA')s
