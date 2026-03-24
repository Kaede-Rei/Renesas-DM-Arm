import wifi
import time
import random

if __name__ == "__main__":
    comms = wifi.WifiServer()
    comms.start()

    last_send_time = time.ticks_ms()

    while True:
        frames, mcu_ready = comms.process()

        for frame in frames:
            print("Received:", frame)

        now = time.ticks_ms()
        if mcu_ready and time.ticks_diff(now, last_send_time) >= 20:
            last_send_time = now

            # 写好坐标转换函数 -> weed_id, x, y, z, confidence = convert_to_data()
            weed_id = random.randint(0, 16)
            x = random.uniform(0.0, 3.0)
            y = random.uniform(0.0, 3.0)
            z = random.uniform(0.0, 3.0)
            confidence = random.random()

            packet = f"WEED:{weed_id},{x:.2f},{y:.2f},{z:.2f},{confidence:.2f}".encode()
            comms.send(packet)
