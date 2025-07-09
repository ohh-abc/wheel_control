#!/usr/bin/env python3
import os
import time
import threading
import pyaudio
import numpy as np
import websocket
import json
import base64
import hashlib
import hmac
from urllib.parse import urlencode
from datetime import datetime
from time import mktime
from wsgiref.handlers import format_date_time
import ssl

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 讯飞 API 参数
XFYUN_APPID = os.getenv('XFYUN_APPID')
XFYUN_APIKEY = os.getenv('XFYUN_APIKEY')
XFYUN_APISECRET = os.getenv('XFYUN_APISECRET')

# 音频参数
RATE_ORIGINAL = 44100
RATE_TARGET = 16000
CHANNELS = 1
FORMAT = pyaudio.paInt16
CHUNK = 1024
RESAMPLE_RATIO = RATE_ORIGINAL // RATE_TARGET

# 🛠 设置为你实际查到的 PyAudio 设备编号（与 ALSA card 0 对应）
INPUT_DEVICE_INDEX = 0  # 修改为你自己机器上查到的值


def print_device_list(p):
    print("🔍 可用音频输入设备列表：")
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        if info.get('maxInputChannels', 0) > 0:
            print(f"  [{i}] {info['name']} (输入通道数: {info['maxInputChannels']})")


class Ws_Param:
    def __init__(self, app_id, api_key, api_secret):
        self.APPID = app_id
        self.APIKey = api_key
        self.APISecret = api_secret
        self.CommonArgs = {"app_id": self.APPID}
        self.BusinessArgs = {
            "domain": "iat", "language": "zh_cn", "accent": "mandarin",
            "vinfo": 1, "vad_eos": 5000
        }

    def create_url(self):
        url = 'wss://ws-api.xfyun.cn/v2/iat'
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))
        signature_origin = f"host: ws-api.xfyun.cn\ndate: {date}\nGET /v2/iat HTTP/1.1"
        signature_sha = hmac.new(self.APISecret.encode('utf-8'),
                                 signature_origin.encode('utf-8'),
                                 digestmod=hashlib.sha256).digest()
        signature = base64.b64encode(signature_sha).decode('utf-8')
        authorization_origin = (
            f'api_key="{self.APIKey}", algorithm="hmac-sha256", '
            f'headers="host date request-line", signature="{signature}"'
        )
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode('utf-8')
        return url + '?' + urlencode({
            "authorization": authorization,
            "date": date,
            "host": "ws-api.xfyun.cn"
        })


def resample_audio(data):
    audio_data = np.frombuffer(data, dtype=np.int16)
    resampled = np.mean(audio_data.reshape(-1, RESAMPLE_RATIO), axis=1)
    return resampled.astype(np.int16).tobytes()


class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("语音控制节点已启动（指定设备模式）")
        self.stop_event = threading.Event()

        self.audio = pyaudio.PyAudio()
        print_device_list(self.audio)

        try:
            self.stream = self.audio.open(
                format=FORMAT,
                channels=CHANNELS,
                rate=RATE_ORIGINAL,
                input=True,
                input_device_index=INPUT_DEVICE_INDEX,
                frames_per_buffer=CHUNK
            )
        except Exception as e:
            self.get_logger().error(f"打开麦克风失败: {e}")
            return

        self.thread = threading.Thread(target=self.speech_loop, daemon=True)
        self.thread.start()

    def destroy_node(self):
        self.stop_event.set()
        if hasattr(self, 'stream'):
            self.stream.stop_stream()
            self.stream.close()
            self.audio.terminate()
        super().destroy_node()

    def handle_result(self, text):
        self.get_logger().info(f"识别到: {text}")
        msg = Twist()
        if "前进" in text:
            msg.linear.x = 0.2
        elif "后退" in text:
            msg.linear.x = -0.2
        elif "左转" in text:
            msg.angular.z = 0.5
        elif "右转" in text:
            msg.angular.z = -0.5
        elif "停止" in text:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            self.get_logger().info("未识别出控制指令")
            return
        self.publisher_.publish(msg)

    def speech_loop(self):
        while not self.stop_event.is_set():
            self.run_one_session()

    def run_one_session(self):
        ws_param = Ws_Param(XFYUN_APPID, XFYUN_APIKEY, XFYUN_APISECRET)
        ws_url = ws_param.create_url()
        audio_buffer = bytearray()
        recognition_complete = threading.Event()

        def on_message(ws, message):
            msg = json.loads(message)
            if msg.get("code", -1) != 0:
                self.get_logger().error(f"识别失败: {msg.get('message')}")
                recognition_complete.set()
                return
            data = msg.get("data", {})
            words = []
            for item in data.get("result", {}).get("ws", []):
                for w in item.get("cw", []):
                    words.append(w.get("w", ''))
            sentence = "".join(words)
            if sentence:
                self.handle_result(sentence)
            if data.get("status") == 2:
                recognition_complete.set()

        def on_open(ws):
            ws.send(json.dumps({
                "common": ws_param.CommonArgs,
                "business": ws_param.BusinessArgs,
                "data": {
                    "status": 0,
                    "format": "audio/L16;rate=16000",
                    "audio": "",
                    "encoding": "raw"
                }
            }))

            def send_audio():
                while not recognition_complete.is_set() and not self.stop_event.is_set():
                    raw_data = self.stream.read(CHUNK, exception_on_overflow=False)
                    resampled = resample_audio(raw_data)
                    audio_buffer.extend(resampled)
                    if len(audio_buffer) >= 2048:
                        chunk = bytes(audio_buffer)
                        audio_buffer.clear()
                        ws.send(json.dumps({
                            "data": {
                                "status": 1,
                                "format": "audio/L16;rate=16000",
                                "audio": base64.b64encode(chunk).decode('utf-8'),
                                "encoding": "raw"
                            }
                        }))
                    time.sleep(0.02)
                ws.send(json.dumps({
                    "data": {"status": 2, "format": "audio/L16;rate=16000", "audio": "", "encoding": "raw"}
                }))

            threading.Thread(target=send_audio, daemon=True).start()

        ws = websocket.WebSocketApp(
            ws_url,
            on_open=on_open,
            on_message=on_message,
            on_error=lambda ws, err: self.get_logger().error(f"WebSocket错误: {err}"),
            on_close=lambda ws, code, msg: self.get_logger().info("识别结束，等待下一轮命令...")
        )

        ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})
        time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点手动中断")
    finally:
        node.get_logger().info("退出语音控制节点")
        node.destroy_node()
        rclpy.shutdown()
