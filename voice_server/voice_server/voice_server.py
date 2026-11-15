#!/usr/bin/env python3
import math
import os
import random
import re
import time
import datetime
import wave
import threading
import io
import ctypes
import concurrent.futures
import numpy as np

# from Third_Party.msi_amr_ros2.Nav2_Amcl.goto_aruco.goto_aruco.GoToArucoActionServer import GoToArucoActionServer
# from Third_Party.msi_amr_ros2.Nav2_Amcl.goto_aruco.goto_aruco.GoToArucoActionClient import GoToArucoActionClient
# from Third_Party.msi_amr_ros2.Nav2_Amcl.ros2_aruco.ros2_aruco.ros2_aruco.aruco_node import ArucoNode
# from ros2_aruco.aruco_node import ArucoNode
# from goto_aruco.GoToArucoActionClient import GoToArucoActionClient
# from goto_aruco.GoToArucoActionServer import GoToArucoActionServer
# from goto_aruco_action_interface.action import GotoAruco

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from action_msgs.msg import GoalStatus

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped # 導航座標格式
from nav2_msgs.action import NavigateToPose, BackUp # Nav2 導航 Action

from cv_bridge import CvBridge

# 語音、視覺與 AI 相關套件
import cv2
import torch
import pyaudio
import sounddevice as sd
from pydub import AudioSegment
from pymouth import VTSAdapter, DBAnalyser

from ultralytics import YOLO
from google import genai
from google.cloud import speech, texttospeech
import requests
import openwakeword
from geometry_msgs.msg import Twist # 確保有引入這個

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

# ⚠️ 請替換成你新 Package 中實際定義的 Action
from breakfast_deliver_interfaces.action import VoiceTask 
from ament_index_python.packages import get_package_share_directory

# --- 屏蔽 ALSA 報錯 ---
ERROR_HANDLER_FUNC = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p)
def py_error_handler(filename, line, function, err, fmt): pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
try:
    asound = ctypes.cdll.LoadLibrary('libasound.so.2')
    asound.snd_lib_error_set_handler(c_error_handler)
except OSError:  # 🔴 修正：指定異常類型而不是裸 except
    pass

# --- 全域參數設定 ---
# ⚠️ 注意：這裡的路徑要改成你新 package 的資料夾名稱
pkg_share_dir = get_package_share_directory('breakfast_deliver') 

GOOGLE_GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
WAKEWORD_MODEL_PATH = os.path.join(pkg_share_dir, 'wakeword', 'WakeWord', 'mei.onnx')
YOLO_MODEL_PATH = os.path.join(pkg_share_dir, 'wakeword', 'yolov10n.pt')


class DeliveryBrain(Node):
    def __init__(self):
        super().__init__('delivery_brain_node')

        # 1. 系統狀態鎖 - 🌟 重要的狀態管理機制
        #    ============================================
        #    - system_active: 整個系統是否啟動（由 RQT 按鈕控制）
        #    - is_processing: 是否正在與客人互動或處理中（防止喚醒詞/視覺重複觸發）
        #    - is_navigating: 是否正在送餐導航中
        #    - post_delivery_mode: 是否在送餐後的互動模式（講笑話/查天氣）
        #    - is_docking: 是否正在進行 ArUco 精準對位
        #    - last_visual_trigger_time: 上一次視覺觸發的時間（用於冷卻機制）
        #
        #    🔴 重要規則：
        #    1. is_processing 解鎖時必須同時更新 last_visual_trigger_time（防止立即重複觸發）
        #    2. 異常情況下（導航失敗、ArUco失敗、語音異常）也要完全解鎖所有鎖
        #    3. post_delivery_mode 結束時要呼叫 delivery_service("home") 自動回家
        self.system_active = False
        self.is_processing = False
        self.is_navigating = False   # 🌟 新增：送餐移動狀態鎖
        self.post_delivery_mode = False  # 🌟 新增：post-delivery 對話模式標記
        self.conversation_count = 0
        self.api_executor = concurrent.futures.ThreadPoolExecutor(max_workers=3)

        self.last_visual_trigger_time = 0
        self.visual_cooldown = 30 # 抵達後冷卻一下再打招呼
        self.frame_count = 0
        self.detect_every_n_frames = 5
        self.last_detections = []  # 🌟 新增：用來記住上一次的偵測結果
        self.latest_depth_img = None
        self.mic_free_event = threading.Event()
        self.mic_free_event.set()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_location = "home" # 🌟 新增：記錄目前在哪裡，預設在 home 基地
        self.current_step_origin = "home"

        # 新增：用來存放當前任務的步驟佇列
        self.current_route_steps = []
        self.target_destination = ""

        # 🌟 2. 餐桌地圖座標字典 (x, y, 旋轉四元數 w)
        # ⚠️ 請替換成你實際場地的座標！
        self.table_poses = {
             "1": {"x": 1.53, "y": -2.299, "z": 0.962, "w": 0.27},
            #"1": {"x": 1.426, "y": -2.555, "z": 0.962, "w": 0.27},
            "2": {"x": -0.4215, "y": -1.375, "z": 0.962, "w": 0.27},
            #"3": {"x": 0.02-1.9087, "y": -6.095+1.6088, "z": -0.218, "w": 0.975},
            "3": {"x": -1.518, "y": -4.528, "z": -0.218, "w": 0.975},
            "4": {"x": 2.174-1.9087, "y": -6.91+1.6088, "z": -0.218, "w": 0.975},    
            #"home": {"x": 1.824, "y": -0.206, "z": 0.962, "w": 0.27},
            "home": {"x": 2.019, "y": -0.3599, "z": 0.962, "w": 0.27},
            # 👇 新增四個角落的中繼站座標 (請替換為實際座標) L:下  R:上
            "TL_corner": {"x": 4.36-1.9087, "y": -4.196+1.6088, "z": 0.962, "w": 0.27}, 
            # "BL_corner": {"x": 0.597, "y": -2.623, "z": -0.218, "w": 0.975},
            "BL_corner": {"x": -1.589, "y": -1.04, "z": 0.962, "w": 0.27}, # 改設得離桌子比較遠一點
            "TR_corner": {"x": 3.216-1.9087, "y": -7.625+1.6088, "z": -0.218, "w": 0.975},
            # "BR_corner": {"x": -1.119, "y": -5.435, "z": 0.977, "w": 0.21}
            "BR_corner": {"x": -3.323, "y": -3.552, "z": -0.218, "w": 0.975},
            "Center": {"x": -1.13-1.9087, "y": -3.83+1.6088, "z": 0.53, "w": 0.84},
            #"Backup_pos": {"x": 0.654, "y": 0.353, "z": 0.962, "w": 0.27}, 
            "Backup_pos": {"x": 0.9655, "y": 0.3, "z": 0.962, "w": 0.27}, 
        }

        # 動作定義：
        # "nav"    -> 使用 NavigateToPose，允許轉向 (用於 Standby 之間與直行)
        # "backup" -> 使用 Backup Action，純直線倒車 (絕不轉彎)
        
        self.routes = {
            # === 從 Home 出發 (假設 Home 在底部中央) ===
            ("home", "1"): [
                {"action": "forward_cmdvel", "target": "Backup_pos"}, # 先到備用點，調整方向
                {"action": "nav", "target": "BL_corner"}, # 先到左下 Standby 轉向朝上
                {"action": "backup", "target": "1"}     # 🌟 倒車進 1 號桌
            ],
            ("home", "2"): [
                {"action": "forward_cmdvel", "target": "Backup_pos"}, # 先到備用點，調整方向
                {"action": "nav", "target": "BL_corner"}, 
                {"action": "backup", "target": "2"}     # 🌟 倒車進 2 號桌
            ],
            ("home", "3"): [
                {"action": "forward_cmdvel", "target": "Backup_pos"}, # 先到備用點，調整方向
                {"action": "nav", "target": "BR_corner"}, # 到右下 Standby 轉向朝下
                {"action": "nav", "target": "3"}  
            ],
            ("home", "4"): [
                {"action": "forward_cmdvel", "target": "Backup_pos"}, # 先到備用點，調整方向
                {"action": "nav", "target": "BR_corner"},
                # {"action": "nav", "target": "TR_corner"}, # 繞到右上 Standby 轉向朝下
                # {"action": "nav", "target": "4"}     # 直走進 4 號
                {"action": "nav", "target": "4"}
            ],

            # === 從 1 號桌出發 (車頭朝上) ===
            ("1", "home"): [
                #{"action": "backup", "target": "TL_corner"}, 
                #{"action": "nav", "target": "BL_corner"}, 
                {"action": "nav", "target": "Backup_pos"},
                {"action": "backup_cmdvel", "target": "home"}
            ],
            ("1", "2"): [
                {"action": "backup", "target": "TL_corner"}, 
                # {"action": "nav", "target": "TL_corner"}, # 往前開出站到左上
                # {"action": "nav", "target": "BL_corner"}, # 繞回左下
                # {"action": "nav", "target": "2"}     # 重新直走進 2 號
                {"action": "nav", "target": "2"}     
            ],
            ("1", "3"): [
                {"action": "backup", "target": "TL_corner"}, 
                #{"action": "nav", "target": "TL_corner"}, 
                {"action": "nav", "target": "BL_corner"},
                {"action": "nav", "target": "BR_corner"}, # 繞到右下
                {"action": "nav", "target": "3"}  # 倒車進 3 號
            ],
            ("1", "4"): [
                {"action": "backup", "target": "TL_corner"}, 
                #{"action": "nav", "target": "TL_corner"}, 
                {"action": "nav", "target": "BL_corner"},
                #{"action": "nav", "target": "BL_corner"}, 
                {"action": "nav", "target": "BR_corner"}, 
                #{"action": "nav", "target": "TR_corner"}, 
                #{"action": "nav", "target": "4"}
                {"action": "nav", "target": "4"}
            ],

            # === 從 2 號桌出發 (車頭朝上) ===
            ("2", "home"): [
                #{"action": "nav", "target": "BL_corner"}, 
                {"action": "nav", "target": "Backup_pos"},
                {"action": "backup_cmdvel", "target": "home"}
            ],
            ("2", "1"): [
                {"action": "backup", "target": "1"} 
            ],
            ("2", "3"): [
                {"action": "nav", "target": "BL_corner"}, 
                {"action": "nav", "target": "BR_corner"}, 
                {"action": "nav", "target": "3"}
            ],
            ("2", "4"): [
                {"action": "nav", "target": "BL_corner"}, 
                {"action": "nav", "target": "BR_corner"}, 
                #{"action": "nav", "target": "TR_corner"}, 
                #{"action": "nav", "target": "4"}
                {"action": "nav", "target": "4"}
            ],

            # === 從 3 號桌出發 (車頭朝下) ===
            ("3", "home"): [
                {"action": "backup", "target": "BR_corner"}, 
                {"action": "nav", "target": "Backup_pos"},
                {"action": "backup_cmdvel", "target": "home"}
            ],
            ("3", "1"): [
                {"action": "backup", "target": "BR_corner"}, 
                {"action": "nav", "target": "BL_corner"}, 
                {"action": "nav", "target": "1"}
            ],
            ("3", "2"): [
                {"action": "backup", "target": "BR_corner"}, 
                {"action": "nav", "target": "BL_corner"}, 
                {"action": "nav", "target": "2"}
            ],
            ("3", "4"): [
                # {"action": "nav", "target": "BR_corner"}, # 往前開出站
                # {"action": "nav", "target": "TR_corner"}, # 繞到右上
                # {"action": "nav", "target": "4"}     # 重新直走進 4 號
                {"action": "nav", "target": "4"}
            ],

            # === 從 4 號桌出發 (車頭朝下) ===
            ("4", "home"): [
                {"action": "backup", "target": "BR_corner"}, # 直走經過 3，到右下
                {"action": "nav", "target": "Backup_pos"},
                {"action": "backup_cmdvel", "target": "home"}
            ],
            ("4", "1"): [
                {"action": "backup", "target": "BR_corner"}, 
                {"action": "nav", "target": "BL_corner"}, 
                {"action": "nav", "target": "1"}
            ],
            ("4", "2"): [
                {"action": "backup", "target": "BR_corner"}, 
                {"action": "nav", "target": "BL_corner"}, 
                {"action": "nav", "target": "2"}
            ],
            ("4", "3"): [
                {"action": "backup", "target": "3"} 
            ],
        }

        # 3. 初始化 AI 模型
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.yolo_model = YOLO(YOLO_MODEL_PATH)
        self.gemini_client = genai.Client(api_key=GOOGLE_GEMINI_API_KEY)
        self.cv_bridge = CvBridge()
        
        # 4. 建立 ROS2 通訊節點
        self.chat_pub = self.create_publisher(String, '/breakfast_robot/chat_log', 10)
        self.image_pub = self.create_publisher(Image, '/breakfast_robot/person_image', 10)
        
        image_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.depth_sub = self.create_subscription(Image, '/dummy_depth', self.depth_callback, image_qos)
        self.color_sub = self.create_subscription(Image, '/image_raw', self.video_callback, image_qos)
        
        # 5. Action Server (接收 RQT 按鈕) 與 Nav2 Action Client (控制底盤)
        self.tools = [self.get_weather_internal, self.delivery_service]
        self._action_server = ActionServer(self, VoiceTask, '/breakfast_voice_task', self.execute_callback)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.backup_client = ActionClient(self, BackUp, '/backup')

        #  === AruCo 初始化 GoToArucoActionClient（用於送餐後的位置調整）===
        #self.aruco_client = ActionClient(self, GotoAruco, 'go_to_aruco')
        # 修改 aruco_client 的 get_result_callback，使其完成後自動進入 post_delivery_chat
        self.is_docking = False  # 🌟 新增：是否正在對 ArUco 進行最後調整

        self.get_logger().info('🤖 小美 送餐大腦啟動！等待 RQT 啟動指令...')
        threading.Thread(target=self.wakeword_thread, daemon=True).start()

    def change_goal_tolerance(self, xy_goal_tolerance=0.20, yaw_goal_tolerance=0.15):
        # 建立一個 Client 來呼叫 /controller_server 的參數修改服務
        client = self.create_client(SetParameters, '/controller_server/set_parameters')
        
        # 等待服務啟動
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /controller_server 參數服務...')

        # 建立參數修改請求
        request = SetParameters.Request()
        param = Parameter()
        param.name = 'general_goal_checker.xy_goal_tolerance' # 請確認你 yaml 裡的名稱
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(xy_goal_tolerance)
        request.parameters = [param]

        request = SetParameters.Request()
        param = Parameter()
        param.name = 'general_goal_checker.yaw_goal_tolerance' # 請確認你 yaml 裡的名稱
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(yaw_goal_tolerance)
        request.parameters = [param]


        # 發送請求 (這裡使用非同步呼叫避免卡死)
        future = client.call_async(request)
        self.get_logger().info(f'🔄 已將 xy_goal_tolerance 動態調整為: {xy_goal_tolerance} m')
        self.get_logger().info(f'🔄 已將 yaw_goal_tolerance 動態調整為: {yaw_goal_tolerance} rad')

    def wakeword_thread(self):
        """背景持續監聽喚醒詞的執行緒"""
        try:
            self.get_logger().info(f"🔊 嘗試載入喚醒詞模型: {WAKEWORD_MODEL_PATH}")
            oww_model = openwakeword.Model(wakeword_models=[WAKEWORD_MODEL_PATH], inference_framework="onnx")
            self.get_logger().info("✅ 喚醒詞模型載入成功！")
        except Exception as e:
            self.get_logger().error(f"❌ 喚醒詞模型載入失敗: {e}")
            return

        pa = pyaudio.PyAudio()
        self.mic_stream = None

        # 🌟 改寫：用 system_active 作為循環條件
        while self.system_active:
            try:
                # 🌟 系統忙碌時：主動釋放麥克風給 STT 使用
                if self.is_processing:
                    if self.mic_stream:
                        try:
                            self.mic_stream.stop_stream()
                            self.mic_stream.close()
                        except:
                            pass
                        self.mic_stream = None
                    self.mic_free_event.set()
                    time.sleep(0.1)  # 🌟 改成 0.1 秒（快 10 倍！）
                    continue

                # 🌟 關鍵防護網：如果鎖解開了，但麥克風是關的，要小心地重開！
                if self.mic_stream is None:
                    try:
                        self.mic_free_event.clear()
                        self.mic_stream = pa.open(
                            format=pyaudio.paInt16,
                            channels=1,
                            rate=16000,
                            input=True,
                            frames_per_buffer=1280
                        )
                        self.get_logger().info("🎤 喚醒詞麥克風已安全重啟！")
                    except Exception as e:
                        # 如果 ALSA 還沒釋放設備，印出警告，然後「等一下再試」
                        # 絕對不能讓 Exception 殺死這個 Thread！
                        self.get_logger().warn(f"等待麥克風設備釋放中... ({e})")
                        time.sleep(0.5)
                        continue  # 回到 while 迴圈開頭重試

                data = self.mic_stream.read(1280, exception_on_overflow=False)

                # 🌟 麥克風數位增益 (DSP)，讓喚醒更靈敏
                audio_float = np.frombuffer(data, dtype=np.int16).astype(np.float32)
                audio_float *= 2.0  # 放大音量 2 倍
                audio_frame = np.clip(audio_float, -32768, 32767).astype(np.int16)  # 防破音限制
                oww_model.predict(audio_frame)

                for mdl, score in oww_model.prediction_buffer.items():
                    # 只在信心度較高時才打印
                    if score[-1] > 0.05:
                        self.get_logger().info(f"🎙️  喚醒詞模型 {mdl} 信心度: {score[-1]:.4f}")
                    if score[-1] > 0.2:
                        self.is_processing = True
                        # 🔴 修正：喚醒詞觸發時也要重置視覺冷卻時間，防止視覺和喚醒詞同時觸發
                        self.last_visual_trigger_time = time.time()
                        if self.mic_stream:
                            self.mic_stream.stop_stream()
                            self.mic_stream.close()
                            self.mic_stream = None

                        # 🌟 導航狀態攔截：送餐中如果聽到名字，回覆正在忙
                        if self.is_navigating:
                            self._tts_and_play("我正在送餐中，請稍後喔！")
                            self.is_processing = False
                            oww_model.reset()
                            break

                        if self.is_docking:
                            self._tts_and_play("我正在做最後的調整，請稍後喔！")
                            self.is_processing = False
                            oww_model.reset()
                            break

                        self.get_logger().info(f"✨ 喚醒詞觸發！信心度: {score[-1]:.2f}")
                        self._tts_and_play("在的，請問要送到幾號桌？")
                        self.publish_chat("系統", "聽取指令中...")
                        threading.Thread(target=self._voice_logic_task).start()
                        oww_model.reset()
                        break

            except Exception as e:
                # 如果是設備被佔用，不要噴 error，改用 warn 並等久一點
                if "Device or resource busy" in str(e) or "Input overflowed" in str(e):
                    self.get_logger().warn(f"⚠️  麥克風設備繁忙: {e}")
                    time.sleep(0.5)
                else:
                    self.get_logger().debug(f"喚醒詞監聽暫時中斷: {e}")
                    time.sleep(0.5)

        # 系統關閉時的清理
        if self.mic_stream:
            try:
                self.mic_stream.stop_stream()
                self.mic_stream.close()
            except:
                pass
        pa.terminate()
        self.get_logger().info("🔌 喚醒詞線程已安全關閉")

    # ==========================================
    # [視覺與影像處理區塊]
    # ==========================================
    def depth_callback(self, msg):
        try:
            raw_depth = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            self.latest_depth_img = cv2.resize(raw_depth, (1280, 720), interpolation=cv2.INTER_NEAREST)
        except Exception:  # 🔴 修正：指定異常類型
            pass

    def video_callback(self, color_msg):
        if not self.system_active: return
        self.frame_count += 1
        
        try:
            # 完全繞過 cv_bridge 轉換影像，效能極限榨取
            # raw_data = np.frombuffer(color_msg.data, dtype=np.uint8)
            # expected_size = color_msg.height * color_msg.width * 3
            # if raw_data.size != expected_size: return

            # full_img = raw_data.reshape((color_msg.height, color_msg.width, 3))
            # img_bgr = cv2.cvtColor(full_img, cv2.COLOR_RGB2BGR) if color_msg.encoding == 'rgb8' else full_img
            # img = cv2.resize(img_bgr, (1280, 720))
            img_bgr = self.cv_bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            img = cv2.resize(img_bgr, (1280, 720))
            closest_dist = 999.0
            depth_img = self.latest_depth_img

            if self.frame_count % self.detect_every_n_frames == 0:
                results = self.yolo_model(img, device=self.device, verbose=False)
                current_detections = []
                for r in results:
                    for box in r.boxes:
                        if int(box.cls[0]) == 0:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])  # 🔴 修正：先取得座標
                            cx = (x1 + x2) // 2  # 🔴 修正：再計算中心點
                            cy = (y1 + y2) // 2
                            dist = 0.5 # 假距離
                            if depth_img is not None:
                                try:
                                    h, w = depth_img.shape[:2]
                                    real_cy, real_cx = int(cy * h / 720), int(cx * w / 1280)
                                    if 0 <= real_cy < h and 0 <= real_cx < w:
                                        dist = depth_img[real_cy, real_cx] / 1000.0
                                except Exception:  # 🔴 修正：指定異常類型
                                    pass

                            if 0 < dist < closest_dist:
                                closest_dist = dist
                            current_detections.append((x1, y1, x2, y2, dist))
                # 覆蓋舊記憶
                self.last_detections = current_detections

            # 🌟 畫框框：不管這幀有沒有跑 YOLO，都把記憶中的框畫上去！
            for (x1, y1, x2, y2, dist) in self.last_detections:
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(img, f"{dist:.2f}m", (x1, y1-10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            now = time.time()
            #self.get_logger().info(f"👀 看到人了！距離: {closest_dist:.2f}m, 忙碌中: {self.is_processing}, 冷卻倒數: {60 - (time.time() - self.last_visual_trigger_time):.0f}秒")
            # 🌟 導航狀態攔截：送餐中看到人不理他
            if 0 < closest_dist < 1.2 and not self.is_processing and not self.is_navigating and not getattr(self, 'is_docking', False) and (now - self.last_visual_trigger_time > self.visual_cooldown):
                self.is_processing = True
                self.last_visual_trigger_time = now
                self.get_logger().info('👀 偵測到客人，自動打招呼！')
                # threading.Thread(
                #     target=self.process_gemini_and_speak, 
                #     args=("偵測到有人靠近。請主動溫馨地打招呼，並詢問『請問要送到幾號桌？』", True), 
                #     daemon=True
                # ).start()

                # 隨機挑一句，感覺比較生動
                greetings = [
                    "你好呀！請問餐點要送到幾號桌呢？",
                    "歡迎光臨！請問要送到幾號桌？",
                    "哈囉！我是小美，請問要幫你送到幾號桌？"
                ]
                greeting_text = random.choice(greetings)
                
                # 直接印出並講話，跳過 Gemini 思考！
                self.publish_chat("小美", greeting_text)
                
                def instant_greet_and_listen():
                    self._tts_and_play(greeting_text) # 生成語音並播放
                    # 講完話後，直接開啟聽筒模式
                    self.publish_chat("系統", "正在聽取您的回覆...")
                    self._voice_logic_task() 

                threading.Thread(target=instant_greet_and_listen, daemon=True).start()

            msg_out = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.image_pub.publish(msg_out)
        except Exception:  # 🔴 修正：指定異常類型
            pass

    # ==========================================
    # [系統與語音邏輯區塊]
    # ==========================================
    def publish_chat(self, sender, message):
        msg = String()
        msg.data = f"[{sender}]: {message}"
        self.chat_pub.publish(msg)

    def execute_callback(self, goal_handle):
        self.get_logger().info("🎬 收到 Master Node 的送餐啟動指令！連線測試成功！")
        # 🔴 修正：檢查是否正在送餐或處理中
        if self.is_processing or self.is_navigating:
            goal_handle.abort()
            return VoiceTask.Result(success=False, final_response="系統正在忙碌中，請稍候！")
        # 🌟 關鍵 1：打開總開關，讓相機和喚醒詞開始工作！
        self.system_active = True

        # 🌟 關鍵 2：千萬不要鎖定系統！(確保 is_processing 是 False)
        # 這樣下一秒相機 (video_callback) 才能立刻去判斷距離 < 1.2m
        self.is_processing = False

        self.last_visual_trigger_time = 0
        # 中斷可能還在聽的麥克風
        if self.mic_stream:
            try:
                self.mic_stream.stop_stream()
                self.mic_stream.close()
                self.mic_stream = None
            except: pass

        self.publish_chat("系統", "已強制重置狀態，準備就緒。")
        goal_handle.succeed()
        return VoiceTask.Result(success=True, final_response="啟動完畢")

    def process_gemini_and_speak(self, prompt, auto_listen=True):
        # self.conversation_count += 1

        # # 🌟 修正：如果已經連問 3 次都沒結果，就強制結束對話，回歸待機
        # if self.conversation_count > 3:
        #     self._tts_and_play("抱歉，我可能聽不太清楚。我先回待機模式，您可以再喊我一次喔！")
        #     self.conversation_count = 0
        #     self.is_processing = False
        #     return

        response_text = self.gemini_brain(prompt)
        if not response_text or response_text.strip() == "":
            response_text = "我聽不太清楚，可以再說一次嗎？"

        self.publish_chat("小美", response_text)
        self._tts_and_play(response_text)

        # 判斷是否要繼續聽 (導航時不聽)
        # 🌟 新增：在post-delivery模式下（is_navigating=False），根據should_listen判斷
        should_listen = auto_listen and not self.is_navigating and ("？" in response_text or "?" in response_text)

        # 🌟 新增：檢查結束詞彙（參考v2.py的做法）
        # 這些詞彙表示post-delivery對話應該結束（允許客人說「謝謝」、「再見」等來結束對話）
        if any(end in response_text for end in ["祝您", "再見", "美好的一天", "請隨時告訴我喔", "進入冷卻", "待機"]):
            should_listen = False

        if should_listen:
            self.publish_chat("系統", "正在聽取您的回覆...")
            threading.Thread(target=self._voice_logic_task).start()
        else:
            # ❌ 舊的邏輯：
            # self.last_visual_trigger_time = time.time()
            # self.is_processing = False

            # 🌟 新增：檢查是否在 post-delivery 對話模式
            if self.post_delivery_mode:
                self.publish_chat("系統", "🏠 post-delivery 對話結束，準備返回廚房...")
                self.post_delivery_mode = False  # 解除標記
                # 🌟 新增：自動回家！
                self.delivery_service("home")
            else:
                # 正常對話結束的情況
                if not self.is_navigating:
                    self.publish_chat("系統", "對話結束，進入冷卻。")
                    self.last_visual_trigger_time = time.time()

                self.is_processing = False

    def _voice_logic_task(self):
        try:
            time.sleep(0.2)
            self.publish_chat("系統", "請說話...")

            # 使用 Streaming STT，使用者講完話馬上回傳，不用等
            text = self._streaming_stt()
            self.get_logger().info(f"🎤 STT 辨識結果: '{text}'")

            if text and text.strip():
                # 🌟 改進：強化字符串修正邏輯，補捉 STT 常見誤譯
                correct_text = text

                # === 原有的修正規則 ===
                correct_text = correct_text.replace("單號桌", "3號桌").replace("單號", "3號").replace("單桌", "3桌")
                correct_text = correct_text.replace("當號桌", "3號桌").replace("當號", "3號").replace("當桌", "3桌")
                correct_text = correct_text.replace("是號桌", "4號桌").replace("是號", "4號").replace("是桌", "4桌")
                correct_text = correct_text.replace("字號桌", "4號桌").replace("字號", "4號").replace("字桌", "4桌")

                # === 🌟 新增：針對 1 號常見誤譯 ===
                correct_text = correct_text.replace("一會", "1號").replace("一會桌", "1號桌")
                correct_text = correct_text.replace("一號後", "1號").replace("一後", "1號")

                # === 🌟 新增：針對 2 號常見誤譯 ===
                correct_text = correct_text.replace("二會", "2號").replace("二號後", "2號")

                # === 🌟 新增：針對 4 號常見誤譯 ===
                correct_text = correct_text.replace("四會", "4號").replace("四號後", "4號")
                correct_text = correct_text.replace("司號", "4號").replace("司號桌", "4號桌")

                # === 🌟 新增：其他常見誤譯 ===
                correct_text = correct_text.replace("一號桌", "1號桌").replace("二號桌", "2號桌")
                correct_text = correct_text.replace("三號桌", "3號桌").replace("四號桌", "4號桌")

                self.publish_chat("User", correct_text)

                # 🔴 關鍵改進：在 post-delivery 對話中，先檢查結束詞彙
                # 如果客人說了結束詞彙，直接結束對話，不讓 Gemini 決定
                if self.post_delivery_mode:
                    end_keywords = ["謝謝", "感謝", "不用了", "不必", "再見", "拜拜", "可以了", "好了", "行了", "謝", "沒有了", "沒了", "結束", "完了", "夠了", "不說了", "不講了", "不聊了", "先這樣", "先這樣吧", "回家", "回廚房", "可以了", "回去"]
                    if any(end in correct_text for end in end_keywords):
                        self.publish_chat("小美", "祝您享用美食，有美好的一天！")
                        self._tts_and_play("祝您享用美食，有美好的一天！")
                        self.post_delivery_mode = False
                        self.is_processing = False
                        self.last_visual_trigger_time = time.time()
                        self.delivery_service("home")
                        return

                # ==========================================
                # 🌟 終極殺手鐧：本地攔截！0 延遲，免扣 Gemini 額度
                # ==========================================
                target_id = self.parse_location(correct_text)

                # 如果有抓到目標，而且在我們的地圖裡，就直接送餐！
                if target_id and target_id in self.table_poses:
                    self.get_logger().info(f"⚡ [本地攔截] 成功辨識桌號: {target_id}，跳過 Gemini API！")

                    # 自己準備回覆的台詞
                    if target_id == "home":
                        reply_text = "好的，我現在返回廚房。"
                    else:
                        reply_text = f"好的，馬上為您送到 {target_id} 號桌。"

                    self.publish_chat("小美", reply_text)
                    self._tts_and_play(reply_text)

                    # 直接在背景呼叫你的導航流程
                    self.delivery_service(target_id)

                    self.is_processing = False

                # 如果找不到桌號 (例如客人說 "今天天氣真好" 或 "你是誰")
                else:
                    # 🔴 新增：檢查文字是否太短或像是提問詞（不像桌號）
                    questionable_words = ["幾號", "幾", "哪號", "哪", "多少號", "幾號桌", "哪號桌", "你好", "呃", "嗯"]
                    # 🌟 用 or：如果文字長度 ≤ 4 秒 或 包含提問詞，就認為不像桌號
                    if len(correct_text) <= 4 or any(word in correct_text for word in questionable_words):
                        # 直接要求客人提供具體桌號，不浪費 Gemini 額度
                        retry_msg = "請說出具體的桌號，例如：一號、二號、三號或四號。"
                        self.publish_chat("小美", retry_msg)
                        self._tts_and_play(retry_msg)
                        # 🔴 修正：不要解鎖系統，而是繼續監聽用戶的回覆
                        self.publish_chat("系統", "正在聽取您的回覆...")
                        threading.Thread(target=self._voice_logic_task).start()
                        self.get_logger().info(f"⚠️  用戶說的不像桌號，已要求重新輸入")
                    else:
                        self.get_logger().info("☁️ [雲端處理] 聽不懂桌號，交給 Gemini 處理閒聊...")
                        self.process_gemini_and_speak(correct_text)

            else:
                # 🌟 新增註解：無聲情況下的處理（post-delivery對話無回應時）
                # _streaming_stt() 會在3秒無聲時結束，所以這裡會拿到空字符串
                self.publish_chat("系統", "未偵測到有效語音。")

                # 🔴 修正：檢查是否在 post-delivery 對話模式，如果是則自動結束並回家
                if self.post_delivery_mode:
                    self.publish_chat("系統", "🏠 對話結束，準備返回廚房...")
                    self.post_delivery_mode = False
                    self.delivery_service("home")
                else:
                    # 正常對話中無聲，只解鎖系統
                    self.is_processing = False
        except Exception as e:
            # 🔴 修正：異常時必須確保解鎖 is_processing，並重置冷卻時間
            self.get_logger().error(f"語音邏輯出錯: {e}")
            self.last_visual_trigger_time = time.time()
            self.is_processing = False

    # (STT 與 TTS 函式保持學弟的版本即可，為節省版面此處省略內部實作，請直接複製剛才的)
    def _streaming_stt(self):
        """Google Cloud 串流語音辨識 (邊聽邊辨識)
        🌟 改進：
        1. 加入 Hints（期望詞彙）改善桌號辨識準度
        2. 檢查信心度，若太低會警告
        3. 調整無聲偵測時間（2.5 秒）
        4. 最長錄音時間延長到 10 秒
        5. 加入音量監測，判斷是否有實際講話
        """
        client = speech.SpeechClient()

        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=16000,
            language_code="zh-TW",
            # 🌟 新增：告訴 STT 我們期望的桌號詞彙
            speech_contexts=[
                speech.SpeechContext(
                    phrases=["一號", "二號", "三號", "四號", "1號", "2號", "3號", "4號",
                             "一號桌", "二號桌", "三號桌", "四號桌", "1號桌", "2號桌", "3號桌", "4號桌",
                             "廚房", "回廚房", "回家"]
                )
            ]
        )
        streaming_config = speech.StreamingRecognitionConfig(
            config=config,
            interim_results=True
        )

        CHUNK = 1024
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000,
                        input=True, frames_per_buffer=CHUNK)

        def request_generator():
            last_audio_time = time.time()
            # 🌟 改進：最多錄音 10 秒鐘
            for _ in range(0, int(16000 / CHUNK * 9)):
                data = stream.read(CHUNK, exception_on_overflow=False)

                # 🌟 新增：檢查音量是否有聲
                audio = np.frombuffer(data, dtype=np.int16)
                audio_mean = np.abs(audio).mean()

                if audio_mean > 50:  # 有實際音量波動（不是靜默）
                    last_audio_time = time.time()
                # 🌟 改進：改成 2.5 秒無聲則提早切斷（而不是 3.0）
                elif time.time() - last_audio_time > 2.5:
                    break

                yield speech.StreamingRecognizeRequest(audio_content=data)

        responses = client.streaming_recognize(config=streaming_config, requests=request_generator())
        final_transcript = ""
        confidence = 0.0

        for response in responses:
            for result in response.results:
                if result.is_final and result.alternatives:
                    final_transcript = result.alternatives[0].transcript
                    confidence = result.alternatives[0].confidence  # 🌟 新增：取得信心度

        stream.stop_stream()
        stream.close()
        p.terminate()

        # 🌟 新增：如果信心度太低，警告可能辨識有誤
        if final_transcript and confidence < 0.6:
            self.get_logger().warn(f"⚠️  STT 信心度偏低: {confidence:.2f}，辨識結果可能不準確 ('{final_transcript}')")
        elif final_transcript:
            self.get_logger().info(f"✅ STT 信心度: {confidence:.2f}")
        else:
            # 🔴 新增：診斷日誌，幫助除錯「未偵測到有效語音」的問題
            self.get_logger().warn(f"⚠️  STT 沒有偵測到任何語音，請檢查麥克風或說話音量")

        return final_transcript if final_transcript else None
    
    def _tts_and_play(self, text):
        """🌟 將語音傳送給 VTuber Studio 進行嘴型連動"""
        now = time.time()
        client = texttospeech.TextToSpeechClient()
        voice = texttospeech.VoiceSelectionParams(language_code="cmn-TW", name="cmn-TW-Wavenet-A")
        
        # VTuber 連動需要 LINEAR16 的 wav 格式
        audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.LINEAR16, 
            sample_rate_hertz=44100,
            speaking_rate=1.2,
            pitch=5.0
        )
        
        try:
            res = client.synthesize_speech(
                input=texttospeech.SynthesisInput(text=text), 
                voice=voice, 
                audio_config=audio_config
            )

            # 在記憶體處理並計算播放時間
            audio_data = io.BytesIO(res.audio_content)
            audio_segment = AudioSegment.from_wav(audio_data)
            audio_duration = audio_segment.duration_seconds
            
            wav_path = "temp.wav"
            with open(wav_path, "wb") as f:
                f.write(res.audio_content)

            # 呼叫 VTuber Studio 連動播放
            # ⚠️ 注意：output_device=5 是學弟的設備號碼，若沒聲音，請改成 None 讓它用預設設備
            try:
                # Vtuber Studio 跑在本地 不用打 ws_uri=target_ws
                # output_device 看自己電腦的播放設備清單
                # target_ws = 'ws://100.79.190.70:8001' 
                # temperature 可以改開口大小
                with VTSAdapter(DBAnalyser(temperature=10)) as a:
                    a.action(audio=wav_path, samplerate=44100, output_device=None)
                    time.sleep(audio_duration + 0.65) # 緩衝等嘴巴閉合
            except Exception as e:
                self.get_logger().warn(f"VTuber Studio 連線失敗，改用備用播放器: {e}")
                # 備用方案：如果 VTS 沒開，使用原本的 aplay 或 pydub 播放
                os.system(f"aplay -q {wav_path}")

            if os.path.exists(wav_path):
                os.remove(wav_path)
                
        except Exception as e:
            self.get_logger().error(f"TTS 生成或播放時發生錯誤: {e}")

    # ==========================================
    # [Gemini 大腦與 Tools]
    # ==========================================
    # def gemini_brain(self, user_input):
    #     instruction = """
    #     ## ROLE
    #     你是一個溫暖貼心的繁體中文送餐機器人 小美。
    #     你的主要任務是詢問客人「請問要送到幾號桌？」，並在確認後執行送餐。

    #     ## STRICT RULES
    #     1. **FUNCTION CALL FIRST**: 當使用者表達「送餐、幾號桌、回到起點」時，或是使用者【單純只回答數字與桌號】（例如只說：「3號」、「一桌」），這代表他在回答你的送餐問題，你 **必須、且只能** 呼叫 `delivery_service()` 工具。
    #     2. **STATUS BOUNDARY**: 呼叫工具後，請「完全照抄」工具回傳的字串作為最終回覆，嚴禁自己發明或預告對話。
    #     3. **TONE AND STYLE**: 回答要溫暖貼心。
    #     """
    #     try:
    #         response = self.gemini_client.models.generate_content(
    #             model="gemini-3.1-flash-lite-preview",
    #             contents=[user_input],
    #             config={"tools": self.tools, "system_instruction": instruction}
    #         )
    #         return response.text if response.text else "抱歉，無法處理。"
    #     except Exception as e:
    #         self.get_logger().error(f"Gemini API Error: {e}") 
    #         return "大腦連線異常。"

    
    def gemini_brain(self, user_input):
        instruction = """
        ## ROLE
        你是一個溫暖貼心的繁體中文送餐機器人 小美。
        你的主要任務是詢問客人「請問要送到幾號桌？」，並在確認後執行送餐。

        ## STRICT RULES
        1. **FUNCTION CALL FIRST**: 當使用者表達「送餐、幾號桌、回到起點」時，或是使用者【單純只回答數字與桌號】（例如只說：「3號」、「一桌」），這代表他在回答你的送餐問題，你 **必須、且只能** 呼叫 `delivery_service()` 工具。
        2. **STATUS BOUNDARY**: 呼叫工具後，請「完全照抄」工具回傳的字串作為最終回覆，嚴禁自己發明或預告對話。
        3. **TONE AND STYLE**: 回答要溫暖貼心。
        4. **POST-DELIVERY CHAT**: 餐點已送達後，若客人還在，你可以：
           - 詢問有沒有其他需要幫助的地方
           - 講個笑話或有趣的故事讓他們開心
           - 主動詢問是否要幫客人查詢天氣
           - 記得在回覆中包含「？」來與客人互動，保持對話連續
        """

        models_to_try = [
            ("gemini-3.1-flash-lite-preview", 1), 
            ("gemini-2.5-flash", 1)               
        ]

        for model_name, max_retries in models_to_try:
            for attempt in range(max_retries):
                try:
                    start_time = time.time() # ⏳ 按下碼表
                    self.get_logger().info(f"🧠 開始呼叫 {model_name}...")
                    
                    future = self.api_executor.submit(
                        self.gemini_client.models.generate_content,
                        model=model_name,
                        contents=[user_input],
                        config={"tools": self.tools, "system_instruction": instruction}
                    )
                    
                    # 先放寬到 15 秒，看它到底需要多久
                    response = future.result(timeout=8.0)  
                    
                    elapsed = time.time() - start_time # ⏱️ 計算花費時間
                    self.get_logger().info(f"✅ {model_name} 回覆成功！耗時: {elapsed:.2f} 秒")
                    
                    return response.text if response.text else "抱歉，無法處理。"
                    
                except concurrent.futures.TimeoutError:
                    elapsed = time.time() - start_time
                    self.get_logger().warn(f"⚠️ {model_name} 已讀不回 (撐了 {elapsed:.2f} 秒)，強制切換！")
                    time.sleep(1.0) 
                    break

                except Exception as e:
                    error_msg = str(e)
                    if "503" in error_msg or "UNAVAILABLE" in error_msg or "429" in error_msg:
                        self.get_logger().warn(f"⚠️ {model_name} 伺服器忙碌中 ({error_msg[:20]})... 切換大腦！")
                        time.sleep(1.0)
                    else:
                        self.get_logger().error(f"❌ {model_name} 發生未知錯誤: {error_msg}")
                        break 

        return "抱歉，目前雲端大腦大塞車，請您稍後再試喔。"

    def get_weather_internal(self, location: str):
        url = f"http://api.weatherapi.com/v1/current.json?key=040e24d691134027aa8115215262102&q={location}&aqi=no"
        try:
            return requests.get(url).json()
        except Exception as e:
            return {"error": str(e)}
    
    def chinese_to_number(self, text):
        """將中文數字轉換為整數 (保留 GPT 的好寫法)"""
        zh_map = {
            "零":0, "一":1, "二":2, "兩":2, 
            "三":3, "單":3,
            "四":4, "是":4,
            "五":5, "六":6, "七":7,
            "八":8, "九":9, "十":10
        }
        if "十" in text:
            parts = text.split("十")
            first = zh_map.get(parts[0], 0) if parts[0] else 0
            second = zh_map.get(parts[1], 0) if len(parts) > 1 and parts[1] else 0
            if parts[0] == "": 
                return 10 + second
            return first * 10 + second
        return zh_map.get(text, None)
    
    def parse_location(self, text: str):
        """從長句子中精準萃取目的地"""
        # 1️⃣ 處理 Home 的情況
        for word in ["home", "起點", "原點", "回去", "廚房"]:
            if word in text.lower():
                return "home"

        # 2️⃣ 🌟 核心防禦：正規表達式綁定「號/桌」
        # 尋找「阿拉伯數字」或「中文數字」，且後面跟著 "號" 或 "桌" 的組合
        # 例如："送到12號"、"去三桌"，但會自動忽略 "等一下"
        match = re.search(r'(\d+|[一二兩三四五六七八九十]+)(?:號|桌)', text)

        # 3️⃣ 備用方案：如果 Gemini 很乖，只傳了單純的 "3" 或 "三"，沒有加"號桌"
        if not match and len(text) <= 5: 
            match = re.search(r'(\d+|[一二兩三四五六七八九十]+)', text)

        if match:
            num_str = match.group(1) # 取出括號中配對到的數字部分
            
            # 如果是阿拉伯數字 (例如 "12")，直接回傳
            if num_str.isdigit():
                return num_str
                
            # 如果是中文 (例如 "十二")，呼叫轉換函式
            return str(self.chinese_to_number(num_str))

        return None
    
    def _execute_next_route_step(self):
        """接力賽引擊：從佇列拿出下一步並執行"""

        # 1. 檢查是否已經走完所有步驟
        if not self.current_route_steps:
            self.get_logger().info(f"🎯 抵達最終目的地: {self.target_destination}！")
            # 解除導航鎖定
            self.is_navigating = False
            self.current_location = self.target_destination # 更新車子的最新位置 # string

            # 🔴 關鍵修正：區分「正常送餐完成」vs「返回 home 完成」
            # 防止無限進入 post-delivery 對話的 bug
            if self.target_destination == "home":
                # =====================================================
                # 情況 B：返回廚房完成 → 完全解鎖系統回歸待機
                # =====================================================
                self.is_processing = False        # 完全解鎖系統
                self.last_visual_trigger_time = time.time()  # 重置視覺冷卻
                self.get_logger().info("✅ 返回廚房完成，系統進入待機模式。")
                self.publish_chat("系統", "🏠 小美已返回廚房，準備好為下位客人服務！")

            else:
                # =====================================================
                # 情況 A：正常送餐完成（來到客人的桌子） → 進入 post-delivery 對話
                # =====================================================
                self.is_processing = True  # 進入 post-delivery 對話模式（鎖定系統）
                # 🌟 狀態轉換說明：
                #   - is_navigating: True → False （送餐完成）
                #   - is_processing: False → True （進入 post-delivery 對話模式，鎖定系統防止其他觸發）
                #   - post_delivery_mode: False → True （在 _post_delivery_routine 中設置）
                #
                # 這樣做的目的是：
                #   1. 防止視覺系統在送餐途中立即觸發下一次接待
                #   2. process_gemini_and_speak 會在對話結束時根據 post_delivery_mode 決定是否自動回家

                # ==call aruco===
                #self.start_aruco_docking()
                threading.Thread(target=self._post_delivery_routine, daemon=True).start()

            return

        # 2. 拿出這一步的指令
        step = self.current_route_steps.pop(0)
        action_type = step["action"]
        target_name = step["target"]

        # ==========================================
        # 情況 A：使用一般導航 (nav)
        # ==========================================
        if action_type == "forward_cmdvel":
            self.get_logger().info(f"🚶 [FORWARD] 從 {self.current_step_origin} 前進至 {target_name}")
            ck1 = self.table_poses[self.current_step_origin]
            ck2 = self.table_poses[target_name]

            dx = ck2['x'] - ck1['x']
            dy = ck2['y'] - ck1['y']
            dist = math.hypot(dx, dy)
            speed = 0.2  # 前進速度

            duration = dist / speed # 國小數學：時間 = 距離 / 速度
            # 🌟 定義一個背景小任務來踩油門與煞車
            def blind_forward_task():
                move_msg = Twist()
                start_time = time.time()
                
                while time.time() - start_time < duration:
                    elapsed = time.time() - start_time
                    
                    # 簡單的起步緩衝 (前 0.5 秒慢慢加速到 0.2)
                    if elapsed < 0.5:
                        current_speed = speed * (elapsed / 0.5) 
                    else:
                        current_speed = speed
                        
                    move_msg.linear.x = current_speed
                    self.cmd_vel_pub.publish(move_msg)
                    time.sleep(0.05) # 提高頻率到 20Hz 讓動作更順暢
                
                # 停止
                stop_msg = Twist()
                stop_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(stop_msg)
                
                self.get_logger().info("✅ 盲駛前進完成！")
                
                # 手動觸發下一步
                self.current_step_origin = target_name
                self._execute_next_route_step()

            # 丟到背景執行，才不會卡住主程式
            threading.Thread(target=blind_forward_task, daemon=True).start()

        # ==========================================
        # 情況 B：使用一般導航 (nav)
        # ==========================================
        elif action_type == "nav":
            if target_name == "Backup_pos":
                self.change_goal_tolerance(0.08, 0.05)
            if target_name in ["3", "4"]:
                self.change_goal_tolerance(0.15, 0.06)
            if target_name == "BL_corner":
                self.change_goal_tolerance(0.25, 0.06)
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = self.table_poses[target_name]["x"]
            goal_pose.pose.position.y = self.table_poses[target_name]["y"]
            goal_pose.pose.orientation.z = self.table_poses[target_name]['z']
            goal_pose.pose.orientation.w = self.table_poses[target_name]["w"]

            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = goal_pose
            self.get_logger().info(f"🚀 [NAV] 前往中繼站/目標: {target_name}")

            future = self.nav_client.send_goal_async(nav_goal)
            future.add_done_callback(self.nav_response_callback)

            self.current_step_origin = target_name

        # ==========================================
        # 情況 C：使用直線倒車 (backup)
        # ==========================================
        elif action_type == "backup_cmdvel":
            self.get_logger().info(f"🔙 [BACKUP] 從 {self.current_step_origin} 後退至 {target_name}")
            ck1 = self.table_poses[self.current_step_origin]
            ck2 = self.table_poses[target_name]

            dx = ck2['x'] - ck1['x']
            dy = ck2['y'] - ck1['y']
            dist = math.hypot(dx, dy) + 0.03
            speed = 0.2  # 倒車速度調慢

            duration = dist / speed # 國小數學：時間 = 距離 / 速度
            # 🌟 定義一個背景小任務來踩油門與煞車
            def blind_backup_task():
                # 🌟 確認 self.cmd_vel_pub 是發布到 '/cmd_vel_nav' (交給 Smoother)
                start_time = time.time()
                move_msg = Twist()
                
                # 持續踩油門直到時間到
                while time.time() - start_time < duration:
                    elapsed = time.time() - start_time
                    
                    # 🌟 倒車柔和起步緩衝 (前 0.5 秒慢慢加速到 -0.2)
                    if elapsed < 0.5:
                        current_speed = -speed * (elapsed / 0.5) 
                    else:
                        current_speed = -speed
                        
                    move_msg.linear.x = current_speed
                    self.cmd_vel_pub.publish(move_msg)
                    time.sleep(0.05) # 🌟 提高頻率到 20Hz (0.05秒)，讓速度曲線更滑順
                
                # 柔和踩煞車 (如果是交給 velocity_smoother，直接給 0.0 它會幫你滑行煞停)
                stop_msg = Twist()
                stop_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(stop_msg)
                
                self.get_logger().info("✅ 盲駛倒車完成！")
                
                # 手動觸發下一步
                self.current_step_origin = target_name
                self._execute_next_route_step()

            # 丟到背景執行，才不會卡住主程式
            threading.Thread(target=blind_backup_task, daemon=True).start()
        # ==========================================
        # 情況 D：使用 Nav2 的直線倒車 (BackUp Action)
        # ==========================================
        elif action_type == "backup":
            backup_goal = BackUp.Goal()
            self.get_logger().info(f"🔙 [BACKUP] 從 {self.current_step_origin} 後退至 {target_name}")
            
            ck1 = self.table_poses[self.current_step_origin]
            ck2 = self.table_poses[target_name]

            dx = ck2['x'] - ck1['x']
            dy = ck2['y'] - ck1['y']
            dist = math.hypot(dx, dy)
            
            # 🌟 必備防禦：因為 Nav2 會看地圖避障，必須扣除安全距離以免觸發狀態碼 6！
            # 假設扣除 0.2 公尺，讓車子停在桌子前緣而不撞上去
            safe_dist = dist - 0.20 
            if safe_dist < 0.0: 
                safe_dist = 0.0

            # ⚠️ Backup Action 只能直線倒車，給單一距離值 (給正值)
            backup_goal.target.x = safe_dist
            backup_goal.speed = 0.20  # 倒車速度放慢以策安全
            
            self.get_logger().info(f"📏 預估安全倒車距離: {safe_dist:.3f}m (原始 dist={dist:.3f}m)")

            # 非同步發送 Action
            future = self.backup_client.send_goal_async(backup_goal)
            
            # 🌟 因為我們是共用同一個 callback，所以不管 Nav 還是 Backup，
            # 成功或失敗都會回到 nav_response_callback -> nav_result_callback 去觸發下一步
            future.add_done_callback(self.nav_response_callback)
            
            # 🌟 只更新這一步的起點名稱，千萬不能在這裡呼叫 _execute_next_route_step()！
            self.current_step_origin = target_name


    def delivery_service(self, location: str):
        """🌟 呼叫 Nav2 執行導航的 Tool"""
        # 🔴 修正：檢查是否已經在送餐中，避免任務被覆蓋
        if self.is_navigating:
            return "我正在送餐中，請稍候！"

        self.get_logger().info(f"=== [DEBUG] Gemini 成功呼叫了 delivery_service 工具！ 目標: {location} ===")

        # 呼叫解析器
        target_id = self.parse_location(location)

        # 防呆：如果解析失敗，或者解析出來的桌號不在地圖字典裡
        if not target_id or target_id not in self.table_poses:
            self.get_logger().warn(f"=== [DEBUG] 無法解析目標桌號 ===")
            return f"抱歉，我聽不太懂 '{location}' 是哪裡，可以請您確認一下桌號嗎？"
        else:
            self.get_logger().info(f"=== [DEBUG] 解析器成功解析出目標桌號: {target_id} ===")
        
        # 👉 補上這段防呆機制：等 2 秒看看 Nav2 有沒有回應
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 導航伺服器未啟動！")
            return "抱歉，導航系統尚未準備好，目前無法送餐。"
        
        route_key = (self.current_location, target_id)

        if route_key not in self.routes:
            self.get_logger().error(f"❌ 找不到從 {self.current_location} 到 {target_id} 的路徑設定！")
            return "抱歉，系統找不到這條路線的規劃。"

        # 複製路徑清單到任務佇列
        self.current_route_steps = list(self.routes[route_key])
        self.target_destination = target_id
        self.current_step_origin = self.current_location
        self.is_navigating = True
        self.get_logger().info(f"📍 準備從 {self.current_location} 導航至 {target_id}")

        self._execute_next_route_step()
        # # 打包 Nav2 Goal
        # goal_pose = PoseStamped()
        # goal_pose.header.frame_id = 'map'
        # goal_pose.header.stamp = self.get_clock().now().to_msg()
        # goal_pose.pose.position.x = self.table_poses[target_id]["x"]
        # goal_pose.pose.position.y = self.table_poses[target_id]["y"]
        # goal_pose.pose.orientation.z = self.table_poses[target_id]['z']
        # goal_pose.pose.orientation.w = self.table_poses[target_id]["w"]

        # nav_goal = NavigateToPose.Goal()
        # nav_goal.pose = goal_pose

        # # 鎖定導航狀態
        # self.is_navigating = True
        # self.get_logger().info(f"📍 準備導航至: {target_id}")

        # # 非同步發送目標給 Nav2
        # send_goal_future = self.nav_client.send_goal_async(nav_goal)
        # send_goal_future.add_done_callback(self.nav_response_callback)

        return f"已收到，準備送往 {target_id} 號桌。"
    

    # ==========================================
    # [Nav2 回報邏輯]
    # ==========================================
    def nav_response_callback(self, future):
        """確認 Nav2 是否接受導航任務"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Nav2 拒絕了導航請求！")
            self.is_navigating = False
            return
        
        self.get_logger().info("✅ Nav2 接受任務，開始移動...")
        self.nav_result_future = goal_handle.get_result_async()
        self.nav_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """導航抵達目的地時觸發"""
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ 單步完成！")

            # 這裡未來可以加入機械手臂放餐點的程式碼！
            # ... arm_put_down_food() ...

            # 👉 丟給背景執行緒去講話，不要卡死 ROS
            # 🌟 新增：如果是抵達中繼站，稍微等個 1.5 秒讓定位收斂
            if self.current_route_steps: # 代表還沒到終點，還在中繼站
                if self.current_step_origin == "Backup_pos":
                    self.change_goal_tolerance(0.20, 0.15) # 抵達 Backup_pos 後放寬容錯，幫助最後對位
                elif self.current_step_origin in ["3", "4"]:
                    self.change_goal_tolerance(0.20, 0.16)
                elif self.current_step_origin == "BL_corner":
                    self.change_goal_tolerance(0.20, 0.15) # 抵達 BL_corner 後放寬容錯，幫助最後對位
                time.sleep(1.5)
            # 🌟 關鍵：這一步完成了，繼續觸發下一步！
            self._execute_next_route_step()

        else:
            self.get_logger().error(f"❌ 導航失敗或被取消，狀態碼: {status}")
            self.is_navigating = False
            self.is_processing = False  # 🔴 修正：明確清除處理狀態
            # 🔴 修正：導航失敗後也要重置視覺冷卻時間，防止立即被視覺重複觸發
            self.last_visual_trigger_time = time.time()
            self.current_route_steps.clear() # 失敗就清空佇列

            # 🔴 修正：通知使用者導航失敗
            error_msg = "送餐中遇到問題，我回到待機模式喔。"
            self.publish_chat("小美", error_msg)
            threading.Thread(target=self._tts_and_play, args=(error_msg,)).start()

    def cancel_delivery(self):
        """🔴 新增：取消送餐任務（用於緊急情況）"""
        if self.is_navigating:
            self.nav_client.cancel_goal()  # 取消 Nav2 任務
            self.is_navigating = False
            self.is_processing = False
            self.current_route_steps.clear()

            cancel_msg = "已取消送餐任務。"
            self.publish_chat("小美", cancel_msg)
            self.get_logger().info("🛑 送餐任務已被取消")
        else:
            self.get_logger().info("沒有進行中的送餐任務")

    # ==========================================
    # ArUco Action 非同步接力區 (請貼在類別內)
    # ==========================================

    # def start_aruco_docking(self):
    #     """當導航抵達桌邊時呼叫此函數，啟動 ArUco 對位"""
    #     self.get_logger().info('📍 抵達桌邊附近，開始 ArUco 精準對位...')
        
    #     # 鎖上系統，避免小美這時候亂講話或亂跑
    #     self.is_docking = True 
        
    #     # 準備要發送的目標參數
    #     goal_msg = GotoAruco.Goal()
    #     goal_msg.distance_to_pose = 0.29 # 距離調整到 29 公分，根據實際測試微調
    #     goal_msg.linear_tolerance = 0.1
    #     goal_msg.orientation_tolerance = 0.01

    #     # 🌟 非同步發送目標！程式執行完這行會立刻往下走，不會卡死
    #     self.get_logger().info('📤 正在發送對位指令給底盤...')
    #     # 確保伺服器有在線上 (等待最多 3 秒)
    #     if not self.aruco_client.wait_for_server(timeout_sec=3.0):
    #         self.get_logger().error('❌ 找不到 ArUco 伺服器！')
    #         self.is_docking = False
    #         return
    #     self._send_goal_future = self.aruco_client.send_goal_async(goal_msg)
        
    #     # 交代系統：如果伺服器回覆了，請去執行 goal_response_callback
    #     self._send_goal_future.add_done_callback(self.goal_response_callback)

    # def goal_response_callback(self, future):
    #     """接力棒 1：確認伺服器有沒有接受我們的對位請求"""
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().warn('⚠️ ArUco 伺服器拒絕了對位請求！')
    #         self.is_docking = False # 解鎖
    #         self._tts_and_play("抱歉，我無法靠近桌子。")
    #         return

    #     self.get_logger().info('✅ ArUco 伺服器已接受請求，正在對位中...')
        
    #     # 交代系統：請繼續等，直到對位「完全結束」，再去執行 get_result_callback
    #     self._get_result_future = goal_handle.get_result_async()
    #     self._get_result_future.add_done_callback(self.get_result_callback)

    # def get_result_callback(self, future):
    #     """接力棒 2：對位完全結束，檢查是成功還是失敗"""
    #     result = future.result().result
    #     status = future.result().status

    #     # 這裡會判斷 GoToArucoActionServer 回傳的 success 變數
    #     if result.success:
    #         self.get_logger().info('🎉 ArUco 精準對位大成功！')
    #         self.on_aruco_complete() # 呼叫最後的完成動作
    #     else:
    #         self.get_logger().warn('❌ ArUco 對位失敗！')
    #         self.is_docking = False # 解鎖對位
    #         # 🔴 修正：ArUco 對位失敗時要完全解鎖系統狀態，避免系統卡住
    #         self.is_processing = False
    #         # 🔴 修正：重置視覺冷卻時間，這樣客人還可以重新觸發系統
    #         self.last_visual_trigger_time = time.time()
    #         self._tts_and_play("桌子好像有點歪，我對不準。")

    # def on_aruco_complete(self):
    #     """接力棒 3：對位成功後的完美收尾"""
        
    #     # 1. 解除底盤鎖，同時立刻鎖上大腦！(防 YOLO 亂入)
    #     self.is_docking = False  
    #     self.is_processing = True 

    #     # 2. 只開「一個」Thread 去跑完整的送餐後流程
    #     threading.Thread(target=self._post_delivery_routine, daemon=True).start()

    # 獨立出來的背景任務函數
    def _post_delivery_routine(self):
        try:
            # 步驟一：先講固定的送達台詞
            self.publish_chat("小美", "餐點已為您送達，請慢用！")
            self._tts_and_play("餐點已為您送達，請慢用！")

            # 步驟二：稍微停頓一下
            time.sleep(1.0)
            self.publish_chat("系統", "準備與客人互動中...")
            self.post_delivery_mode = True  # 🌟 標記進入 post-delivery 對話模式

            # 步驟三：詢問客人的選擇（笑話 vs 天氣）
            question_text = "請問有什麼需要我為您服務的嗎？"
            self.publish_chat("小美", question_text)
            self._tts_and_play(question_text)

            time.sleep(0.5)
            self.publish_chat("系統", "等待客人回答中...")

            # 步驟四：監聽客人的回答
            choice = self._listen_for_choice()

            # 步驟五：根據客人選擇執行對應的服務
            # if choice == 'joke':
            #     prompt = """你的唯一工作就是講個有趣、溫暖的笑話讓這位客人開心。

            #     【步驟】
            #     1. 講一個精簡的笑話（不要太長）
            #     2. 講完後，直接問「你覺得怎樣？」
            #     3. 等待客人的反應

            #     【重要】如果客人說以下任何話語，你就應該立即結束並說「祝您享用美食，有美好的一天！」：
            #     - "謝謝" / "感謝" / "不用了" / "不必"
            #     - "我要吃飯了" / "我先吃" / "拜拜" / "再見" / "可以了" / "好了"
            #     - 或任何表達滿足、完成、準備離開的意思"""
            #     self.publish_chat("系統", "客人選擇：笑話模式")

            # elif choice == 'weather':
            #     prompt = """你的唯一工作就是幫客人查詢天氣資訊。

            #     【步驟】
            #     1. 立即呼叫 get_weather_internal 工具查詢「台灣」的天氣
            #     2. 根據結果簡潔地告訴客人：天氣、氣溫、建議
            #     3. 直接問「這樣你知道要不要多加衣服了嗎？」
            #     4. 等待客人的反應

            #     【重要】如果客人說以下任何話語，你就應該立即結束並說「祝您享用美食，有美好的一天！」：
            #     - "謝謝" / "感謝" / "不用了" / "不必"
            #     - "我要吃飯了" / "我先吃" / "拜拜" / "再見" / "可以了" / "好了"
            #     - 或任何表達滿足、完成、準備離開的意思"""
            #     self.publish_chat("系統", "客人選擇：天氣模式")

            # 🔴 修正：改用簡潔的 if-else 結構，移除冗余的 None 檢查
            # 因為 _listen_for_choice() 只返回 None 或非空字符串
            # 所以進入 else 時，choice 必定是 None，不需再檢查
            if choice:
                prompt = f"""客人選擇了這個服務: {choice}。請根據客人的選擇提供對應的服務，並且在回覆中包含「？」來與客人互動，保持對話連續。

                【重要】如果客人說以下任何話語，你就應該立即結束並說「祝您享用美食，有美好的一天！」：
                - "謝謝" / "感謝" / "不用了" / "不必"
                - "我要吃飯了" / "我先吃" / "拜拜" / "再見" / "可以了" / "好了"
                - 或任何表達滿足、完成、準備離開的意思"""
                self.publish_chat("系統", f"客人選擇了這個服務: {choice}")

            else:
                # 客人沒有明確選擇或說了結束詞彙（choice 必定是 None）
                ending_text = "祝您享用美食，有美好的一天！"
                self.publish_chat("小美", ending_text)
                self._tts_and_play(ending_text)

                # 🔴 關鍵：立即設置長冷卻時間，防止視覺系統立刻重新觸發
                self.last_visual_trigger_time = time.time()
                time.sleep(0.5)  # 稍微停頓讓客人有時間反應

                self.post_delivery_mode = False
                self.is_processing = False  # 🔴 提前解鎖，不要等導航完成
                self.delivery_service("home")
                return

            # 步驟六：發送給大腦執行選定的服務
            self.process_gemini_and_speak(prompt, auto_listen=True)

        except Exception as e:
            self.get_logger().error(f"送餐後互動失敗: {e}")
            # 🔴 修正：萬一出錯，要完全解鎖系統，避免系統卡住
            self.is_processing = False
            self.post_delivery_mode = False
            self.last_visual_trigger_time = time.time()

    def _listen_for_choice(self):
        """🌟 監聽客人的選擇（笑話或天氣）"""
        try:
            text = self._streaming_stt()

            # 🔴 修正：檢查不只是 text 是否存在，還要檢查是否只含空白
            # 防止空白字符串被當成有效的客人回應
            if text and text.strip():
                text = text.strip()  # 清理前後空白

                # 判斷客人說的是笑話還是天氣
                # if any(word in text.lower() for word in ["笑話", "joke", "笑", "逗樂", "趣"]):
                #     return 'joke'
                # elif any(word in text.lower() for word in ["天氣", "weather", "天氣", "溫度", "氣溫", "下雨"]):
                #     return 'weather'

                # 🔴 修正：與 _voice_logic_task() 保持一致的結束詞彙列表
                # 確保兩個地方的檢查邏輯統一，防止客人說"回去"等詞被遺漏
                end_keywords = ["謝謝", "感謝", "不用了", "不必", "再見", "拜拜",
                               "可以了", "好了", "行了", "謝", "沒有了", "沒了",
                               "結束", "完了", "夠了", "不說了", "不講了", "不聊了",
                               "先這樣", "先這樣吧", "回家", "回廚房", "回去"]

                if any(end in text for end in end_keywords):
                    return None
                else:
                    return text  # 回傳原始文字讓 Gemini 判斷

            # 無回應則視為客人不想要服務
            return None

        except Exception as e:
            self.get_logger().error(f"監聽客人選擇失敗: {e}")
            return None

            


def main(args=None):
    rclpy.init(args=args)
    from rclpy.executors import MultiThreadedExecutor
    node = DeliveryBrain()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()