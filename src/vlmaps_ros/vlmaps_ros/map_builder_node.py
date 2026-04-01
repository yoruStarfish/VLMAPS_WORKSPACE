import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess
from pathlib import Path


class VLMapsBuilder(Node):
    def __init__(self):
        super().__init__('vlmaps_builder_node')
        # 建立一個名為 /build_vlmap 的服務 (Service)
        self.srv = self.create_service(Trigger, '/build_vlmap', self.build_map_callback)
        self.get_logger().info("✅ VLMaps 建圖大腦已啟動！隨時等待您的『建圖指令』...")

    def build_map_callback(self, request, response):
        self.get_logger().info("🚀 收到指令！開始進行 VLMaps 3D 語意建圖...")
        
        # 你收集資料的資料夾路徑 (剛好對應 data_collector 存的地方)
        current_file = Path(__file__).resolve()
        base_dir = '/home;/robotic/vlmaps_ws' # 以使用者主目錄為基底
        data_dir = os.path.join(base_dir, 'dataset') # 你可以改成你想要的資料夾
        
        # 你剛剛存檔的獨立建圖腳本路徑
        script_path = '/home/robotic/vlmaps/run_map_builder.py' # 請替換成你實際的路徑

        try:
            self.get_logger().info("🧠 正在啟動 PyTorch 與 LSeg 模型，這可能需要幾分鐘...")
            
            # 使用 subprocess 呼叫外部程式，這樣跑完後系統會自動把珍貴的 GPU 記憶體釋放掉！
            process = subprocess.Popen(
                ['python3', script_path, '--data_dir', data_dir],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True
            )
            
            # 即時印出 AI 模型的下載進度或處理進度條
            for line in process.stdout:
                print(line, end='') 
            
            process.wait()

            if process.returncode == 0:
                response.success = True
                response.message = "🎉 建圖大功告成！地圖已儲存完畢。"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = "❌ 建圖失敗，請檢查上方錯誤訊息。"
                self.get_logger().error(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f"系統呼叫失敗: {str(e)}"
            self.get_logger().error(response.message)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = VLMapsBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()