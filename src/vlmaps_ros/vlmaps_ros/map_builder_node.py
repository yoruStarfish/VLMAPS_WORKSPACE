import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess

class VLMapsBuilder(Node):
    def __init__(self):
        super().__init__('vlmaps_builder_node')
        self.srv = self.create_service(Trigger, '/build_vlmap', self.build_map_callback)
        self.get_logger().info("VLMaps 建圖啟動！隨時等待您的『建圖指令』...")

    def build_map_callback(self, request, response):
        self.get_logger().info("收到指令！開始進行 VLMaps 3D 語意建圖...")
        
        base_dir = os.path.expanduser('~/vlmaps_ws') 
        data_dir = os.path.join(base_dir, 'dataset') 
        
        # 修正：移除了字串開頭的斜線，確保路徑正確拼接
        script_path = os.path.join(base_dir, 'vlmapsSrc/run_map_builder.py') 

        try:
            self.get_logger().info("🧠 正在啟動 PyTorch 與 LSeg 模型，這可能需要幾分鐘...")
            
            process = subprocess.Popen(
                ['python3', script_path, '--data_dir', data_dir],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, # 把錯誤訊息也導向 stdout 一起印出
                universal_newlines=True
            )
            
            # 修正：使用 ROS 2 專用的 Logger 來逐行印出子程式的進度
            for line in process.stdout:
                # 使用 strip() 移除換行符號，避免輸出多餘的空白行
                clean_line = line.strip()
                if clean_line:
                    self.get_logger().info(f"[vlmaps_builder] {clean_line}")
            
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