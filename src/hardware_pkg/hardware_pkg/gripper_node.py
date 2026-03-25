# import rclpy
# from rclpy.node import Node
# from std_srvs.srv import SetBool
# import serial
# import time

# class GripperNode(Node):
#     def __init__(self):
#         super().__init__('gripper_node')
#         self.srv = self.create_service(SetBool, 'control_gripper', self.control_cb)
#         try:
#             self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
#             time.sleep(2.0)
#             self.get_logger().info("✅ Gripper Serial Connected")
#         except Exception as e:
#             self.get_logger().error(f"❌ Serial Error: {e}")

#     def control_cb(self, request, response):
#         if request.data: # True -> Grip
#             self.ser.write(b"grip\n")
#             response.message = "Grip Success"
#         else: # False -> Open
#             self.ser.write(b"open\n")
#             response.message = "Open Success"
#         response.success = True
#         return response

# def main():
#     rclpy.init()
#     rclpy.spin(GripperNode())
#     rclpy.shutdown()

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import serial
import time

class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')
        self.srv = self.create_service(SetBool, 'control_gripper', self.control_cb)
        
        try:
            # 시리얼 포트 연결 (노트북 설정에 맞춰 ACM0 또는 ACM1 확인 필요)
            self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
            
            # 아두이노/그리퍼 컨트롤러 리셋 대기
            time.sleep(2.0)
            self.get_logger().info("✅ Gripper Serial Connected")

            # [수정 사항] 노드 시작 시 자동으로 그리퍼를 엽니다.
            self.get_logger().info("➡️ Initializing Gripper: Sending 'open'...")
            self.ser.write(b"open\n")
            
        except Exception as e:
            self.get_logger().error(f"❌ Serial Error: {e}")

    def control_cb(self, request, response):
        """
        Service Callback
        request.data 가 True면 grip, False면 open 명령을 보냅니다.
        """
        try:
            if request.data:  # True -> Grip
                self.ser.write(b"grip\n")
                self.get_logger().info("📌 Sent: grip")
                response.message = "Grip Command Sent"
            else:             # False -> Open
                self.ser.write(b"open\n")
                self.get_logger().info("📌 Sent: open")
                response.message = "Open Command Sent"
            
            response.success = True
        except Exception as e:
            self.get_logger().error(f"❌ Service Error: {e}")
            response.success = False
            response.message = str(e)
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
            node.get_logger().info("✅ Serial Closed")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()