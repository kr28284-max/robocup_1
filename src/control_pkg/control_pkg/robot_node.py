import rclpy
from rclpy.node import Node
from srvs_pkg.srv import GetTargetPose
from std_srvs.srv import Trigger
import rbpodo as rb
import numpy as np

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')
        self.robot = rb.Cobot("10.0.2.7")
        self.rc = rb.ResponseCollector()
        self.robot.set_operation_mode(self.rc, rb.OperationMode.Real)
        
        self.srv_home = self.create_service(Trigger, '/robot_home', self.home_cb)
        self.srv_move = self.create_service(GetTargetPose, '/robot_move_step', self.move_step_cb)
        
        # 오프셋 및 속도 설정
        self.CAM_X_OFF = -51.0
        self.CAM_Y_OFF = 32.0
        self.L_VEL = 500
        self.L_ACC = 800
        
        self.get_logger().info("✅ Robot Node Ready")

    def wait_move(self, name="MOVE"):
        # 1초 동안 이동 시작을 기다림
        started = self.robot.wait_for_move_started(self.rc, 1.0).is_success()
        if not started:
            self.get_logger().warn(f"⚠️ {name} START SKIPPED (이미 정렬되어 있거나 하드웨어 무시됨)")
            return True
            
        self.robot.wait_for_move_finished(self.rc)
        return True

    def home_cb(self, req, res):
        self.robot.move_j(self.rc, np.array([-90.0, 0.0, 90.0, 0.0, 90.0, 0.0], dtype=float), 255, 255)
        self.wait_move("HOME")
        res.success = True
        return res

    def move_step_cb(self, req, res):
        try:
            if req.target_size == "YAW":
                # 💡 핵심: Yaw 값이 0에 가까우면 (예: 0.5도 미만) 이동 생략
                if abs(req.yaw) < 0.01:
                    self.get_logger().info(f"⏭️ YAW 값이 0에 가까움({req.yaw:.2f}도). 정렬을 생략하고 진행합니다.")
                    res.success = True
                    return res
                    
                self.robot.move_l_rel(self.rc, np.array([0,0,0,0,0, req.yaw], dtype=float), self.L_VEL, self.L_ACC, rb.ReferenceFrame.Tool)
                self.wait_move("YAW")
                
            elif req.target_size == "XY":
                dx = -(req.x * 1000.0) + self.CAM_Y_OFF
                dy = (req.y * 1000.0) + self.CAM_X_OFF
                self.robot.move_l_rel(self.rc, np.array([dy, dx, 0, 0, 0, 0], dtype=float), self.L_VEL, self.L_ACC, rb.ReferenceFrame.Tool)
                self.wait_move("XY")
                
            elif req.target_size == "Z":
                self.robot.move_l_rel(self.rc, np.array([0, 0, req.z, 0, 0, 0], dtype=float), self.L_VEL, self.L_ACC, rb.ReferenceFrame.Tool)
                self.wait_move(f"Z_MOVE({req.z:.1f})")
            
            res.success = True
        except Exception as e:
            self.get_logger().error(f"❌ Move Error: {e}")
            res.success = False
            
        return res

def main():
    rclpy.init()
    rclpy.spin(RobotNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()