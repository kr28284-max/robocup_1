import rclpy
from rclpy.node import Node
from srvs_pkg.srv import GetTargetPose
from std_srvs.srv import SetBool, Trigger
import time

class MasterNode2(Node):
    def __init__(self):
        super().__init__('master_node')
        self.cli_v = self.create_client(GetTargetPose, '/get_target_pose')
        self.cli_r = self.create_client(GetTargetPose, '/robot_move_step')
        self.cli_g = self.create_client(SetBool, '/control_gripper')
        self.cli_h = self.create_client(Trigger, '/robot_home')
        
        self.Z_OFF = -85.0
        self.Z_MARGIN = 20.0
        self.BLOCK_H = 16.0
        self.WAIT_TIME = 1.5 

    def call(self, cli, req):
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {cli.srv_name}...')
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def check_color_exists(self, color):
        p = self.call(self.cli_v, GetTargetPose.Request(target_color=color))
        return p.success

    def find_target_with_retry(self, color, retries=4):
        for i in range(retries):
            p = self.call(self.cli_v, GetTargetPose.Request(target_color=color))
            if p.success:
                return p
            self.get_logger().warn(f"⚠️ [{color}] 찾는 중... 대기 ({i+1}/{retries})")
            time.sleep(1.0) 
        return None

    def pick_target(self, color):
        self.get_logger().info(f"\n--- PICK TARGET: [{color.upper()}] ---")
        
        p = self.find_target_with_retry(color)
        if not p: return False
        req = GetTargetPose.Request(); req.yaw = p.yaw; req.target_size = "YAW"
        self.call(self.cli_r, req)
        time.sleep(self.WAIT_TIME) 

        p = self.find_target_with_retry(color)
        if not p: return False
        req = GetTargetPose.Request(); req.x = p.x; req.y = p.y; req.target_size = "XY"
        self.call(self.cli_r, req)
        time.sleep(self.WAIT_TIME) 

        p = self.find_target_with_retry(color)
        if not p: return False
        z_move = p.z * 1000.0 + self.Z_OFF
        self.call(self.cli_r, GetTargetPose.Request(z=z_move - self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME) 
        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME) 

        self.call(self.cli_g, SetBool.Request(data=True))
        time.sleep(self.WAIT_TIME) 
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME) 
        return True

    # 💡 yaw_offset 매개변수를 추가했습니다. (기본값은 0.0)
    def insert_to_target(self, color, yaw_offset=0.0):
        self.get_logger().info(f"\n--- STACK ON: [{color.upper()}] (Yaw Offset: +{yaw_offset}도) ---")
        time.sleep(1.0)

        # 1. YAW
        p = self.find_target_with_retry(color, retries=5)
        if not p: return False
        req_y = GetTargetPose.Request()
        # 💡 카메라가 측정한 기본 yaw 값에 우리가 원하는 추가 각도(offset)를 더해서 로봇에 명령을 내립니다.
        req_y.yaw = p.yaw + yaw_offset 
        req_y.target_size = "YAW"
        self.call(self.cli_r, req_y)
        time.sleep(self.WAIT_TIME) 
        
        # 2. XY
        p = self.find_target_with_retry(color)
        if not p: return False
        req_xy = GetTargetPose.Request(); req_xy.x = p.x; req_xy.y = p.y; req_xy.target_size = "XY"
        self.call(self.cli_r, req_xy)
        time.sleep(self.WAIT_TIME) 

        # 3. Z
        p = self.find_target_with_retry(color)
        if not p: return False
        z_move = (p.z * 1000.0 + self.Z_OFF) - self.BLOCK_H
        self.call(self.cli_r, GetTargetPose.Request(z=z_move - self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME) 
        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME) 

        # 4. Open
        self.call(self.cli_g, SetBool.Request(data=False))
        time.sleep(self.WAIT_TIME) 
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME) 
        return True

    def run(self):
        self.get_logger().info("🚀 STARTING 3-STEP ASSEMBLY SEQUENCE")
        
        self.call(self.cli_h, Trigger.Request())
        self.call(self.cli_g, SetBool.Request(data=False))
        time.sleep(1.0) 
        
        # ---------------------------------------------------------
        # [당근 조립] 1.노란색 -> 2.노란색 -> 3.초록색
        # ---------------------------------------------------------

        self.get_logger().info("🥕 [당근 조립] 1단계: 노란색(2x2) -> 노란색(2x2)")
        if self.check_color_exists("2x2_yellow"):
            self.get_logger().info("🎯 1차 조건 충족! (노랑 2x2 픽 -> 노랑 2x2 스택)")
            if self.pick_target("2x2_yellow"):
                self.call(self.cli_h, Trigger.Request())
                if self.insert_to_target("2x2_yellow"):
                    self.get_logger().info("✅ 당근 1단계 완료")
                    
                    self.call(self.cli_h, Trigger.Request())
                    self.get_logger().info("🥕 [당근 조립] 2단계: 초록색(2x2) -> 노란색(2x2)")
                    if self.check_color_exists("2x2_blue") and self.check_color_exists("2x2_yellow"):
                        self.get_logger().info("🎯 2차 조건 충족! (초록 2x2 픽 -> 노랑 2x2 스택)")
                        if self.pick_target("2x2_blue"):
                            self.call(self.cli_h, Trigger.Request())
                            if self.insert_to_target("2x2_yellow"):
                                self.get_logger().info("✅ 당근 완성!")
                            else:
                                self.get_logger().error("❌ 노란색 스택 실패!")
                        else:
                            self.get_logger().error("❌ 초록색 픽업 실패!")
                    else:
                        self.get_logger().info("⏭️ 2차 작업(초록->노랑) 생략.")
                        
                else:
                    self.get_logger().error("❌ 노란색 스택 실패!")
            else:
                self.get_logger().error("❌ 노란색 픽업 실패!")
        else:
            self.get_logger().info("⏭️ 1차 작업(노랑->노랑) 생략.")
        
        # ---------------------------------------------------------
        # [신호등 조립] 1.초록색 -> 2.노란색 -> 3.빨간색
        # ---------------------------------------------------------
        self.call(self.cli_h, Trigger.Request())
        
        self.get_logger().info("🚦 [신호등 조립] 1단계: 노란색(2x2) -> 초록색(2x2)")
        if self.check_color_exists("2x2_yellow") and self.check_color_exists("2x2_blue"):
            self.get_logger().info("🎯 1차 조건 충족! (노랑 2x2 픽 -> 초록 2x2 스택)")
            if self.pick_target("2x2_yellow"):
                self.call(self.cli_h, Trigger.Request())
                if self.insert_to_target("2x2_blue"):
                    self.get_logger().info("✅ 신호등 1단계 완료")
                    
                    self.call(self.cli_h, Trigger.Request())
                    self.get_logger().info("🚦 [신호등 조립] 2단계: 빨간색(2x2) -> 노란색(2x2)")
                    if self.check_color_exists("2x2_red") and self.check_color_exists("2x2_yellow"):
                        self.get_logger().info("🎯 2차 조건 충족! (빨강 2x2 픽 -> 노랑 2x2 스택)")
                        if self.pick_target("2x2_red"):
                            self.call(self.cli_h, Trigger.Request())
                            if self.insert_to_target("2x2_yellow"):
                                self.get_logger().info("✅ 신호등 완성!")
                            else:
                                self.get_logger().error("❌ 노란색 스택 실패!")
                        else:
                            self.get_logger().error("❌ 빨간색 픽업 실패!")
                    else:
                        self.get_logger().info("⏭️ 2차 작업(빨강->노랑) 생략.")
                        
                else:
                    self.get_logger().error("❌ 초록색 스택 실패!")
            else:
                self.get_logger().error("❌ 노란색 픽업 실패!")
        else:
            self.get_logger().info("⏭️ 1차 작업(노랑->초록) 생략.")

        # ---------------------------------------------------------
        # 최종 종료
        # ---------------------------------------------------------
        self.call(self.cli_h, Trigger.Request())
        self.get_logger().info("🎉 ALL SEQUENCE DONE")

def main():
    rclpy.init()
    node = MasterNode2()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()