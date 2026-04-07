import rclpy
from rclpy.node import Node
from srvs_pkg.srv import GetTargetPose
from std_srvs.srv import SetBool, Trigger
import time
import math

class MasterNode(Node):
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
       
        self.STUD_PITCH = 0.016
        self.YAW_TUNE = 0.0  # 필요시 여기서 영점 조절 (예: -1.5)
       
        # 🌟 가장 깨끗하게 인식되었을 때의 타겟 좌표를 기억하는 변수
        self.last_perfect_pose = None

    def call(self, cli, req):
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {cli.srv_name}...')
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def count_color(self, color):
        p = self.call(self.cli_v, GetTargetPose.Request(target_color=f"count_{color}"))
        return int(p.x) if p.success else 0

    def find_target_with_retry(self, color, retries=4):
        for i in range(retries):
            p = self.call(self.cli_v, GetTargetPose.Request(target_color=color))
            if p.success:
                return p
            self.get_logger().warn(f"⚠️ [{color}] 타겟 찾는 중... ({i+1}/{retries})")
            time.sleep(1.0)
        return None
    
    def get_dist(self, p1, p2):
        """두 지점 사이의 평면 거리(m)를 계산"""
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
    # =========================================================================
    # Big_Tree,Ice_cream만 pick_fresh_target사용
    # ==========================================================================
    def pick_fresh_target(self, color, exclude_pose=None, threshold=0.035, layer_index=0):
        # 🌟 좌표 필터 방식 삭제. 대신 'far_' 접두사를 붙여 로봇이 이동해도 무조건 멀리 있는 블록을 추적
        target_req = f"far_{color}"
        
        target_p = None
        for i in range(5):
            p = self.call(self.cli_v, GetTargetPose.Request(target_color=target_req))
            if p.success:
                target_p = p 
                break
            time.sleep(1.0)

        if not target_p:
            return False

        # --- 로봇 이동 시작 ---
        # [STEP 1] YAW 회전
        req_y = GetTargetPose.Request(); req_y.yaw = target_p.yaw; req_y.target_size = "YAW"
        self.call(self.cli_r, req_y)
        time.sleep(self.WAIT_TIME)

        # 🌟 [STEP 2] XY 이동 (회전 후에도 가까운 베이스 무시하고 계속 멀리 있는 블록 추적)
        p_retry = self.find_target_with_retry(target_req)
        if not p_retry: return False
        req_xy = GetTargetPose.Request(); req_xy.x = p_retry.x; req_xy.y = p_retry.y; req_xy.target_size = "XY"
        self.call(self.cli_r, req_xy)
        time.sleep(self.WAIT_TIME)

        # [STEP 3] Z 하강 및 집기
        p_z = self.find_target_with_retry(target_req)
        if not p_z: return False
        
        z_move = (p_z.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
        self.call(self.cli_r, GetTargetPose.Request(z=z_move - self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME)

        self.call(self.cli_g, SetBool.Request(data=True))
        time.sleep(self.WAIT_TIME)
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        return True


    def pick_target(self, color, layer_index=0, offset_studs_x=0.0, offset_studs_y=0.0, exclude_pose=None):
        self.get_logger().info(f"\n--- PICK TARGET: [{color.upper()}] ---")
        p = self.find_target_with_retry(color)
        if not p: return False
       
        req = GetTargetPose.Request(); req.yaw = p.yaw; req.target_size = "YAW"
        self.call(self.cli_r, req)
        time.sleep(self.WAIT_TIME)

        p = self.find_target_with_retry(color)
        if not p: return False

        dx = offset_studs_x * self.STUD_PITCH
        dy = offset_studs_y * self.STUD_PITCH
        yaw_rad = math.radians(p.yaw)
        real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
        real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

        target_x = p.x + real_offset_x
        target_y = p.y + real_offset_y

        req = GetTargetPose.Request(); req.x = target_x; req.y = target_y; req.target_size = "XY"
        self.call(self.cli_r, req)
        time.sleep(self.WAIT_TIME)

        p = self.find_target_with_retry(color)
        if not p: return False
        z_move = (p.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
        self.call(self.cli_r, GetTargetPose.Request(z=z_move - self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME)

        self.call(self.cli_g, SetBool.Request(data=True))
        time.sleep(self.WAIT_TIME)
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        return True

    def blind_insert(self, base_pose, layer_index, yaw_offset=0.0, release_gripper=True, regrip=False, offset_studs_x=0.0, offset_studs_y=0.0):
        self.get_logger().info(f"\n--- BLIND STACK (메모리 사용): Layer {layer_index} (Y Offset: {offset_studs_y}) ---")
        time.sleep(1.0)

        dx = offset_studs_x * self.STUD_PITCH
        dy = offset_studs_y * self.STUD_PITCH

        yaw_rad = math.radians(base_pose.yaw)
        real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
        real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

        target_x = base_pose.x + real_offset_x
        target_y = base_pose.y + real_offset_y

        req_xy = GetTargetPose.Request()
        req_xy.x = target_x; req_xy.y = target_y; req_xy.target_size = "XY"
        self.call(self.cli_r, req_xy)
        time.sleep(self.WAIT_TIME)

        target_yaw = base_pose.yaw + yaw_offset + self.YAW_TUNE
        while target_yaw > 90.0: target_yaw -= 180.0
        while target_yaw < -90.0: target_yaw += 180.0

        self.get_logger().info(f"🔄 [YAW 회전] 계산된 고정 각도 회전: {target_yaw:.1f}도")
        req_y = GetTargetPose.Request()
        req_y.yaw = target_yaw; req_y.target_size = "YAW"
        self.call(self.cli_r, req_y)
        time.sleep(self.WAIT_TIME)

        z_move = (base_pose.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
       
        self.call(self.cli_r, GetTargetPose.Request(z=z_move - self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN , target_size="Z"))
        time.sleep(self.WAIT_TIME)

        

        if release_gripper:
            self.call(self.cli_g, SetBool.Request(data=False))
            time.sleep(self.WAIT_TIME)
        # self.call(self.cli_r, GetTargetPose.Request(z=30.0, target_size="Z"))
        # time.sleep(self.WAIT_TIME)
        return True

    def visual_insert(self, target_color, layer_index, release_gripper=True, yaw_offset=0.0, offset_studs_x=0.0, offset_studs_y=0.0):
        self.get_logger().info(f"\n--- VISUAL STACK: [{target_color.upper()}] (Layer +{layer_index}, Y Offset: {offset_studs_y}) ---")
        time.sleep(1.0)

        p = self.find_target_with_retry(target_color)
        if not p: return False
       
        target_yaw = p.yaw + yaw_offset + self.YAW_TUNE
        while target_yaw > 90.0: target_yaw -= 180.0
        while target_yaw < -90.0: target_yaw += 180.0

        self.get_logger().info(f"🔄 [YAW 회전] 시각 보정 기반 회전: {target_yaw:.1f}도")
        req_y = GetTargetPose.Request()
        req_y.yaw = target_yaw; req_y.target_size = "YAW"
        self.call(self.cli_r, req_y)
        time.sleep(self.WAIT_TIME)

        p = self.find_target_with_retry(target_color)
        if not p: return False

        # 메모리 로직
        self.last_perfect_pose = p

        dx = offset_studs_x * self.STUD_PITCH
        dy = offset_studs_y * self.STUD_PITCH
        yaw_rad = math.radians(p.yaw)
        real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
        real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

        target_x = p.x + real_offset_x
        target_y = p.y + real_offset_y

        self.get_logger().info(f"➡️ [XY 이동] 시각 보정 기반 최적 오프셋 적용")
        req_xy = GetTargetPose.Request()
        req_xy.x = target_x; req_xy.y = target_y; req_xy.target_size = "XY"
        self.call(self.cli_r, req_xy)
        time.sleep(self.WAIT_TIME)

        z_move = (p.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
       
        self.call(self.cli_r, GetTargetPose.Request(z=z_move - self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN , target_size="Z"))
        time.sleep(self.WAIT_TIME)

        if release_gripper:
            self.call(self.cli_g, SetBool.Request(data=False))
            time.sleep(self.WAIT_TIME)
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        return True

    def get_best_build_plan(self, current_inventory):
        recipes = {
            'studs_y': {'4x2_red': 2, '2x2_red': 2, '2x2_yellow': 1},
            'battery': {'2x2_yellow': 1, '2x2_blue': 1},
            'magnet': {'2x2_blue': 1, '2x2_red': 1},
            'e_stop': {'2x2_red': 1, '4x2_yellow': 1},
            'carrot': {'2x2_blue': 1, '2x2_yellow': 2},
            'traffic_light': {'2x2_red': 1, '2x2_yellow': 1, '2x2_blue': 1},
            'small_tree': {'2x2_red': 1, '4x2_red': 1, '2x2_yellow': 1},
            'hammer': {'4x2_blue': 1, '2x2_red': 2},
            'big_carrot': {'2x2_yellow': 2, '4x2_yellow': 1, '2x2_blue': 1},
            'burger': {'4x2_yellow': 2, '4x2_red': 1, '2x2_red': 1},
            'ice_cream': {'2x2_yellow': 2, '4x2_yellow': 1, '2x2_red': 1, '2x2_blue': 1},
            'big_tree': {'2x2_yellow': 1, '2x2_red': 2, '4x2_red': 2}
        }
        best_plan = []
        min_remainder = sum(current_inventory.values())b

        def dfs(inv, current_plan):
            nonlocal best_plan, min_remainder
            made_any = False
            for name, recipe in recipes.items():
                can_make = True
                for color, count in recipe.items():
                    if inv.get(color, 0) < count:
                        can_make = False
                        break
                if can_make:
                    made_any = True
                    new_inv = inv.copy()
                    for color, count in recipe.items():
                        new_inv[color] -= count
                    dfs(new_inv, current_plan + [name])
           
            if not made_any:
                remainder = sum(inv.values())
                if remainder < min_remainder:
                    min_remainder = remainder
                    best_plan = current_plan
                elif remainder == min_remainder:
                    if len(current_plan) < len(best_plan):
                        best_plan = current_plan

        dfs(current_inventory, [])
        return best_plan

    # --- 2~3개 조합 (Visual Insert 적용) ---

    def build_battery(self):
        self.get_logger().info("🔋 [배터리] 노란색(Pick) -> 파란색(Base)")
        if self.pick_target("2x2_yellow"):
            self.call(self.cli_h, Trigger.Request())
            self.visual_insert("2x2_blue", layer_index=1)
            self.get_logger().info("✅ 배터리 조립 완료!")

    def build_magnet(self):
        self.get_logger().info("🧲 [자석] 파란색(Pick) -> 빨간색(Base)")
        if self.pick_target("2x2_blue"):
            self.call(self.cli_h, Trigger.Request())
            self.visual_insert("2x2_red", layer_index=1)
            self.get_logger().info("✅ 자석 조립 완료!")

    def build_e_stop(self):
        self.get_logger().info("🛑 [비상정지] 빨간색(Pick) -> 노란색4x2(Base)")
        if self.pick_target("2x2_red"):
            self.call(self.cli_h, Trigger.Request())
            self.visual_insert("4x2_yellow", layer_index=1, yaw_offset=-90.0)
            self.get_logger().info("✅ 비상정지 조립 완료!")

    def build_carrot(self):
        self.get_logger().info("🥕 [당근] 노란색(Pick) -> 파란색(Base) -> 노란색(Pick)")
        if self.pick_target("2x2_yellow"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("2x2_blue", layer_index=1):
                self.call(self.cli_h, Trigger.Request())
                if self.pick_target("2x2_yellow"):
                    self.call(self.cli_h, Trigger.Request())
                    # 1층에 놓인 노란색을 베이스로 삼아 1층 높이 더 올리기
                    self.visual_insert("2x2_yellow", layer_index=1)
                    self.get_logger().info("✅ 당근 완성!")

    def build_traffic_light(self):
        self.get_logger().info("🚦 [신호등] 노란색(Pick) -> 파란색(Base) -> 빨간색(Pick)")
        if self.pick_target("2x2_yellow"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("2x2_blue", layer_index=1):
                self.call(self.cli_h, Trigger.Request())
                if self.pick_target("2x2_red"):
                    self.call(self.cli_h, Trigger.Request())
                    # 파란색이 가려졌으니, 방금 놓은 노란색을 타겟으로!
                    self.visual_insert("2x2_yellow", layer_index=1)
                    self.get_logger().info("✅ 신호등 완성!")

    def build_small_tree(self):
        self.get_logger().info("🌳 [작은 나무] 빨강4x2(Pick) -> 노랑2x2(Base) -> 빨강2x2(Pick)")
        if self.pick_target("4x2_red"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("2x2_yellow", layer_index=1):
                self.call(self.cli_h, Trigger.Request())
                if self.pick_target("2x2_red"):
                    self.call(self.cli_h, Trigger.Request())
                    # 가려진 2x2 노랑 대신, 방금 놓은 4x2 빨강을 타겟으로!
                    self.visual_insert("4x2_red", layer_index=1)
                    self.get_logger().info("✅ 작은 나무 완성!")

    def build_hammer(self):
        self.get_logger().info("🔨 [망치] 빨강2x2(Pick) -> 빨강2x2(Base) -> 파랑4x2(Pick)")
        if self.pick_target("2x2_red"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("2x2_red", layer_index=1):
                self.call(self.cli_h, Trigger.Request())
                if self.pick_target("4x2_blue"):
                    self.call(self.cli_h, Trigger.Request())
                    # 가려진 0층 빨강 대신 1층 빨강을 타겟으로! 간섭 회피용 90도 회전
                    self.visual_insert("2x2_red", layer_index=1, yaw_offset=0.0)
                    self.get_logger().info("✅ 망치 완성!")

    # --- 4개 조합 (Big Carrot, Burger) ---
    def build_big_carrot(self):
        self.get_logger().info("🥕🥕 [큰 당근] 노랑2x2(Pick) -> 노랑2x2(Base) -> 노랑4x2(Pick) -> 파랑2x2(Pick)")
        if self.pick_target("2x2_yellow"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("2x2_yellow", layer_index=1):
               
                self.call(self.cli_h, Trigger.Request())
                if self.pick_target("4x2_yellow"):
                    self.call(self.cli_h, Trigger.Request())
                    if self.visual_insert("2x2_yellow", layer_index=2, yaw_offset=0.0):
                       
                        self.call(self.cli_h, Trigger.Request())
                        if self.pick_target("2x2_blue"):
                            self.call(self.cli_h, Trigger.Request())
                            self.visual_insert("4x2_yellow", layer_index=1)
                            self.get_logger().info("✅ 대왕 당근 완성!")

    def build_burger(self):
        self.get_logger().info("🍔 [버거] 노랑4x2(Base) -> 빨강4x2(Offset Y -1) -> 빨강2x2(Offset Y +2) -> 노랑4x2(Top)")
        if self.pick_target("4x2_red"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("4x2_yellow", layer_index=1, offset_studs_y=-1.0):
                saved_base_bun_pose = self.last_perfect_pose
               
                self.call(self.cli_h, Trigger.Request())
                if self.pick_target("2x2_red"):
                    self.call(self.cli_h, Trigger.Request())
                   
                    # 🌟 간섭 회피를 위해 yaw_offset=0.0 으로 복구
                    if self.visual_insert("4x2_red", layer_index=0, yaw_offset=0.0, offset_studs_y=3.0):
                       
                        self.call(self.cli_h, Trigger.Request())
                        if self.pick_target("4x2_yellow"):
                            self.call(self.cli_h, Trigger.Request())
                           
                            if saved_base_bun_pose:
                                self.get_logger().info("🧠 [메모리 사용] 덩어리 인식 오류 방지: 최초 바닥 빵의 좌표를 기억해서 정중앙에 덮습니다!")
                                self.blind_insert(saved_base_bun_pose, layer_index=2, offset_studs_y=1.0)
                                self.get_logger().info("✅ 버거 완성!")
                            else:
                                self.get_logger().warn("❌ 저장된 좌표가 없습니다. 조립 실패.")

    def build_ice_cream(self):
        self.get_logger().info("🍦 [아이스크림] 모듈형 조립 전략")

        self.get_logger().info("[Phase 1] 하단 조립: 노랑4x2(Pick) -> 노랑2x2(Base)")
        if self.pick_target("4x2_yellow"):
            self.call(self.cli_h, Trigger.Request())
            if not self.visual_insert("2x2_yellow", layer_index=1):
                self.get_logger().warn("❌ 하단 모듈 조립 실패")
                return

        self.get_logger().info("[Phase 2-1] 파랑2x2(Pick) -> 빨강2x2 옆에 배치 (바닥)")
        self.call(self.cli_h, Trigger.Request())
        if self.pick_target("2x2_blue"):
            self.call(self.cli_h, Trigger.Request())
            if not self.visual_insert("2x2_red", layer_index=0, offset_studs_y=2.0):
                self.get_logger().warn("❌ 파란색 블록 배치 실패")
                return

        self.get_logger().info("[Phase 2-2] 노랑2x2(Pick) -> 파랑2x2(Base, offset_y=-1)로 결합")
        self.call(self.cli_h, Trigger.Request())
        if self.pick_target("2x2_yellow"):
            self.call(self.cli_h, Trigger.Request())
            if not self.visual_insert("2x2_blue", layer_index=1, offset_studs_y=-1.0):
                self.get_logger().warn("❌ 상단 모듈 결합 실패")
                return

        self.get_logger().info(" [Phase 3] 최종 결합: 상단 모듈 들어서 하단 모듈(노랑4x2) 위에 꽂기!")
        self.call(self.cli_h, Trigger.Request())
        if self.pick_target("2x2_yellow", layer_index=0.5):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("4x2_yellow", layer_index=2):
                self.get_logger().info("✅🎉 5단 아이스크림 완벽하게 완성!")
            else:
                self.get_logger().warn("❌ 최종 층 올리기 실패")

    # ==========================================
    # 예제 studs_y
    # ==========================================

    # def build_studs_y(self):
    #     self.get_logger().info("🧱 [Phase 1] 4x2 red(-1.0) -> 2x2 yellow(0.0)")
    #     if self.pick_target("4x2_red", offset_studs_y=-1.025):
    #         self.call(self.cli_h, Trigger.Request())
    #         if self.visual_insert("2x2_yellow", layer_index=1, offset_studs_y=0.0):
    #             # Phase 1 베이스(2x2_yellow) 위치를 메모리에 저장
    #             saved_phase1_base = self.last_perfect_pose
    #             self.call(self.cli_h, Trigger.Request())

    #             self.get_logger().info("🧱 [Phase 2] 4x2 red(-1.85) -> 2x2 red(0.0) (그리퍼 유지)")
    #             if self.pick_target("4x2_red", offset_studs_y=-1.825):
    #                 self.call(self.cli_h, Trigger.Request())
    #                 # 그리퍼 열지 않고 결합
    #                 if self.visual_insert("2x2_red", layer_index=0.7, offset_studs_y=0.0, release_gripper=False):
                        
    #                     self.get_logger().info("🧱 [Phase 3] 페이즈 1 위치로 이동하여 6x2 결합 (메모리 사용)")
    #                     self.call(self.cli_h, Trigger.Request())
    #                     # Phase 1 베이스 옆에 나란히 결합하여 6x2를 만듦 (방향에 따라 -2.0일 수 있음)
    #                     if self.blind_insert(saved_phase1_base, layer_index=2, offset_studs_y=-2.0):
                            
    #                         self.get_logger().info("🧱 [Phase 4] 2x2 red -> 6x2 중앙에 배치 (메모리 사용)")
    #                         self.call(self.cli_h, Trigger.Request())
    #                         if self.pick_target("2x2_red", offset_studs_y=0.0):
    #                             self.call(self.cli_h, Trigger.Request())
    #                             # Phase 1 베이스(0.0)와 Phase 2 베이스(2.0)의 정중앙(1.0) 3층(layer_index=2)에 결합
    #                             if self.blind_insert(saved_phase1_base, layer_index=2, offset_studs_y=1.0):
    #                                 self.get_logger().info("✅ 최종 조립 완료!")

    # def build_studs_y(self):
    #     self.get_logger().info("🧱 [STUDS_Y] 4x2 앞단 파지 -> 2x2 결합(유지) -> 4x2 추가 결합")
        
    #     if self.pick_target("4x2_red", offset_studs_y=-1.85):
    #         self.call(self.cli_h, Trigger.Request())
            
    #         # 1. 2x2_red에 먼저 결합하고 그리퍼를 열지 않음 (release_gripper=False)
    #         if self.visual_insert("2x2_red", layer_index=1, offset_studs_y=0.0, release_gripper=False):
    #             self.call(self.cli_h, Trigger.Request())
                
    #             # 2. 남은 비어있는 공간을 바닥의 다른 4x2_red에 결합하여 6x2 형태로 만듦 (그리퍼 해제)
    #             # (offset_studs_y 수치는 실제 방향에 따라 3.0 또는 -3.0 등으로 조정 필요)
    #             self.visual_insert("4x2_red", layer_index=1, offset_studs_y=-3.0, release_gripper=True)
    #             self.get_logger().info("✅ STUDS_Y 6x2 조립 완료!")

    def build_studs_y(self):
        self.get_logger().info("🧱 [초기화] 맨 처음 2x2_yellow 위치 스캔 및 기억")
        # 페이즈 4를 위해 노란색 블록의 위치를 가장 먼저 찾아 저장합니다.
        p_yellow = self.find_target_with_retry("2x2_yellow")
        if not p_yellow:
            self.get_logger().warn("❌ 바닥에 2x2_yellow가 안 보입니다. 조립을 취소합니다.")
            return
        saved_yellow_pose = p_yellow
        self.call(self.cli_h, Trigger.Request())

        self.get_logger().info("🧱 [Phase 1] 4x2_red(-1.85) 파지 -> 2x2_red(0.0) 결합 (그리퍼 유지)")
        if self.pick_target("4x2_red", offset_studs_y=-1.84):
            self.call(self.cli_h, Trigger.Request())
            
            # 그리퍼 열지 않고 결합
            if self.visual_insert("2x2_red", layer_index=1, offset_studs_y=0.0, release_gripper=False):
                self.call(self.cli_h, Trigger.Request())
                time.sleep(1.0) # 카메라가 흔들림을 잡고 바닥을 볼 수 있도록 약간의 대기

                self.get_logger().info("🧱 [Phase 2] 바닥의 다른 4x2_red 스캔 및 6x2 조립 (그리퍼 해제)")
                # 🌟 수정됨: visual_insert 대신 홈 위치에서 깨끗하게 1회 스캔하여 좌표를 먼저 확보합니다.
                p_4x2_base = self.find_target_with_retry("4x2_red")
                if not p_4x2_base:
                    self.get_logger().warn("❌ 바닥에 다른 4x2_red가 안 보입니다.")
                    return
                
                # 시야가 가려지기 전의 정확한 바닥 좌표를 저장!
                saved_6x2_pose = p_4x2_base 
                
                # 🌟 수정됨: 스캔된 좌표로 블라인드 결합을 수행해 2차 스캔(오인식)을 원천 차단합니다.
                if self.blind_insert(saved_6x2_pose, layer_index=1, offset_studs_y=-3.0, release_gripper=True):
                    self.call(self.cli_h, Trigger.Request())

                    self.get_logger().info("🧱 [Phase 3] 2x2_red 파지 -> 6x2 중심에 결합 (그리퍼 유지)")
                    if self.pick_target("2x2_red", offset_studs_y=-0.2):
                        self.call(self.cli_h, Trigger.Request())
                        
                        # 아까 저장해둔 깨끗한 6x2 베이스 좌표로 블라인드 이동
                        if self.blind_insert(saved_6x2_pose, layer_index=2, offset_studs_y=-1.0, release_gripper=False):
                            self.call(self.cli_h, Trigger.Request())

                            self.get_logger().info("🧱 [Phase 4] 덩어리를 2x2_yellow 중앙에 최종 결합")
                            # 맨 처음에 기억해둔 노란색 블록 위치로 블라인드 이동 후 그리퍼 해제
                            if self.blind_insert(saved_yellow_pose, layer_index=1.5, offset_studs_y=0.0):
                                self.get_logger().info("✅ 최종 조립 시퀀스 완벽 종료!")

    # def build_studs_y(self):
    #     self.get_logger().info("🧱 [초기화] 맨 처음 2x2_yellow 위치 스캔 및 기억")
    #     # 페이즈 4를 위해 노란색 블록의 위치를 가장 먼저 찾아 저장합니다.
    #     p_yellow = self.find_target_with_retry("2x2_yellow")
    #     if not p_yellow:
    #         self.get_logger().warn("❌ 바닥에 2x2_yellow가 안 보입니다. 조립을 취소합니다.")
    #         return
    #     saved_yellow_pose = p_yellow
    #     self.call(self.cli_h, Trigger.Request())

    #     self.get_logger().info("🧱 [Phase 1] 4x2_red(-1.85) 파지 -> 2x2_red(0.0) 결합 (그리퍼 유지)")
    #     if self.pick_target("4x2_red", offset_studs_y=-1.85):
    #         self.call(self.cli_h, Trigger.Request())
            
    #         # 그리퍼 열지 않고 결합
    #         if self.visual_insert("2x2_red", layer_index=1, offset_studs_y=0.0, release_gripper=False):
    #             self.call(self.cli_h, Trigger.Request())

    #             self.get_logger().info("🧱 [Phase 2] 다른 4x2_red(-3.0)에 결합하여 6x2 완성 (그리퍼 해제)")
    #             # 그리퍼를 열고 6x2 덩어리를 바닥에 내려놓음
    #             if self.visual_insert("4x2_red", layer_index=1, offset_studs_y=-3.05, release_gripper=True):
    #                 # 내려놓은 기준점(바닥의 4x2_red) 위치를 기억
    #                 saved_6x2_pose = self.last_perfect_pose 
    #                 self.call(self.cli_h, Trigger.Request())

    #                 self.get_logger().info("🧱 [Phase 3] 2x2_red 파지 -> 6x2 중심에 결합 (그리퍼 유지)")
    #                 if self.pick_target("2x2_red", offset_studs_y=0.0):
    #                     self.call(self.cli_h, Trigger.Request())
                        
    #                     # 카메라가 가려지므로 저장된 6x2 위치를 이용해 블라인드 인서트. 
    #                     # 🌟 중심 오프셋 -2.0으로 설정 (방향 안 맞으면 2.0으로 수정!)
    #                     if self.blind_insert(saved_6x2_pose, layer_index=2, offset_studs_y=2.0, release_gripper=False):
    #                         self.call(self.cli_h, Trigger.Request())

    #                         self.get_logger().info("🧱 [Phase 4] 덩어리를 2x2_yellow 중앙에 최종 결합")
    #                         # 맨 처음에 기억해둔 노란색 블록 위치로 블라인드 이동 후 그리퍼 해제
    #                         if self.blind_insert(saved_yellow_pose, layer_index=1, offset_studs_y=0.0):
    #                             self.get_logger().info("✅ 최종 조립 시퀀스 완벽 종료! 고생하셨습니다 퇴근하세요!!")

    # def build_big_tree(self):
    #     self.get_logger().info("🎄 [큰 나무] 상단 모듈(잎) 조립 후 기둥(노랑2x2)에 결합!")

    #     # ==========================================
    #     # [Phase 1] 상단 모듈 (나뭇잎 부분: 2, 3, 4층) 만들기
    #     # ==========================================
    #     self.get_logger().info("🧱 [Phase 1-1] 빨강2x2(Pick) -> 빨강4x2 옆에 배치 (바닥)")
    #     self.call(self.cli_h, Trigger.Request())

    #     if self.pick_target("2x2_red"):
    #         self.call(self.cli_h, Trigger.Request())
    #         # 1-1 조립 성공 시에만 내부 로직(Shake 및 다음 단계) 실행
    #         if self.visual_insert("4x2_red", layer_index=0, offset_studs_y=3.0):
                
    #             # 🏠 홈 복귀 및 정밀 스캔(Shake) 시작
    #             self.get_logger().info("🏠 홈 복귀 및 정밀 스캔(Shake) 시작...")
    #             self.call(self.cli_h, Trigger.Request())
    #             time.sleep(1.0)
                
    #             # 🔄 Shake 로직: Yaw를 좌우로 흔들어 좌표 갱신
    #             self.get_logger().info("🔄 시야각 보정 중 (+5도 -> -5도)...")
    #             self.call(self.cli_r, GetTargetPose.Request(yaw=10.0, target_size="YAW"))
    #             time.sleep(1.5)
    #             self.call(self.cli_r, GetTargetPose.Request(yaw=-10.0, target_size="YAW"))
    #             time.sleep(2.0)
                
                
    #             # 🎯 조립 구역 좌표(무시용) 확정
    #             saved_leaf_base = self.find_target_with_retry("4x2_red")
                
    #             if not saved_leaf_base:
    #                 self.get_logger().error("❌ 조립 구역 좌표 갱신 실패")
    #                 return

    #             # -----------------------------------------------------------
    #             # [Phase 1-2] 빨강4x2(Pick) -> 두 블록을 이어주는 3층 배치
    #             # -----------------------------------------------------------
    #             self.get_logger().info("🧱 [Phase 1-2] 빨강4x2(Pick) -> 3층 배치 시작")
    #             self.call(self.cli_h, Trigger.Request())
    #             # 갱신된 saved_leaf_base를 사용하여 조립 뭉치 무시
    #             if self.pick_fresh_target("4x2_red", exclude_pose=saved_leaf_base):
    #                 self.call(self.cli_h, Trigger.Request())
    #                 if not self.blind_insert(saved_leaf_base, layer_index=1, offset_studs_y=0):
    #                     return

    #             # -----------------------------------------------------------
    #             # [Phase 1-3] 빨강2x2(Pick) -> 가장 꼭대기 4층 배치
    #             # -----------------------------------------------------------
    #             self.get_logger().info("🧱 [Phase 1-3] 빨강2x2(Pick) -> 4층 배치 시작")
    #             self.call(self.cli_h, Trigger.Request())
    #             if self.pick_target("2x2_red"):
    #                 self.call(self.cli_h, Trigger.Request())
    #                 if not self.blind_insert(saved_leaf_base, layer_index=2, offset_studs_y=0):
    #                     return

    #     # ==========================================
    #     # [Phase 2] 최종 결합 (상단 뭉치를 노랑 기둥에 꽂기)
    #     # ==========================================
    #     self.get_logger().info("🚀 [Phase 2] 최종 결합 시작!")
    #     self.call(self.cli_h, Trigger.Request())
       
    #     if self.pick_target("4x2_red", layer_index=0):
    #         self.call(self.cli_h, Trigger.Request())
    #         if self.visual_insert("2x2_yellow", layer_index=2):
    #             self.get_logger().info("✅🎉 빅트리 완성!")
    #         else:
    #             self.get_logger().warn("❌ 최종 결합 실패")

    def run(self):
        self.get_logger().info("🚀 STARTING VISUAL-STACK ASSEMBLY SEQUENCE (Full Recipe Mode)")
        self.call(self.cli_h, Trigger.Request())
        self.call(self.cli_g, SetBool.Request(data=False))
        time.sleep(1.0)
       
        self.get_logger().info("👀 필드 블록 스캔 중...")
        inventory = {
            "2x2_yellow": self.count_color("2x2_yellow"),
            "2x2_blue": self.count_color("2x2_blue"),
            "2x2_red": self.count_color("2x2_red"),
            "2x2_green": self.count_color("2x2_green"),
            "4x2_yellow": self.count_color("4x2_yellow"),
            "4x2_red": self.count_color("4x2_red"),
            "4x2_blue": self.count_color("4x2_blue")
        }
        self.get_logger().info(f"📦 현재 인벤토리: {inventory}")

        best_plan = self.get_best_build_plan(inventory)
       
        if not best_plan:
            self.get_logger().warn("❌ 조립 가능한 조합이 없습니다.")
        else:
            self.get_logger().info(f"🧠 최적 계획: {best_plan}")
            for item in best_plan:
                self.get_logger().info(f"▶️ 작업 시작: {item.upper()}")
                if item == 'battery': self.build_battery()
                elif item == 'studs_y': self.build_studs_y()
                elif item == 'magnet': self.build_magnet()
                elif item == 'e_stop': self.build_e_stop()
                elif item == 'carrot': self.build_carrot()
                elif item == 'traffic_light': self.build_traffic_light()
                elif item == 'small_tree': self.build_small_tree()
                elif item == 'hammer': self.build_hammer()
                elif item == 'big_carrot': self.build_big_carrot()
                elif item == 'burger': self.build_burger()
                elif item == 'ice_cream': self.build_ice_cream()
                elif item == 'big_tree': self.build_big_tree()
                   
                self.call(self.cli_h, Trigger.Request())
                time.sleep(1.0)

        self.call(self.cli_h, Trigger.Request())
        self.get_logger().info("🎉 ALL SEQUENCE DONE")

def main():
    rclpy.init()
    node = MasterNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()