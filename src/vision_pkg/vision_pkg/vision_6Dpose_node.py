import rclpy
from rclpy.node import Node
from srvs_pkg.srv import GetTargetPose
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import cv2
import time
import math

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.srv = self.create_service(GetTargetPose, '/get_target_pose', self.get_pose_cb)
        self.model = YOLO("/home/user/duplo_ws/best.pt")

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        self.latest_color = None
        self.latest_depth = None
        self.latest_results = None

        # 🌟 금지 구역 좌표를 저장할 변수 추가
        self.exclude_x = None
        self.exclude_y = None

        self.create_timer(0.033, self.visualize_callback)
        self.get_logger().info("✅ Vision Node: 1층 높이 필터링 및 2.0초 정밀 스캔 모드 가동")

    def calculate_refined_yaw(self, rect):
        (cx, cy), (w, h), angle = rect
        if w < h:
            yaw = angle
        else:
            yaw = angle + 90.0

        if yaw > 90: yaw -= 180
        if yaw < -90: yaw += 180
        return yaw

    def visualize_callback(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            aligned = self.align.process(frames)
            self.latest_depth = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            
            if not color_frame or not self.latest_depth: return

            self.latest_color = np.asanyarray(color_frame.get_data())
            self.latest_results = self.model(self.latest_color, verbose=False)[0]

            # 🌟 YOLO의 기본 plot 대신 원본 이미지를 복사해서 직접 그립니다.
            display_img = self.latest_color.copy()
            cv2.circle(display_img, (320, 240), 5, (255, 255, 255), -1) # 중심점

            if self.latest_results.boxes is not None:
                for i, box in enumerate(self.latest_results.boxes):
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    u, v = int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)
                    yaw = 0.0

                    # Yaw 계산 (Mask가 있을 때)
                    if self.latest_results.masks is not None and len(self.latest_results.masks.xy) > i:
                        pts = np.int32([self.latest_results.masks.xy[i]])
                        M = cv2.moments(pts)
                        if M["m00"] != 0:
                            u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                            rect = cv2.minAreaRect(pts)
                            yaw = self.calculate_refined_yaw(rect)
                    
                    z_val = self.latest_depth.get_distance(u, v)
                    if z_val > 0:
                        x_r, y_r, _ = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], z_val)
                        
                        # 🌟 상태 결정 로직 (마스터 좌표 기준)
                        color = (0, 255, 0) # 기본 초록색
                        status_txt = "OK"
                        
                        if self.exclude_x is not None and self.exclude_y is not None:
                            # 금지 구역 좌표와의 거리 계산
                            dist = math.sqrt((x_r - self.exclude_x)**2 + (y_r - self.exclude_y)**2)
                            if dist < 0.05: # 5cm 이내
                                color = (0, 0, 255) # 빨간색
                                status_txt = "IGNORED"

                        # 박스 및 정보 그리기
                        cv2.rectangle(display_img, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), color, 2)
                        cv2.putText(display_img, f"[{status_txt}] X:{x_r*1000:.0f} Y:{y_r*1000:.0f}", (xyxy[0], xyxy[1] - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        cv2.putText(display_img, f"Yaw:{yaw:.1f}", (xyxy[0], xyxy[3] + 20), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            cv2.imshow("Vision Debug View (Dynamic Filter)", display_img)
            cv2.waitKey(1)
        except Exception:
            pass

    def get_valid_depth(self, depth_frame, u, v, search_radius=10):
        z = depth_frame.get_distance(u, v)
        if z > 0: return z
        for r in range(1, search_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nu, nv = u + dx, v + dy
                    if 0 <= nu < 640 and 0 <= nv < 480:
                        z = depth_frame.get_distance(nu, nv)
                        if z > 0: return z
        return 0.0

    def get_pose_cb(self, request, response):
        raw_target = request.target_color.lower()

        self.exclude_x = None
        self.exclude_y = None
        is_center_mode = False
        is_far_mode = False  # 🌟 추가된 far 모드 플래그

        # 🌟 1. 모드 분리
        if raw_target.startswith("center_"):
            is_center_mode = True
            raw_target = raw_target.replace("center_", "")
        elif raw_target.startswith("far_"):
            is_far_mode = True
            raw_target = raw_target.replace("far_", "")

        # 2. '|' 구분자가 있는지 확인 (이하 기존 코드 유지...)
        if '|' in raw_target:
            parts = raw_target.split('|')
            target = parts[0]            
            self.exclude_x = float(parts[1])
            self.exclude_y = float(parts[2])
            self.get_logger().info(f"🛡️ 금지 구역 설정: X={self.exclude_x:.3f}, Y={self.exclude_y:.3f}")
        else:
            target = raw_target
        
        target = target.replace("far_", "").replace("center_", "")

        if target.startswith("count_"):
            search_color = target.replace("count_", "")
            start_time = time.time()
            max_count = 0
            while time.time() - start_time < 0.5:
                try:
                    frames = self.pipeline.wait_for_frames(timeout_ms=500)
                    aligned = self.align.process(frames)
                    color_f = aligned.get_color_frame()
                    if not color_f: continue

                    img = np.asanyarray(color_f.get_data())
                    results = self.model(img, verbose=False)[0]
                    
                    if results.boxes is not None:
                        current_count = sum(1 for box in results.boxes if search_color in results.names[int(box.cls[0])].lower())
                        max_count = max(max_count, current_count)
                except Exception:
                    pass
            response.success = True
            response.x, response.y, response.z, response.yaw = float(max_count), 0.0, 0.0, 0.0
            return response

        # -------------------------------------------------------------
        # 🌟 정밀 스캔 데이터 수집 시간을 2.0초로 늘려 안정성 극대화
        self.get_logger().info(f"🔍 '{target}' 정밀 측정 (바닥 1층 탐색, 2.0초 데이터 수집 중...)")
        
        samples = []
        start_time = time.time()
        
        # 💡 1.2 -> 2.0으로 늘려서 흔들림 없는 확실한 중간값을 뽑아냅니다.
        while time.time() - start_time < 2.0:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=500)
                aligned = self.align.process(frames)
                depth_f = aligned.get_depth_frame()
                color_f = aligned.get_color_frame()
                
                if not color_f or not depth_f: continue

                img = np.asanyarray(color_f.get_data())
                results = self.model(img, verbose=False)[0]
                if results.boxes is None: continue

                frame_targets = []
                for i, box in enumerate(results.boxes):
                    cls_name = results.names[int(box.cls[0])].lower()
                    if target in cls_name:
                        u, v, yaw = 0, 0, 0.0
                        if results.masks is not None and len(results.masks.xy) > i:
                            pts = np.int32([results.masks.xy[i]])
                            M = cv2.moments(pts)
                            if M["m00"] != 0:
                                u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                                rect = cv2.minAreaRect(pts)
                                yaw = self.calculate_refined_yaw(rect)
                        else:
                            xyxy = box.xyxy[0].cpu().numpy()
                            u, v = int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)

                        z = self.get_valid_depth(depth_f, u, v)

                        if z > 0:
                            # 🌟 1. 로봇 기준 3D 좌표 미리 계산
                            x_r, y_r, _ = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], z)

                            # 🌟 2. 동적 거리 필터: 마스터가 준 좌표와 현재 블록 사이의 거리 계산
                            if self.exclude_x is not None and self.exclude_y is not None:
                                dist_to_exclude = math.sqrt((x_r - self.exclude_x)**2 + (y_r - self.exclude_y)**2)
                                if dist_to_exclude < 0.05: # 5cm 이내면 무시
                                    continue 

                            dist = ((u - 320) ** 2 + (v - 240) ** 2) ** 0.5
                            frame_targets.append({'u': u, 'v': v, 'z': z, 'yaw': yaw, 'dist': dist})

                        
                if frame_targets:
                    max_z = max(t['z'] for t in frame_targets)
                    ground_targets = [t for t in frame_targets if abs(max_z - t['z']) < 0.015]
                    
                    best_t = None
                    if is_center_mode:
                        best_t = min(ground_targets, key=lambda t: t['dist'])
                    elif is_far_mode:
                        # 🌟 far_ 모드: 화면에서 가장 위쪽(v좌표가 가장 작은)에 있는 블록 = 가장 멀리 있는 새 4x2 블록 선택
                        best_t = min(ground_targets, key=lambda t: t['v'])
                    else:
                        # 기존 모드: 가장 가까운 블록 선택
                        max_y = -999.0
                        for t in ground_targets:
                            _, ty, _ = rs.rs2_deproject_pixel_to_point(self.intrinsics, [t['u'], t['v']], t['z'])
                            if ty > max_y:
                                max_y = ty
                                best_t = t
                            
                    if best_t:
                        x, y, z = rs.rs2_deproject_pixel_to_point(self.intrinsics, [best_t['u'], best_t['v']], best_t['z'])
                        samples.append([x, y, z, best_t['yaw']])
                
                time.sleep(0.01)
            except Exception:
                continue

        if len(samples) < 5:
            self.get_logger().error(f"❌ 인식 실패 (수집 프레임 부족)")
            response.success = False
            return response

        samples = np.array(samples)
        
        if is_center_mode:
            # 🌟 센터 모드일 때는 중앙값(median)을 사용하여 정밀도 유지
            final_pose = np.median(samples, axis=0)
        else:
            # 초기 탐색 시: X(실제 Y)가 가장 큰 샘플로 확정
            best_idx = np.argmax(samples[:, 1])
            final_pose = samples[best_idx]
        
        response.success = True
        response.x, response.y, response.z, response.yaw = \
            float(final_pose[0]), float(final_pose[1]), float(final_pose[2]), float(final_pose[3])

        self.get_logger().info(f"🎯 타겟 확정: X:{response.x*1000:.1f}, Y:{response.y*1000:.1f}, Yaw:{response.yaw:.1f}")
        return response

def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()