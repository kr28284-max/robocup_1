# import rclpy
# from rclpy.node import Node
# from srvs_pkg.srv import GetTargetPose
# import numpy as np
# import pyrealsense2 as rs
# from ultralytics import YOLO
# import cv2

# class VisionNode(Node):
#     def __init__(self):
#         super().__init__('vision_node')
#         self.srv = self.create_service(GetTargetPose, '/get_target_pose', self.get_pose_cb)
#         self.model = YOLO("/home/da/duplo_ws/best.pt")

#         self.pipeline = rs.pipeline()
#         config = rs.config()
#         config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#         config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#         profile = self.pipeline.start(config)
#         self.align = rs.align(rs.stream.color)
#         self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

#         self.latest_color = None
#         self.latest_depth = None
#         self.latest_results = None

#         self.create_timer(0.033, self.visualize_callback)
#         self.get_logger().info("✅ Vision Node: 타겟 스위칭 방지(락온) 및 Yaw 시각화 모드 가동")

#     def visualize_callback(self):
#         try:
#             frames = self.pipeline.wait_for_frames(timeout_ms=1000)
#             aligned = self.align.process(frames)
#             self.latest_depth = aligned.get_depth_frame()
#             color_frame = aligned.get_color_frame()
#             if not color_frame or not self.latest_depth: return

#             self.latest_color = np.asanyarray(color_frame.get_data())
#             self.latest_results = self.model(self.latest_color, verbose=False)[0]

#             # YOLO가 기본적으로 그려주는 이미지 (바운딩 박스, 클래스 이름 등)
#             display_img = self.latest_results.plot()

#             # 💡 [추가된 부분] 화면에 인식된 모든 블록의 Yaw 값을 계산해서 텍스트로 띄워줍니다.
#             if self.latest_results.boxes is not None:
#                 for i, box in enumerate(self.latest_results.boxes):
#                     yaw = 0.0
#                     xyxy = box.xyxy[0].cpu().numpy()
#                     u = int((xyxy[0] + xyxy[2]) / 2)
#                     v = int((xyxy[1] + xyxy[3]) / 2)

#                     # 마스크가 있으면 회전 각도(Yaw) 계산
#                     if self.latest_results.masks is not None and len(self.latest_results.masks.xy) > i:
#                         pts = np.int32([self.latest_results.masks.xy[i]])
#                         M = cv2.moments(pts)
#                         if M["m00"] != 0:
#                             # 중심점 정밀 보정
#                             u = int(M["m10"] / M["m00"])
#                             v = int(M["m01"] / M["m00"])
#                             # 각도 계산
#                             rect = cv2.minAreaRect(pts)
#                             yaw = rect[2] % 90.0
#                             if yaw > 45: yaw -= 90
                    
#                     # 블록 중심 (u, v) 좌표 근처에 노란색 텍스트로 Yaw 값 표시
#                     text = f"Yaw: {yaw:.1f}"
#                     cv2.putText(display_img, text, (u - 40, v + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

#             cv2.imshow("Real-time Segmentation", display_img)
#             cv2.waitKey(1)
#         except Exception:
#             pass

#     def get_valid_depth(self, u, v, search_radius=10):
#         z = self.latest_depth.get_distance(u, v)
#         if z > 0: return z
        
#         for r in range(1, search_radius + 1):
#             for dx in range(-r, r + 1):
#                 for dy in range(-r, r + 1):
#                     nu, nv = u + dx, v + dy
#                     if 0 <= nu < 640 and 0 <= nv < 480:
#                         z = self.latest_depth.get_distance(nu, nv)
#                         if z > 0: return z
#         return 0.0

#     def get_pose_cb(self, request, response):
#         target = request.target_color.lower()
        
#         if self.latest_results is None or self.latest_results.boxes is None:
#             response.success = False
#             return response

#         valid_targets = []

#         # 1. 화면에 보이는 해당 색상의 모든 블록 정보를 수집합니다.
#         for i, box in enumerate(self.latest_results.boxes):
#             cls_name = self.latest_results.names[int(box.cls[0])].lower()
            
#             if target in cls_name:
#                 xyxy = box.xyxy[0].cpu().numpy()
#                 u = int((xyxy[0] + xyxy[2]) / 2)
#                 v = int((xyxy[1] + xyxy[3]) / 2)
#                 yaw = 0.0

#                 if self.latest_results.masks is not None and len(self.latest_results.masks.xy) > i:
#                     pts = np.int32([self.latest_results.masks.xy[i]])
#                     M = cv2.moments(pts)
#                     if M["m00"] != 0:
#                         u = int(M["m10"] / M["m00"])
#                         v = int(M["m01"] / M["m00"])
#                         rect = cv2.minAreaRect(pts)
#                         yaw = rect[2] % 90.0
#                         if yaw > 45: yaw -= 90

#                 z = self.get_valid_depth(u, v)
#                 if z > 0:
#                     valid_targets.append({'u': u, 'v': v, 'z': z, 'yaw': yaw, 'name': cls_name})

#         if not valid_targets:
#             response.success = False
#             return response

#         # 2. [1차 필터] 가장 바닥에 있는 Z값(가장 큰 거리)을 찾고, 오차범위(3cm) 내의 바닥 블록들을 추려냅니다.
#         max_z = max(t['z'] for t in valid_targets)
#         bottom_blocks = [t for t in valid_targets if t['z'] >= max_z - 0.03]

#         # 3. [2차 필터] 바닥 블록들 중 '화면 중앙(320, 240)'과 가장 가까운 블록을 최종 선택합니다. (락온 효과)
#         best_target = None
#         min_dist = float('inf')
        
#         for t in bottom_blocks:
#             # 피타고라스 정리로 화면 중앙과의 픽셀 거리 계산 (해상도 640x480 기준 중앙)
#             dist_to_center = ((t['u'] - 320) ** 2 + (t['v'] - 240) ** 2) ** 0.5
#             if dist_to_center < min_dist:
#                 min_dist = dist_to_center
#                 best_target = t

#         # 4. 최종 선택된 타겟의 3D 좌표 변환 및 전송
#         if best_target:
#             x, y, z_val = rs.rs2_deproject_pixel_to_point(
#                 self.intrinsics, [best_target['u'], best_target['v']], best_target['z']
#             )
            
#             response.success = True
#             response.x, response.y, response.z, response.yaw = float(x), float(y), float(z_val), float(best_target['yaw'])
            
#             self.get_logger().info(f"🎯 최종 타겟: {best_target['name']} (Z: {best_target['z']:.3f}, 중심거리: {min_dist:.1f}px)")
#             return response

#         response.success = False
#         return response

# def main():
#     rclpy.init()
#     rclpy.spin(VisionNode())
#     cv2.destroyAllWindows()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from srvs_pkg.srv import GetTargetPose
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import cv2

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.srv = self.create_service(GetTargetPose, '/get_target_pose', self.get_pose_cb)
        self.model = YOLO("/home/da/duplo_ws/best.pt")

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

        self.create_timer(0.033, self.visualize_callback)
        self.get_logger().info("✅ Vision Node: 타겟 스위칭 방지(중앙 락온) 및 Yaw 시각화 모드 가동")

    def visualize_callback(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            aligned = self.align.process(frames)
            self.latest_depth = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not color_frame or not self.latest_depth: return

            self.latest_color = np.asanyarray(color_frame.get_data())
            self.latest_results = self.model(self.latest_color, verbose=False)[0]

            display_img = self.latest_results.plot()

            # 화면에 인식된 모든 블록의 Yaw 값을 계산해서 텍스트로 띄워줍니다.
            if self.latest_results.boxes is not None:
                for i, box in enumerate(self.latest_results.boxes):
                    yaw = 0.0
                    xyxy = box.xyxy[0].cpu().numpy()
                    u = int((xyxy[0] + xyxy[2]) / 2)
                    v = int((xyxy[1] + xyxy[3]) / 2)

                    if self.latest_results.masks is not None and len(self.latest_results.masks.xy) > i:
                        pts = np.int32([self.latest_results.masks.xy[i]])
                        M = cv2.moments(pts)
                        if M["m00"] != 0:
                            u = int(M["m10"] / M["m00"])
                            v = int(M["m01"] / M["m00"])
                            rect = cv2.minAreaRect(pts)
                            yaw = rect[2] % 90.0
                            if yaw > 45: yaw -= 90
                    
                    # 블록 중심 (u, v) 좌표 근처에 노란색 텍스트로 Yaw 값 표시
                    text = f"Yaw: {yaw:.1f}"
                    cv2.putText(display_img, text, (u - 40, v + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow("Real-time Segmentation", display_img)
            cv2.waitKey(1)
        except Exception:
            pass

    def get_valid_depth(self, u, v, search_radius=10):
        z = self.latest_depth.get_distance(u, v)
        if z > 0: return z
        
        for r in range(1, search_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nu, nv = u + dx, v + dy
                    if 0 <= nu < 640 and 0 <= nv < 480:
                        z = self.latest_depth.get_distance(nu, nv)
                        if z > 0: return z
        return 0.0

    def get_pose_cb(self, request, response):
        target = request.target_color.lower()
        
        if self.latest_results is None or self.latest_results.boxes is None:
            response.success = False
            return response

        valid_targets = []

        # 1. 화면에 보이는 해당 색상의 모든 블록 정보를 수집
        for i, box in enumerate(self.latest_results.boxes):
            cls_name = self.latest_results.names[int(box.cls[0])].lower()
            
            if target in cls_name:
                xyxy = box.xyxy[0].cpu().numpy()
                u = int((xyxy[0] + xyxy[2]) / 2)
                v = int((xyxy[1] + xyxy[3]) / 2)
                yaw = 0.0

                if self.latest_results.masks is not None and len(self.latest_results.masks.xy) > i:
                    pts = np.int32([self.latest_results.masks.xy[i]])
                    M = cv2.moments(pts)
                    if M["m00"] != 0:
                        u = int(M["m10"] / M["m00"])
                        v = int(M["m01"] / M["m00"])
                        rect = cv2.minAreaRect(pts)
                        yaw = rect[2] % 90.0
                        if yaw > 45: yaw -= 90

                z = self.get_valid_depth(u, v)
                if z > 0:
                    # 💡 핵심: 블록의 중심점(u, v)과 카메라 화면 정중앙(320, 240) 사이의 거리를 계산합니다.
                    dist = ((u - 320) ** 2 + (v - 240) ** 2) ** 0.5
                    valid_targets.append({'u': u, 'v': v, 'z': z, 'yaw': yaw, 'name': cls_name, 'dist': dist})

        if not valid_targets:
            response.success = False
            return response

        # 2. 💡 [강력한 필터] Z값(깊이) 비교를 없애고, 무조건 '화면 중앙과 가장 가까운 블록(min dist)'을 타겟으로 고정합니다.
        best_target = min(valid_targets, key=lambda t: t['dist'])

        # 3. 최종 선택된 타겟의 3D 좌표 변환 및 전송
        x, y, z_val = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, [best_target['u'], best_target['v']], best_target['z']
        )
        
        response.success = True
        response.x, response.y, response.z, response.yaw = float(x), float(y), float(z_val), float(best_target['yaw'])
        
        self.get_logger().info(f"🎯 락온 완료: {best_target['name']} (중심거리: {best_target['dist']:.1f}px, Z: {best_target['z']:.3f})")
        return response

def main():
    rclpy.init()
    rclpy.spin(VisionNode())
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()