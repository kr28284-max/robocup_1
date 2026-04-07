# duplo_ws
# 🏆 RoboCup 2026: 지능형 듀플로 조립 로봇 (Intelligent Duplo Assembly System)

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?style=flat&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=flat&logo=python&logoColor=white)
![YOLOv8](https://img.shields.io/badge/YOLO-v8_Segmentation-00FFFF?style=flat&logo=yolo&logoColor=black)
![RealSense](https://img.shields.io/badge/Intel_RealSense-RGBD-0071C5?style=flat&logo=intel&logoColor=white)

## 📌 프로젝트 개요 (Overview)
본 프로젝트는 **로보컵(RoboCup) Industrial - RoboCup@Work** 미션 수행을 위한 **지능형 자동 조립 시스템**입니다. 

단순한 반복 동작을 넘어, **YOLOv8 Segmentation**과 **RGB-D 카메라(RealSense)**를 통해 실시간으로 필드 상황을 파악합니다. 수집된 인벤토리 데이터를 바탕으로 **DFS(깊이 우선 탐색) 알고리즘**을 가동하여, 제한된 블록 내에서 점수를 최대화할 수 있는 최적의 조립 리스트를 스스로 생성하고 실행하는 것이 핵심입니다.

## 🧱 조립 가능한 듀플로 레시피 (Assembly Recipes)
인벤토리 스캔 결과에 따라 아래 12가지 레시피 중 최적의 조합을 선택하여 조립합니다.

* **기본 조합:** 🔋 배터리, 🧲 자석, 🛑 비상정지
* **중급 조합:** 🥕 당근, 🚦 신호등, 🌳 작은 나무, 🔨 망치
* **고급 조합:** 🥕 큰 당근, 🍔 버거, 🍦 아이스크림, 🌳 큰 나무, 🧱 Studs Y (5단 고난도 결합)

---

## 🚀 핵심 기능 및 트러블슈팅 (Key Features & Dev Log)

### 1. 인벤토리 기반 DFS 최적 조립 계획 (Inventory-Aware DFS Planning)
* **문제점:** 필드에 흩어진 블록들이 어떤 조합에 최적인지 계산하지 않고 탐욕적(Greedy)으로 집을 경우, 나중에 더 높은 점수의 구조물을 만들 블록이 부족해지는 상황이 발생했습니다.
* **해결책:** 조립 시작 전 비전 노드에 `count_` 요청을 보내 전체 블록 수량을 파악합니다. 이후 **DFS 알고리즘 기반 시뮬레이션**을 통해 잔여 블록이 가장 적게 남는 최적의 조립 순서(`best_plan`)를 찾아내어 순차적으로 실행합니다.

### 2. 정밀도 극대화를 위한 2.0초 Median Scan (Refined Vision Scan)
* **문제점:** 로봇 팔 이동 직후 발생하는 잔진동과 RGB-D 센서의 노이즈로 인해 순간적인 좌표 오차가 발생하여 조립 정밀도가 떨어졌습니다.
* **해결책:** 비전 노드의 정밀 측정 모드 시간을 **2.0초로 확장**했습니다. 2초간 수집된 수많은 프레임 중 **중간값(Median)**을 추출함으로써 노이즈와 진동의 영향을 배제한 확실한 타겟 좌표를 확보했습니다.

### 3. 동적 금지 구역 설정 (Dynamic Exclude Zone Filter)
* **문제점:** 이미 조립이 완료된 구조물 근처의 다른 블록을 잡으려 할 때, 비전 노드가 기존 구조물을 타겟으로 중복 인식하여 충돌이 발생하는 논리적 충돌이 있었습니다.
* **해결책:** 마스터 노드에서 현재 조립된 위치를 비전 노드에 `exclude_x/y`로 전달합니다. 비전 노드는 해당 좌표의 5cm 이내 객체를 **`IGNORED`(빨간색) 상태로 분류하여 인식 대상에서 제외**함으로써 간섭 없는 연속 조립을 가능케 했습니다.

### 4. 오클루전 극복: 메모리 기반 블라인드 스택 (Blind Stacking)
* **문제점:** 3단 이상 적재 시 로봇 그리퍼와 구조물이 카메라 시야를 완전히 가려 베이스 블록의 좌표를 잃어버리는 오클루전(Occlusion) 현상이 발생했습니다.
* **해결책:** 인식이 가장 명확한 1층 시점의 좌표를 메모리(`last_perfect_pose`)에 저장합니다. 이후 상단 적재 시에는 카메라를 보지 않고 **메모리된 좌표에 블록 높이(`BLOCK_H`) 오프셋만 더해 투입**하는 'Blind Insert' 전략으로 정밀도를 유지했습니다.

### 5. 타겟 추적 최적화 (Center & Far Targeting Mode)
* **문제점:** 여러 블록이 뭉쳐 있을 때 단순히 가장 가까운 블록만 잡으려다 조립 동선이 꼬이는 문제가 발생했습니다.
* **해결책:** 상황에 따라 `center_`(중앙 기준 최단거리)와 `far_`(화면 상단 기준 최원거리) 모드를 선택 적용합니다. 특히 **`far_` 모드를 통해 구조물 뒤쪽의 블록을 안전하게 확보**하여 그리퍼 충돌을 원천 차단했습니다.

---

## 🚀 Key Technical Challenges & Solutions

### 1. Segmentation Mask-based Yaw Calculation
- **Problem:** 일반적인 Bounding Box 방식은 블록이 비스듬히 놓였을 때 회전 각도(Yaw) 오차가 커 결합이 실패함.
- **Solution:** YOLOv8의 **Segmentation Mask** 데이터를 OpenCV의 `minAreaRect`로 분석하여 실제 블록의 장축 방향을 산출하고, `calculate_refined_yaw` 함수를 통해 ±90도 오차를 자동 보정하는 정밀 알고리즘을 구현했습니다.

### 2. Multi-stage Modular Assembly (Ice Cream)
- **Problem:** 5단 이상의 높은 구조물을 한 번에 수직 적재하는 것은 물리적 공차 누적으로 성공률이 낮음.
- **Solution:** 하단(노란색 베이스)과 상단(파란색/노란색 결합부)을 각기 다른 위치에서 조립한 뒤, 최종적으로 상단 모듈을 통째로 들어 하단 위에 꽂는 **모듈형 분할 조립 전략**으로 안정성을 확보했습니다.

### 3. Systematic Coordinate Consistency (master_node5)
- **Problem:** 로봇 팔의 YAW 회전 후 XY 이동 시 툴 좌표계의 축 변화로 인한 오프셋 밀림 현상.
- **Solution:** `Yaw(허공 회전) -> XY(타겟 위 정렬) -> Z(수직 삽입)` 순으로 제어 시퀀스를 고도화하여 상대 좌표계의 물리적 오차를 제로화했습니다.

---

## 📂 실행 구조 (Architecture)
* **`vision_6Dpose_node.py`**: YOLOv8 + RealSense 기반 3D 좌표 및 정밀 Yaw 산출
* **`master_node5.py`**: DFS 최적화 시나리오 및 조립 공정 메인 제어
* **`robot_node.py`**: 로봇 팔 하위 하드웨어 제어 인터페이스
* **`gripper_node.py`**: 다이나믹셀 기반 그리퍼 파지 제어

---

## 👨‍💻 Author
* **정동혁 (Jeong Dong-hyeok)**
* Incheon National University, Dept. of Electrical Engineering
* 47th Student Council Reserve Forces President
