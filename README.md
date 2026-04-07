# duplo_ws
🏆 RoboCup 2026: 지능형 듀플로 조립 로봇 (Duplo Assembly Robot System)
📌 프로젝트 개요 (Overview)
본 프로젝트는 로보컵(RoboCup) manipulation 미션 수행을 위한 AI 비전 기반 자동 조립 시스템입니다. YOLOv8 Segmentation 모델과 RealSense RGB-D 카메라를 활용하여 필드의 블록 상태를 실시간으로 파악하고, DFS(깊이 우선 탐색) 알고리즘을 통해 주어진 자원 내에서 점수를 최대화할 수 있는 최적의 조립 시나리오를 스스로 설계하여 실행합니다.

단순한 반복 동작이 아닌, 비전 피드백과 메모리 기반 제어를 결합하여 고도의 정밀도가 요구되는 5단 적재 및 복합 구조물 조립을 완수합니다.

🧱 조립 가능한 듀플로 레시피 (Assembly Recipes)
현재 인벤토리 상태에 따라 다음 12가지 레시피 중 최적의 조합을 선택하여 조립합니다.

기본 조합: 🔋 배터리, 🧲 자석, 🛑 비상정지

중급 조합: 🥕 당근, 🚦 신호등, 🌳 작은 나무, 🔨 망치

고급 조합: 🥕 큰 당근, 🍔 버거, 🍦 아이스크림, 🌳 큰 나무, 🧱 Studs Y (5단 결합)

🚀 핵심 기능 및 트러블슈팅 (Key Features & Dev Log)
1. 인벤토리 기반 DFS 최적 조립 계획 (Inventory-Aware DFS Planning)
문제점: 필드에 블록이 무작위로 흩어져 있을 때, 특정 레시피만 고집하면 자원이 낭비되거나 조립 도중 블록이 부족해 멈추는 현상이 발생했습니다.

해결책: 조립 전 비전 노드의 count_ 모드를 호출하여 전체 인벤토리를 파악합니다. 이후 DFS 알고리즘을 통해 모든 경우의 수를 시뮬레이션하여, **잔여 블록이 가장 적게 남는 최적의 조립 리스트(best_plan)**를 생성하여 순차적으로 집행합니다.

2. 고정밀 2.0초 정밀 스캔 (Refined Median Scan)
문제점: 로봇 팔의 이동 직후 발생하는 잔진동과 뎁스 카메라 특유의 노이즈로 인해 목표 좌표가 순간적으로 튀어 조립에 실패하는 사례가 있었습니다.

해결책: 비전 노드에 2.0초 정밀 데이터 수집 로직을 구현했습니다. 2초간 들어오는 수많은 프레임의 좌표 데이터 중 **중간값(Median)**을 추출함으로써, 진동이나 일시적인 인식 오류에 영향을 받지 않는 "확정 타겟 좌표"를 확보하여 성공률을 극대화했습니다.

3. 동적 금지 구역 설정 (Dynamic Exclude Zone)
문제점: 조립 중인 구조물 옆의 재료 블록을 집으려 할 때, 이미 완성된 구조물을 재료로 착각하여 이중 인식을 하거나 충돌하는 문제가 발생했습니다.

해결책: 마스터 노드에서 비전 노드에 요청을 보낼 때 exclude_x/y 좌표를 함께 전달합니다. 비전 노드는 전달받은 좌표의 5cm 이내에 있는 객체를 빨간색(IGNORED)으로 표시하며 인식 대상에서 제외하여, 간섭 없이 다음 블록만 정확히 찾아낼 수 있게 설계했습니다.

4. 시각 차폐 극대화: 메모리 기반 블라인드 스택 (Blind Stacking)
문제점: 3단 이상의 높은 구조물을 쌓을 때, 로봇 그리퍼와 이미 쌓인 블록들이 카메라 시야를 가려(Occlusion) 베이스 블록의 좌표를 잃어버리는 현상이 잦았습니다.

해결책: 인식이 가장 완벽한 시점의 좌표를 last_perfect_pose 변수에 저장합니다. 이후 가려진 위치에 블록을 꽂아야 할 때는 실시간 인식 대신 저장된 메모리 좌표에 높이(BLOCK_H) 오프셋만 계산하여 투입하는 방식을 적용하여 "눈을 감고도 정확히 꽂는" 동작을 구현했습니다.

5. 타겟팅 모드 분리 (Targeting Modes: Center & Far)
문제점: 여러 블록이 뭉쳐 있을 때 로봇과 가장 가까운 블록만 잡으려다 보니, 특정 조립(예: 아이스크림) 시 구조물 뒤쪽의 블록을 활용하지 못하는 한계가 있었습니다.

해결책: center_ 모드(화면 중앙 기준 최단 거리)와 far_ 모드(V좌표 기준 가장 멀리 있는 블록)를 구현했습니다. 특히 로봇 이동 후에도 가장 멀리 있는 새 블록을 추적하는 far_ 모드를 통해 조립 중인 구조물과의 간섭을 원천 차단했습니다.

🚀 Key Technical Challenges & Solutions
1. Advanced Yaw Calculation using Segmentation Mask
Problem: 단순 Bounding Box 기반 인식으로는 듀플로 블록의 정확한 회전 각도(Yaw)를 얻기 어려워 결합 실패 발생.

Solution: YOLOv8의 Segmentation Mask 데이터를 OpenCV의 minAreaRect로 처리하여 블록의 장축 방향을 계산하고, calculate_refined_yaw 함수를 통해 ±90도 오차를 보정한 정밀 각도를 산출.

2. Robust 5-Layer Assembly Strategy (Studs Y)
Problem: Studs Y 레시피와 같이 5단에 달하는 복잡한 결합 시 시각적 혼동이 극심함.

Solution: 1단계에서 베이스 좌표를 미리 스캔하여 저장하고, 이후 모든 결합을 Blind Insert로 처리하여 시각적 노이즈를 배제하는 전략으로 성공률 확보.

3. Modular Construction Logic (Ice Cream)
Problem: 한 번에 5단을 쌓는 것은 물리적 오차가 누적되어 불가능에 가까움.

Solution: 하단 모듈과 상단 모듈을 각각 별도의 위치에서 조립한 뒤, 최종적으로 상단 모듈을 들어 하단 모듈 위에 결합하는 분할 조립(Modular Assembly) 방식 채택.

👨‍💻 Author
정동혁 (Jeong Dong-hyeok)

Incheon National University, Dept. of Electrical Engineering

Major in Robotics & Control Engineering
