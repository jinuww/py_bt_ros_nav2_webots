# py_bt_ros_nav2_webots

<p align="center">
  <img src="docs/hero.gif" alt="BT Nav2 Webots Mission Demo" width="600" />
</p>

<h1 align="center">Webots + Nav2 + Behavior Tree 미션 데모</h1>

<p align="center">
  Webots 환경에서 TurtleBot3가 Behavior Tree를 기반으로 <b>목표 이동 → 이미지 캡처 → 원위치 복귀</b>를 수행하는 프로젝트
</p>

<p align="center">
  <!-- 예시 뱃지들, 필요 없으면 지워도 됨 -->
  <img src="https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white" />
  <img src="https://img.shields.io/badge/Sim-Webots-00BFFF?logo=webots&logoColor=white" />
  <img src="https://img.shields.io/badge/Robot-TurtleBot3-green?logo=ros" />
  <img src="https://img.shields.io/badge/BT-py_bt_ros-blueviolet" />
  <img src="https://img.shields.io/badge/Language-Python3-yellow" />
</p>

---

## ✨ 프로젝트 소개

본 프로젝트는 Webots 시뮬레이터에서 TurtleBot3가 다음과 같은 **3단계 미션**을 수행

1. **목표 지점 이동**  
   - RViz에서 `/bt/goal_pose` 로 목표를 지정하면  
   - Nav2의 `/navigate_to_pose` 액션을 통해 해당 지점까지 자율 주행

2. **도착 지점에서 이미지 1장 캡처**  
   - `/TurtleBot3Burger/front_camera/image_color` 토픽을 수신하는  
     이미지 캡처 서비스 서버(`/bt/capture_image`)를 호출하여  
   - 도착 시점의 카메라 이미지를 저장

3. **초기 위치로 복귀(Return)**  
   - 첫 goal이 들어올 때의 `/amcl_pose` 를 `home_pose` 로 저장해 두었다가  
   - Nav2 액션을 다시 사용해 최초 위치로 되돌아감

이 과정 전체는 `py_bt_ros` 를 활용한 **Behavior Tree**로 제어합니다.



## 🧠 Behavior Tree 구조

프로젝트에서 사용한 Behavior Tree의 핵심 구조는 아래와 같습니다.

<pre>
<code>
Sequence
 ├─ HasGoal        # goal 수신 여부 + home_pose 저장
 ├─ MoveToGoal     # Nav2 /navigate_to_pose 로 목표 지점 이동
 ├─ CaptureImage   # /bt/capture_image 서비스 호출 (이미지 1장 저장)
 └─ Return         # home_pose 로 복귀 후 블랙보드 초기화
</code>
</pre>

## 🏗 시스템 구조
<pre>
<code>
RViz (/bt/goal_pose)          Camera (/TurtleBot3Burger/front_camera/image_color)
          │                                         │
          ▼                                         ▼
    [py_bt_ros]                             [ImageCaptureNode]
    ├─ HasGoal                              └─ /bt/capture_image (Trigger 서비스)
    ├─ MoveToGoal ──────> Nav2 /navigate_to_pose
    ├─ CaptureImage ────> /bt/capture_image
    └─ Return ─────────> Nav2 /navigate_to_pose

</code>
</pre>


## 📂 프로젝트 구조

<pre>
<code>
.
├── README.md
├── py_bt_ros/
│   ├── config_nav2.yaml              # Nav2 + Webots 환경용 BT 설정
│   └── scenarios/
│       └── nav2_webots/
│           ├── bt_nodes.py           # HasGoal / MoveToGoal / CaptureImage / Return
│           └── default_bt.xml        # Behavior Tree 정의
└── bt_image_capture/                 # 이미지 캡처 서비스 패키지
    ├── package.xml
    ├── setup.py
    └── bt_image_capture/
        └── take_picture_node.py      # /bt/capture_image 서비스 서버 노드
</code>
</pre>


## 🚀 실행방법
1) Webots + Nav2 실행
<pre>
<code>
cd ~/webots_ros2_ws
source install/local_setup.bash

ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true
</code>
</pre>

2) 이미지 캡처 서비스 노드 실행
<pre>
<code>
cd ~/webots_ros2_ws
source install/local_setup.bash

ros2 run bt_image_capture bt_take_picture
</code>
</pre>

3) Behavior Tree 실행 (py_bt_ros)
<pre>
<code>
cd ~/py_bt_ros
source ~/webots_ros2_ws/install/local_setup.bash

python3 main.py --config config_nav2.yaml
</code>
</pre>

4) RViz에서 미션 수행
  - RViz에서 2D Goal Pose 도구 선택

  - Topic을 /bt/goal_pose 로 변경

  - 지도의 임의의 지점을 클릭하여 목표 설정

  - 이후 자동 플로우

    - 로봇이 Nav2를 통해 목표 지점으로 이동

    - 도착 후 이미지 1장 캡처 (~/bt_images/ 에 저장)

    - 최초 위치로 복귀(Return)

## 🧩 사용 기술 & 역할

- ROS2 Humble

  - Nav2 액션 클라이언트/서버 연동

  - 서비스/토픽 인터페이스 설계

- py_bt_ros

  - Behavior Tree 노드 설계 (Condition / Action)

  - 블랙보드 기반 상태 공유 및 미션 관리

- Webots + TurtleBot3

  - 시뮬레이션 환경 설정 및 센서/액추에이터 연동

- OpenCV + cv_bridge

  - ROS Image → OpenCV 이미지 변환

  - 특정 시점(도착 시점)의 이미지 파일 저장 로직 구현

- Python

  - ROS2 노드, BT 노드, 서비스 서버/클라이언트 구현
