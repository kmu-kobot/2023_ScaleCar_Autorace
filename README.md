# 스케일카 기반 자율주행 경진대회

2023년 하계 혁신융합대학 스케일카 기반 자율주행 경진대회

[대회 공지 링크](https://coss.kookmin.ac.kr/fvedu/community/notice.do?mode=view&articleNo=5904366&article.offset=10&articleLimit=10)

## 📺 시연 영상

추후 업로드

## 소개

- 도로 주행
    - **Sliding Window**
    - slidewindow.py
- 어린이 보호 구역
    - [**Aruco Marker Detector** - fiducial](https://github.com/UbiquityRobotics/fiducials/)
    - 
- Rubber cone 주행
    - [Obstacle Detector](https://github.com/tysik/obstacle_detector)
- 정적 장애물
    - [Obstacle Detector](https://github.com/tysik/obstacle_detector)
- 동적 장애물
    - OpenCV

```bash
git clone https://github.com/kmu-kobot/2023_ScaleCar_Autorace.git
catkin_make
roslaunch main main.launch
```

## 시스템 구성 및 아키텍처

![rosgraph](https://github.com/kmu-kobot/2023_ScaleCar_Autorace/assets/84698896/40a653a7-ce15-47c8-a24b-b4c1ff280f5d)

## 💻 개발 환경 및 개발 언어

- 운영체제: Ubuntu 20.04, ROS noetic
- IDE: Visual Studio Code
- 개발 언어: Python 3.10.4
- 협업 툴: Github, Notion

## 팀 정보

국민대학교 소프트웨어융합대학 임베디드 소프트웨어 동아리 **KOBOT** 12기 ROBOT 팀

| 이름 | 이메일 | 담당 |
| --- | --- | --- |
| [안선영](https://github.com/SeoooooNyeong) | bm9024@kookmin.ac.kr | TM, 시뮬레이션 환경 구축 |
| [안지한](https://github.com/Anjihan) | jihan5575@kookmin.ac.kr | ROS, HW 담당 |
| [이세현](https://github.com/sehyeon518) | lifethis21@kookmin.ac.kr | 알고리즘, 주행 제어 |
| [차예찬](https://github.com/ChaNeeeeeee) | 3004yechan@kookmin.ac.kr | 영상처리, 알고리즘 |
