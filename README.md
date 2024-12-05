### 카나야마 컨트롤러를 이용한 차량 제어 ROS 노드 구현

본 레포지토리는 성균관대학교 기계공학과 수업 '모바일시스템제어'의 최종 프로젝트에 사용되는 코드임.

1. 사용법
    1. 환경 준비
        1. ```git clone https://github.com/tepk2924/Controller.git```
        2. 일단 ROS1 환경 (Noetic)을 구성하고, 파이썬을 이용할 수 있도록 세팅.
        3. requirments.txt에 적힌 파이썬 라이브러리 설치 (가상환경 추천).
        4. ```cd ros_ws```
        5. ```catkin_make``` 혹은, 라이브러리를 가상환경에 설치했을 경우, ```catkin_make -DPYTHON_EXECUTABLE=<가상환경 경로>/bin/python3```
    2. ROS로 돌리기

2. 목적   
    차를 Carla 시뮬레이터 상에서 경로(Lap)을 코스를 이탈하지 않고 최대한 빠르게 주행. 차의 경로를 나타내는 waypoint(x, y, z)들의 좌표들을 미리 받음. 차의 현재 좌표와 방향, 그리고 조향각을 실시간으로 입력으로 받으면, waypoint의 좌표와 같이 처리해서 차의 조향각과 스로틀, 그리고 브레이크값을 산출한다.