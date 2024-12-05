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
차를 Carla 시뮬레이터 상에서 경로(Lap)을 코스를 이탈하지 않고 최대한 빠르게 주행.   
차의 경로를 나타내는 waypoint(x, y, z)들의 좌표들을 미리 받음.   
차의 현재 좌표와 방향, 그리고 조향각을 실시간으로 입력으로 받으면, waypoint의 좌표와 같이 처리해서 차의 조향각과 스로틀, 그리고 브레이크값을 산출한다.

3. 이론   
(모든 각은 $(-\pi, \pi) 범위에 있는 것으로 가정하며, 각의 덧셈과 뺄셈 후에는 이 범위에 들어오도록 $2\pi$ 씩 더해지고 빼진다고 가정한다.)
    1. waypoint의 차 기준 상대 좌표 (에러)   
    차의 절대좌표계 기준 현재 위치와 방향을 각각 $x_c$, $y_c$, ${\theta}_c$ 라고 하고, 차가 현재 도달하고자 하는 절대좌표계 기준 waypoint의 위치와 방향을 $x_r$, $y_r$, ${\theta}_r$ 라고 하자.   
    또한, waypoint는 기준 속력 $v_r$ 과 기준 각속도 ${\omega}_r$ 또한 가진다.   
    waypoint의 차 좌표계 기준 위치 $x_e$, $y_e$ 와 방향 ${\theta}_e$ 은 다음과 같다.   

```math
\begin{bmatrix} x_e \\\ y_e \end{bmatrix} = \begin{bmatrix} \cos({\theta}_c) & \sin({\theta}_c) \\\ -\sin({\theta}_c) & \cos({\theta}_c) \end{bmatrix}\begin{bmatrix} x_r - x_c \\\ y_r - y_e \end{bmatrix}
```

    2. 카나야마 컨트롤러   
    카나야마 컨트롤러는 waypoint의 에러 $x_e$, $y_e$ 와 방향 ${\theta}_e$ 와 기준 속력 $v_r$ 과 각속도 ${\omega}_r$ 를 입력으로 받아 차가 내야하는 속력과 각속도를 산출한다.
    카나야마 컨르롤러의 게인 $K_x$, $K_y$, 그리고 $K_{\omega}$ 를 설정할 수 있으며, 컨트롤러가 산출하는 속력을 $v$, 각속도를 ${\omega}$라 한다면, 아래와 같다.

    ```latex
    v = v_r \cos({\theta}_e) + K_{x}x_{e}
    ```