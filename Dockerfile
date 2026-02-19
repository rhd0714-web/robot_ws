# 1. 베이스 이미지: ROS 2 Humble (Desktop 버전이 GUI 도구 포함됨)
FROM osrf/ros:humble-desktop

# 2. 필수 도구 설치 (기존에 에러 났던 라이브러리들 미리 다 설치!)
# - python3-pip: 파이썬 패키지 관리자
# - libxcb...: Qt(화면) 관련 에러 방지용
# - nano, git: 기본 편집기
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    nano \
    git \
    libxcb-cursor0 \
    libxcb-xinerama0 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    ros-humble-plotjuggler-ros \
    && rm -rf /var/lib/apt/lists/*

# 3. 파이썬 라이브러리 설치 (무조코, OpenCV 등)
# - numpy: ROS 2 Humble과 호환되는 1.x 버전
# - opencv: numpy 1.x를 지원하는 마지막 버전
# - mediapipe & protobuf: 버전 짝꿍을 맞춰서 에러 방지
# - PyQt6: 대시보드 GUI용
RUN pip3 install --upgrade pip
RUN pip3 install \
    "numpy==1.26.4" \
    "opencv-python==4.10.0.84" \
    "mediapipe==0.10.9" \
    "protobuf==3.20.3" \
    mujoco \
    transforms3d \
    PyQt6

# 4. 사용자(User) 생성 (ID: 1000)
# 리눅스끼리는 파일 권한이 꼬일 수 있어서, 내 컴퓨터 유저랑 ID를 맞추는 게 중요합니다.
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 5. 하드웨어 권한 부여 (그룹이 없으면 강제로 만들고 추가함)
RUN groupadd -f video && \
    groupadd -f render && \
    groupadd -f dialout && \
    usermod -aG video,render,dialout $USERNAME

# 6. 환경 설정 (.bashrc에 자동 등록)
USER $USERNAME
# 터미널 켤 때마다 ROS2 기본 설정 자동 로드
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# 터미널 켤 때마다 내 워크스페이스 설정 자동 로드
# (주의: 처음엔 빌드가 안 돼서 에러가 날 수 있지만, 빌드 후엔 자동으로 적용됨)
RUN echo "source /home/user/robot_ws/install/setup.bash" >> ~/.bashrc
# 컬러 출력 등 편의 설정 (선택사항)
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc

# 7. 작업 폴더 설정
WORKDIR /home/$USERNAME/robot_ws

# 8. 시작 명령어
CMD ["/bin/bash"]
