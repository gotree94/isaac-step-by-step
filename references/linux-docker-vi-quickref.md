# Linux · Docker · vi 빠른 참조

> Isaac Sim 5.1 커리큘럼 진행 시 자주 사용하는 명령어 모음
> **목적**: 설치부터 ROS2 디버깅, 파일 편집까지 한 번에 찾아보기

---

## 목차

1. [Linux 기본 명령어](#1-linux-기본-명령어)
2. [파일 · 디렉토리 조작](#2-파일--디렉토리-조작)
3. [사용자 · 권한](#3-사용자--권한)
4. [프로세스 · 시스템](#4-프로세스--시스템)
5. [네트워크](#5-네트워크)
6. [패키지 관리 (apt)](#6-패키지-관리-apt)
7. [환경 변수 · Shell](#7-환경-변수--shell)
8. [Docker 명령어](#8-docker-명령어)
9. [vi / vim 편집기](#9-vi--vim-편집기)
10. [Isaac Sim 자주 쓰는 명령어](#10-isaac-sim-자주-쓰는-명령어)

---

## 1. Linux 기본 명령어

### 1.1 길찾기

```bash
pwd                       # 현재 디렉토리 출력
ls                        # 파일 목록
ls -la                    # 상세 목록 (숨김 파일 포함)
ls -lh                    # 사람이 읽기 쉬운 크기 표시
cd ~                      # 홈 디렉토리로 이동
cd /isaac-sim             # 절대 경로로 이동
cd ..                     # 상위 디렉토리
cd -                      # 직전 디렉토리로 이동
tree -L 2                 # 디렉토리 트리 (깊이 2)
```

### 1.2 파일 보기

```bash
cat file.txt              # 파일 내용 출력 (짧은 파일)
less file.txt             # 스크롤 가능하게 보기 (q로 종료)
head -20 file.txt         # 처음 20줄
tail -50 file.txt         # 마지막 50줄
tail -f file.log          # 로그 실시간 추적 (Ctrl+C 종료)
nl file.txt               # 줄 번호와 함께 출력
wc -l file.txt            # 줄 수 세기
```

### 1.3 검색

```bash
grep "error" log.txt                  # 문자열 검색
grep -r "isaacsim" /etc               # 디렉토리 재귀 검색
grep -ri "error" /var/log             # 대소문자 무시 검색
grep -rn "import" src/                # 줄 번호 포함
grep -rl "SimulationApp" .            # 파일명만 출력
find . -name "*.py"                   # 파일명으로 검색
find . -type f -name "*isaac*"        # 파일명 패턴 검색
find . -size +100M                    # 100MB 이상 파일
which python3                         # 실행 파일 위치 찾기
whereis isaacsim                      # 바이너리 + 소스 + 매뉴얼 위치
```

---

## 2. 파일 · 디렉토리 조작

```bash
mkdir dir_name                        # 디렉토리 생성
mkdir -p a/b/c                        # 상위 디렉토리 함께 생성
touch file.txt                        # 빈 파일 생성 / 타임스탬프 갱신
cp source.txt dest.txt                # 파일 복사
cp -r src_dir/ dest_dir/              # 디렉토리 복사
mv old.txt new.txt                    # 이름 변경 / 이동
rm file.txt                           # 파일 삭제
rm -rf dir/                           # 디렉토리 강제 삭제 (주의!)
rm -i file.txt                        # 삭제 전 확인
ln -s /real/path link_name           # 심볼릭 링크 생성
```

### 자주 사용하는 조합

```bash
# Isaac Sim 설치 경로 확인
ls -la /isaac-sim

# 로그 파일 꼬리 보기
tail -f ~/.local/share/ov/pkg/isaac-sim-5.1.0/logs/*.log

# 특정 확장자만 찾아서 복사
find . -name "*.usd" -exec cp {} /backup/ \;

# 큰 파일 찾기 (정렬)
du -sh * | sort -hr
du -sh /isaac-sim/* | sort -hr | head -10
```

---

## 3. 사용자 · 권한

```bash
whoami                                # 현재 사용자
id                                    # 사용자 ID 정보
sudo command                          # 관리자 권한 실행
sudo -i                               # root shell 진입
sudo su -                             # root로 전환
chmod +x script.sh                    # 실행 권한 추가
chmod 755 script.sh                   # rwxr-xr-x
chmod 644 file.txt                    # rw-r--r--
chown user:group file.txt             # 소유자 변경
sudo !!                               # 직전 명령을 sudo로 재실행
```

### 권한 숫자 의미

| 숫자 | 권한 | 의미 |
|------|------|------|
| 7 | rwx | 읽기+쓰기+실행 |
| 6 | rw- | 읽기+쓰기 |
| 5 | r-x | 읽기+실행 |
| 4 | r-- | 읽기 전용 |
| 0 | --- | 권한 없음 |

`chmod 755` = 소유자(7) + 그룹(5) + 기타(5)

---

## 4. 프로세스 · 시스템

```bash
ps aux                                # 모든 프로세스 목록
ps aux | grep isaac                   # Isaac Sim 프로세스 찾기
top                                   # 실시간 프로세스 모니터 (q 종료)
htop                                  # 향상된 top (설치 필요)
kill 1234                             # PID로 프로세스 종료
kill -9 1234                          # 강제 종료 (SIGKILL)
pkill -f isaacsim                     # 이름으로 프로세스 종료
nvidia-smi                            # GPU 상태 확인
nvidia-smi -l 2                       # 2초마다 GPU 상태 갱신
free -h                               # 메모리 사용량
df -h                                 # 디스크 사용량
du -sh /isaac-sim                     # 특정 디렉토리 용량
uptime                                # 시스템 가동 시간
dmesg | grep nvidia                   # NVIDIA 드라이버 메시지
```

### Isaac Sim GPU 모니터링

```bash
# GPU 사용량 실시간 확인
watch -n 1 nvidia-smi

# 특정 GPU 정보만
nvidia-smi --query-gpu=name,temperature.gpu,utilization.gpu,memory.used --format=csv

# Isaac Sim 프로세스 메모리 확인
ps aux --sort=-%mem | grep isaac | head -5
```

---

## 5. 네트워크

```bash
ip a                                  # 네트워크 인터페이스 정보
ifconfig                              # (구) 네트워크 정보
ping 192.168.1.1                      # 연결 확인
ping -c 4 google.com                  # 4번만 ping
ss -tuln                              # 열린 포트 확인
netstat -tuln                         # (구) 열린 포트 확인
curl http://localhost:8011            # HTTP 요청 보내기
curl -X POST http://localhost:8000    # POST 요청
wget https://url.com/file.zip         # 파일 다운로드
nc -zv 192.168.1.100 22              # 특정 포트 연결 테스트
```

### ROS2 네트워크

```bash
# ROS2 도메인 ID 확인/설정
echo $ROS_DOMAIN_ID
export ROS_DOMAIN_ID=42

# ROS2 노드 목록
ros2 node list

# ROS2 토픽 목록
ros2 topic list -t                   # 토픽 타입 포함

# ROS2 멀티캐스트 테스트
ros2 multicast receive               # 수신 테스트
ros2 multicast send                  # 송신 테스트
```

---

## 6. 패키지 관리 (apt)

```bash
sudo apt update                       # 패키지 목록 갱신
sudo apt upgrade                      # 설치된 패키지 업그레이드
sudo apt install 패키지명              # 패키지 설치
sudo apt remove 패키지명               # 패키지 삭제
sudo apt purge 패키지명                # 설정 파일까지 삭제
sudo apt autoremove                   # 불필요 패키지 정리
apt search 키워드                      # 패키지 검색
apt show 패키지명                      # 패키지 정보
dpkg -l | grep nvidia                 # 설치된 NVIDIA 패키지 확인
```

### Isaac Sim 관련 패키지

```bash
# Python 3.11
sudo apt install python3.11 python3.11-venv python3.11-dev

# Isaac Sim 의존성
sudo apt install libegl1 libgl1-mesa-glx libxi6 libxrandr2

# nvidia-container-toolkit
sudo apt install nvidia-container-toolkit
sudo systemctl restart docker
```

---

## 7. 환경 변수 · Shell

```bash
# 환경 변수
export MY_VAR=값                      # 임시 설정
echo $MY_VAR                          # 값 확인
echo $PATH                            # 실행 경로 확인
env                                   # 전체 환경 변수 출력

# 영구 설정 (~/.bashrc 또는 ~/.bash_aliases)
echo 'export OMNIVERSE_ACCEPT_EULA=YES' >> ~/.bashrc
source ~/.bashrc                      # 설정 즉시 적용

# PATH에 추가
export PATH=$PATH:/isaac-sim

# Alias (단축 명령어)
alias ll='ls -la'
alias gs='git status'
alias isaac='cd /isaac-sim && ./isaac-sim.sh'
```

### .bashrc 꿀팁

```bash
# 자주 쓰는 설정을 ~/.bashrc 또는 ~/.bash_aliases 에 추가

# Isaac Sim alias
alias isaac='cd /isaac-sim && ./isaac-sim.selector.sh'

# GPU 모니터링 alias
alias gpumon='watch -n 1 nvidia-smi'

# ROS2 alias
alias ros2top='ros2 topic list -t'

# Python 가상환경 자동 활성화
alias isaacenv='source ~/isaac-step-curriculum/env_isaacsim/bin/activate'

# 긴 명령어 저장
echo "isaacenv" >> ~/.bashrc
```

---

## 8. Docker 명령어

```bash
docker pull nvcr.io/nvidia/isaac-sim:5.1.0    # 이미지 다운로드
docker images                                  # 로컬 이미지 목록

# Isaac Sim Container 실행 (Headless)
docker run --rm --gpus all -it \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  bash -c "cd /isaac-sim && ./isaac-sim.sh --headless"

# GUI 모드 (X11 공유)
docker run --rm --gpus all -it \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  nvcr.io/nvidia/isaac-sim:5.1.0

# Container 내부 bash 접속
docker run --rm --gpus all -it \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  --entrypoint /bin/bash \
  nvcr.io/nvidia/isaac-sim:5.1.0

# 실행 중인 Container 목록
docker ps
docker ps -a                              # 종료된 Container 포함

# Container 중지/삭제
docker stop CONTAINER_ID
docker rm CONTAINER_ID

# Container 로그
docker logs CONTAINER_ID
docker logs -f CONTAINER_ID               # 실시간 로그

# 이미지 삭제
docker rmi nvcr.io/nvidia/isaac-sim:5.1.0

# 시스템 정리
docker system prune -a                    # 사용하지 않는 모든 리소스 삭제

# NVIDIA Container Toolkit 확인
docker run --rm --gpus all nvidia/cuda:12.0-base nvidia-smi

# Docker Compose
docker-compose up -d                      # 백그라운드 실행
docker-compose down                       # 종료
```

### Isaac Sim Docker 유용 팁

```bash
# 데이터 볼륨 마운트
docker run --rm --gpus all -it \
  -v $(pwd)/projects:/workspace/projects \    # 프로젝트 디렉토리 공유
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -e OMNIVERSE_ACCEPT_EULA=YES \
  nvcr.io/nvidia/isaac-sim:5.1.0

# Port 포워딩 (WebRTC / ROS2)
docker run --rm --gpus all -it \
  -p 8011:8011 \                              # WebRTC 스트리밍
  -p 8888:8888 \                              # Jupyter 등
  nvcr.io/nvidia/isaac-sim:5.1.0
```

---

## 9. vi / vim 편집기

> Isaac Sim은 대부분 설정 파일이 `.json`, `.usd`, `.yaml` 형식입니다.
> vi 하나만 알면 파일 편집에 문제없습니다.

### 9.1 vi 시작과 종료

```bash
vi file.txt                           # 파일 열기
vim file.py                           # vim으로 열기 (향상된 vi)
```

| 명령 | 의미 |
|------|------|
| `vi 파일명` | 파일 열기 |
| `:q` | 종료 |
| `:q!` | 저장 없이 강제 종료 |
| `:w` | 저장 |
| `:wq` 또는 `ZZ` | 저장 후 종료 |
| `:wq!` | 강제 저장 후 종료 |
| `:e!` | 마지막 저장 상태로 되돌리기 |

### 9.2 모드 전환

```
NORMAL 모드        ← vi 시작 시 기본 모드 (명령 입력)
  │
  ├─ i  →  INSERT 모드 (텍스트 입력)
  ├─ v  →  VISUAL 모드 (블록 선택)
  └─ :  →  COMMAND-LINE 모드 (ex 명령어)

ESC 키를 누르면 항상 NORMAL 모드로 돌아옴
```

| 키 | 동작 |
|-----|-------|
| `ESC` | NORMAL 모드로 전환 |
| `i` | 현재 커서 위치에서 INSERT 시작 |
| `I` | 줄 맨 앞에서 INSERT 시작 |
| `a` | 현재 커서 다음에서 INSERT 시작 |
| `A` | 줄 맨 끝에서 INSERT 시작 |
| `o` | 아래 줄에 새 줄 추가 후 INSERT |
| `O` | 위 줄에 새 줄 추가 후 INSERT |
| `v` | VISUAL 모드 (방향키로 블록 선택) |
| `V` | 줄 단위 VISUAL 모드 |

### 9.3 커서 이동 (NORMAL 모드)

| 키 | 동작 |
|-----|-------|
| `h` `j` `k` `l` | ← ↓ ↑ → |
| `w` | 다음 단어로 |
| `b` | 이전 단어로 |
| `0` | 줄 맨 앞 |
| `$` | 줄 맨 끝 |
| `gg` | 파일 맨 처음 |
| `G` | 파일 맨 끝 |
| `:42` | 42번째 줄로 이동 |
| `Ctrl+d` | 반 페이지 아래로 |
| `Ctrl+u` | 반 페이지 위로 |
| `Ctrl+f` | 한 페이지 아래로 |
| `Ctrl+b` | 한 페이지 위로 |

### 9.4 편집 / 삭제 / 복사 / 붙여넣기

| 명령 | 동작 |
|------|------|
| `x` | 커서 위치 문자 삭제 |
| `dd` | 현재 줄 삭제 (잘라내기) |
| `3dd` | 3줄 삭제 |
| `dw` | 단어 삭제 |
| `D` | 줄 끝까지 삭제 |
| `yy` | 현재 줄 복사 |
| `3yy` | 3줄 복사 |
| `p` | 아래에 붙여넣기 |
| `P` | 위에 붙여넣기 |
| `u` | 실행 취소 (Undo) |
| `Ctrl+r` | 다시 실행 (Redo) |
| `.` | 마지막 명령 반복 |
| `>>` | 들여쓰기 (오른쪽) |
| `<<` | 내어쓰기 (왼쪽) |

### 9.5 검색 / 치환

```vim
/검색어           " 아래로 검색 (n = 다음, N = 이전)
?검색어           " 위로 검색
:%s/old/new/g     " 파일 전체 치환
:%s/old/new/gc    " 치환 시 확인
:5,20s/old/new/g  " 5~20번째 줄만 치환
:noh              " 검색 하이라이트 제거
```

### 9.6 여러 파일 / 창

```bash
vi file1.py file2.py              # 여러 파일 열기
:ls                               # 열린 파일 목록
:bnext 또는 :bn                    " 다음 파일
:bprev 또는 :bp                    " 이전 파일
:bd                               " 현재 파일 닫기
:sp file2.py                      " 수평 분할
:vsp file2.py                     " 수직 분할
Ctrl+w w                          " 창 전환
Ctrl+w q                          " 현재 창 닫기
```

### 9.7 vi 설정 (.vimrc)

```vim
" ~/.vimrc 에 추가
set number                        " 줄 번호 표시
set hlsearch                      " 검색 하이라이트
set incsearch                     " 점진적 검색
set tabstop=4                     " 탭 = 4칸
set shiftwidth=4                  " 자동 들여쓰기 = 4칸
set expandtab                     " 탭 → 공백 변환
set autoindent                    " 자동 들여쓰기
set mouse=a                       " 마우스 지원
syntax on                         " 구문 강조
```

### 9.8 실전 예제 — Isaac Sim 설정 파일 수정

```bash
# 1. 설정 열기
vi ~/isaac-step-curriculum/config/settings.json

# 2. 검색 (NORMAL 모드에서)
/RTX                            # RTX 관련 설정 찾기

# 3. 수정 (i로 INSERT 모드)

# 4. 저장 후 종료
:wq

# 5. 다른 파일 열기 (vi 종료하지 않고)
:vsp ~/.bashrc
Ctrl+w w                        # 창 전환
```

---

## 10. Isaac Sim 자주 쓰는 명령어

```bash
# GUI 실행
isaacsim

# Headless 실행
isaacsim --headless

# 특정 USD 파일 로드
isaacsim /path/to/scene.usd

# WebRTC 활성화 실행
isaacsim --enable-rtx-webrtc --webrtc-server-port 8011

# Extension 개발 모드
isaacsim --ext-folder /workspace/exts

# Python 스크립트 실행 (pip 설치 기준)
python -c "
from isaacsim import SimulationApp
app = SimulationApp({'headless': True})
# ... your code ...
app.close()
"

# Compatibility Check
/isaac-sim/isaac-sim.compatibility_check.sh

# App Selector
/isaac-sim/isaac-sim.selector.sh
```

---

> **참고**: 이 문서는 Isaac Sim 5.1 기준으로 작성되었으며, Ubuntu 22.04 LTS 환경을 기준으로 합니다.
> 더 자세한 정보는 각 명령어의 `man` 페이지 (`man ls`, `man vi`)를 참고하세요.
