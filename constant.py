# ==========================================================
# 1. [FIXED] 시스템 설계 상수 (수정 금지 - 환경 설정)

# --- 통신 전력 관련 (기기별 분리) ---
P_TX_TV = 0.2                   # Target Vehicle(차량) 송신 전력: 23dBm (약 0.2W)
P_TX_SV = 0.2                   # Service Vehicle(차량) 송신 전력: 23dBm (약 0.2W)
P_TX_RSU = 1.0                  # RSU 송신 전력: 30dBm (1.0W) - 차량보다 높게 설정
TYPE_TV = "TV"
TYPE_SV = "SV"

# --- 채널 및 물리 상수 (계산된 고정값) ---
NOISE_POWER = 1.38e-13          # 노이즈 전력 (Watts) - 계산식: 10^((-174+70)/10)/1000
WAVELENGTH = 0.05               # 파장: 5.9GHz 기준 약 0.05m
PATH_LOSS_EXP = 2.7             # 경로 손실 지수 (Urban 환경)
BW = 10e6                       # 대역폭 (10MHz)

# ==========================================================
# 2. [VARIABLE -> FIXED] 실험 후 고정
# --- 통신 및 자원 제약 ---
ONE_HOP_LIMIT = 250             # 1-hop 통신 범위 (250m)
SV_RESOURCE = 5 * (10**9)                # SV 연산 자원: 5 GHz
COMP_INTENSITY = 1000           # 연산 복잡도 (1000 cycles/bit)
WEIGHT_SV_DIR = 0.5                # SV 선택 시 방향 유사도 가중치
WEIGHT_SV_RATE = 0.5               # SV 선택 시 전송 효율 가중치
WEIGHT_RL_DELAY = 0.5              # relay 선택 시 딜레이 가중치
WEIGHT_RL_MOB = 0.5                # relay 선택 시 이동성 안정성 가중치

# ==========================================================
# 3. [VARIABLE] 실험 파라미터 (그래프 추출 시 수정)

# --- 실험 1: 데이터 크기 및 리턴 비율 ---
TASK_MIN_MBITS = 0.5             # 최소 0.5 Mbits
TASK_MAX_MBITS = 1.5             # 최소 1.5 Mbits
ALPHA = 0.3                      # 리턴 데이터 비율 (0.1 ~ 1.0 가변) -> 실험 시 사용할 디폴트 값
ALPHA_X_AXIS = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]       # 그래프 그릴 때 사용

# --- 실험 2: 환경 배치 및 밀도 ---
TOTAL_VEHICLES = 300            # 총 차량 수는 고정
TV_DENSITY_LIST = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9] # x축: 전체 중 TV가 차지하는 비율
CURRENT_TV_DENSITY = 0.8        # 현재 실험값

# --- 실험 3: 제약 조건 및 가중치 ---
MAX_LATENCY = 1.0               # 최대 허용 지연 (1s)
BACKHAUL_DELAY = 0.003          # RSU 백홀 지연 (3ms)
WEIGHT_F = 0.5                  # 실패율 가중치
WEIGHT_D = 0.5                  # 딜레이 가중치
