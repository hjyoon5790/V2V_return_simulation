import math
import numpy as np
import traci
import random
import constant as c    # 상수 파일 임포트

""" 세 점 A,BC의 방향성을 외적으로 계산 """
def ccw(A, B, C):
    return (B[0] - A[0]) * (C[1] - A[1]) - (B[1] - A[1]) * (C[0] - A[0])

""" 선분 AB(차량 통신선)와 선분 CD(건물 벽면)가 교차하는지 판별 """
def is_intersect(A, B, C, D):
    c2 = ccw(A, B, C) * ccw(A, B, D)     # C2: 통신 링크 기준 벽 끝점들의 위치
    c1 = ccw(C, D, A) * ccw(C, D, B)     # C1: 벽 기준 차량들의 위치
    if c1 < 0 and c2 < 0:
        return True
    return False

""" TV와 SV 사이에 NLOS 여부 확인 """
def check_los(tv_pos, sv_pos, building_walls):
    A = tv_pos
    B = sv_pos
    
    for wall in building_walls:
        C, D = wall[0], wall[1]
        if is_intersect(A, B, C, D):
            return False    # 하나라도 교차하면 NLOS
    return True         # 어떤 벽과도 교차하지 않으면 LOS
    
""" 한 번의 TV 요청에 사용할 동일한 Task 조건 생성 """
def generate_random_task():
    # 1. 이번 TV의 랜덤 task 크기 생성 (2~4 Mbits)
    task_mbits = random.uniform(c.TASK_MIN_MBITS, c.TASK_MAX_MBITS)
    task_bits = task_mbits * (10**6)    # 비트 단위로 변환
    computation_intensity = random.uniform(c.COMP_MIN_INTENSITY, c.COMP_MAX_INTENSITY)
    
    # 2. 연산 시간(Comp Delay) 미리 계산
    t_comp = (task_bits * computation_intensity) / c.SV_RESOURCE
    latency_constraint = random.uniform(c.MIN_LATENCY_CONSTRAINT, c.MAX_LATENCY_CONSTRAINT)
    
    return task_bits, t_comp, latency_constraint

""" 두 좌표(pos1과 pos2) 사이의 거리를 계산하는 함수 """
def calculate_distance(pos1, pos2):
    # pos1[0]은 x좌표, pos1[1]은 y좌표
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

""" 거리에 따른 전파 감소(Path Loss)를 계산하는 함수 """
def calculate_channel_gain(distance):
    if distance == 0:
        return 0
    # 물리 공식 구현: (파장 / (4 * pi * 거리)) ^ 경로손실지수
    return (c.WAVELENGTH / (4 * math.pi * distance)) ** c.PATH_LOSS_EXP

""" V2I 전용 채널 이득 계산 """
def calculate_v2i_gain(distance):
    if distance == 0: return 0
    return (c.WAVELENGTH / (4 * math.pi * distance)) ** c.V2I_PATH_LOSS_EXP

""" 두 차량의 이동성 안정성 점수 구하는 함수(벡터 내적 이용) """
def calculate_mobility_stability(id1, id2, step_info):
    # 차량의 속력(speed)과 방향(angle) 가져옴
    speed1 = step_info[id1]['speed']
    angle1 = math.radians(step_info[id1]['angle'])
    speed2 = step_info[id2]['speed']
    angle2 = math.radians(step_info[id2]['angle'])
    
    # 정지 상태일 경우 분모가 0이 되는 것을 방지 (중립값 0 반환)
    if speed1 < 0.1 or speed2 < 0.1:
        return 0.0

    # 속력과 각도를 이용해 x축, y축 방향의 벡터(화살표) 길이 구함
    v1 = (speed1 * math.sin(angle1), speed1 * math.cos(angle1))
    v2 = (speed2 * math.sin(angle2), speed2 * math.cos(angle2))

    dot_product = v1[0]*v2[0] + v1[1]*v2[1]
    mag1 = math.sqrt(v1[0]**2 + v1[1]**2)
    mag2 = math.sqrt(v2[0]**2 + v2[1]**2)

    # 코사인 유사도 S = (A·B) / (|A||B|)
    return dot_product / (mag1 * mag2)

""" 차량과 고정된 RSU 사이의 이동성 안정성 점수 구하는 함수 """
def calculate_rsu_mobility_stability(veh_id, step_info):
    veh_pos = step_info[veh_id]['pos']
    speed = step_info[veh_id]['speed']
    angle = math.radians(step_info[veh_id]['angle'])
    rsu_pos = c.RSU_POS  # 토폴로지에 RSU는 하나이므로 상수를 직접 사용

    if speed < 0.1: return 0.0

    # 1. 차량의 속도 벡터 (v_x, v_y)
    v_veh = (speed * math.sin(angle), speed * math.cos(angle))
    
    # 2. 차량에서 RSU를 향하는 방향 벡터 (u_x, u_y)
    u_to_rsu = (rsu_pos[0] - veh_pos[0], rsu_pos[1] - veh_pos[1])
    mag_u = math.sqrt(u_to_rsu[0]**2 + u_to_rsu[1]**2)
    
    if mag_u < 0.1: return 1.0 # RSU 바로 위라면 최상급 안정성

    # 3. 두 벡터 사이의 코사인 유사도 계산
    dot_product = v_veh[0]*u_to_rsu[0] + v_veh[1]*u_to_rsu[1]
    return dot_product / (speed * mag_u)

""" 다른 TV들이 쏘는 전파가 내가 고르려는 SV에게 얼마나 잡음인지 계산(잠재 지연) """
def get_total_interference(tx_id, rx_id, step_info):
    total_interf = 0        # 간섭을 더할 변수 초기화
    
    # 수신측(rx)의 좌표 결정 (ID가 들어오면 위치를 찾고, 좌표가 직접 들어오면 그대로 사용)
    rx_pos = step_info[rx_id]['pos'] if isinstance(rx_id, str) else rx_id

    # 나(TV)를 제외한 다른 TV를 찾음
    for other_v, info in step_info.items():
        if other_v != tx_id and info['type'] == c.TYPE_TV:
            # 다른 TV와 타겟 SV 사이의 거리 구함 + 벽에 막힌 전파 무시
            other_pos = info['pos']
            if not check_los(other_pos, rx_pos, c.BUILDING_WALLS):
                continue
            dist_to_rx = calculate_distance(other_pos, rx_pos)
            gain = calculate_channel_gain(dist_to_rx)
            total_interf += (c.P_TX_TV * gain)
    
    return total_interf

""" TV를 위해 최적의 SV 선택하는 함수 """
def proposed_sv_selection(tv_id, step_info, task_info, n_sv_sharing=1):
    # [추가] 최대 접속 인원 초과 시 실패 처리
    if n_sv_sharing > c.MAX_N_SHARING:
        return None, c.PENALTY_DELAY

    task_bits = task_info['task_bits']
    t_comp = task_info['t_comp']
    latency_constraint = task_info['lat_constraint']
    tv_pos = step_info[tv_id]['pos']
    sv_candidates = []
    
    # 3. 범위 내 SV 탐색 및 필터링 (Type & Distance & Delay)
    for v_id, info in step_info.items():
        if v_id == tv_id:
            continue
        if info['type'] == c.TYPE_SV:
            sv_pos = info['pos']
            dist = calculate_distance(tv_pos, sv_pos)   
            
            # (1) 통신 범위 확인
            if dist <= c.ONE_HOP_LIMIT:
                # (2) LOS 필터링
                if not check_los(tv_pos, sv_pos, c.BUILDING_WALLS):
                    continue 
                # SV 후보의 상세 정보 계산
                cos_sim = calculate_mobility_stability(tv_id, v_id, step_info)
                mob_score = (cos_sim + 1) / 2  # 0~1 사이로 정규화
                # 통신 속도(Rate) 계산
                gain = calculate_channel_gain(dist)
                interf = get_total_interference(tv_id, v_id, step_info)
                sinr = (c.P_TX_TV * gain) / (interf + c.NOISE_POWER)
                
                # [수정] 대역폭 1/N 분할 적용
                rate = (c.BW / n_sv_sharing) * math.log2(1 + sinr) if sinr > 0 else 0
                
                if rate > 0:
                    t_tx = task_bits / rate
                    # [수정] 가용 CPU 자원을 N등분 (연산 시간 t_comp는 N배 증가)
                    t_off = t_tx + (t_comp * n_sv_sharing)
                
                    # 최대 허용 지연을 넘지 않는 후보만 추가
                    if t_off <= latency_constraint:
                        sv_candidates.append({'id': v_id, 'rate': rate, 'mob_score': mob_score, 't_off': t_off})
                
    # 4. 후보 없음: 1-hop 이내에 차량 없음 + 딜레이 제약
    if not sv_candidates:
        ### 여기에 나중에 RSU에게 넘기는 코드 추가
        return None, c.PENALTY_DELAY
    
    # 5. 정규화 및 최종 점수 계산(살아남은 후보끼리 경쟁)
    # rate가 가장 높은 SV 찾기(분모 0 방지)
    max_rate = max(sv['rate'] for sv in sv_candidates)
    best_sv_id = None
    max_final_score = -1        ## 최고 점수를 찾기 위해 가장 낮은 값으로 초기화
    
    for sv in sv_candidates:
        eff_score = sv['rate'] / max_rate if max_rate > 0 else 0    # 속도 정규화 점수
        # 최종 점수 = (속도 가중치 * 효율점수) + (방향 가중치 * 방향점수)
        sv['final_score'] = (c.WEIGHT_SV_RATE * eff_score) + (c.WEIGHT_SV_DIR * sv['mob_score'])
        
        # 최대 점수 갱신
        if sv['final_score'] > max_final_score:
            max_final_score = sv['final_score']
            best_sv_id = sv['id']
            best_t_off = sv['t_off']
            
    return best_sv_id, best_t_off

""" [모델링 3, 4, 5단계] 리턴 경로 평가 및 최종 전송 로직 """
def proposed_evaluate_return_path(tv_id, sv_id, step_info, task_info, t_off, n_sv_sharing=1, n_rl_sharing=1):    
    tv_pos = step_info[tv_id]['pos']
    sv_pos = step_info[sv_id]['pos']
    d_out = task_info['task_bits'] * c.ALPHA
    t_rem = task_info['lat_constraint'] - t_off
    
    if t_rem <= 0: return False, c.PENALTY_DELAY

    # 3. 리턴 경로 상태 확인 (Direct Link Check)
    dist = calculate_distance(sv_pos, tv_pos)
    # 1-hop 이내이고 가시선(LOS)이 확보된 경우 우선적으로 직접 전송
    if dist <= c.ONE_HOP_LIMIT and check_los(tv_pos, sv_pos, c.BUILDING_WALLS):
        gain = calculate_channel_gain(dist)
        interf = get_total_interference(sv_id, tv_id, step_info)
        # [수정] SV의 리턴 대역폭도 공유
        rate = (c.BW / n_sv_sharing) * math.log2(1 + (c.P_TX_SV * gain) / (interf + c.NOISE_POWER)) if (interf + c.NOISE_POWER) > 0 else 0
        if rate > 0:
            t_return = d_out / rate
            if t_return <= t_rem:
                return True, t_off + t_return

    # 4 & 5 통합. 최적 릴레이 선택 (Relay Vehicle + RSU Candidate Pool)
    relay_candidates = []
    min_cost = float('inf')

    # --- 후보 1: 주변 차량들 (V2V Relay) ---
    for rl_id, info in step_info.items():
        
        if rl_id in [tv_id, sv_id]: continue
        rl_pos = info['pos']
        
        dist_s_r = calculate_distance(sv_pos, rl_pos)
        dist_r_t = calculate_distance(rl_pos, tv_pos)
            
        # 릴레이도 각 링크가 1-hop 범위 내에 있어야 함
        if dist_s_r > c.ONE_HOP_LIMIT or dist_r_t > c.ONE_HOP_LIMIT:
            continue
        
        # 릴레이 노드 용량 초과 체크
        if n_rl_sharing > c.MAX_N_SHARING: continue

        # 1차 필터링: 양방향 LOS 확보
        if check_los(sv_pos, rl_pos, c.BUILDING_WALLS) and check_los(rl_pos, tv_pos, c.BUILDING_WALLS):
            # SV -> RL 전송 속도
            # [수정] 릴레이 노드의 대역폭 공유 적용
            rate_s_r = (c.BW / n_rl_sharing) * math.log2(1 + (c.P_TX_SV * calculate_channel_gain(dist_s_r)) / (get_total_interference(sv_id, rl_id, step_info) + c.NOISE_POWER))
            rate_r_t = (c.BW / n_rl_sharing) * math.log2(1 + (c.P_TX_SV * calculate_channel_gain(dist_r_t)) / (get_total_interference(rl_id, tv_id, step_info) + c.NOISE_POWER))
            
            if rate_s_r > 0 and rate_r_t > 0:
                t_rl = (d_out / rate_s_r) + (d_out / rate_r_t)
                if t_rl <= t_rem:
                    # 2차 점수 산정 (이동성 안정성)
                    s_sv = calculate_mobility_stability(rl_id, sv_id, step_info)
                    s_tv = calculate_mobility_stability(rl_id, tv_id, step_info)
                    s_mob = (s_sv + s_tv) / 2
                    s_mob_tilde = (s_mob + 1) / 2  # 모델링의 tilde{S}_mob^(k)
                    
                    # 최종 비용 함수 계산
                    v_cost = c.WEIGHT_RL_DELAY * (t_rl / t_rem) + c.WEIGHT_RL_MOB * (1 - s_mob_tilde)
                    relay_candidates.append({'type': 'VEHICLE', 'cost': v_cost, 'total_time': t_off + t_rl})

    # --- 후보 2: 인프라 (RSU Relay/Fallback) ---
    # RSU는 항상 고정된 위치에 있으며 일반적으로 높은 곳에 설치되어 LOS 환경을 가정함
    dist_s_rsu = calculate_distance(sv_pos, c.RSU_POS)
    dist_rsu_t = calculate_distance(c.RSU_POS, tv_pos)
    
    # RSU 용량 초과 체크
    if n_rl_sharing > c.MAX_N_SHARING: pass # RSU는 용량이 크다고 가정할 수도 있지만, 요청대로 체크 적용 가능

    # [수정] RSU는 대역폭 공유 시 BW_RSU를 사용하며, 간섭 없이 NOISE만 고려 (OFDMA 가정)
    rate_s_rsu = (c.BW_RSU / n_rl_sharing) * math.log2(1 + (c.P_TX_SV * calculate_v2i_gain(dist_s_rsu)) / c.NOISE_POWER)
    rate_rsu_t = (c.BW_RSU / n_rl_sharing) * math.log2(1 + (c.P_TX_RSU * calculate_v2i_gain(dist_rsu_t)) / c.NOISE_POWER)
    
    if rate_s_rsu > 0 and rate_rsu_t > 0:
        t_rsu = (d_out / rate_s_rsu) + c.BACKHAUL_DELAY + (d_out / rate_rsu_t)
        if t_rsu <= t_rem:
            # RSU와의 상대적 이동성 점수 계산 (단일 RSU이므로 차량 ID만 전달)
            s_sv_rsu = calculate_rsu_mobility_stability(sv_id, step_info)
            s_tv_rsu = calculate_rsu_mobility_stability(tv_id, step_info)
            
            s_mob_avg = (s_sv_rsu + s_tv_rsu) / 2
            s_mob_tilde = (s_mob_avg + 1) / 2  # [-1, 1] 범위를 [0, 1]로 정규화
            
            rsu_cost = c.WEIGHT_RL_DELAY * (t_rsu / t_rem) + c.WEIGHT_RL_MOB * (1 - s_mob_tilde)
            relay_candidates.append({'type': 'RSU', 'cost': rsu_cost, 'total_time': t_off + t_rsu})

    # 모든 후보(차량 + RSU) 중 최적의 경로 선택
    if relay_candidates:
        best_candidate = min(relay_candidates, key=lambda x: x['cost'])
        return True, best_candidate['total_time']

    return False, c.PENALTY_DELAY # 모든 경로 실패



""" [비교 스킴1] distance-based greedy """
def distance_greedy_sv_selection(tv_id, step_info, task_info, n_sv_sharing=1):
    # [추가] 최대 접속 인원 초과 시 실패 처리
    if n_sv_sharing > c.MAX_N_SHARING:
        return None, c.PENALTY_DELAY

    task_bits = task_info['task_bits']
    t_comp = task_info['t_comp']
    latency_constraint = task_info['lat_constraint']
    
    tv_pos = step_info[tv_id]['pos']   # 현재 TV 위치 가져오기
    best_sv_id = None
    min_dist = float('inf')     # 가장 짧은 거리를 찾기 위해 무한대로 초기화
    
    # 1. 거리만 보고 가까운 SV 뽑음    
    for v_id, info in step_info.items():
        if v_id == tv_id:
            continue            # 나 자신은 제외
            
        # 차량 타입이 SV인 경우만 확인
        if info['type'] == c.TYPE_SV:
            sv_pos = info['pos']
            dist = calculate_distance(tv_pos, sv_pos)    # 거리 계산
                
            # 1-hop 범위 안에 있고 지금까지 찾은 거리보다 가까우면 갱신 
            if dist <= c.ONE_HOP_LIMIT:
                if dist < min_dist:
                    min_dist = dist
                    best_sv_id = v_id
    if best_sv_id is None:              # 반경 내 SV가 아예 없으면 실패
        return None, c.PENALTY_DELAY

    sv_pos = step_info[best_sv_id]['pos']

    # 2. 채점 (Validation): 거리만 보고 가장 가까운 SV를 골랐으므로, 여기서 LOS를 체크함
    # 만약 NLOS라면 다른 대안을 찾지 않고 그리디 원칙에 따라 바로 실패(None) 반환
    if not check_los(tv_pos, sv_pos, c.BUILDING_WALLS):
        return None, c.PENALTY_DELAY

    dist = calculate_distance(tv_pos, sv_pos)
    gain = calculate_channel_gain(dist)
    interf = get_total_interference(tv_id, best_sv_id, step_info)
    sinr = (c.P_TX_TV * gain) / (interf + c.NOISE_POWER)
    rate = (c.BW / n_sv_sharing) * math.log2(1 + sinr) if sinr > 0 else 0
                
    # 3. 채점 (Validation): 거리만 보고 고른 SV가 실제로 지연 시간 조건을 만족하는지 확인
    if rate > 0:
        t_tx = task_bits / rate
        t_off = t_tx + (t_comp * n_sv_sharing)
        if t_off <= latency_constraint:
            return best_sv_id, t_off
    return None, c.PENALTY_DELAY     # 시간 제약 넘겼으니 실패

def distance_greedy_evaluate_return_path(tv_id, sv_id, step_info, task_info, t_off, n_sv_sharing=1, n_rl_sharing=1):
    # [수정] SV의 리턴 대역폭도 공유 적용
    tv_pos = step_info[tv_id]['pos']
    sv_pos = step_info[sv_id]['pos']
    d_out = task_info['task_bits'] * c.ALPHA
    t_rem = task_info['lat_constraint'] - t_off
    
    if t_rem <= 0: return False, c.PENALTY_DELAY

    # 1. SV에서 TV로 직접 전송(Direct) 가능한지 확인 (거리 기준)
    dist_direct = calculate_distance(sv_pos, tv_pos)
    if dist_direct <= c.ONE_HOP_LIMIT:
        # 채점: 거리 안에 있다면 LOS 여부를 확인 (실패 시 릴레이 탐색으로 넘어감)
        if check_los(sv_pos, tv_pos, c.BUILDING_WALLS):
            gain = calculate_channel_gain(dist_direct)
            interf = get_total_interference(sv_id, tv_id, step_info)
            rate = (c.BW / n_sv_sharing) * math.log2(1 + (c.P_TX_SV * gain) / (interf + c.NOISE_POWER)) if (interf + c.NOISE_POWER) > 0 else 0
            if rate > 0:
                t_return = d_out / rate
                if t_return <= t_rem:
                    return True, t_off + t_return

    # 2. 최단 거리 릴레이 후보 선정 (Pure Greedy Selection)
    # 가시선(LOS)이나 전송 속도를 고려하지 않고, 오직 SV와 가장 가까운 노드 하나만 고름
    best_rl_id = None
    min_dist = float('inf')
    is_rsu_selected = False

    # 차량들 중 가장 가까운 노드 탐색
    for v_id, info in step_info.items():
        if v_id in [tv_id, sv_id]: continue
        d = calculate_distance(sv_pos, info['pos'])
        if d < min_dist:
            min_dist = d
            best_rl_id = v_id
            is_rsu_selected = False

    # RSU 거리와 비교하여 더 가까우면 RSU 선택
    dist_s_rsu = calculate_distance(sv_pos, c.RSU_POS)
    if dist_s_rsu < min_dist:
        min_dist = dist_s_rsu
        is_rsu_selected = True
        best_rl_id = None

    # 3. 채점 (Validation): 선정된 가장 가까운 후보가 실제로 통신 가능한지 확인
    if is_rsu_selected:
        dist_rsu_t = calculate_distance(c.RSU_POS, tv_pos)
        # RSU 채점 (거리 제약 및 시간 확인)
        if min_dist <= c.ONE_HOP_LIMIT and dist_rsu_t <= c.ONE_HOP_LIMIT:
            rate_s_rsu = (c.BW_RSU / n_rl_sharing) * math.log2(1 + (c.P_TX_SV * calculate_v2i_gain(min_dist)) / c.NOISE_POWER)
            rate_rsu_t = (c.BW_RSU / n_rl_sharing) * math.log2(1 + (c.P_TX_RSU * calculate_v2i_gain(dist_rsu_t)) / c.NOISE_POWER)
            if rate_s_rsu > 0 and rate_rsu_t > 0:
                t_rsu = (d_out / rate_s_rsu) + c.BACKHAUL_DELAY + (d_out / rate_rsu_t)
                if t_rsu <= t_rem:
                    return True, t_off + t_rsu
    elif best_rl_id:
        rl_pos = step_info[best_rl_id]['pos']
        dist_r_t = calculate_distance(rl_pos, tv_pos)
        # 차량 릴레이 채점 (거리, LOS, 시간 확인)
        if min_dist <= c.ONE_HOP_LIMIT and dist_r_t <= c.ONE_HOP_LIMIT:
            if check_los(sv_pos, rl_pos, c.BUILDING_WALLS) and check_los(rl_pos, tv_pos, c.BUILDING_WALLS):
                rate_s_r = (c.BW / n_rl_sharing) * math.log2(1 + (c.P_TX_SV * calculate_channel_gain(min_dist)) / (get_total_interference(sv_id, best_rl_id, step_info) + c.NOISE_POWER))
                rate_r_t = (c.BW / n_rl_sharing) * math.log2(1 + (c.P_TX_SV * calculate_channel_gain(dist_r_t)) / (get_total_interference(best_rl_id, tv_id, step_info) + c.NOISE_POWER))
                if rate_s_r > 0 and rate_r_t > 0:
                    t_relay = (d_out / rate_s_r) + (d_out / rate_r_t)
                    if t_relay <= t_rem:
                        return True, t_off + t_relay

    # 선정된 '가장 가까운 노드'가 채점(LOS/지연시간)에서 탈락하면 다른 노드를 찾지 않고 즉시 실패
    return False, c.PENALTY_DELAY