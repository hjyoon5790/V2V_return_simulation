import math
import numpy as np
import traci
import random
import constant as c    # 상수 파일 임포트

""" 두 좌표(pos1과 pos2) 사이의 거리를 계산하는 함수 """
def calulate_distance(pos1, pos2):
    # pos1[0]은 x좌표, pos1[1]은 y좌표
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

""" 거리에 따른 전파 감소(Path Loss)를 계산하는 함수 """
def calculate_channel_gain(distance):
    if distance == 0:
        return 0
    # 물리 공식 구현: (파장 / (4 * pi * 거리)) ^ 경로손실지수
    return (c.WAVELENGTH / (4 * math.pi * distance)) ** c.PATH_LOSS_EXP

""" 두 차량의 방향 유사도 구하는 함수(코사인 유사도 이용) """
def calculate_cosine_similarity(v1_id, v2_id):
    # 차량의 속력(speed)과 방향(angle) 가져옴
    speed1 = traci.vehicle.getSpeed(v1_id)
    angle1 = traci.vehicle.getAngle(v1_id)
    speed2 = traci.vehicle.getSpeed(v2_id)
    angle2 = traci.vehicle.getAngle(v2_id)
    
    # 두 차량 중 하나가 정차(속도 = 0) 중이면 유사도 0 (분모 0 방지)
    if speed1 == 0 or speed2 == 0:
        return 0.0
    
    # SUMO의 각도(Angle)를 Radian으로 단위 변환(파이썬이 알아듣도록)
    rad1 = math.radians(angle1)
    rad2 = math.radians(angle2)
    
    # 속력과 각도를 이용해 x축, y축 방향의 벡터(화살표) 길이 구함
    v1_x = speed1 * math.sin(rad1)
    v1_y = speed1 * math.cos(rad1)
    v2_x = speed2 * math.sin(rad2)
    v2_y = speed2 * math.cos(rad2)
    
    # 코사인 유사도 공식
    dot_product = (v1_x * v2_x) + (v1_y * v2_y)     # 내적
    magnitude_multi = speed1 * speed2               # 크기의 곱
    
    return dot_product / magnitude_multi            # -1(정반대)에서 1(같은 방향)사이의 값

""" 다른 TV들이 쏘는 전파가 내가 고르려는 SV에게 얼마나 잡음인지 계산(잠재 지연) """
def get_total_interference(sv_pos, other_tv_positions):
    total_interf = 0
    
    # traci 통신 없이 밖에서 받아온 리스트만 반복
    for other_pos in other_tv_positions:
        dist = calulate_distance(other_pos, sv_pos)
        gain = calculate_channel_gain(dist)
        total_interf += (c.P_TX_TV * gain)
        
    return total_interf

""" TV를 위해 최적의 SV 선택하는 함수 """
def sv_selection(tv_id):
    tv_pos = traci.vehicle.getPosition(tv_id)
    sv_candidates = []
    all_vehicles = traci.vehicle.getIDList()
    
    # 1. 이번 TV의 랜덤 task 크기 생성 (2~4 Mbits)
    task_mbits = random.uniform(c.TASK_MIN_MBITS, c.TASK_MAX_MBITS)
    task_bits = task_mbits * (10**6)    # 비트 단위로 변환
    
    # 2. 연산 시간(Comp Delay) 미리 계산
    t_comp = (task_bits * c.COMP_INTENSITY) / c.SV_RESOURCE
    
    sv_list = []
    other_tv_positions = []
    
    # 3. 범위 내 SV 탐색 및 필터링 (Type & Distance & Delay)
    for v_id in all_vehicles:
        if v_id == tv_id:
            continue
        v_type = traci.vehicle.getTypeID(v_id)
        if v_type == c.TYPE_TV:
            other_tv_positions.append(traci.vehicle.getPosition(v_id))
        elif v_type == c.TYPE_SV:
            sv_list.append((v_id, traci.vehicle.getPosition(v_id)))
    
    for v_id, sv_pos in sv_list:
        dist = calulate_distance(tv_pos, sv_pos)
        # (1) 통신 범위 확인
        if dist <= c.ONE_HOP_LIMIT:
            #SV 후보의 상세 정보 계산
            cos_sim = calculate_cosine_similarity(tv_id, v_id)
            dir_score = (cos_sim + 1) / 2
            # 통신 속도(Rate) 계산
            gain = calculate_channel_gain(dist)
            interf = get_total_interference(sv_pos, other_tv_positions)
            sinr = (c.P_TX_TV * gain) / (interf + c.NOISE_POWER)
            rate = c.BW * math.log2(1 + sinr)   # Shannon Capacity
                
            # (2) 지연 시간 조건 확인
            if rate <=0:
                continue        # 통신 불가능하면 패스
                
            t_tx = task_bits / rate     # 전송 시간 = 데이터 크기/속도
            t_off = t_tx + t_comp
                
            # 최대 지연 허용 지연(1초)을 넘으면 후보 리스트에 넣지 않음(탈락!)
            if t_off > c.MAX_LATENCY:
                continue
                
            # 조건 통과한 SV만 후보 리스트에 넣기
            sv_candidates.append({
                'id': v_id,
                'rate': rate,
                'dir_score': dir_score,
                't_off': t_off      # 나중에 확인용으로 저장
            })
                
    # 4. 후보 없음: 1-hop 이내에 차량 없음 + 딜레이 제약
    if not sv_candidates:
        ### 여기에 나중에 RSU에게 넘기는 코드 추가
        return None
    
    # 5. 정규화 및 최종 점수 계산(살아남은 후보끼리 경쟁)
    # rate가 가장 높은 SV 찾기(분모 0 방지)
    max_rate = max(sv['rate'] for sv in sv_candidates)
    
    best_sv_id = None
    max_final_score = -1        ## 절대 나올 수 없는 점수인 -1로 초기화(최대 점수 갱신용)
    
    for sv in sv_candidates:
        # 효율 점수(eff_score) 계산: 내 속도 / 1등 속도
        if max_rate > 0:
            sv['eff_score'] = sv['rate'] / max_rate
        else:
            sv['eff_score'] = 0
            
        # 최종 점수 = (속도 가중치 * 효율점수) + (방향 가중치 * 방향점수)
        sv['final_score'] = (c.WEIGHT_SV_RATE * sv['eff_score']) + (c.WEIGHT_SV_DIR * sv['dir_score'])
        
        # 최대 점수 갱신
        if sv['final_score'] > max_final_score:
            max_final_score = sv['final_score']
            best_sv_id = sv['id']
            
    return best_sv_id