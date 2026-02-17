import math
import numpy as np
import traci
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
def get_total_interference(tv_id, sv_pos):
    total_interf = 0        # 간섭을 더할 변수 초기화
    all_vehicles = traci.vehicle.getIDList()
    
    # 나(TV)를 제외한 다른 TV를 찾음
    for other_v in all_vehicles:
        if other_v != tv_id and traci.vehicle.getTypeID(other_v) == c.TYPE_TV:
            # 다른 TV와 타겟 SV 사이의 거리 구함
            other_pos = traci.vehicle.getPosition(other_v)
            dist_to_sv = calulate_distance(other_pos, sv_pos)
            
            # 다른 TV가 SV에게 미치는 간섭을 계산해 누적
            gain = calculate_channel_gain(dist_to_sv)
            total_interf += (c.P_TX_TV * gain)
    
    return total_interf

""" TV를 위해 최적의 SV 선택하는 함수 """
def sv_selection(tv_id):
    tv_pos = traci.vehicle.getPosition(tv_id)
    sv_candidates = []
    all_vehicles = traci.vehicle.getIDList()
    
    # 1. 범위 내 SV 필터링 (Type & Distance)
    for v_id in all_vehicles:
        if v_id == tv_id:
            continue
        if traci.vehicle.getTypeID(v_id) == c.TYPE_SV:
            sv_pos = traci.vehicle.getPosition(v_id)
            dist = calulate_distance(tv_pos, sv_pos)
            
            # 리스트 맨 뒤에 새로운 데이터(딕셔너리) 추가
            if dist <= c.ONE_HOP_LIMIT:
                sv_candidates.append({
                    'id': v_id,
                    'pos': sv_pos,
                    'dist': dist
                })
    if not sv_candidates:
        return None         # 1-hop 이내에 차량 없음
    
    # 2. SV 후보의 상세 점수 계산(방향 유사도 & 통신 속도)
    for sv in sv_candidates:
        # 코사인 유사도 계산
        cos_sim = calculate_cosine_similarity(tv_id, sv['id'])
        sv['dir_score'] = (cos_sim + 1) / 2
        
        # 통신 속도 계산    ## 질문: 문법 + 전부 같은 시기(t_0)에서 계산되는게 맞는지
        gain = calculate_channel_gain(sv['dist']) 
        interf = get_total_interference(tv_id, sv['pos'])   # 실제 간섭값 가져오기
        
        # SINR 구함
        sinr = (c.P_TX_TV * gain) / (interf + c.NOISE_POWER)
        # 샤논의 capacity formula
        sv['rate'] = c.BW * math.log2(1 + sinr)
        
    # 3. 정규화 및 최적 SV 선택
    max_rate = max(sv['rate'] for sv in sv_candidates)      # 후보들의 rate 중 최댓값 찾기
    
    for sv in sv_candidates:
        # 가장 빠른 차를 1점으로 두고 나머지를 비율로 계산
        sv['eff_score'] = sv['rate'] / max_rate if max_rate > 0 else 0  ## 질문: 문법
        
        # 최종 점수 계산
        sv['final_score'] = (c.WEIGHT_SV_RATE * sv['eff_score']) + (c.WEIGHT_SV_DIR * sv['dir_score'])
        
    best_sv = max(sv_candidates, key=lambda x: x['final_score'])
    
    return best_sv['id']       # 최종 점수가 높은 차량의 id만 알려줌