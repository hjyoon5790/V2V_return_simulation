import traci
import random
import constant as c

''' TV와 SV 비율에 맞춰 차량 맵에 소환 '''
def spawn_vehicles(density):
    # 1. 소환할 차량 대수 계산
    num_tv = int(c.TOTAL_VEHICLES * density)
    num_sv = c.TOTAL_VEHICLES - num_tv
    
    # 2. 현재 맵에 존재하는 경로 이름 가져오기 -> 차가 다닐 경로가 반드시 있어야 함.
    route_list = traci.route.getIDList()
        
    # SUMO의 기본 차종을 복사해 TV라는 새로운 차종 만들고 빨간색으로 칠함
    traci.vehicletype.copy("DEFAULT_VEHTYPE", c.TYPE_TV)
    traci.vehicletype.setColor(c.TYPE_TV, (255, 0, 0, 255))
    
    # SV라는 차종을 만들고 파란색으로 칠함
    traci.vehicletype.copy("DEFAULT_VEHTYPE",c.TYPE_SV)
    traci.vehicletype.setColor(c.TYPE_SV, (0, 0, 255, 255))
    
    
    vehicle_types = [c.TYPE_TV] * num_tv + [c.TYPE_SV] * num_sv     # TV와 SV의 타입 이름을 차량 수만큼 리스트에 담기
    random.shuffle(vehicle_types)       # 리스트의 순서를 무작위로 섞음
    
    # ID에 붙일 번호를 따로 카운트
    tv_count = 0
    sv_count = 0
    
    for i, v_type in enumerate(vehicle_types):
        chosen_route = random.choice(route_list)       # 존재하는 여러 경로 중 하나를 무작위로 선택해 출발(300대가 한 경로에 몰리지 않도록)
        
        if v_type == c.TYPE_TV:
            v_id = f"tv_{tv_count}"
            tv_count += 1
        else:
            v_id = f"sv_{sv_count}"
            sv_count += 1
        traci.vehicle.add(v_id, chosen_route, typeID=v_type)    # 차량 ID, 경로 ID, typeID
    
    print(f"차량 세팅 완료! [TV: {num_tv}대 | SV: {num_sv}대] - 섞어서 소환")