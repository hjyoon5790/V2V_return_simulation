import traci
import constant as c

''' TV와 SV 비율에 맞춰 차량 맵에 소환 '''
def spawn_vehicles():
    # 1. 소환할 차량 대수 계산
    num_tv = int(c.TOTAL_VEHICLES * c.CURRENT_TV_DENSITY)
    num_sv = c.TOTAL_VEHICLES - num_tv
    
    # 2. 현재 맵에 존재하는 경로 이름 가져오기 -> 차가 다닐 경로가 반드시 있어야 함.
    route_list = traci.route.getIDList()
    if len(route_list) == 0:
        print("🚨에러: 맵에 설정된 경로(ROUTE)가 전혀 없습니다!")
        return
    # 첫 번째 경로 선택
    valid_route = route_list[0]
    
    # SUMO의 기본 차종을 복사해 TV라는 새로운 차종 만들고 빨간색으로 칠함
    traci.vehicletype.copy("DEFAULT_VEHTYPE", c.TYPE_TV)
    traci.vehicletype.setColor(c.TYPE_TV, (255, 0, 0, 255))
    
    # SV라는 차종을 만들고 파란색으로 칠함
    traci.vehicletype.copy("DEFAULT_VEHTYPE",c.TYPE_SV)
    traci.vehicletype.setColor(c.TYPE_SV, (0, 0, 255, 255))
    
    
    # 3. TV 소환
    for i in range(num_tv):
        traci.vehicle.add(f"tv_{i}", valid_route, typeID=c.TYPE_TV)     ## 질문: 문법
    
    # 4. SV 소환
    for i in range(num_sv):
        traci.vehicle.add(f"sv_{i}", valid_route, typeID=c.TYPE_SV)
        
    print(f"차량 세팅 완료! [TV: {num_tv}대 | SV: {num_sv}대]")