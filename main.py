import traci    ##라이브러리 가져옴
import os
import sys

# 1. 혹시 남아있을지 모르는 유령 SUMO들을 강제로 다 끄고 시작
# ('새롭게 생성'하기 위한 가장 확실한 밑작업)
os.system("taskkill /f /im sumo.exe /t 2> nul")
os.system("taskkill /f /im sumo-gui.exe /t 2> nul")

print("♻️ 기존 프로세스 정리 완료! 새 시뮬레이션을 생성합니다.")

# 2. 실행 설정
sumo_binary = "sumo-gui"
sumo_config = "map_data/osm.sumocfg"

# SUMO 실행 코드
sumo_cmd=[sumo_binary, "-c", sumo_config, "--start"] ##SUMO를 켜기 위한 명령어 세트
# label을 붙여주면 '이 연결은 내 꺼야!'라고 명시하는 효과가 있어 더 안정적

try:
    # 3. 포트 번호를 지정하지 않고 시작 (label만 부여)
    # 이렇게 하면 파이썬이 알아서 비어있는 포트를 찾아 새 창을 띄움.
    traci.start(sumo_cmd, label="sim1")
    print("새로운 SUMO-GUI가 성공적으로 실행 됨")   ##파이썬으로 SUMO-GUI 실행 및 연결 -> SUMO 창 열림

    #100초(step) 동안 시뮬레이션 돌리면서 데이터 뽑기
    for step in range(100):     ##0~99까지 반복
        traci.simulationStep()  ## 시뮬레이션 시간을 1초(1스텝)만 앞으로 감기
    
        #현재 지도 위에 있는 모든 차량의 ID 가져오기
        vehicle_ids=traci.vehicle.getIDList()
    
        #지도에 차가 1대라도 있으면 터미널에 몇 대인지 출력하기
        if len(vehicle_ids)>0:
            # print(f"[{step}초] 현재 지도 위 차량 수: {len(vehicle_ids)}대")
            target_veh = vehicle_ids[0]     ##명단 중에 첫 번째 차량을 타겟으로 잡음
            x, y = traci.vehicle.getPosition(target_veh)    ##타겟 차량의 현재 위치
            speed_ms = traci.vehicle.getSpeed(target_veh)   ##타겟 차량의 현재 속도(m/s)
        
            speed_kmh = speed_ms * 3.6  ## 단위 변경 m/s -> km/h
            ## 소수점 둘째 자리까지 출력(차량 | 위치 | 속도)
            print(f"[{step}초] 타겟 차량: {target_veh} | 위치: (X: {x:.2f}, Y:{y:.2f}) | 속도: {speed_kmh} km/h")
    
    # 바로 닫히지 않게 사용자의 입력 기다림
    print("시뮬레이션 계산 완료!")
    input("Press Enter to close the SUMO window...")
    
    #시뮬레이션 창 닫기
    traci.close()
    print("simulation finished!")
except Exception as e:
    print(f"새 시뮬레이션 생성 중 에러 발생")