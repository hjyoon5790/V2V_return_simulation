import traci
import os

''' SUMO 시작하는 함수. config_path에 아무것도 안 넣으면 기본으로 이 경로 실행 '''
def start_sumo(config_path="map_data/osm.sumocfg"):
    os.system("taskkill /f /im sumo.exe /t 2> nul")
    print(f"SUMO 시뮬레이션 환경 준비(맵:{config_path})")   ## 질문: print(f"~~")랑 print("~~")랑 뭐가 다른지
    sumo_cmd = ["sumo", "-c", config_path, "--no-warnings", "--no-step-log"]
    traci.start(sumo_cmd)

""" SUMO 엔진을 안전하게 종료하는 함수 """
def close_sumo():
    print("시뮬레이션을 종료하고 환경 닫음")
    traci.close()