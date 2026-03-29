import traci
import os

''' SUMO 시작하는 함수. config_path에 아무것도 안 넣으면 기본으로 이 경로 실행 '''
def start_sumo(config_path="map_data/manhattan.sumocfg"):
    os.system("taskkill /f /im sumo.exe /t 2> nul")
    print(f"SUMO 시뮬레이션 환경 준비(맵:{config_path})")
    # 생성된 route 파일을 로드하도록 -r 옵션 추가
    sumo_cmd = [
        "sumo-gui", "-c", config_path, 
        "-r", "map_data/generated.rou.xml",
        "--start", "--quit-on-end", 
        "--no-warnings", "--no-step-log"
    ]
    traci.start(sumo_cmd)

""" SUMO 엔진을 안전하게 종료하는 함수 """
def close_sumo():
    print("시뮬레이션을 종료하고 환경 닫음")
    traci.close()