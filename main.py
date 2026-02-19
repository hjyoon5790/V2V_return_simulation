import traci
import constant as c
import environment as env
import utils
import algorithm as alg
import utils_graph as ug

def run_simulation():
    # 1. 시뮬레이션 환경 켜기
    env.start_sumo()
    
    # 2. 차량 소환
    utils.spawn_vehicles()
    
    step = 0
    history_step = []
    history_sv_success = []
    history_sv_fail = []
    
    # 3. 메인 루프
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        
        # 이번 스텝(현재 1초) 동안의 성공/실패율 셀카운터 준비
        current_step_success = 0
        current_step_fail = 0
        
        # 도로 위 모든 차량 검색
        all_vehicles = traci.vehicle.getIDList()
        for v_id in all_vehicles:
            # TV를 발견하면 알고리즘 실행
            if traci.vehicle.getTypeID(v_id) == c.TYPE_TV:
                selected_sv = alg.sv_selection(v_id)
                
                if selected_sv:
                    current_step_success += 1
                    # print(f"[Stpe {step}] TV({v_id})가 최적의 SV({selected_sv})를 찾았습니다!")
                else:
                    current_step_fail += 1
                    # print(f"🚨[Step {step}] TV({v_id})는 주변에 조건이 맞는 SV가 없습니다!")
        
        # 모든 차량 검사가 끝나면 이번 스텝의 결과를 리스트에 저장
        history_step.append(step)
        history_sv_success.append(current_step_success)
        history_sv_fail.append(current_step_fail)
        
        # 테스트용이므로 1000초까지만 돌리고 강제 종료
        if step > 1000:
            print("1000초 도달. 시뮬레이션 중단.")
            break
        
    # 4. 환경 끄기
    env.close_sumo()
    
    # 5. 시뮬레이션 끝난 후 모아둔 데이터 그래프 그리기
    print("📊 시뮬레이션 종료. 결과를 바탕으로 그래프를 생성...")
    ug.plot_sv_selection_stats(history_step, history_sv_success, history_sv_fail)
    
if __name__ == "__main__":
    run_simulation()