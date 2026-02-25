import traci
import constant as c
import environment as env
import utils
import algorithm as alg
import utils_graph as ug

def run_single_simulation(density):
    env.start_sumo()        # 1. 시뮬레이션 환경 켜기
    utils.spawn_vehicles(density)   # 2. 해당 밀도로 차량 소환
    
    step = 0
    total_requests = 0      # 전체 요청 횟수
    success_proposed = 0       # 성공 횟수
    success_greedy = 0          # distance-based greedy 성공 횟수
    
    vehicle_type_cache = {}     # 차량 타입을 저장해둘 딕셔너리
    
    # 3. 메인 루프
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()      # 1초 진행
        step += 1
        
        if 200 <= step <= 800 and step % 5 == 0:
            all_vehicles = traci.vehicle.getIDList()
            # --- [디버깅] 100스텝마다 현재 맵에 차가 몇 대인지 출력 ---
            if step % 100 == 0:
                print(f" [Step {step}] 맵 위 차량: {len(all_vehicles)}대 (목표: {c.TOTAL_VEHICLES}대)")
            # --------------------------------------------------------
            
            step_info = {}
            for v_id in all_vehicles:
            # TV를 발견하면 알고리즘 실행
                if v_id not in vehicle_type_cache:          # 딕셔너리에 없는 차(새로 들어온 차)만 Traci에 물어봄
                    vehicle_type_cache[v_id] = traci.vehicle.getTypeID(v_id)
                step_info[v_id] = {
                    'type': vehicle_type_cache[v_id],
                    'pos': traci.vehicle.getPosition(v_id),
                    'speed': traci.vehicle.getSpeed(v_id),
                    'angle': traci.vehicle.getAngle(v_id)
                }
            for v_id in all_vehicles:
                if step_info[v_id]['type'] == c.TYPE_TV:   # 저장된 캐시에서 타입 확인
                    total_requests += 1
                    
                    # 내 알고리즘 실행 및 결과 기록
                    if alg.sv_selection(v_id, step_info):      # SV 선택 시도. SV값을 돌려주면 True, 안 돌려주면 False로 취급됨
                        success_proposed += 1      # 성공 시 횟수 증가
                    if alg.sv_selection_distance_greedy(v_id, step_info):
                        success_greedy += 1
                        
        if step > 800:
            break       # 실험은 800초까지만 진행
    # 4. 환경 끄기
    env.close_sumo()
    
    # --- [디버깅] 실제 몇 번 요청이 잇었고 성공했는지 출력 ---
    print(f"  >[결과] 총 TV 요청 횟수: {total_requests}번")
    print(f"  >[결과] Proposed 성공: {success_proposed}번 | Greedy 성공: {success_greedy}번")
    # 5. 성공률 계산(0으로 나누기 방지)
    rate_proposed = ((success_proposed / total_requests) * 100) if total_requests > 0 else 0
    rate_greedy = ((success_greedy / total_requests) * 100) if total_requests > 0 else 0
    return rate_proposed, rate_greedy
    
if __name__ == "__main__":
    # 결과를 저장할 리스트 두 개 준비
    proposed_rates = []
    greedy_rates = []
    
    # constant.py에 정의된 density 리스트를 하나씩 꺼내 반복
    for d in c.TV_DENSITY_LIST:
        print(f"--- 현재 TV 밀도 {d} 실험 중 ---")
        p_rate, g_rate = run_single_simulation(d)     # 시뮬레이션 실행 -> 두 개의 결괏값 받음
        
        proposed_rates.append(p_rate)
        greedy_rates.append(g_rate)
        
        print(f"Proposed: {p_rate:.2f}% | Greedy: {g_rate:.2f}%")
        
    # 모든 밀도 실험이 끝나면 최종 그래프 그리기
    ug.plot_success_rate_by_density(c.TV_DENSITY_LIST, proposed_rates, greedy_rates)