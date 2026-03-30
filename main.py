import traci
import constant as c
import environment as env
import utils
import algorithm as alg
import utils_graph as ug
import random
import numpy as np
import matplotlib.pyplot as plt

def run_single_simulation(density):
    utils.generate_route_file(density) # 1. 해당 밀도로 .rou.xml 파일 생성
    env.start_sumo()                   # 2. 생성된 파일을 로드하며 시뮬레이션 시작
    
    step = 0
    total_requests = 0      # 전체 요청 횟수
    success_proposed = 0       # 성공 횟수
    success_greedy = 0          # distance-based greedy 성공 횟수
    
    vehicle_type_cache = {}     # 차량 타입을 저장해둘 딕셔너리
    
    # 3. 메인 루프
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()      # 1초 진행
        step += 1
        
        # 원래 나오던 유령 차들은 나오자마자 삭제
        for v in traci.simulation.getDepartedIDList():
            if not (v.startswith("tv_") or v.startswith("sv_")):
                traci.vehicle.remove(v)

        # Step 30부터 500까지 5스텝마다 측정
        if 30 <= step <= 500 and step % 5 == 0:
            vehicle_ids = traci.vehicle.getIDList()
            veh_count = len(vehicle_ids)             # 맵 위 차량 대수 계산
            real_veh_count = traci.vehicle.getIDCount()     # Traci가 인식하는 차량 수 (유령차 제거 후)
            # --- [디버깅] 30스텝마다 현재 맵에 차가 몇 대인지 출력 ---
            if step % 30 == 0:
                print(f" [Step {step}] 맵 위 차량: {veh_count}대 (목표: {c.TOTAL_VEHICLES}대)")
                print(f"  > 맵 위 실제 차량 수: {real_veh_count}대")
            # --------------------------------------------------------
            
            step_info = {}
            for v_id in vehicle_ids:
                if v_id not in vehicle_type_cache:          # 딕셔너리에 없는 차(새로 들어온 차)만 Traci에 물어봄
                    vehicle_type_cache[v_id] = traci.vehicle.getTypeID(v_id)
                step_info[v_id] = {
                    'type': vehicle_type_cache[v_id],
                    'pos': traci.vehicle.getPosition(v_id),
                    'speed': traci.vehicle.getSpeed(v_id),
                    'angle': traci.vehicle.getAngle(v_id)
                }
            for v_id in vehicle_ids:
                if step_info[v_id]['type'] == c.TYPE_TV:   # 저장된 캐시에서 타입 확인
                    total_requests += 1
                    
                    task_bits, t_comp, lat_constraint = alg.generate_random_task()
                    task_info_original = {
                        'task_bits': task_bits,
                        't_comp': t_comp,
                        'lat_constraint': lat_constraint
                    }
                    task_for_proposed = task_info_original.copy()
                    task_for_greedy = task_info_original.copy()

                    # 내 알고리즘 실행 및 결과 기록
                    best_sv_p, t_off_p = alg.sv_selection(v_id, step_info, task_for_proposed)
                    if best_sv_p:
                        # SV가 선택되었다면 리턴 경로(Direct, Relay, RSU)까지 평가
                        is_success, _ = alg.evaluate_return_path(v_id, best_sv_p, step_info, task_for_proposed, t_off_p)
                        if is_success:
                            success_proposed += 1

                    best_sv_g, t_off_g = alg.sv_selection_distance_greedy(v_id, step_info, task_for_greedy)
                    if best_sv_g:
                        is_success_g, _ = alg.evaluate_return_path(v_id, best_sv_g, step_info, task_for_greedy, t_off_g)
                        if is_success_g:
                            success_greedy += 1
                        
        if step > 500:
            break
    # 4. 환경 끄기
    env.close_sumo()
    
    # --- [디버깅] 실제 몇 번 요청이 잇었고 성공했는지 출력 ---
    print(f"  >[결과] 총 TV 요청 횟수: {total_requests}번")
    # print(f"  >[결과] Proposed 성공: {success_proposed}번 | Greedy 성공: {success_greedy}번")
    # 5. 성공률 계산(0으로 나누기 방지)
    rate_proposed = ((success_proposed / total_requests) * 100) if total_requests > 0 else 0
    rate_greedy = ((success_greedy / total_requests) * 100) if total_requests > 0 else 0
    return rate_proposed, rate_greedy
    
if __name__ == "__main__":
    # 결과를 저장할 리스트 두 개 준비
    proposed_rates = []
    greedy_rates = []
    NUM_TRIALS = 5  # 각 밀도별 반복 횟수 설정
    
    # constant.py에 정의된 density 리스트를 하나씩 꺼내 반복
    for d in c.TV_DENSITY_LIST:
        print(f"--- 현재 TV 밀도 {d} 실험 중 ---")

        trial_proposed = []
        trial_greedy = []
        
        # 각 밀도당 5번 반복 실행
        for i in range(NUM_TRIALS):
            print(f"  [시행 {i+1}/{NUM_TRIALS}]")
            rate_proposed, rate_greedy = run_single_simulation(d)
            trial_proposed.append(rate_proposed)
            trial_greedy.append(rate_greedy)
        
        avg_p = sum(trial_proposed) / NUM_TRIALS
        avg_g = sum(trial_greedy) / NUM_TRIALS
        
        # 평균값만 결과 리스트에 추가
        proposed_rates.append(avg_p)
        greedy_rates.append(avg_g)
        
        print(f"==> 밀도 {d} 최종 결과 - Proposed 평균: {avg_p:.2f}% | Greedy 평균: {avg_g:.2f}%")
        
    # 모든 밀도 실험이 끝나면 최종 그래프 그리기
    ug.plot_success_rate_by_density(c.TV_DENSITY_LIST, proposed_rates, greedy_rates)