import traci
import constant as c
import environment as env
import utils
import algorithm as alg
import utils_graph as ug
import matplotlib as plt

def run_single_simulation(tv_ratio):
    # 1. 시뮬레이션 환경 켜기
    env.start_sumo()    
    # 2. 차량 소환
    utils.spawn_vehicles(total_count=c.TOTAL_VEHICLES, ratio=tv_ratio)
    
    step = 0
    total_sv_success = 0
    total_sv_requests = 0
    
    # 3. 메인 루프
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        if step < 200: continue
        
        # 도로 위 모든 차량 검색
        all_v = traci.vehicle.getIDList()
        sv_info_list = []
        other_tv_positions = []
        tv_tasks = []   # (id, info_dict) 리스트
        
        for v_id in all_v:
            v_type = traci.vehicle.getTypeID(v_id)
            v_data = {
                'pos': traci.vehicle.getPosition(v_id),
                'speed': traci.vehicle.getSpeed(v_id),
                'angle': traci.vehicle.getAngle(v_id)
            }
            
            if v_type == c.TYPE_TV:
                other_tv_positions.append(v_data['pos'])
                tv_tasks.append((v_id, v_data))
            elif v_type == c.TYPE_SV:
                sv_info_list.append((v_id, v_data))
        
        for tv_id, tv_info in tv_tasks:
            # TV를 발견하면 알고리즘 실행
            selected_sv = alg.sv_selection(tv_id, tv_info, sv_info_list, other_tv_positions)
            total_sv_requests += 1
            if selected_sv: total_sv_success += 1    
        if step > 600:
            print("600초 도달. 시뮬레이션 중단.")
            break
        
    # 4. 환경 끄기
    env.close_sumo()
    return (total_sv_success / total_sv_requests * 100) if total_sv_requests > 0 else 0
    
if __name__ == "__main__":
    final_results = []
    iterations = 5      # 각 비율당 5번씩 테스트해서 평균 내기
    
    for ratio in c.TV_DENSITY_LIST:
        print(f"--- Processing TV Ratio: {int(ratio*100)} ---")
        temp_rates = []
        for i in range(iterations):
            rate = run_single_simulation(ratio)
            temp_rates.append(rate)
        avg_rate = sum(temp_rates)  / iterations
        final_results.append(avg_rate)
        print(f"평균 성공률: {avg_rate:.2f}%")
    ug.plot_tv_ratio_performance(c.TV_DENSITY_LIST, final_results)