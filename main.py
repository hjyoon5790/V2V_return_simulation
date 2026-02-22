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
    
    # 3. 메인 루프
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()      # 1초 진행
        step += 1
        
        if 200 <= step <= 500 and step % 5 == 0:
            all_vehicles = traci.vehicle.getIDList()
            for v_id in all_vehicles:
            # TV를 발견하면 알고리즘 실행
                if traci.vehicle.getTypeID(v_id) == c.TYPE_TV:      # TV 찾으면
                    total_requests += 1     # 요청 시도 횟수 증가
                    
                    # 내 알고리즘 실행 및 결과 기록
                    if alg.sv_selection(v_id):      # SV 선택 시도. SV값을 돌려주면 True, 안 돌려주면 False로 취급됨
                        success_proposed += 1      # 성공 시 횟수 증가
                    if alg.sv_selection_distance_greedy(v_id):
                        success_greedy += 1
                        
        if step > 500:
            break       # 실험은 500초까지만 진행
    # 4. 환경 끄기
    env.close_sumo()
    
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