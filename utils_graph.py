import matplotlib.pyplot as plt

''' 밀도에 따른 성공률 그래프 함수 '''
def plot_success_rate_by_density(densities, proposed_rates, greedy_rates):
    # 1. 그림판 크기 설정
    plt.figure(figsize=(10, 6))
    
    # 2. 내 알고리즘 그래프 (파란색, 네모 점) -> 색상과 마커를 다르게 표시
    plt.plot(densities, proposed_rates, marker='s', color='blue',
             linestyle='-', linewidth=2, label='Proposed Scheme (SINR+Dir)')
    # 내 알고리즘 점 위에 숫자 표시
    for x, y in zip(densities, proposed_rates):
        plt.text(x, y + 2, f"{y:.2f}", ha='center', va='bottom', fontsize=9, color='blue', fontweight='bold')
    
    # 3. 비교 스킴1: distance-based greedy 그래프 (빨간색, 엑스 점) -> 색상과 마커를 다르게 표시
    plt.plot(densities, greedy_rates, marker='x', color='red',
             linestyle='--', linewidth=2, label='Distance-based Greedy')
    for x, y in zip(densities, greedy_rates):
        plt.text(x, y - 2, f"{y:.2f}", ha='center', va='top', fontsize=9, color='red', fontweight='bold')
    
    # 4. 그래프 제목과 축 이름
    plt.title("Success Rate Comparision by TV Density, ALPHA=0.7", fontsize=14)
    plt.xlabel("TV Density (Ratio of TVs in network)", fontsize=12)
    plt.ylabel("Success Rate (%)", fontsize=12)
    plt.grid(True, alpha=0.3)       # 5. 모눈종이 격자 켜기(켜기, 투명도)
    plt.ylim(0, 105)                # 6. Y축 범위 (0% ~ 100%)
    plt.legend()                    # 7. 범례 표시 (어떤 선이 어떤 알고리즘인지 설명창 띄우기)
    plt.show()                      # 8. 그래프 출력


""" 시간에 따른 SV 선택 성공/실패 횟수와 성공률을 시각화하는 함수 """
def plot_sv_selection_stats(steps, sv_successes, sv_failures):
    # SV 선택 성공률 계산
    sv_rates = []
    for s, f in zip(sv_successes, sv_failures):     # zip()은 여러 개의 리스트를 같은 순서끼리 묶어주는 함수(리스트 길이가 같을 때)
        sv_total = s + f
        if sv_total > 0:        # 시도가 0번이었으면 ZeroDivisionError가 발생하므로 이를 방지
            sv_rates.append((s / sv_total) * 100)
        else:
            sv_rates.append(0)      # 시도가 없었으면 0으로 기록
            
    # 그래프 그리기 -> 큰 도화지(fig) 안에 개별 도화지(ax) 여러 장 만듦. 이때 x축 공유
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize = (10, 8), sharex=True)
    
    # 1. 위쪽 그래프: 성공/실패 횟수
    # alpha는 투명도
    ax1.plot(steps, sv_successes, label='SV selection Success', color='green', marker='o', markersize=3, alpha=0.7)
    ax1.plot(steps, sv_failures, label='SV selection Fail', color='red', marker='x',markersize=3, alpha=0.7)
    ax1.set_ylabel("Number of Requests")
    ax1.set_title("SV Selection Performance (Real-time Monitor)")
    ax1.grid(True, linestyle='--', alpha=0.6)   # 모눈종이를 점선으로 약간 투명하게 그림
    
    # 2. 아래쪽 그래프: 성공률 (%)
    ax2.plot(steps, sv_rates, color='blue',linewidth=2)     # linewidth는 선의 굵기
    ax2.axhline(y=90, color='orange', linestyle='--', label='Target 90%')   # 목표선
    ax2.set_ylabel("Success Rate (%)")
    ax2.set_xlabel("Simulation Step (sec)")
    ax2.set_ylim(-5, 105)   # 0~100% 좀 더 여유있게
    ax2.grid(True, linestyle='--',alpha=0.6)
    
    plt.tight_layout()      # 두 그래프가 겹치거나 글씨가 잘리지 않도록 알아서 여백 조정하는 함수
    plt.show()