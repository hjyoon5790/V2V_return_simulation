import matplotlib.pyplot as plt

"""SV 선택 성공/실패 횟수와 성공률을 시각화하는 함수"""
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