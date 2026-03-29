import random
import constant as c
import xml.etree.ElementTree as ET

def generate_route_file(density, net_file="map_data/manhattan.net.xml", output_file="map_data/generated.rou.xml"):
    """ 밀도에 따라 차량의 출발지, 목적지, 타입을 정한 .rou.xml 파일을 생성 """
    
    try:
        # 1. 네트워크 파일에서 도로(edge) 목록 추출
        tree = ET.parse(net_file)
        root = tree.getroot()
        all_edges = [edge.get('id') for edge in root.findall('edge') if ":" not in edge.get('id')]

        # 2. 차량 대수 계산
        num_tv = int(c.TOTAL_VEHICLES * density)
        num_sv = c.TOTAL_VEHICLES - num_tv
        vehicle_types = [c.TYPE_TV] * num_tv + [c.TYPE_SV] * num_sv
        random.shuffle(vehicle_types)

        # 3. XML 파일 쓰기
        with open(output_file, "w") as f:
            f.write('<routes>\n')
            # vType 정의: 여기서 사용자님이 설정하신 가속도, 최대속도 등을 고정합니다.
            f.write(f'    <vType id="{c.TYPE_TV}" accel="2.6" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" color="1,0,0" speedFactor="normc(0.8,0.1)"/>\n')
            f.write(f'    <vType id="{c.TYPE_SV}" accel="2.6" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" color="0,0,1" speedFactor="normc(0.8,0.1)"/>\n')

            tv_idx, sv_idx = 0, 0
            for i, v_type in enumerate(vehicle_types):
                v_id = f"tv_{tv_idx}" if v_type == c.TYPE_TV else f"sv_{sv_idx}"
                if v_type == c.TYPE_TV: tv_idx += 1
                else: sv_idx += 1
                
                start = random.choice(all_edges)
                end = random.choice(all_edges)
                f.write(f'    <trip id="{v_id}" type="{v_type}" depart="{i * 0.5:.1f}" from="{start}" to="{end}" departLane="best" departPos="base"/>\n')
            
            f.write('</routes>\n')
        print(f"루트 파일 생성 완료: {output_file} [TV: {num_tv} | SV: {num_sv}]")
    except Exception as e:
        print(f"파일 생성 중 오류 발생: {e}")