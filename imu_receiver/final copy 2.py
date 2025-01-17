import socket
import json
import numpy as np
import ahrs
from ahrs.common.orientation import q_prod, q_conj, acc2q, am2q, q2R, q_rot
from scipy import signal

# UDP 소켓 설정
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
BUFFER_SIZE = 1024

# UDP 소켓 생성
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# Sample period (시간 간격)
samplePeriod = 0.01  # 예시 값

# IMU 데이터 초기화
acc_magFilt = np.zeros(100)  # 예시 초기화
quat = np.zeros((100, 4))  # 쿼터니언 초기화

# Mahony 필터 초기화
mahony = ahrs.filters.Mahony(Kp=1, Ki=0, KpInit=1, frequency=1/samplePeriod)
q = np.array([1.0, 0.0, 0.0, 0.0])

# LP 필터 및 HP 필터 설계
filtCutOffLP = 5  # Low-pass filter cutoff (Hz)
filtCutOffHP = 0.1  # High-pass filter cutoff (Hz)
b_lp, a_lp = signal.butter(1, (2*filtCutOffLP)/(1/samplePeriod), 'lowpass')
b_hp, a_hp = signal.butter(1, (2*filtCutOffHP)/(1/samplePeriod), 'highpass')

# 실시간 데이터 처리
while True:
    data, addr = sock.recvfrom(BUFFER_SIZE)  # UDP로 데이터 수신
    
    try:
        # JSON 형식으로 파싱
        imu_data = json.loads(data.decode('utf-8'))
        
        # 필요한 IMU 데이터 추출
        acc_x = imu_data.get('accelX', 0.0)
        acc_y = imu_data.get('accelY', 0.0)
        acc_z = imu_data.get('accelZ', 0.0)
        gyr_x = imu_data.get('gyroX', 0.0)
        gyr_y = imu_data.get('gyroY', 0.0)
        gyr_z = imu_data.get('gyroZ', 0.0)
        
        # 쿼터니언 데이터 추출
        quat_x = imu_data.get('quatX', 0.0)
        quat_y = imu_data.get('quatY', 0.0)
        quat_z = imu_data.get('quatZ', 0.0)
        quat_w = imu_data.get('quatW', 1.0)  # 기본값으로 [1, 0, 0, 0] 설정
        
        time = imu_data.get('accelTime', 0.0)

        # 가속도 데이터 계산 (필터링)
        acc_mag = np.sqrt(acc_x**2 + acc_y**2 + acc_z**2)  # 가속도 크기 계산
        
        # Low-pass 필터 적용
        acc_magFilt = signal.filtfilt(b_lp, a_lp, acc_magFilt, padtype='odd', padlen=3*(max(len(b_lp), len(a_lp))-1))

        acc_magFilt = np.abs(acc_magFilt)

        # High-pass 필터 적용 (HP 필터)
        acc_magFilt = signal.filtfilt(b_hp, a_hp, acc_magFilt, padtype='odd', padlen=3*(max(len(b_hp), len(a_hp))-1))

        stationary = acc_magFilt < 0.05 # 정지 상태 감지, 기준 가속도
        
        # Mahony 필터로 쿼터니언 업데이트
        gyr = np.array([gyr_x, gyr_y, gyr_z]) * np.pi / 180  # 각속도(rad/s)
        acc = np.array([acc_x, acc_y, acc_z])
        
        
        # 쿼터니언 데이터로부터 회전된 가속도 계산
        q_received = np.array([quat_w, quat_x, quat_y, quat_z])  # 수신된 쿼터니언
        acc_rot = q_rot(q_conj(q_received), np.array([acc_x, acc_y, acc_z]))  # 회전된 가속도 계산
        
        # 포지션 계산 (이전 계산 방식 이어서)
        pos = np.zeros(3)
        vel = np.zeros(3)
        vel = vel + acc_rot * samplePeriod

        if stationary == True:
            vel = np.zeros(3)

        velDrift = np.zeros(3)
        # stationaryStart = np.where(np.)    
        pos = pos + vel * samplePeriod
        
        # 결과 출력
        print(f"Position: {pos}")
        print(f"Orientation (Quat): {q_received}")
        
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
