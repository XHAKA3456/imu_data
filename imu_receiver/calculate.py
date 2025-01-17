import socket
import json
import numpy as np
import ahrs
from scipy import signal
from ahrs.common.orientation import q_rot, q_conj

# UDP socket 설정
UDP_IP = "0.0.0.0"  # 수신 IP (로컬호스트)
UDP_PORT = 5005  # 수신 포트 번호
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# 샘플 주기 설정
samplePeriod = 1 / 256

# Mahony 필터 초기화
mahony = ahrs.filters.Mahony(Kp=1, Ki=0, KpInit=1, frequency=1/samplePeriod)
q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

# 초기 설정
stationary_threshold = 0.05  # 정지 상태 판별 임계값
stationary = False  # 초기에는 움직임 상태로 설정

# 속도 및 위치 계산을 위한 변수
vel = np.zeros(3)
pos = np.zeros(3)

# 필터 설정
def hp_filter(data, cutoff, sample_period):
    b, a = signal.butter(1, (2 * cutoff) / (1 / sample_period), 'highpass')
    return signal.filtfilt(b, a, data, padtype='odd', padlen=3 * (max(len(b), len(a)) - 1))

def lp_filter(data, cutoff, sample_period):
    b, a = signal.butter(1, (2 * cutoff) / (1 / sample_period), 'lowpass')
    return signal.filtfilt(b, a, data, padtype='odd', padlen=3 * (max(len(b), len(a)) - 1))

min_data_len = 5  # 최소 데이터 길이 (필요한 만큼 조정)
accX_buffer = []  # 가속도 X 데이터 버퍼
accY_buffer = []  # 가속도 Y 데이터 버퍼
accZ_buffer = []  # 가속도 Z 데이터 버퍼

# 실시간 데이터 처리 루프
while True:
    # UDP로부터 데이터 수신
    data, addr = sock.recvfrom(1024)
    imu_data = json.loads(data.decode('utf-8'))

    # IMU 데이터 추출
    quatX = imu_data.get('quatX', 0.0)
    quatY = imu_data.get('quatY', 0.0)
    quatZ = imu_data.get('quatZ', 0.0)
    quatW = imu_data.get('quatW', 0.0)
    accX_buffer.append(imu_data.get('accelX', 0.0))
    accY_buffer.append(imu_data.get('accelY', 0.0))
    accZ_buffer.append(imu_data.get('accelZ', 0.0))
    gyrX = imu_data.get('gyroX', 0.0)
    gyrY = imu_data.get('gyroY', 0.0)
    gyrZ = imu_data.get('gyroZ', 0.0)
    time = imu_data.get('accelTime',0.0)/1000
    
    if len(accX_buffer) >= min_data_len:

        accX = np.array(accX_buffer)
        accY = np.array(accY_buffer)
        accZ = np.array(accZ_buffer)

        # 가속도계의 크기 계산
        acc_mag = np.sqrt(accX**2 + accY**2 + accZ**2)

        # 고주파 필터링 (고속 신호 제거)
        acc_magFilt = hp_filter(acc_mag, 0.001, samplePeriod)

        # 절대값 취하기
        acc_magFilt = np.abs(acc_magFilt)

        # 저주파 필터링 (잡음 제거)
        acc_magFilt = lp_filter(acc_magFilt, 5, samplePeriod)

        # 정지 여부 감지
        stationary = acc_magFilt < stationary_threshold

        # 쿼터니언 업데이트
        quat = np.array([quatW, quatX, quatY, quatZ])
        
        # 회전 행렬 업데이트
        gyr = np.array([gyrX, gyrY, gyrZ]) * np.pi / 180  # 각속도 (rad/s)
        acc = np.array([accX, accY, accZ])  # 가속도
        q = mahony.updateIMU(q, gyr=gyr, acc=acc)

        # initial convergence
        initPeriod = 2
        indexSel = time<=time+initPeriod
        gyr=np.zeros(3, dtype=np.float64)
        acc = np.array([np.mean(accX[indexSel]), np.mean(accY[indexSel]), np.mean(accZ[indexSel])])
        mahony = ahrs.filters.Mahony(Kp=1, Ki=0,KpInit=1, frequency=1/samplePeriod)
        q = np.array([1.0,0.0,0.0,0.0], dtype=np.float64)
        for i in range(0, 2000):
            q = mahony.updateIMU(q, gyr=gyr, acc=acc)

        # For all data
        for t in range(0,time.size):
            if(stationary[t]):
                mahony.Kp = 0.5
            else:
                mahony.Kp = 0
            gyr = np.array([gyrX[t],gyrY[t],gyrZ[t]])*np.pi/180
            acc = np.array([accX[t],accY[t],accZ[t]])
            quat[t,:]=mahony.updateIMU(q,gyr=gyr,acc=acc)

        # -------------------------------------------------------------------------
        # Compute translational accelerations

        # Rotate body accelerations to Earth frame
        acc = []
        for x,y,z,q in zip(accX,accY,accZ,quat):
            acc.append(q_rot(q_conj(q), np.array([x, y, z])))
        acc = np.array(acc)
        acc = acc - np.array([0,0,1])
        acc = acc * 9.81

        # Compute translational velocities
        # acc[:,2] = acc[:,2] - 9.81

        # acc_offset = np.zeros(3)
        vel = np.zeros(acc.shape)
        for t in range(1,vel.shape[0]):
            vel[t,:] = vel[t-1,:] + acc[t,:]*samplePeriod
            if stationary[t] == True:
                vel[t,:] = np.zeros(3)

        # Compute integral drift during non-stationary periods
        velDrift = np.zeros(vel.shape)
        stationaryStart = np.where(np.diff(stationary.astype(int)) == -1)[0]+1
        stationaryEnd = np.where(np.diff(stationary.astype(int)) == 1)[0]+1
        for i in range(0,stationaryEnd.shape[0]):
            driftRate = vel[stationaryEnd[i]-1,:] / (stationaryEnd[i] - stationaryStart[i])
            enum = np.arange(0,stationaryEnd[i]-stationaryStart[i])
            drift = np.array([enum*driftRate[0], enum*driftRate[1], enum*driftRate[2]]).T
            velDrift[stationaryStart[i]:stationaryEnd[i],:] = drift

        # Remove integral drift
        vel = vel - velDrift

        # -------------------------------------------------------------------------
        # Compute translational position
        pos = np.zeros(vel.shape)
        for t in range(1,pos.shape[0]):
            pos[t,:] = pos[t-1,:] + vel[t,:]*samplePeriod

        # 출력 예시 (디버깅용)
        print(f"Position: {pos}, Velocity: {vel}")
