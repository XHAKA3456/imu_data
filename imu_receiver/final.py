import json
import socket
from ahrs.common.orientation import q_prod, q_conj, acc2q, am2q, q2R, q_rot
import numpy as np
from scipy import signal
import ahrs

class IMUReceiver:
    def __init__(self, ip, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.samplePeriod = 1 / 256  # 샘플 주기 설정
        self.mahony = ahrs.filters.Mahony(Kp=1, Ki=0, KpInit=1, frequency=1/self.samplePeriod)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.vel = np.zeros(3)
        self.pos = np.zeros(3)
        self.stationary = np.zeros(100, dtype=bool)  # For example, a history of 100 samples
        self.filtCutOffLP = 5
        self.filtCutOffHP = 0.1

    def get_real_time_data(self):
        """UDP로 실시간 IMU 데이터를 받아오기."""
        data, addr = self.sock.recvfrom(1024)  # UDP로 데이터 수신
        imu_data = json.loads(data.decode('utf-8'))  # JSON 파싱

        # IMU 데이터 추출 (기본값 0.0 설정)
        quatX = imu_data.get('quatX', 0.0)
        quatY = imu_data.get('quatY', 0.0)
        quatZ = imu_data.get('quatZ', 0.0)
        quatW = imu_data.get('quatW', 0.0)
        accX = imu_data.get('accelX', 0.0)
        accY = imu_data.get('accelY', 0.0)
        accZ = imu_data.get('accelZ', 0.0) + 0.98
        gyrX = imu_data.get('gyroX', 0.0)
        gyrY = imu_data.get('gyroY', 0.0)
        gyrZ = imu_data.get('gyroZ', 0.0)

        return quatX, quatY, quatZ, quatW, accX, accY, accZ, gyrX, gyrY, gyrZ

    def update(self):
        """IMU 데이터를 업데이트하고 처리."""
        quatX, quatY, quatZ, quatW, accX, accY, accZ, gyrX, gyrY, gyrZ = self.get_real_time_data()

        # # 가속도 벡터와 각속도를 사용하여 쿼터니언 업데이트
        # gyr = np.array([gyrX, gyrY, gyrZ]) * np.pi / 180  # 각속도는 라디안 단위로 변환
        # acc = np.array([accX, accY, accZ])

        # Mahony 필터로 쿼터니언 업데이트
        self.quat = np.array([quatW, quatX, quatY, quatZ])

        b_lp, a_lp = signal.butter(1, (2*self.filtCutOffLP)/(1/self.samplePeriod), 'lowpass')
        b_hp, a_hp = signal.butter(1, (2*self.filtCutOffHP)/(1/self.samplePeriod), 'highpass')

        acc_mag = np.sqrt(accX**2 + accY**2 + accZ**2)  # 가속도 크기 계산

        acc_mag = np.array(acc_mag)  # 최소 7개의 값으로 배열을 만듦

        # # Low-pass 필터 적용
        # acc_magFilt = signal.filtfilt(b_lp, a_lp,acc_mag)

        # acc_magFilt = np.abs(acc_mag)

        # # High-pass 필터 적용 (HP 필터)
        # acc_magFilt = signal.filtfilt(b_hp, a_hp, acc_magFilt)

        stationary = acc_mag < 0.1 # 정지 상태 감지, 기준 가속도

        # self.stationary 배열을 갱신
        # self.stationary[-1] = stationary[-1]        

        # 가속도 회전 적용
        acc_rot = q_rot(q_conj(self.quat), np.array([accX, accY, accZ]))
        # acc_rot = acc_rot - np.array([0, 0, 1])  # 중력 벡터 제외
        acc_rot *= 9.81  # 중력 가속도 적용

        # 이동 속도와 위치 업데이트
        self.vel += acc_rot * self.samplePeriod
        if stationary == True:
            self.vel = np.zeros(3)
        # self.pos += self.vel * self.samplePeriod

        # 속도 드리프트 제거
        velDrift = np.zeros(self.vel.shape)

        # Update stationary history
        self.stationary = np.roll(self.stationary, -1)  # Shift array left
        self.stationary[-1] = stationary  # Add latest stationary state

        stationaryStart = np.where(np.diff(self.stationary.astype(int)) == -1)[0] + 1
        stationaryEnd = np.where(np.diff(self.stationary.astype(int)) == 1)[0] + 1

        if stationaryStart.size > 0 and stationaryEnd.size > 0:
            for i in range(stationaryEnd.shape[0]):
                if stationaryEnd[i] < len(self.vel) and stationaryStart[i] < len(self.vel):
                    driftRate = self.vel[stationaryEnd[i] - 1, :] / (stationaryEnd[i] - stationaryStart[i])
                    enum = np.arange(0, stationaryEnd[i] - stationaryStart[i])
                    drift = np.array([enum * driftRate[0], enum * driftRate[1], enum * driftRate[2]]).T
                    velDrift[stationaryStart[i]:stationaryEnd[i], :] = drift

        self.vel -= velDrift  # 드리프트 보정
        self.pos += self.vel * self.samplePeriod  # 위치 업데이트

        print(f"Position: {self.pos}, Quaternion: {self.quat}")

# 실행 예시
imu_receiver = IMUReceiver(ip="0.0.0.0", port=5005)

# 데이터 수신 및 처리
while True:
    imu_receiver.update()
