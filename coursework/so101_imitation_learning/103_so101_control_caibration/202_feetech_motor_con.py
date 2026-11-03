import serial
import time
import struct

class MiniFeetechDriver:
    # Feetech 모터 주요 레지스터 주소 (STS3215 기준)
    REG_GOAL_POSITION = 42
    REG_PRESENT_POSITION = 56
    REG_TORQUE_ENABLE = 40

    def __init__(self, port='/dev/ttyUSB0', baudrate=1000000):
        # 1. 시리얼 포트 설정 (LeRobot도 1M baudrate 사용)
        self.ser = serial.Serial(port, baudrate, timeout=0.05)
        
    def _send_packet(self, motor_id, instruction, parameters):
        """Feetech 프로토콜에 맞춘 패킷 생성 및 전송"""
        length = len(parameters) + 2
        packet = [0xFF, 0xFF, motor_id, length, instruction] + parameters
        
        # Checksum 계산 (LeRobot의 _checksum 함수와 동일한 논리)
        checksum = ~(sum(packet[2:]) & 0xFF) & 0xFF
        packet.append(checksum)
        
        self.ser.write(bytearray(packet))
        return self.ser.read(100) # 응답 패킷 읽기 (생략 가능하나 확인용으로 권장)

    def set_torque(self, motor_id, enable):
        """모터의 토크를 켜거나 끕니다 (1: On, 0: Off)"""
        # [주소, 값] 순서로 파라미터 전달
        self._send_packet(motor_id, 0x03, [self.REG_TORQUE_ENABLE, 1 if enable else 0])

    def set_position(self, motor_id, position):
        """목표 위치로 이동 (position: 0 ~ 4095)"""
        # 16비트 데이터를 2개의 8비트로 분리 (Little Endian)
        pos_low = position & 0xFF
        pos_high = (position >> 8) & 0xFF
        self._send_packet(motor_id, 0x03, [self.REG_GOAL_POSITION, pos_low, pos_high])

    def get_position(self, motor_id):
        """현재 위치 값을 읽어옴"""
        # 0x02: READ instruction (주소 56번부터 2바이트 읽기)
        response = self._send_packet(motor_id, 0x02, [self.REG_PRESENT_POSITION, 2])
        
        if len(response) >= 8: # 패킷 구조: FF FF ID LEN ERR VAL_L VAL_H CHK
            # 응답 패킷에서 데이터 추출
            pos_low = response[5]
            pos_high = response[6]
            return (pos_high << 8) | pos_low
        return None

# --- 실행 예시 ---
if __name__ == "__main__":
    driver = MiniFeetechDriver(port='/dev/ttyUSB0') # 본인의 포트에 맞게 수정
    
    MOTOR_ID = 1
    driver.set_torque(MOTOR_ID, True)
    
    print(f"현재 위치: {driver.get_position(MOTOR_ID)}")
    
    # 2048(중간 지점)로 이동
    driver.set_position(MOTOR_ID, 1000)
    driver.set_torque(MOTOR_ID, False)