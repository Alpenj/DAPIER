import time
from motor_control import MiniFeetechDriver

def set_motor_id(port='/dev/ttyUSB0'):
    driver = MiniFeetechDriver(port=port)
    
    # Feetech 모터 ID 설정 레지스터 주소 (EEPROM)
    REG_ID = 5
    # EEPROM 쓰기 방지 해제 레지스터 (일부 모델 필요)
    REG_LOCK = 55 

    print("=== Feetech 모터 ID 설정 도구 ===")
    print("주의: 반드시 설정할 '하나의 모터'만 커넥터에 연결하세요.")
    
    # 1. 현재 모터의 ID를 모를 경우를 대비해 브로드캐스트 ID(0xFE) 사용 가능
    # 하지만 안전을 위해 현재 ID를 입력받거나 1번(기본값)을 시도합니다.
    current_id = int(input("현재 모터의 ID를 입력하세요 (모르면 1 또는 254): "))
    new_id = int(input("새로 부여할 ID를 입력하세요 (1~6): "))

    if not (1 <= new_id <= 253):
        print("잘못된 ID 범위입니다.")
        return

    # 2. 쓰기 잠금 해제 (필요한 경우)
    driver._send_packet(current_id, 0x03, [REG_LOCK, 0])
    time.sleep(0.1)

    # 3. ID 변경 명령 (WRITE_DATA)
    # 파라미터: [레지스터 주소, 설정할 값]
    print(f"ID {current_id}를 {new_id}로 변경 시도 중...")
    driver._send_packet(current_id, 0x03, [REG_ID, new_id])
    
    # EEPROM에 저장될 때까지 잠시 대기
    time.sleep(0.5)

    # 4. 확인 과정
    print(f"확인 중... 새 ID({new_id})로 응답을 시도합니다.")
    result = driver.get_position(new_id)
    
    if result is not None:
        print(f"성공! 모터 ID가 {new_id}로 변경되었습니다. (현재 위치: {result})")
    else:
        print("실패: 모터가 새 ID에 응답하지 않습니다. 연결을 확인하세요.")

if __name__ == "__main__":
    set_motor_id()
