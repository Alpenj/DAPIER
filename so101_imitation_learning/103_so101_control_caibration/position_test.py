import time
from motor_control import MiniFeetechDriver

def monitor_motor_position(port="/dev/ttyUSB0", motor_id=1):
    """
    지정한 모터의 현재 위치를 실시간으로 출력합니다.
    """
    try:
        driver = MiniFeetechDriver(port=port)
        
        print(f"--- 모터 {motor_id} 모니터링 시작 ---")
        print("토크를 OFF합니다. 로봇을 손으로 움직여보세요.")
        
        # 손으로 움직일 수 있게 토크 해제
        driver.set_torque(motor_id, False)
        time.sleep(0.1)

        print("현재 위치 (Ctrl+C를 누르면 종료됩니다):")
        
        while True:
            pos = driver.get_position(motor_id)
            
            if pos is not None:
                # \r을 사용하여 한 줄에서 숫자가 업데이트되도록 표기
                print(f"\r[Motor {motor_id}] Current Position: {pos:<5}", end="")
            else:
                print(f"\r[Motor {motor_id}] 데이터를 읽을 수 없음...", end="")
            
            time.sleep(0.05)  # 20Hz 주기로 업데이트

    except KeyboardInterrupt:
        print("\n\n모니터링을 종료합니다.")
    except Exception as e:
        print(f"\n에러 발생: {e}")

if __name__ == "__main__":
    # 포트와 모터 ID가 맞는지 확인하세요 (현재 ID 6으로 설정)
    monitor_motor_position(port="/dev/ttyUSB0", motor_id=1)