import time
import serial
import RPi.GPIO as GPIO
from ultralytics import YOLO
import cv2
import numpy as np
import os
from picamera2 import Picamera2
import datetime

# ==========================================================
# ★ IMU 제거 → 회전 시간으로 판단
# ==========================================================
ROTATE_TIME_SEC = 8 # ← 실험으로 조정
# 오른→왼→오른→왼 반복
last_turn_right = False       # 이전 턴 방향 기록

# ==========================================================
# YOLO 모델(ncnn) 경로
# ==========================================================
YOLO_MODEL_PATH = "/home/seung/1114/yolo/best.pt"  # ncnn 모델 디렉토리/파일

# ==========================================================
# 저장 폴더
# ==========================================================
SAVE_DIR = "/home/seung/pictures_1129"
os.makedirs(SAVE_DIR, exist_ok=True)

# ==========================================================
# ★ 수정 필요: 아두이노 Serial 포트
# ==========================================================
SERIAL_PORT = '/dev/ttyACM0'   # ← 아두이노가 잡힌 /dev/ttyUSBo
BAUD = 9600

# ==========================================================
# GPIO 핀
# ==========================================================
PUMP_PIN = 18         # 릴레이
TRIG = 4              # 초음파
ECHO = 17
ULTRA_EN = 5
CAM_EN = 6

# ==========================================================
# 카메라 설정
# ==========================================================
CAM_W, CAM_H = 960, 720       # 라즈베리 카메라 해상도 #느려서 낮춤

# ==========================================================
# 동작 파라미터
# ==========================================================
########## ★ 수정: 초음파 주기 0.5초로 줄임 ##########
ULTRA_INTERVAL = 0.5           # 초음파 체크 주기(초)
CAM_INTERVAL   = 0.5           # 카메라 + YOLO 주기(초)
CLIFF_THRESHOLD = 40.0         # cliff 판정 거리(cm)
# pitch_threshold = 10.0         # pitch 기준 각도(±10°)
STOP_STABILIZE_SEC = 1       # STOP 후 안정화 시간(초)


# ==========================================================
# GPIO 초기화
# ==========================================================
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # 초음파 I/O
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
    GPIO.output(TRIG, False)

    # 각 센서 Enable 핀
    GPIO.setup(ULTRA_EN, GPIO.OUT)
    GPIO.setup(CAM_EN, GPIO.OUT)
    # GPIO.setup(IMU_EN, GPIO.OUT)

    # 물펌프 릴레이
    GPIO.setup(PUMP_PIN, GPIO.OUT, initial=GPIO.LOW)

    # 초기 상태: 초음파 ON, 카메라 ON, IMU OFF
    GPIO.output(ULTRA_EN, True)
    GPIO.output(CAM_EN, True)


# ==========================================================
# 아두이노 Serial 초기화
# ==========================================================
def setup_serial():
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
    time.sleep(2)               # ★ Serial 안정화 대기 (추가)
    ser.flush()                 # ★ 버퍼 초기화
    return ser

# ==========================================================
# 초음파 거리 측정 (cm) #timeout 제외 및 코드 정리
# ==========================================================
def read_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start = time.time()
    while GPIO.input(ECHO) == 0:
        if time.time() - start > 0.02:
            return None
    pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        if time.time() - pulse_start > 0.02:
            return None
    pulse_end = time.time()

    return round((pulse_end - pulse_start) * 17150, 2)


# ==========================================================
# contam 시퀀스에서 "마지막에 연속된 1의 개수" 계산
# ==========================================================
def tail_ones(seq):
    count = 0
    for v in reversed(seq):
        if v == 1:
            count += 1
        else:
            break
    return count


# ==========================================================
# 물펌프 패턴 (duration초 동안 0.2s ON / 0.8s OFF 반복)
# ==========================================================
def pump_pattern(duration_sec):
    t_end = time.time() + duration_sec
    while time.time() < t_end:
        GPIO.output(PUMP_PIN, GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(PUMP_PIN, GPIO.LOW)
        time.sleep(0.8)


# ==========================================================
# STOP + 안정화
# ==========================================================
def send_stop(ser):
    ser.write(b"STOP\n")
    time.sleep(STOP_STABILIZE_SEC)


# ----------------------------------------------------------
# 아두이노로 모드 명령 보내기
# ----------------------------------------------------------

def safe_backoff(ser, dur=1.0):
    ser.write(b"BACK_SHORT\n")      # 아두이노에서 1초 후진
    time.sleep(dur + 0.2)           # 통신 / 관성 보정

def enter_fwd_dry(ser):
    ser.write(b"STOP\n")
    time.sleep(0.5)
    ser.write(b"FWD_DRY\n")
    time.sleep(1.5)               # ★ 추가: 모터 전환 안정화


def enter_rev_wet(ser, duration_sec):
    ser.write(b"STOP\n")
    time.sleep(0.5)
    cmd = f"REV_WET:{duration_sec}\n"
    ser.write(cmd.encode())
    # 물펌프 패턴은 라즈베리에서 릴레이로 제어
    pump_pattern(duration_sec*3)##############################
    send_stop(ser)

    time.sleep(1.5)               # ★ 추가: 후진→정지→전진 전환 안정화


def rotate_left(ser):
    ser.write(b"STOP\n")
    time.sleep(0.5)
    ########## ★ 수정: 후진 1초 추가 ##########
    safe_backoff(ser, 1.5)
    # ← 추가된 후진 1초 구간
    time.sleep(0.5)
    ser.write(b"ROTATE_L\n")
    time.sleep(1.5)                # ★ 추가: 회전 후 안정화


def rotate_right(ser):
    ser.write(b"STOP\n")
    time.sleep(0.5)
    safe_backoff(ser, 1.5)          # ← 추가된 후진 1초 구간
    ser.write(b"ROTATE_R\n")
    time.sleep(1.5)                # ★ 추가: 회전 후 안정화


# ==========================================================
# MAIN LOOP
# ==========================================================
def main():

    global last_turn_right
    setup_gpio()
    ser = setup_serial()
    time.sleep(2)          # 아두이노가 리셋 후 완전히 준비되도록

    # YOLO 모델 로드
    model = YOLO(YOLO_MODEL_PATH)

    last_cam_time = 0.0
    ########## ★ 수정: 초음파용 타이머 + 2연속 cliff 상태 변수 추가 ##########
    last_ultra_time = 0.0          # 마지막 초음파 측정 시각
    last_cliff_raw = False         # 직전 초음파가 cliff였는지 여부(1회 감지)

    contam_seq = []   # 카메라 결과(0/1) 시퀀스
    frame_id = 1      # 저장 이미지 번호

    # 시작은 직진 + 건식 청소 모드
    enter_fwd_dry(ser)

    # ======================================================
    # ★ Picamera2 초기화
    # ======================================================
    picam = Picamera2()
    config = picam.create_video_configuration(
        {"size": (CAM_W, CAM_H)}
    )
    picam.configure(config)
    picam.start()
    time.sleep(0.2)                    # ★ 센서 안정화
    # ======================================================

    while True:
        now = time.time()

        # -------------------------------------------------
        # 1) 초음파 체크 (ULTRA_INTERVAL 주기) + 2연속 cliff 확인
        # -------------------------------------------------
        ########## ★ 수정 블록
        ##########           
        cliff_confirmed = False
        dist = None

        if now - last_ultra_time >= ULTRA_INTERVAL:
            last_ultra_time = now
            
            dist = read_distance()
            if dist is not None:
                print(f"[ULTRA] {dist} cm")

            if dist is not None and dist >= CLIFF_THRESHOLD:
                cliff_confirmed = True

        ################################################################

        # ===== cliff 확정된 경우에만 기존 cliff 로직 실행 =====
        if cliff_confirmed:


            # 1) 오염 시퀀스가 있으면 → 청소 먼저
            if 1 in contam_seq:
                # contam_seq의 끝에서부터 연속된 1 개수
                count_ones = tail_ones(contam_seq)
                if count_ones > 1:
                    print(f"[CLIFF] cliff + contamination → REV_WET for {count_ones+2} sec")

                    # 청소 모드 동안 센서값은 무시 (Enable 끔)
                    GPIO.output(ULTRA_EN, False)
                    GPIO.output(CAM_EN, False)
                    # GPIO.output(IMU_EN, False)

                    # 후진 + 습식 청소
                    enter_rev_wet(ser, count_ones+2)

                    # 시퀀스 초기화
                    contam_seq = []

                    # 다시 기본 상태: 초음파/카메라 ON, IMU OFF, 직진 + 건식
                    GPIO.output(ULTRA_EN, True)
                    GPIO.output(CAM_EN, True)
                    # GPIO.output(IMU_EN, False)
                    enter_fwd_dry(ser)

                    # 이번 루프에서는 회전은 하지 않고 종료.
                    # 다음 루프에서 초음파가 다시 cliff를 감지하면 그때 회전 모드 진입.
                    time.sleep(ULTRA_INTERVAL)
                    continue

            # 2) 오염이 없으면 바로 회전 모드 진입
            print("[CLIFF] no contamination → enter rotation mode")

            # 회전 모드: 초음파/카메라 OFF, IMU ON
            GPIO.output(ULTRA_EN, False)
            GPIO.output(CAM_EN, False)
            # GPIO.output(IMU_EN, True)

            # --- 회전 방향 결정: 오른→왼→오른→왼 ---
            if last_turn_right:
                print("[ROTATE] LEFT (toggle)")
                rotate_left(ser)
                last_turn_right = False
            else:
                print("[ROTATE] RIGHT (toggle)")
                rotate_right(ser)
                last_turn_right = True

            # --- 시간 기반 회전 ---
            print(f"[ROTATE] rotating for {ROTATE_TIME_SEC} sec")
            time.sleep(ROTATE_TIME_SEC)
            send_stop(ser)

            # 회전 후 다시 직진 모드
            GPIO.output(ULTRA_EN, True)
            GPIO.output(CAM_EN, True)
            enter_fwd_dry(ser)

            time.sleep(ULTRA_INTERVAL)
            continue

        # -------------------------------------------------
        # 2) 카메라 + YOLO (1초 주기)
        #    - 직진 모드에서만 오염 감지
        # -------------------------------------------------
        if now - last_cam_time >= CAM_INTERVAL:
            last_cam_time = now

            try:
                frame = picam.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            except Exception as e:
                print(f"[CAM] capture failed: {e}")
                time.sleep(ULTRA_INTERVAL)
                continue

            # YOLO 실행
            results = model(frame, verbose=False)
            detections = results[0].boxes
            n_detect = len(detections)

            # contam: 0(clean) / 1(contam)
            contam = 1 if n_detect > 0 else 0
            print(f"[YOLO] contam={contam}  detect_count={n_detect}")

            # bounding box 표시 (시각화용)
            if n_detect > 0:
                for box in detections:
                    xyxy = box.xyxy.cpu().numpy().astype(int)[0]
                    cv2.rectangle(
                        frame,
                        (xyxy[0], xyxy[1]),
                        (xyxy[2], xyxy[3]),
                        (0, 255, 0),
                        3
                    )

            # 이미지 저장
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = f"{SAVE_DIR}/frame_{frame_id:04d}_label{contam}_{timestamp}.jpg"
            cv2.imwrite(save_path, frame)
            frame_id += 1

            # 오염 시퀀스 업데이트
            contam_seq.append(contam)

            # 1→0 패턴(오염 구간 끝)일 때도 후진+습식 수행
            if len(contam_seq) >= 2:
                # 마지막이 0이고, 바로 전이 1이면 "오염 구간이 끝난 순간"
                if contam_seq[-1] == 0 and contam_seq[-2] == 1:
                    count_ones = tail_ones(contam_seq[:-1])  # 마지막 0은 제외
                    if count_ones < 2:
                        contam_seq = []
                        continue
                    if count_ones > 0:
                        print(f"[REVERSE] natural end of contamination → {count_ones+2} sec")
                        # 청소 모드 동안은 초음파/카메라/IMU 모두 무시
                        GPIO.output(ULTRA_EN, False)
                        GPIO.output(CAM_EN, False)
                        # GPIO.output(IMU_EN, False)

                        enter_rev_wet(ser, count_ones+2)
                        contam_seq = []

                        # 다시 기본 상태 복귀
                        GPIO.output(ULTRA_EN, True)
                        GPIO.output(CAM_EN, True)
                        # GPIO.output(IMU_EN, False)
                        enter_fwd_dry(ser)

        # -------------------------------------------------
        # 루프 주기 유지
        # -------------------------------------------------
        time.sleep(ULTRA_INTERVAL)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Program terminated.")
