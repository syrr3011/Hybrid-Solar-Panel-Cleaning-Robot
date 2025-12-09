#include <AFMotor.h>

// --------------------------------------------------
// Motor channel mapping (하드웨어 연결)
// --------------------------------------------------
// motor1: 왼쪽 바퀴 2개 병렬
// motor2: 오른쪽 바퀴 2개 병렬
// motor3: 습식 브러시 모터
// motor4: 건식 브러시 모터
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// --------------------------------------------------
// 속도 설정 (0 ~ 255)  ★ 필요하면 여기 값만 바꾸면 됨
// --------------------------------------------------

// 직진 속도
const int SPEED_LEFT_FWD   = 130;   // motor1 직진 속도
const int SPEED_RIGHT_FWD  = 130;   // motor2 직진 속도

// 후진 속도
const int SPEED_LEFT_REV   = 135;   // motor1 후진 속도
const int SPEED_RIGHT_REV  = 135;   // motor2 후진 속도

// 회전 속도 (한쪽 바퀴만 전진)
// ROTATE_L: 오른쪽 바퀴만 전진 → 좌회전
// ROTATE_R: 왼쪽 바퀴만 전진 → 우회전
const int SPEED_ROTATE_LEFT  = 240; // 좌회전 시 motor2 속도
const int SPEED_ROTATE_RIGHT = 240; // 우회전 시 motor1 속도
const int SPEED_30 = 30;


// 브러시 속도
const int SPEED_WET_BRUSH  = 200;   // motor3 (습식 브러시)
const int SPEED_DRY_BRUSH  = 200;   // motor4 (건식 브러시)


// --------------------------------------------------
String command;
// --------------------------------------------------

void setup() {
  Serial.begin(9600);

  // 초기 속도 설정 (실제 동작에서는 각 함수에서 다시 세팅함)
  motor1.setSpeed(SPEED_LEFT_FWD);
  motor2.setSpeed(SPEED_RIGHT_FWD);
  motor3.setSpeed(SPEED_WET_BRUSH);
  motor4.setSpeed(SPEED_DRY_BRUSH);

  stopAll();
}

void loop() {
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();

    // -----------------------------
    // REV_WET:<duration>
    // -----------------------------
    if (command.startsWith("REV_WET")) {
      int idx = command.indexOf(":");
      if (idx > 0 && idx < command.length() - 1) {
        String durStr = command.substring(idx + 1);
        durStr.trim();
        float dur = durStr.toFloat();   // 초 단위 duration
        revWet(dur);
      }
    }

    // -----------------------------
    // FWD_DRY
    // -----------------------------
    else if (command == "FWD_DRY") {
      fwdDry();
    }

    // -----------------------------
    // ROTATE_L (좌회전)
    // -----------------------------
    else if (command == "ROTATE_L") {
      rotate_L();
    }

    // -----------------------------
    // ROTATE_R (우회전)
    // -----------------------------
    else if (command == "ROTATE_R") {
      rotate_R();
    }

    else if (command == "BACK_SHORT") {
      backShort();
    }

    // -----------------------------
    // STOP
    // -----------------------------
    else if (command == "STOP") {
      stopAll();
    }
  }
}

// --------------------------------------------------
// 직진 + 건식 청소 모드 (기본 상태)
//  - motor1, motor2: FORWARD
//  - motor4: ON (건식 브러시)
//  - motor3: OFF (습식 브러시)
// --------------------------------------------------
void fwdDry() {
  // 바퀴 속도 (전진용)
  motor1.setSpeed(SPEED_LEFT_FWD);
  motor2.setSpeed(SPEED_RIGHT_FWD);

  // 브러시 속도
  motor4.setSpeed(SPEED_DRY_BRUSH);

  // 바퀴 전진
  motor1.run(FORWARD);
  motor2.run(FORWARD);

  // 브러시 동작
  motor4.run(BACKWARD);   // 건식 브러시 ON
  motor3.run(RELEASE);   // 습식 브러시 OFF
}

// --------------------------------------------------
// 후진 + 습식 청소 모드
//  - motor1, motor2: BACKWARD
//  - motor3: ON (습식 브러시)
//  - motor4: OFF (건식 브러시)
//  - duration 초 후 stopAll()
//  - 물펌프 패턴(0.2초 ON / 0.8초 OFF)은 라즈베리에서 릴레이로 제어
// --------------------------------------------------
void revWet(float dur) {
  unsigned long total_ms = (unsigned long)(dur * 1000);

//#######################
  // 방향 전환 사이 잠깐 정지 시간 (ms)
  const unsigned long SWITCH_PAUSE_MS = 50;  // 0.05초


  // 건식 브러시 OFF
  motor4.run(RELEASE);

  // --------------------------------------------------
  // 1) 첫 번째 구간: 후진 dur + 습식 브러시(FORWARD)
  // --------------------------------------------------
  motor3.setSpeed(SPEED_WET_BRUSH);
  motor3.run(FORWARD);          // 후진 → 브러시 FORWARD

  motor1.setSpeed(SPEED_LEFT_REV);
  motor2.setSpeed(SPEED_RIGHT_REV);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);

  delay(total_ms);

  // 방향 전환 전 잠시 정지
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);          // ★ 브러시도 잠깐 쉬어야 전환 충격 줄어듦
  delay(SWITCH_PAUSE_MS);

  // --------------------------------------------------
  // 2) 두 번째 구간: 전진 dur + 습식 브러시(BACKWARD)
  // --------------------------------------------------
  motor3.run(BACKWARD);         // 전진 → 브러시 BACKWARD

  motor1.setSpeed(SPEED_LEFT_FWD);
  motor2.setSpeed(SPEED_RIGHT_FWD);
  motor1.run(FORWARD);
  motor2.run(FORWARD);

  delay(total_ms);

  // 다시 잠깐 정지
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  delay(SWITCH_PAUSE_MS);

  // --------------------------------------------------
  // 3) 세 번째 구간: 후진 dur + 습식 브러시(FORWARD)
  // --------------------------------------------------
  motor3.run(FORWARD);          // 후진 → 브러시 FORWARD

  motor1.setSpeed(SPEED_LEFT_REV);
  motor2.setSpeed(SPEED_RIGHT_REV);

  motor1.run(BACKWARD);
  motor2.run(BACKWARD);

  delay(total_ms);

  // --------------------------------------------------
  // 4) 종료
  // --------------------------------------------------
  stopAll();  // 모든 모터 OFF
}
//#######################


// --------------------------------------------------
// 후진 1초 (회전 전 안전 확보)
// --------------------------------------------------
void backShort() {
  motor1.setSpeed(SPEED_LEFT_REV);
  motor2.setSpeed(SPEED_RIGHT_REV);

  motor1.run(BACKWARD);
  motor2.run(BACKWARD);

  delay(1500);   // 1.5초 후진

  stopAll();
}

// --------------------------------------------------
// 좌회전 (ROTATE_L)
//  - IMU + 라즈베리에서 회전량(180°) 제어
//  - 여기서는 "오른쪽 바퀴만 전진"만 수행
//  - wet/dry 브러시 둘 다 OFF
// --------------------------------------------------
void rotate_L() {
  // 오른쪽 바퀴만 전진
  motor2.setSpeed(SPEED_ROTATE_LEFT);
  motor1.setSpeed(SPEED_30);

  motor1.run(FORWARD);      // 왼쪽 바퀴 정지
  motor2.run(FORWARD);      // 오른쪽 바퀴 전진

  motor3.run(RELEASE);      // 습식 브러시 OFF
  motor4.run(RELEASE);      // 건식 브러시 OFF

  // 회전 종료는 라즈베리에서 IMU로 판단 후 "STOP" 명령 보내서 종료
}

// --------------------------------------------------
// 우회전 (ROTATE_R)
//  - 왼쪽 바퀴만 전진
//  - 회전 종료는 라즈베리 STOP 명령으로
// --------------------------------------------------
void rotate_R() {
  motor1.setSpeed(SPEED_ROTATE_RIGHT);
  motor2.setSpeed(SPEED_30);

  motor1.run(FORWARD);      // 왼쪽 바퀴 전진
  motor2.run(FORWARD);      // 오른쪽 바퀴 전진

  motor3.run(RELEASE);      // 습식 브러시 OFF
  motor4.run(RELEASE);      // 건식 브러시 OFF
}


// --------------------------------------------------
// STOP: 모든 모터 정지
// --------------------------------------------------
void stopAll() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
