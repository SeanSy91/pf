# ESP32-S3 Particle Filter Firmware

이 프로젝트는 `MecanumSensorSimulator`의 serial JSON line을 ESP32-S3에서 받아 파티클 필터로 `x`, `y`, `theta`를 추정합니다.

## Project Layout

```text
platformio.ini
include/
  estimator_config.h
  mecanum.h
  particle_filter.h
  sensor_packet.h
  serial_parser.h
src/
  main.cpp
  mecanum.c
  particle_filter.c
  serial_parser.c
```

Arduino 프레임워크 특성상 진입점은 `src/main.cpp`이지만, 필터와 파서는 C 스타일 모듈로 분리되어 있습니다.

## Serial Input

ESP32-S3는 `Serial`로 simulator의 JSON Lines를 받습니다.

기본값:

```text
baudrate = 115200
dt = 0.1 s
```

Simulator 실행 예:

```powershell
.\dist\MecanumSensorSimulator.exe --port COM3 --baudrate 115200
```

## Build And Upload

VSCode PlatformIO가 설치되어 있으면 프로젝트 폴더를 열고 PlatformIO의 `Build` 또는 `Upload` 버튼을 사용하면 됩니다.

명령줄에서는 다음처럼 빌드할 수 있습니다.

```powershell
pio run
```

현재 작업 환경에서는 `pio`가 PATH에 없을 수 있으므로, 이 경우 아래 명령을 사용할 수 있습니다.

```powershell
& 'C:\Users\seong\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -m platformio run
```

업로드는 보드가 연결된 상태에서:

```powershell
pio run --target upload
```

보드 포트를 직접 지정해야 하면:

```powershell
pio run --target upload --upload-port COM7
```

빌드가 성공하면 펌웨어 파일은 다음 위치에 생성됩니다.

```text
.pio/build/esp32-s3-devkitc-1/firmware.bin
```

## Estimate Output

ESP32-S3는 packet을 처리할 때마다 다음 형식의 JSON을 출력합니다.

```json
{
  "seq": 10,
  "estimate": {
    "x": 1.2345,
    "y": 2.3456,
    "theta": 0.12345
  },
  "neff": 240.5,
  "truth": {
    "x": 1.22,
    "y": 2.35,
    "theta": 0.12
  },
  "error": {
    "x": 0.0145,
    "y": -0.0044,
    "theta": 0.00345
  }
}
```

`truth`와 `error`는 simulator packet의 `state`를 기준으로 출력되는 검증용 값입니다. 실제 센서에서는 제거해도 됩니다.

## Important Config

주요 설정은 `include/estimator_config.h`에서 바꿀 수 있습니다.

```c
#define EST_PARTICLE_COUNT 350
#define EST_INITIAL_X_M 1.0f
#define EST_INITIAL_Y_M 1.0f
#define EST_INITIAL_THETA_RAD 0.0f
#define EST_TAG_OFFSET_X_M 0.0f
#define EST_TAG_OFFSET_Y_M 0.0f
#define EST_TAG_OFFSET_Z_M 0.5f
```

메카넘 휠 파라미터:

```c
#define EST_PPR 36.0f
#define EST_GEAR_RATIO 15.0f
#define EST_WHEEL_RADIUS_M 0.0762f
#define EST_LX_M 0.400f
#define EST_LY_M 0.440f
```

## Algorithm

1. `serial_parser.c`
   - JSON line에서 IMU, wheel ticks, UWB range를 추출합니다.

2. `mecanum.c`
   - 4개 wheel tick을 바디 좌표계 이동량으로 변환합니다.
   - `gyro_z`와 wheel heading delta를 섞어 `dtheta`를 만듭니다.

3. `particle_filter.c`
   - Prediction: 메카넘 오도메트리와 gyro heading으로 particle을 이동시킵니다.
   - Correction: UWB range likelihood로 weight를 갱신합니다.
   - Resampling: effective sample size가 낮으면 systematic resampling을 수행합니다.

## Notes

- 현재 JSON 파서는 simulator 출력 포맷에 맞춘 경량 C 파서입니다.
- UWB 앵커 좌표는 packet에 포함된 값을 그대로 사용합니다.
- UWB distance는 3D 거리로 처리합니다.
- 태그 오프셋은 로봇 중심 기준 바디 좌표계 offset으로 반영됩니다.
- 출력과 입력을 모두 `Serial`로 사용합니다. 실제 하드웨어 연결이 복잡해지면 입력은 `Serial1`, 디버그 출력은 `Serial`로 나누는 편이 좋습니다.
