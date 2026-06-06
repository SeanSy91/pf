# Mecanum Sensor Simulator Serial Output Spec

이 문서는 `MecanumSensorSimulator`가 serial 또는 stdout으로 출력하는 센서 데이터 포맷을 설명합니다.
수신 측에서는 각 라인을 하나의 독립적인 JSON packet으로 파싱하면 됩니다.

## Output Transport

- 기본 출력 주기: `100 ms`
- 기본 주파수: `10 Hz`
- 기본 baudrate: `115200`
- 인코딩: `UTF-8`
- 프레임 형식: `JSON Lines`
- 라인 구분자: `\n`

즉, serial 수신 측에서는 newline 단위로 한 줄을 읽고 `json.loads(line)` 형태로 파싱하면 됩니다.

## Coordinate System

월드 좌표계는 다음 기준을 사용합니다.

- `+X`: 오른쪽
- `+Y`: 위쪽
- `theta = 0 rad`: `+X` 방향
- `theta > 0`: 반시계 방향 회전
- 위치 단위: `m`
- 각도 단위: `rad`

UI 조작 기준은 로봇 바디 좌표계입니다.

- `W`: 현재 heading 방향으로 전진
- `S`: 현재 heading 반대 방향으로 후진
- `A`: 현재 heading 기준 왼쪽으로 횡이동
- `D`: 현재 heading 기준 오른쪽으로 횡이동
- `Z`: 반시계 방향 회전
- `X`: 시계 방향 회전

## Packet Structure

각 출력 packet은 다음 최상위 필드를 가집니다.

```json
{
  "seq": 0,
  "state": {},
  "imu": {},
  "hall_ticks": {},
  "uwb": []
}
```

## Top-Level Fields

| Field | Type | Description |
|---|---:|---|
| `seq` | integer | 0부터 증가하는 packet 번호 |
| `state` | object | 시뮬레이터의 true pose |
| `imu` | object | IMU 가속도/자이로 측정값 |
| `hall_ticks` | object | 4개 휠의 홀센서 tick 증분 |
| `uwb` | array | 해당 주기에 수신된 UWB 앵커 거리 데이터 |

현재 packet에는 timestamp가 포함되지 않습니다. 필터 쪽에서는 고정 주기 `dt = 0.1 s`를 사용하면 됩니다.

## State

```json
{
  "x": 1.045,
  "y": 1.0,
  "theta": 0.0,
  "theta_deg": 0.0
}
```

| Field | Unit | Description |
|---|---:|---|
| `x` | m | 로봇 중심의 월드 X 위치 |
| `y` | m | 로봇 중심의 월드 Y 위치 |
| `theta` | rad | 로봇 heading |
| `theta_deg` | deg | 확인용 heading degree |

`state`는 시뮬레이션 기준의 실제 위치입니다. 파티클 필터 개발 중에는 ground truth 비교용으로 사용할 수 있습니다.

## IMU

```json
{
  "acc_x": 0.478,
  "acc_y": 0.022,
  "acc_z": 1.001,
  "gyro_x": -0.008,
  "gyro_y": -0.011,
  "gyro_z": 0.000
}
```

| Field | Unit | Description |
|---|---:|---|
| `acc_x` | g | 로봇 바디 좌표계 X축 가속도 |
| `acc_y` | g | 로봇 바디 좌표계 Y축 가속도 |
| `acc_z` | g | 로봇 바디 좌표계 Z축 가속도 |
| `gyro_x` | rad/s | X축 각속도 |
| `gyro_y` | rad/s | Y축 각속도 |
| `gyro_z` | rad/s | Z축 각속도, heading 변화율 |

현재 시뮬레이터에서 heading 추정에 가장 직접적으로 사용할 값은 `gyro_z`입니다.

## Hall Sensor Ticks

```json
{
  "front_left": 50,
  "front_right": 50,
  "rear_left": 51,
  "rear_right": 51
}
```

| Field | Description |
|---|---|
| `front_left` | 전방 좌측 휠의 1주기 tick 증분 |
| `front_right` | 전방 우측 휠의 1주기 tick 증분 |
| `rear_left` | 후방 좌측 휠의 1주기 tick 증분 |
| `rear_right` | 후방 우측 휠의 1주기 tick 증분 |

Tick 값은 각 출력 주기 동안의 증분값입니다. 방향에 따라 양수 또는 음수가 나올 수 있습니다.

기본 휠 파라미터는 다음과 같습니다.

| Parameter | Value |
|---|---:|
| `ppr` | `36` |
| `gear_ratio` | `15.0` |
| `wheel_ppr` | `540` |
| `wheel_radius` | `0.0762 m` |
| `Lx` | `0.400 m` |
| `Ly` | `0.440 m` |
| `L = Lx + Ly` | `0.840 m` |

기본 meter per tick:

```text
meter_per_tick = 2*pi*0.0762 / (36*15)
               ~= 0.000887 m/tick
```

휠 순서는 다음과 같습니다.

```text
front_left, front_right, rear_left, rear_right
```

메카넘 휠 배치는 `X` 형태입니다.

## UWB

`uwb`는 매 packet마다 2개 이상 4개 이하의 앵커 데이터가 랜덤으로 들어옵니다.

```json
[
  {
    "id": "A4",
    "x": 0.5,
    "y": 9.5,
    "z": 2.0,
    "distance": 8.711,
    "std": 0.116
  },
  {
    "id": "A1",
    "x": 0.5,
    "y": 0.5,
    "z": 2.0,
    "distance": 2.048,
    "std": 0.157
  }
]
```

| Field | Unit | Description |
|---|---:|---|
| `id` | string | 앵커 ID |
| `x` | m | 앵커 월드 X 위치 |
| `y` | m | 앵커 월드 Y 위치 |
| `z` | m | 앵커 월드 Z 위치 |
| `distance` | m | UWB 태그와 앵커 사이의 측정 거리 |
| `std` | m | 해당 거리 측정의 표준편차 |

`distance`는 3D 거리입니다.

```text
distance ~= sqrt((tag_x-anchor_x)^2 + (tag_y-anchor_y)^2 + (tag_z-anchor_z)^2) + noise
```

파티클 필터에서는 각 파티클의 태그 위치와 앵커 위치로 예측 거리를 계산하고, `std`를 measurement likelihood의 표준편차로 사용하면 됩니다.

예:

```text
residual = measured_distance - predicted_distance
weight *= exp(-0.5 * residual^2 / std^2)
```

## UWB Tag Offset

UWB distance는 로봇 중심이 아니라 UWB 태그 위치 기준입니다.

태그 위치는 설정 파일의 다음 값을 사용합니다.

```json
{
  "tag_offset_x": 0.0,
  "tag_offset_y": 0.0,
  "tag_offset_z": 0.5
}
```

`tag_offset_x`, `tag_offset_y`는 로봇 바디 좌표계 기준입니다. 따라서 파티클 필터에서 파티클 pose가 `(x, y, theta)`일 때 태그의 월드 위치는 다음처럼 계산해야 합니다.

```text
tag_x = x + cos(theta)*tag_offset_x - sin(theta)*tag_offset_y
tag_y = y + sin(theta)*tag_offset_x + cos(theta)*tag_offset_y
tag_z = tag_offset_z
```

## Full Example Packet

```json
{
  "seq": 0,
  "state": {
    "x": 1.045,
    "y": 1.0,
    "theta": 0.0,
    "theta_deg": 0.0
  },
  "imu": {
    "acc_x": 0.4781950671373997,
    "acc_y": 0.021741684130496562,
    "acc_z": 1.000995037134074,
    "gyro_x": -0.007645436509716318,
    "gyro_y": -0.010921732151041415,
    "gyro_z": 0.0003133451683171687
  },
  "hall_ticks": {
    "front_left": 50,
    "front_right": 50,
    "rear_left": 51,
    "rear_right": 51
  },
  "uwb": [
    {
      "id": "A4",
      "x": 0.5,
      "y": 9.5,
      "z": 2.0,
      "distance": 8.710998943615056,
      "std": 0.11611884747163331
    },
    {
      "id": "A1",
      "x": 0.5,
      "y": 0.5,
      "z": 2.0,
      "distance": 2.048037503983292,
      "std": 0.15721599977247408
    },
    {
      "id": "A3",
      "x": 9.5,
      "y": 9.5,
      "z": 2.0,
      "distance": 12.06014287868034,
      "std": 0.15456680648947418
    }
  ]
}
```

실제 serial 출력은 위 JSON이 한 줄로 압축되어 출력됩니다.

## Recommended Receiver Behavior

1. Serial에서 newline 단위로 한 줄씩 읽습니다.
2. 빈 줄은 무시합니다.
3. JSON 파싱 실패 시 해당 라인만 버립니다.
4. `seq`가 건너뛰는 경우 packet drop으로 처리합니다.
5. 필터 prediction에는 `hall_ticks`와 `imu.gyro_z`를 사용합니다.
6. 필터 correction에는 `uwb` 배열의 각 range measurement를 사용합니다.
7. `state`는 실기 데이터에는 없는 ground truth로 보고, 시뮬레이션 검증용으로만 사용합니다.

