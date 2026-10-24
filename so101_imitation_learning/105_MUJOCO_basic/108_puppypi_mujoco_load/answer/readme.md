# MuJoCo에서 4족 로봇/로봇암 XML 설정 시 떨림, free joint, childclass, 충돌, 조명 정리

## 1) 문서 목적

이 문서는 MuJoCo에서 로봇 XML을 올릴 때 자주 발생하는 다음 문제를 **중복 없이 한 번에 정리**하기 위한 실무용 참고 문서입니다.

- 4족 로봇이 허공에 고정되거나 바닥으로 떨어지는 문제
- 다리가 덜덜 떨리는 문제
- `free joint`의 역할
- `actuator`의 `kp`, `kv`, `forcerange`
- `joint`의 `damping`, `frictionloss`, `armature`
- `geom`의 `contype`, `conaffinity`, `rgba`
- `body childclass`를 써야 진동이 잡히는 이유
- 로봇암과 4족 로봇의 구조적 차이
- 화면이 어두운 문제와 조명/색상 설정

---

## 2) 핵심 요약

- **4족 로봇은 보통 floating-base 구조**이므로, 베이스가 월드에 고정되지 않게 하려면 `free joint`가 필요합니다.
- **로봇암은 보통 fixed-base 구조**이므로, 월드 직속 body에 joint가 없으면 자동으로 고정 베이스처럼 동작합니다.
- `actuator`의 `kp`, `kv`만으로는 진동이 완전히 안 잡힐 수 있습니다.이유는 이것이 **제어기(software)** 성격이고, 실제 기구의 **물리적 감쇠/마찰/모터 관성(hardware)** 은 `joint`의 `damping`, `frictionloss`, `armature`가 담당하기 때문입니다.
- `childclass`는 “모터 속성” 자체가 아니라 **하위 body/joint/geom에 default 설정을 일괄 상속하는 수단**입니다.따라서 잘 쓰면 진동 억제에 매우 효과적이지만, 잘못 쓰면 `free joint`나 발 충돌까지 망가뜨릴 수 있습니다.
- 4족 로봇은 바닥과 접촉해야 하므로 발/다리 geom의 충돌이 살아 있어야 합니다. 즉 일반적으로 `contype="1" conaffinity="1"` 같은 충돌 설정이 필요합니다.
- 화면이 어두우면 두 가지를 먼저 봐야 합니다.
  1. `scene.xml`의 `light`, `headlight`, `ambient`
  2. 로봇 `geom`의 `rgba="0 0 0 1"` 여부

---

## 3) 증상별 원인과 해결 방법

### 3.1 로봇이 허공에 고정됨

#### 원인

4족 로봇 베이스가 월드에 대해 자유도가 없기 때문입니다.
MuJoCo에서 `worldbody` 직속 body가 **joint 없이 시작되면 고정 베이스**로 취급됩니다.

즉, 이런 구조는 로봇이 공간에 못 박힌 상태입니다.

```xml
<worldbody>
  <body name="base">
    ...
  </body>
</worldbody>
```

#### 해결

4족 로봇처럼 **떠 있는 베이스(floating base)** 가 필요한 경우 베이스에 `free joint`를 둡니다.

```xml
<body name="base_footprint">
  <freejoint name="root"/>
  ...
</body>
```

또는

```xml
<body name="base_footprint">
  <joint type="free"/>
  ...
</body>
```

#### 참고

- `freejoint`가 있으면 로봇 전체가 중력을 받아 떨어지고, 바닥 충돌이 살아 있으면 착지합니다.
- `freejoint`가 없으면 로봇은 떨어지지 않고 공중 고정 상태에서 관절만 움직일 수 있습니다.

---

### 3.2 다리가 덜덜 떨림

#### 원인

주된 원인은 다음 조합입니다.

1. `actuator`는 있는데 `joint damping/frictionloss/armature`가 부족함
2. `kp`가 큰데 `kv`나 `damping`이 부족함
3. 관성/질량/충돌이 있는 구조에서 제어기만 강하고 기구 감쇠가 약함

즉, 서보가 목표 각도를 강하게 쫓아가지만 기계적인 브레이크가 약해서 **오버슈트와 미세 진동**이 남습니다.

#### 해결

다음 순서로 조정합니다.

1. `position actuator`에 `kp`, `kv` 설정
2. `joint`에 `damping`, `frictionloss`, `armature` 추가
3. 필요 시 `forcerange` 조정
4. 필요 시 `childclass`로 다리 체인 전체에 동일한 설정 상속

예시:

```xml
<default>
  <default class="servo_joint">
    <joint damping="0.3" frictionloss="0.05" armature="0.02" limited="true"/>
    <position kp="45" kv="1.2" forcerange="-1.5 1.5"/>
  </default>
</default>
```

---

### 3.3 로봇이 바닥으로 가라앉음 / 바닥을 뚫고 내려감

#### 원인

대부분 아래 둘 중 하나입니다.

1. **바닥 plane이 없음**
2. 발/다리 geom의 충돌이 꺼져 있음
   (`contype="0" conaffinity="0"` 상속 포함)

특히 `childclass`로 하위 geom에 다음이 상속되면 문제가 큽니다.

```xml
<geom contype="0" conaffinity="0"/>
```

이 설정이 발 geom까지 내려가면, 로봇은 바닥과 접촉 계산을 하지 못해 그대로 통과합니다.

#### 해결

- 바닥 plane 추가
- 발/다리 geom에 충돌 활성화
- `childclass` 적용 범위를 분리
- 충돌 관련 default와 서보 관련 default를 분리하는 것이 가장 안전

예시:

```xml
<geom name="floor" type="plane" size="0 0 0.05" pos="0 0 0"
      contype="1" conaffinity="1" rgba="0.8 0.8 0.8 1"/>
```

발/다리 geom 예시:

```xml
<geom type="mesh" mesh="lf_link2" contype="1" conaffinity="1"/>
```

---

### 3.4 화면이 어두움

#### 원인

보통 두 가지입니다.

1. 조명이 약하거나 범위가 부족함
2. 로봇 mesh가 완전 검정색(`rgba="0 0 0 1"`)임

검정색 geom은 빛을 받아도 형상이 잘 안 보입니다.

#### 해결

##### scene.xml 조정

- `headlight ambient`를 올림
- light를 하나 더 추가
- directional light 위치를 높임

```xml
<visual>
  <headlight diffuse="0.6 0.6 0.6" ambient="0.6 0.6 0.6" specular="0 0 0"/>
</visual>
```

```xml
<light pos="0 0 5" dir="0 0 -1" directional="true" diffuse="0.8 0.8 0.8"/>
<light pos="3 0 5" dir="-0.5 0 -1" directional="true" diffuse="0.4 0.4 0.4"/>
```

##### 로봇 색상 조정

```xml
<geom type="mesh" mesh="base_link" rgba="0.8 0.8 0.8 1"/>
```

---

## 4) MuJoCo 파라미터 역할 정리

## 4.1 actuator 파라미터

`position actuator`는 목표 각도를 따라가게 하는 **제어기**입니다.

| 항목           | 역할                        | 성격           |
| -------------- | --------------------------- | -------------- |
| `position`   | 목표 위치 추종 actuator     | 제어기         |
| `kp`         | 목표 각도로 끌어당기는 강성 | 비례 제어      |
| `kv`         | 속도에 비례한 제동          | 속도 제어/감쇠 |
| `forcerange` | actuator 출력 제한          | 토크 제한      |

### 해석

- `kp`가 크면 더 강하게 버팁니다.
- `kv`가 작으면 반응은 빠르지만 떨릴 수 있습니다.
- `forcerange`가 너무 작으면 다리를 못 버팁니다.
- `forcerange`가 너무 크고 감쇠가 작으면 충돌 시 과격해질 수 있습니다.

예시:

```xml
<position name="act_lb_j1" joint="lb_joint1"
          kp="45" kv="1.2" forcerange="-1.5 1.5"/>
```

---

## 4.2 joint 파라미터

`joint` 파라미터는 실제 기계계의 **물리 특성**을 흉내 냅니다.

| 항목             | 역할                                | 성격      |
| ---------------- | ----------------------------------- | --------- |
| `damping`      | 속도 비례 감쇠                      | 점성 마찰 |
| `frictionloss` | 정지 마찰/기어 마찰                 | 마찰      |
| `armature`     | 모터/감속기 회전자 관성의 등가 반영 | 관성      |
| `limited`      | 관절 제한 사용 여부                 | 제약      |
| `range`        | 관절 가동 범위                      | 제약 범위 |

### 해석

- `damping`은 “움직일수록 잡아주는 힘”
- `frictionloss`는 “작은 힘에는 꿈쩍 안 하는 기어 마찰”
- `armature`는 “모터 로터/감속기 때문에 더 묵직하게 보이게 하는 관성”
- 이 값들은 **바디용** 이 아니라 **관절/모터 기구부 특성**에 더 가깝습니다.

예시:

```xml
<joint name="lb_joint1" axis="1 0 0" range="-2 2"
       damping="0.3" frictionloss="0.05" armature="0.02"/>
```

---

## 4.3 geom 파라미터

### `contype`, `conaffinity`

충돌 그룹을 결정합니다.

- `contype`: 내가 어떤 타입으로 충돌할지
- `conaffinity`: 어떤 타입과 충돌 허용할지

실무적으로는 둘 다 `1`로 두고 시작하는 경우가 많습니다.

```xml
<geom contype="1" conaffinity="1"/>
```

반대로 아래는 충돌 비활성화입니다.

```xml
<geom contype="0" conaffinity="0"/>
```

### `rgba`

렌더링 색상입니다.

```xml
<geom rgba="0.8 0.8 0.8 1"/>
```

- `0 0 0 1`이면 완전 검정색이라 어둡게 보이기 쉽습니다.

---

## 4.4 body의 `childclass`

`childclass`는 해당 body 아래의 자식 요소들이 특정 default class를 상속받게 합니다.

즉, 이것은 “모터 속성” 자체라기보다 **default를 일괄 적용하는 구조적 장치**입니다.

예시:

```xml
<body name="lf_link1" childclass="servo_joint">
  ...
</body>
```

효과:

- 하위 joint에 `damping`, `frictionloss`, `armature` 적용
- 하위 geom에 `contype`, `conaffinity`, `rgba` 적용 가능
- 하위 actuator에는 직접 적용되지 않지만, 관련 default class를 actuator에서 별도 참조할 수 있음

---

## 4.5 free joint의 역할

`free joint`는 6자유도 베이스를 만듭니다.

- 위치 3축(x, y, z)
- 자세 3축(회전)

즉, 4족 로봇이 **땅 위에서 넘어지고, 착지하고, 이동하는 동작**을 하려면 필요합니다.

반면 고정형 로봇암은 보통 필요 없습니다.

---

## 5) 왜 actuator만으로 진동이 완전히 안 잡히고 childclass/joint 기본값이 필요한가

핵심은 **제어기와 기계계가 다르기 때문**입니다.

### actuator가 하는 일

- 목표 각도 추종
- `kp`, `kv` 기반으로 힘 계산
- 즉, “어디로 가야 하는가”를 결정

### joint가 하는 일

- 실제 기어 마찰, 축 감쇠, 로터 관성 반영
- 즉, “얼마나 쉽게 흔들리고, 얼마나 빨리 가라앉는가”를 결정

`actuator`만 강하게 주면 로봇은 목표 각도를 쫓아가지만, 실제 기계가 가진 점성/마찰이 약해서 계속 잔진동이 생길 수 있습니다.
반대로 `joint damping/frictionloss/armature`가 적절하면, actuator가 만든 오버슈트를 기구부가 흡수합니다.

### 왜 childclass가 효과적으로 보이는가

`childclass`를 쓰면 다리 체인 전체의 joint에 같은 `damping`, `frictionloss`, `armature`가 **빠짐없이** 들어갑니다.그래서 개별 관절 누락 없이 설정이 통일되고, 최종 잔진동이 줄어듭니다.

> **정리**
>
> - `actuator`: 소프트웨어 제어 성격
> - `joint`: 하드웨어 물리 성격
> - 둘이 같이 있어야 실제 서보처럼 안정됨

---

## 6) 로봇암(jdcobot100 계열)과 4족 로봇(puppy 계열) 비교

| 항목                        | 로봇암 (jdcobot100 계열) | 4족 로봇 (puppy 계열)       |
| --------------------------- | ------------------------ | --------------------------- |
| 베이스 구조                 | fixed-base               | floating-base               |
| free joint 필요             | 보통 불필요              | 보통 필요                   |
| 월드 직속 body에 joint 없음 | 고정됨                   | 고정되므로 이동 불가        |
| 바닥 충돌 필요성            | 낮음 또는 선택적         | 매우 중요                   |
| `contype/conaffinity`     | 0/0도 가능               | 발/다리는 보통 1/1 필요     |
| 목적                        | 고정된 위치에서 팔 동작  | 중력, 착지, 보행, 자세 유지 |

### 6.1 free joint 필요 여부

- **로봇암**: 바닥/프레임에 고정된 기계로 모델링하므로 `free joint`가 없어도 정상
- **4족 로봇**: 몸통이 공간에서 자유롭게 움직여야 하므로 `free joint` 필요

### 6.2 `contype/conaffinity` 값 차이

- **로봇암**: 지면 접촉이 핵심이 아니고, 종종 자기 몸체 충돌도 끄기 위해 `0 0` 사용
- **4족 로봇**: 발이 바닥을 딛고 서야 하므로 최소한 발/다리/바닥은 충돌 활성화 필요

### 6.3 fixed-base와 floating-base 차이

- **fixed-base**
  - base가 world에 고정
  - 떨어지지 않음
  - 바닥 plane 없이도 유지 가능
- **floating-base**
  - base가 자유롭게 움직임
  - 중력 받음
  - 바닥 충돌 없으면 내려감

---

## 7) childclass를 어디에 적용해야 안전한가

## 7.1 base 전체에 줄 때 생길 수 있는 문제

베이스 상위 body에 `childclass`를 걸면, 의도하지 않은 요소까지 상속될 수 있습니다.

예를 들어 default class 안에 다음이 있으면:

```xml
<geom contype="0" conaffinity="0"/>
<joint damping="0.3" frictionloss="0.05" armature="0.02" limited="true"/>
```

이것이 발 geom이나 특수 joint까지 내려가면 다음 문제가 생길 수 있습니다.

- 발 충돌 꺼짐 → 바닥 통과
- 루트 쪽 joint에 원치 않는 제한/감쇠 적용
- 시뮬레이션 해석이 꼬여 베이스 거동이 이상해짐

---

## 7.2 다리 체인에만 줄 때의 장점

`lb_link1`, `rb_link1`, `rf_link1`, `lf_link1` 같은 **다리 시작 body** 에만 `childclass`를 두면 좋습니다.

장점:

- 다리 관절에만 서보 물성 일괄 적용
- 베이스/floating root에 영향 최소화
- 발 geom 충돌만 별도로 관리 가능

예시:

```xml
<body name="lf_link1" childclass="servo_joint">
  ...
</body>
```

---

## 7.3 충돌이 꺼지는 문제(`contype=0 conaffinity=0`) 주의

> **주의**
> `childclass`에 아래 설정이 있으면 하위 geom 충돌이 전부 꺼질 수 있습니다.
>
> ```xml
> <geom contype="0" conaffinity="0"/>
> ```
>
> 이 상태로 4족 로봇 발에 상속되면, 로봇은 바닥에 착지하지 못하고 그대로 내려갑니다.

### 권장 방식

- **서보용 default class** 와 **충돌용 default class** 를 분리
- 또는 발 geom만 따로 `contype="1" conaffinity="1"` 명시

---

## 8) 추천 XML 패턴 예시

## 8.1 4족 로봇용 default / actuator 예시

```xml
<default>
  <default class="servo_joint">
    <joint damping="0.3" frictionloss="0.05" armature="0.02" limited="true"/>
    <position kp="45" kv="1.2" forcerange="-1.5 1.5"/>
  </default>

  <default class="collide_geom">
    <geom contype="1" conaffinity="1" rgba="0.8 0.8 0.8 1"/>
  </default>
</default>

<actuator>
  <position name="act_lb_j1" joint="lb_joint1" class="servo_joint"/>
  <position name="act_lb_j2" joint="lb_joint2" class="servo_joint"/>
</actuator>
```

### 적용 팁

- `servo_joint`는 joint/actuator 쪽
- `collide_geom`는 발/다리 geom 쪽
- 필요하면 body별로 `childclass="servo_joint"` 사용

---

## 8.2 바닥 plane 추가 예시

```xml
<worldbody>
  <geom name="floor" type="plane" size="0 0 0.05"
        pos="0 0 0" contype="1" conaffinity="1"
        rgba="0.8 0.8 0.8 1"/>
</worldbody>
```

---

## 8.3 4족 로봇의 floating-base 예시

```xml
<worldbody>
  <body name="base_footprint">
    <freejoint name="root"/>
    <body name="base_link" pos="0 0 0.15">
      ...
    </body>
  </body>
</worldbody>
```

---

## 8.4 로봇암용 fixed-base 예시

```xml
<worldbody>
  <body name="arm_base">
    <body name="link1">
      <joint name="joint1" axis="0 0 1"/>
      ...
    </body>
  </body>
</worldbody>
```

설명:

- `arm_base`가 world에 직접 붙어 있고 free joint가 없으므로 고정형입니다.

---

## 9) 튜닝 순서와 체크리스트

## 9.1 먼저 확인할 것

### 구조

- 4족 로봇인가? → `free joint` 필요 여부 확인
- 로봇암인가? → fixed-base로 둘지 확인

### 충돌

- 바닥 plane이 있는가?
- 발 geom의 `contype/conaffinity`가 살아 있는가?
- `childclass`가 충돌을 꺼버리지 않았는가?

### 시각화

- 로봇 색이 완전 검정(`rgba="0 0 0 1"`)은 아닌가?
- 조명이 충분한가?

---

## 9.2 값을 조절하는 순서

1. **구조 먼저**

   - 4족: `free joint` 추가
   - 로봇암: 고정 베이스 확인
2. **충돌 먼저**

   - 바닥 plane 추가
   - 발 충돌 활성화
3. **actuator 기본값**

   - `kp` 설정
   - `kv` 설정
   - `forcerange` 설정
4. **joint 물성 보강**

   - `damping` 올리기
   - `frictionloss` 조금 추가
   - `armature` 소량 추가
5. **childclass 범위 정리**

   - 베이스 전체보다는 다리 체인에 우선 적용

### 권장 튜닝 감각

- 자세를 못 버팀 → `kp` 또는 `forcerange` 증가
- 부르르 떨림 → `kv`, `damping` 증가
- 너무 둔함 → `damping`, `kv` 소폭 감소
- 움직임이 너무 가벼움 → `armature` 소폭 증가

---

## 9.3 흔한 실수

> **주의 1**
> `childclass` 안에 `geom contype="0" conaffinity="0"`를 넣고 다리 전체에 상속시키면 발 충돌까지 꺼집니다.

> **주의 2**
> 4족 로봇에 `free joint`가 없으면 몸통은 떨어지지 않고 공중 고정 상태가 됩니다.

> **주의 3**
> `kp`만 올리고 `kv/damping`을 안 올리면 진동이 커질 수 있습니다.

> **주의 4**
> `forcerange`가 너무 작으면 서보가 자세를 지탱하지 못합니다.

> **주의 5**
> 화면이 어둡다고 조명만 보지 말고 `rgba="0 0 0 1"`도 확인해야 합니다.

> **주의 6**
> 로봇암과 4족 로봇의 XML 구조를 같은 방식으로 해석하면 안 됩니다.
> 로봇암은 fixed-base, 4족은 floating-base라는 차이가 핵심입니다.

---

## 10) 결론

MuJoCo에서 로봇의 떨림과 가라앉음 문제는 대부분 다음 세 층을 구분하면 정리됩니다.

1. **구조**

   - fixed-base인가, floating-base인가
   - 4족 로봇이면 `free joint`가 필요한가
2. **제어**

   - `actuator`의 `kp`, `kv`, `forcerange`
3. **물리**

   - `joint`의 `damping`, `frictionloss`, `armature`
   - `geom`의 `contype`, `conaffinity`
   - `body childclass`의 상속 범위

실무적으로는 다음 원칙이 가장 중요합니다.

- **로봇암은 보통 fixed-base**
- **4족 로봇은 보통 floating-base**
- **actuator만으로는 부족하고, joint 물성이 함께 들어가야 진동이 안정화됨**
- **childclass는 강력하지만 적용 범위를 잘못 잡으면 충돌과 루트 거동까지 망가질 수 있음**
- **4족 로봇은 반드시 바닥 충돌과 발 충돌을 확인해야 함**
- **화면이 어두우면 조명과 geom 색상을 같이 봐야 함**

필요하면 다음 단계로는 이 문서를 기준으로

- **4족 로봇용 안정화 템플릿 XML**
- **로봇암용 fixed-base 템플릿 XML**
- **튜닝용 Python 체크 스크립트**
  형태로 분리해 관리하는 것이 좋습니다.