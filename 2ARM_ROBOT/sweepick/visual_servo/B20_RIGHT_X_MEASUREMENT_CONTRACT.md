# B20 첫 REAL 비접촉 X 측정 계약

대상은 오른팔의 하나의 실제 접근/정렬 방향 `X_receive`다. 영상의 `u`와 물리 X의 부호가 같다고 가정하지 않는다. Y·깊이·회전을 제어값으로 채우거나 완전한 2D/6D 식별을 선행조건으로 요구하지 않는다.

1. `reference_id`는 물리적으로 식별한 현재 Pad/task 관계 또는 확인된 비접촉 기준 자세를 가리킨다. 기준 영상 SHA, actual q/feedback, 실측 gripper opening, camera/mounting identity, 물체 identity·형상·보유 상태와 관측 조건을 결합한다. 비접촉 기준 자세는 사람이 확인한 물리 관계와 목표 특징 정의를 포함해야 한다.
2. `feature_definition`은 같은 물리 target에서 같은 영상량을 측정한다. 현재 구현은 관측된 red component의 centroid와 확인된 Pad/task target의 상대 오차다. 잘린 centroid는 보정에 쓰지 않는다. 부분 모서리는 별도 물리 identity/target definition이 필요하며 이번 구현에는 없다. 가려진 면·전체 중심·Pad 대응을 채우지 않는다.
3. `X_receive_basis`는 A model world에 표현한 단위 방향과 실제 접근 방향의 확인 근거를 포함한다. 실제 q의 단위·관절 순서·calibration/device identity와 FK source를 기록한다. candidate mapping/FK를 물리 ground truth로 승격하지 않는다. 명령 displacement와 actual feedback 기반 FK displacement를 별도 보존한다.
4. `fit_observations`에는 같은 물체·같은 reference의 전후 특징, right actual q/FK, left held evidence, measured opening, camera/state 시각과 출처를 보존한다. 물체가 움직이거나 camera–object/feature 관계를 비교할 수 없으면 fit에서 제외한다. 단순 left torque on은 동일 물체 확인이 아니다.
5. `independent_sign_observation`은 fit에서 제외한 관측으로 실제 보정 방향과 영상 응답의 부호를 검증한다. 동일 fit pair를 다시 평가한 결과를 독립 검증이라 부르지 않는다. 필요한 특징을 잃으면 응답 미관측으로 남기며, 새 숫자 dwell이나 error cap을 만들지 않는다.

위 조건이 확보된 경우에만 기존 `estimate_jacobian`에 1D 실제 displacement와 같은 정의의 feature delta를 넣는다. 최소 scalar 관계는 `J_uX = delta_u / delta_X_actual`이다. 0/비유한 displacement, 출처·context 불일치는 식별 불가다. 분해능 이하의 움직임을 증폭해 새로운 물리 정확도를 주장하지 않는다. 기존 numerical rank 규칙은 수치 식별 가능성일 뿐 물리 noise bound가 아니다.

독립 sign/context 검증 전 fit은 `OBSERVED_SUBSPACE_FIT_REQUIRES_VALIDATION`, `validated=false`다. 검증 근거의 observation/source SHA와 exclusion 사유를 J의 `source_sha256`/context에 결합한 뒤에만 `REAL_MEASURED`, `validated=true`로 전달한다. 외부 producer의 `validated` 주장은 실제 근거를 갖춰야 한다. 어댑터 자체가 물리 사실을 인증하지 않는다.

현재 B20에서는 유효 pair, 물리 목표, 독립 sign 관측이 모두 미확보다. J는 null이며, 14개 REAL 결과는 모두 다음과 같다.

```json
{
  "observed_control_dimensions": 0,
  "estimated_tool_correction": null,
  "executable": false,
  "motor_write_authorized": false
}
```

향후 검증된 1D 제안에서도 실행 권한 두 필드는 false다. A는 자신의 현재 actual pose에서 기존 IK·경로·충돌·Owner·Stage·STOP·writer 계약을 적용한다. gain, motor velocity, collision margin, reach tolerance는 변경하지 않는다. image error를 TCP millimetre accuracy로 환산하지 않는다.
