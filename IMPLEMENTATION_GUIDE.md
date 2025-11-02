# Pure Pursuit Controller Implementation Guide

## 📋 개요
기존 MAP (Model-based Predictive) Controller에 **Pure Pursuit Controller**를 추가 구현하였습니다.  
Launch 시 `controller_mode` 인자를 통해 두 컨트롤러 중 하나를 선택하여 실행할 수 있습니다.

---

## 🗂️ 추가된 파일

### 1. Header Files
- **`include/crazy_controller/pp_controller.hpp`**
  - `PP_Controller_Node` 클래스 선언
  - ROS2 노드 레벨 인터페이스 정의

### 2. Source Files
- **`src/pp_controller.cpp`**
  - `PP_Controller_Node` 구현
  - ROS2 subscriber/publisher 설정
  - `pp_params.yaml` 로드

- **`src/utils_pp.cpp`**
  - `PP_Controller` 알고리즘 구현
  - Pure Pursuit 공식 적용
  - Lookup table 미사용 (직접 계산)

### 3. Configuration
- **`config/pp_params.yaml`** (기존 파일 활용)
  - Pure Pursuit 컨트롤러 튜닝 파라미터

---

## 🔧 수정된 파일

### 1. `include/crazy_controller/utils.hpp`
```cpp
// 추가된 구조체
struct PPResult {
  double steering_angle;
  double speed_now;
  // ... (기타 필드)
};

// 추가된 클래스 (Lookup table 미사용)
class PP_Controller {
  // steer_lookup_ 멤버 변수 없음
  // Pure Pursuit 직접 계산
};
```

### 2. `CMakeLists.txt`
```cmake
# pp_controller_node 실행 파일 추가
add_executable(pp_controller_node
  src/pp_controller.cpp
  src/utils_pp.cpp
)

# 의존성 및 링크 설정
ament_target_dependencies(pp_controller_node ...)
target_link_libraries(pp_controller_node Eigen3::Eigen yaml-cpp)

# Install targets에 추가
install(TARGETS controller_node pp_controller_node ...)
```

### 3. `launch/controller_launch.py`
```python
# PP 파라미터 경로 추가
pp_params_path = PythonExpression([...])

# MAP Controller (controller_mode != 'PP'일 때 실행)
map_controller_node = Node(
    condition=UnlessCondition(...)
)

# Pure Pursuit Controller (controller_mode == 'PP'일 때 실행)
pp_controller_node = Node(
    condition=IfCondition(...)
)
```

---

## ⚙️ Pure Pursuit 알고리즘 구현

### 핵심 공식
```cpp
// src/utils_pp.cpp - calc_steering_angle() 함수
double steering_angle = std::atan((2.0 * 0.33 * std::sin(eta)) / L1_distance);
```

- **Wheelbase (L)**: 0.33m (하드코딩)
- **eta**: Lookahead point와 차량 방향 사이의 각도
- **L1_distance**: Lookahead distance

### Lookup Table 제거
- MAP Controller: `steer_lookup_.lookup_steer_angle(lat_acc, speed)` 사용
- **PP Controller**: 직접 계산 (lateral acceleration 계산 불필요)

### 유지된 Enhancement 기능
- ✅ `speed_lookahead_for_steer`: 속도에 따른 lookahead 조정
- ✅ `speed_adjust_lat_err`: Lateral error 기반 조정
- ✅ `speed_steer_scaling`: 속도에 따른 steering 스케일링
- ✅ Rate limiting (급격한 조향각 변화 방지)
- ✅ Clamping (최대 조향각 제한)

---

## 🚀 빌드 방법

```bash
cd ~/your_ros2_workspace
colcon build --packages-select crazy_controller
source install/setup.bash
```

---

## 🎮 실행 방법

### 1. MAP Controller 실행 (기본값)
```bash
ros2 launch crazy_controller controller_launch.py
```

또는 명시적으로:
```bash
ros2 launch crazy_controller controller_launch.py controller_mode:=MAP
```

### 2. Pure Pursuit Controller 실행
```bash
ros2 launch crazy_controller controller_launch.py controller_mode:=PP
```

### 3. Simulation 모드
```bash
# MAP + Simulation
ros2 launch crazy_controller controller_launch.py mod:=sim

# Pure Pursuit + Simulation
ros2 launch crazy_controller controller_launch.py controller_mode:=PP mod:=sim
```

---

## 🔍 구조 비교

### MAP Controller
```
MAP_Controller_Node (crazy_controller.cpp)
    └─> MAP_Controller (utils.cpp)
            └─> Lookup Table 사용
            └─> Lateral Acceleration 계산
            └─> steer_lookup_.lookup_steer_angle(lat_acc, speed)
```

### Pure Pursuit Controller
```
PP_Controller_Node (pp_controller.cpp)
    └─> PP_Controller (utils_pp.cpp)
            └─> Lookup Table 미사용
            └─> Pure Pursuit 공식 직접 계산
            └─> atan((2*L*sin(eta))/l_d)
```

---

## 📊 파라미터 파일

### MAP Controller
- `config/l1_params.yaml` (real mode)
- `config/l1_params_sim.yaml` (sim mode)

### Pure Pursuit Controller
- `config/pp_params.yaml` (공통)

둘 다 동일한 Lookup Table CSV 파일 경로를 전달받지만, **PP Controller는 실제로 사용하지 않습니다**.

---

## ✅ 검증 체크리스트

- [x] `PP_Controller_Impl` 클래스명 완전 제거
- [x] `PP_Controller` 클래스에서 `steer_lookup_` 멤버 변수 제거
- [x] `utils_pp.cpp`에서 `lookup_steer_angle` 함수 호출 제거
- [x] `utils_pp.cpp`에서 `lat_acc` 계산 제거
- [x] Pure Pursuit 공식 직접 구현 완료
- [x] Enhancement 기능 (scaling, clamping) 유지
- [x] CMakeLists.txt에 pp_controller_node 추가
- [x] Launch 파일에 조건부 로직 추가
- [x] MAP Controller는 영향받지 않음 확인

---

## 🎯 다음 단계

1. **빌드 테스트**
   ```bash
   colcon build --packages-select crazy_controller
   ```

2. **Launch 파일 테스트**
   ```bash
   ros2 launch crazy_controller controller_launch.py controller_mode:=PP
   ```

3. **파라미터 튜닝**
   - `config/pp_params.yaml` 조정
   - 실제 주행 테스트 및 성능 평가

4. **성능 비교**
   - MAP vs Pure Pursuit
   - Lap time, stability, 주행 경로 비교

---

## 📝 주의사항

1. **Lookup Table 경로**: PP Controller는 파라미터로 받지만 사용하지 않습니다.
2. **Wheelbase**: 0.33m로 하드코딩되어 있습니다. 차량이 바뀌면 수정 필요.
3. **Rate Limiting**: 급격한 조향각 변화 방지 (threshold = 0.4 rad)
4. **Maximum Steering Angle**: 0.45 rad (약 25.8도)

---

## 🐛 문제 해결

### 빌드 에러 발생 시
```bash
# 캐시 삭제 후 재빌드
rm -rf build/ install/ log/
colcon build --packages-select crazy_controller
```

### Launch 에러 발생 시
- `controller_mode` 값 확인: `MAP` 또는 `PP`만 가능
- 파라미터 파일 경로 확인: `config/pp_params.yaml` 존재 여부

### 실행 중 에러
- `/planned_waypoints` 토픽 publish 여부 확인
- `/odom` 토픽 publish 여부 확인
- TF transform (`map` → `base_link`) 확인

---

## 👥 Contributors
- Pure Pursuit Controller 구현: 2025-10-25

