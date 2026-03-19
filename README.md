# smart-robot-car
Agile obstacle avoiding smart car project with ultrasonic scanning, MPU6050 heading stabilization, and PID control.

## Overview
`Genius_design/Genius_design.ino` implements a complete control system:
1. Background servo radar scanning at 5 angles: 0°, 45°, 90°, 135°, 180°
2. Real-time yaw stabilization using MPU6050 and straight-line PID
3. Side repulsion-based heading correction when front path is clear
4. Emergency stop and PID-based turning when front obstacle detected

## Features
- Non-blocking radar scan task (servo + ultrasonic distance)
- Predictive gap evaluation for best turn direction
- Adaptive heading correction from side distances
- Precise PID turn with motion control and recovery

## File Structure
- `Genius_design/Genius_design.ino` - main full control logic
- `test/test.ino` - development version
- `move_direction/move_direction.ino` - basic motor direction control
- `obstacle_move/obstacle_move.ino` - primitive obstacle avoidance
- `ultrasound_scan/ultrasound_scan.ino` - scanning example

## Genius_design.ino Control Flow
```mermaid
flowchart TD
    %% 定义节点颜色和样式
    classDef start fill:#d4edda,stroke:#28a745,stroke-width:2px,color:#000
    classDef decision fill:#fff3cd,stroke:#ffc107,stroke-width:2px,color:#000
    classDef danger fill:#f8d7da,stroke:#dc3545,stroke-width:2px,color:#000
    classDef action fill:#cce5ff,stroke:#007bff,stroke-width:2px,color:#000

    A([Start setup]):::start --> B[Init motors, ultrasonic, servo, MPU6050]:::start
    B --> C([Loop start]):::start
    
    C --> D["updateRadarTask()"]:::action
    D --> E["Read scan distances at 45°,90°,135°"]:::action
    E --> F{"frontDist > SafeDistance?"}:::decision
    
    %% 前方安全分支
    F -- Yes --> G[Compute left/right repulsion]:::action
    G --> H["Adjust yaw_target by ±0.5"]:::action
    H --> I["moveStraightPID()"]:::action
    I --> J["delay(10)"]
    J --> C
    
    %% 前方受阻分支
    F -- No --> K["stopCar()"]:::danger
    K --> L["turnAngle = evaluateBestDirection()"]:::action
    L --> M{"turnAngle == 999?"}:::decision
    
    M -- Yes --> N["moveBackward + rotateRelativePID(180)"]:::danger
    M -- No --> O{"turnAngle != 0?"}:::decision
    
    O -- Yes --> P["rotateRelativePID(turnAngle)"]:::action
    O -- No --> Q[skip turn]
    
    P --> R[reset servo forward]:::action
    N --> R
    Q --> R
    
    R --> S["delay(200)"]
    S --> T["delay(10)"]
    T --> C
```

## How to Use
1. Open `Genius_design/Genius_design.ino` in Arduino IDE.
2. Upload to your smart robot car hardware.
3. Place in an open area and test obstacle avoidance.

## Notes
- Tune `SafeDistance`, `WarningDistance`, and PID constants for your chassis.
- For better turn behavior, adjust `turn_Kp/Ki/Kd` and `minTurnSpeed`.
