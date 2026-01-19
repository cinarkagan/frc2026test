# Robot Controller Architecture Refactoring Plan

## Özet
Mevcut FRC robot kodunu yeni bir controller-based mimariye dönüştüreceğiz. Controller'lar robot ile **sadece** `RobotState` ve `RobotAction` üzerinden iletişim kuracak.

## Yeni Klasör Yapısı

```
frc/robot/
├── Main.java
├── Robot.java                          # Controller loop eklendi
├── RobotContainer.java                 # Controller yönetimi
├── Telemetry.java
├── LimelightHelpers.java
│
├── controller/                         # YENİ
│   ├── Controller.java                 # Interface
│   ├── GamepadController.java          # Xbox controller
│   ├── AutoController.java             # State machine
│   └── ControllerManager.java          # Controller switching
│
├── robot/                              # Mevcut kodlar buraya taşınacak (bilerek ayrı)
│   ├── state/
│   │   ├── RobotState.java             # Robot → Controller veri
│   │   └── RobotAction.java            # Controller → Robot komut
│   │
│   ├── RobotCore.java                  # State builder & action executor
│   │
│   ├── subsystems/
│   │   ├── CommandSwerveDrivetrain.java
│   │   ├── ShooterSubsystem.java
│   │   └── LocalizationSubsystem.java
│   │
│   ├── commands/
│   │   ├── TurnToAngle.java
│   │   ├── TurnToTag.java
│   │   └── DriveToTag.java
│   │
│   └── constants/
│       ├── SwerveConstants.java
│       ├── DriveConstants.java
│       └── ShooterConstants.java
│
└── hal/                                # Boş (ileride donanım soyutlaması için)
    └── package-info.java
```

## Paket ve Path Eşlemesi
- `frc/robot/` kökü `package frc.robot;`
- `frc/robot/controller/` -> `package frc.robot.controller;`
- `frc/robot/robot/` -> `package frc.robot.robot;` (bilerek iki katman)

## Yeni Dosyalar

### 1. RobotState.java (robot/state/)
Robot'un anlık durumu - Controller'a gönderilir:
- Position: x, y (metre)
- Heading: derece
- Velocity: vx, vy (m/s), omega (rad/s)
- Shooter: RPM, enabled durumu
- Vision: tag sayısı, pose estimate (Pose2d, metre/radyan)
- Timestamp (s)

### 2. RobotAction.java (robot/state/)
Controller'dan gelen komutlar:
- `DRIVE`: velocityX, velocityY (m/s), rotationRate (rad/s)
- `NAVIGATE`: targetX, targetY (m), opsiyonel heading (deg)
- `TURN`: targetAngle (deg)
- `SHOOTER`: enable/disable/adjustRPM
- `COMPOSITE`: birden fazla action birleşimi (RobotCore çakışmaları çözer)

### 3. Controller.java (controller/)
```java
public interface Controller {
    void onActivate();
    void onDeactivate();
    RobotAction update(RobotState state);
    String getName();
    boolean isFinished();
}
```

### 4. GamepadController.java (controller/)
- Xbox controller input'larını okur
- Stick → drive velocity (max hızlar constructor ile verilir, subsystem'e dokunmaz)
- Bumper → shooter RPM ayarı
- A button → turn to angle

### 5. AutoController.java (controller/)
State Machine ile çalışır:
- `IDLE`: Beklemede
- `DRIVING_TO_POINT`: Hedefe gidiyor
- `TURNING`: Açıya dönüyor
- `WAITING`: Bekliyor
- `SHOOTING`: Atış yapıyor
- `FINISHED`: Tamamlandı

Waypoint sistemi:
```java
autoController
    .addWaypoint(2.0, 0.0)
    .addWaypoint(2.0, 1.0, 90.0)  // heading ile
    .addWaypointWithShoot(3.0, 1.0, 45.0);
```

### 6. RobotCore.java (robot/)
- `buildState()`: Tüm subsystem'lerden veri toplayıp RobotState oluşturur
- `executeAction(RobotAction)`: Action'ı subsystem komutlarına çevirir; command/scheduler ile doğrudan kontrol çakışmasını engeller

### 7. ControllerManager.java (controller/)
- Aktif controller'ı yönetir
- Teleop ↔ Auto geçişlerini sağlar (RobotContainer sadece manager'a mod bildirir)

## Değiştirilecek Dosyalar

### Robot.java
```java
// Her periodic'te:
private void runControllerLoop() {
    RobotState state = robotCore.buildState();
    Controller activeController = controllerManager.getActiveController();
    RobotAction action = activeController.update(state);
    robotCore.executeAction(action);
}
```

### RobotContainer.java
- RobotCore instance'ı oluşturur
- GamepadController ve AutoController oluşturur
- ControllerManager'ı kurar ve `getControllerManager()` ile Robot'a verir
- `prepareAutonomous()`/`prepareTeleop()` sadece manager'a mod bildirir

## Taşınacak Dosyalar

| Eski Konum | Yeni Konum |
|------------|------------|
| subsystems/*.java | robot/subsystems/*.java |
| commands/*.java | robot/commands/*.java |
| constants/*.java | robot/constants/*.java |

## İmplementasyon Sırası

1. Klasör yapısını oluştur ve package path eşlemesini doğrula (`frc.robot.robot` bilerek)
2. Mevcut dosyaları taşı ve package/import'ları güncelle
3. RobotState ve RobotAction sınıflarını (unit'lerle) yaz
4. Controller interface'ini yaz
5. RobotCore sınıfını yaz ve action çakışma kurallarını belirle
6. GamepadController'ı implemente et (max hızlar constructor ile gelir)
7. AutoController state machine'i implemente et (waypoint + opsiyonel heading)
8. ControllerManager'ı yaz ve aktif controller seçim akışını tanımla
9. RobotContainer: controller'ları kur, manager'ı Robot'a ver, mod bildirimlerini ekle
10. Robot.java: controller loop'u ve CommandScheduler akışını güncelle
11. Test et

## Navigation Yaklaşımı

**Simple PID** kullanılacak (DriveToTag benzeri):
- X ve Y eksenleri için ayrı PID controller
- Hedef pozisyona ulaşana kadar velocity komutu
- Tolerance: 0.15m pozisyon, 3° açı
- Opsiyonel heading varsa: aynı anda heading kontrolü yapılır, yoksa mevcut heading korunur

## WPILib Command Entegrasyonu

- `CommandScheduler.run()` hala çalışmaya devam eder
- Subsystem `periodic()` metodları çalışır
- TURN action için mevcut `TurnToAngle` command'ı schedule edilir
- NAVIGATE action için RobotCore içinde PID kontrolü yapılır (heading varsa ya dahili PID ya da TurnToAngle ile koordine edilir)

## Verification

1. Build: `./gradlew build`
2. Deploy: `./gradlew deploy`
3. Test:
   - Teleop: Gamepad ile sürüş kontrol edilir
   - Auto: Waypoint'ler takip edilir
   - Shooter: RPM ayarları çalışır
