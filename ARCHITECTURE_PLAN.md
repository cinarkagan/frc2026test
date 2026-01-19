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
├── robot/                              # Mevcut kodlar buraya taşınacak
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

## Yeni Dosyalar

### 1. RobotState.java (robot/state/)
Robot'un anlık durumu - Controller'a gönderilir:
- Position: x, y (metre)
- Heading: derece
- Velocity: vx, vy, omega
- Shooter: RPM, enabled durumu
- Vision: tag sayısı, pose estimate
- Timestamp

### 2. RobotAction.java (robot/state/)
Controller'dan gelen komutlar:
- `DRIVE`: velocityX, velocityY, rotationRate
- `NAVIGATE`: targetX, targetY (+ opsiyonel heading)
- `TURN`: targetAngle
- `SHOOTER`: enable/disable/adjustRPM
- `COMPOSITE`: birden fazla action birleşimi

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
- Stick → drive velocity
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
- `executeAction(RobotAction)`: Action'ı subsystem komutlarına çevirir

### 7. ControllerManager.java (controller/)
- Aktif controller'ı yönetir
- Teleop ↔ Auto geçişlerini sağlar

## Değiştirilecek Dosyalar

### Robot.java
```java
// Her periodic'te:
private void runControllerLoop() {
    RobotState state = robotCore.buildState();
    RobotAction action = activeController.update(state);
    robotCore.executeAction(action);
}
```

### RobotContainer.java
- RobotCore instance'ı oluştur
- GamepadController ve AutoController oluştur
- ControllerManager ile yönet
- `prepareAutonomous()` ve `prepareTeleop()` metodları

## Taşınacak Dosyalar

| Eski Konum | Yeni Konum |
|------------|------------|
| subsystems/*.java | robot/subsystems/*.java |
| commands/*.java | robot/commands/*.java |
| constants/*.java | robot/constants/*.java |

## İmplementasyon Sırası

1. Klasör yapısını oluştur
2. Mevcut dosyaları taşı ve package'ları güncelle
3. RobotState ve RobotAction sınıflarını yaz
4. Controller interface'ini yaz
5. RobotCore sınıfını yaz
6. GamepadController'ı implemente et
7. AutoController state machine'i implemente et
8. ControllerManager'ı yaz
9. Robot.java ve RobotContainer.java'yı güncelle
10. Test et

## Navigation Yaklaşımı

**Simple PID** kullanılacak (DriveToTag benzeri):
- X ve Y eksenleri için ayrı PID controller
- Hedef pozisyona ulaşana kadar velocity komutu
- Tolerance: 0.15m pozisyon, 3° açı

## WPILib Command Entegrasyonu

- `CommandScheduler.run()` hala çalışmaya devam eder
- Subsystem `periodic()` metodları çalışır
- TURN action için mevcut TurnToAngle command'ı kullanılır
- NAVIGATE action için RobotCore içinde PID kontrolü yapılır

## Verification

1. Build: `./gradlew build`
2. Deploy: `./gradlew deploy`
3. Test:
   - Teleop: Gamepad ile sürüş kontrol edilir
   - Auto: Waypoint'ler takip edilir
   - Shooter: RPM ayarları çalışır
