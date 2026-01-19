---
description: 
---

# Auto Klasörü Refaktör Görevi

## Amaç
`auto` klasöründeki özel scheduler yapısını kaldırarak, WPILib'in native command scheduler'ını kullanacak şekilde yeniden yazmak.

## Görevler

- [/] Auto klasörünü analiz et
  - [x] Mevcut dosyaları incele (AutoAction, AutoExecutor, AutoRoutine vb.)
  - [x] Mevcut DriveToPose PID implementasyonlarını incele
  - [x] Controller yapısını anla
  - [x ] PathPlanner API'sini araştır

- [ x] Yeni dosya yapısını tasarla
  - [x ] controller/auto altında yeni yapıyı tanımla
  - [ x] Command-based mimariyi planla FluentBuilder yapılı olsun

- [ ] Implementasyon
  - [ x] AutoCommands.java - WPILib command factory
  - [ ] DriveToPoseCommand.java - bu mevcut bunun üzerine kodu taz
  - [x ] PathFollowerCommand.java - PathPlanner entegrasyonu
  - [ x] AutoController.java - Basitleştirilmiş controller
  - [x ] AutoConstants.java - Sadeleştirilmiş sabitler

- [ x] Eski dosyaları sil Yeni yapı için kullanılmayacaksa sil 
  - [ x] auto/AutoAction.java
  - [ x] auto/AutoExecutor.java
  - [ ] auto/AutoRoutine.java
  - [ ] autoExampleAutos.java

