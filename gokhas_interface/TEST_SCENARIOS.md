# GokHAS Interface Test Senaryoları

## Test Ortamı Hazırlığı
1. **Terminal 1**: `roscore`
2. **Terminal 2**: `roslaunch gokhas_interface interface.launch`
3. **Terminal 3**: `rosrun gokhas_communication uart_com.py`

## Test Senaryoları - Simulation Mode: FALSE (Gerçek STM32)

### 🔴 Test 1: Sistem Başlatma ve Konfigürasyon Doğrulama
**Amaç**: Konfigürasyonun doğru yüklendiğini ve STM32 bağlantısının kurulduğunu doğrulama

**Kontrol Edilecekler**:
- [ ] Interface terminalinde: `STM32 Simulation Mode: DISABLED`
- [ ] UART terminalinde: `Serial port opened successfully: /dev/ttyUSB1 @ 9600 baud`
- [ ] Sürekli gelen `comStatus=0` mesajları
- [ ] Arayüz LOG kısmında: Başlangıç mesajları

**Beklenen Sonuç**: Sistem başarıyla başlar, STM32 bağlantısı kurulur

---

### 🟡 Test 2: Sistem Aktivasyonu
**Amaç**: Sistem aktivasyonunun gerçek STM32 ile çalıştığını doğrulama

**Test Adımları**:
1. Kırmızı "DEACTIVATED" butonuna bas
2. 5 saniye bekle (activation timeout)

**Kontrol Edilecekler**:
- [ ] Interface terminalinde: `Activation status changed: ACTIVATED`
- [ ] Interface terminalinde: `Activation timeout started (5.0 seconds)`
- [ ] UART terminalinde: `Control command received: comStatus=1, calibStatus=0`
- [ ] UART terminalinde: `Transmitted 8 bytes to STM32`
- [ ] STM32'den gelen yanıt: `comStatus=1`
- [ ] Buton yeşil "ACTIVATED" duruma geçer
- [ ] Arayüz LOG'unda: Aktivasyon mesajları

**Beklenen Sonuç**: Sistem STM32 ile iletişim kurarak aktive olur

---

### 🔵 Test 3: Joint Kontrol Testi
**Amaç**: Joint komutlarının STM32'ye gönderildiğini doğrulama

**Test Adımları**:
1. Sistem aktivasyonu sonrası
2. Herhangi bir joint butonuna bas (örn: J1, J2, J3)
3. Joint slider'larını hareket ettir

**Kontrol Edilecekler**:
- [ ] UART terminalinde: `Joint command received:` mesajları
- [ ] UART terminalinde: `Transmitted 8 bytes to STM32`
- [ ] Joint feedback mesajları
- [ ] Arayüz LOG'unda: Joint komut mesajları

**Beklenen Sonuç**: Joint komutları STM32'ye iletilir ve feedback alınır

---

### 🟢 Test 4: Manual/Auto Mode Değişimi
**Amaç**: Mode değişikliklerinin STM32'ye iletildiğini doğrulama

**Test Adımları**:
1. Sistem aktif iken
2. "MANUAL" butonuna bas → "AUTO" olmalı
3. Tekrar bas → "MANUAL" olmalı

**Kontrol Edilecekler**:
- [ ] Mode butonunun durumu değişir
- [ ] UART terminalinde mode komutları
- [ ] Arayüz LOG'unda mode değişiklik mesajları

**Beklenen Sonuç**: Mode değişiklikleri STM32'ye iletilir

---

### 🟠 Test 5: Kalibrasyon Testi
**Amaç**: Kalibrasyon komutlarının çalıştığını doğrulama

**Test Adımları**:
1. Sistem aktif iken
2. Herhangi bir joint'in "CAL" butonuna bas
3. 5 saniye bekle (calibration duration)

**Kontrol Edilecekler**:
- [ ] UART terminalinde: `calibStatus=1` içeren mesajlar
- [ ] Kalibrasyon timeout mesajları
- [ ] Buton animasyonları (pulse effect)
- [ ] Arayüz LOG'unda: Kalibrasyon mesajları

**Beklenen Sonuç**: Kalibrasyon komutları STM32'ye iletilir

---

### 🔴 Test 6: Sistem Deaktivasyonu
**Amaç**: Sistem kapatma işleminin doğru çalıştığını doğrulama

**Test Adımları**:
1. Sistem aktif iken
2. Yeşil "ACTIVATED" butonuna bas

**Kontrol Edilecekler**:
- [ ] UART terminalinde: `comStatus=0` komutu gönderimi
- [ ] STM32'den `comStatus=0` yanıtları
- [ ] Buton kırmızı "DEACTIVATED" duruma geçer
- [ ] Joint butonları deaktive olur
- [ ] Arayüz LOG'unda: Deaktivasyon mesajları

**Beklenen Sonuç**: Sistem güvenli şekilde deaktive olur

---

### ⚡ Test 7: Error Handling ve Timeout Testi
**Amaç**: Hata durumlarının doğru işlendiğini doğrulama

**Test Adımları**:
1. STM32'yi fiziksel olarak çıkar
2. Aktivasyon butonuna bas
3. 5+ saniye bekle

**Kontrol Edilecekler**:
- [ ] Interface terminalinde: `STM32 communication timeout`
- [ ] UART terminalinde: Serial port hata mesajları
- [ ] Buton tekrar kırmızı duruma döner
- [ ] Arayüz LOG'unda: Timeout ve hata mesajları

**Beklenen Sonuç**: Sistem hata durumlarını graceful olarak yönetir

---

## Test Çalıştırma Sırası

Lütfen şu sırayla testleri yapın ve her test için hem terminal çıktılarını hem de arayüz LOG mesajlarını paylaşın:

### Şimdi Başlayalım:

1. **Terminalleri hazırlayın**:
   ```bash
   # Terminal 1
   roscore
   
   # Terminal 2  
   cd ~/bitirme_ws && source devel/setup.bash
   roslaunch gokhas_interface interface.launch
   
   # Terminal 3
   cd ~/bitirme_ws && source devel/setup.bash  
   rosrun gokhas_communication uart_com.py
   ```

2. **Test 1'i başlatın**: Sistem başlatma ve konfigürasyon doğrulama

Terminalleri hazırladıktan sonra **Test 1**'in sonuçlarını paylaşın, sonra diğer testlere geçeriz.
