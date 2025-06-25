# STM32 Communication Reproduction - Arduino Code

Bu Arduino kodu, yeni ayrı paket yapısıyla STM32 haberleşmesini reproduce eder.

## Paket Yapıları

### Structure 1 (ControlMessage) - 2 byte
- `comStatus` (1 byte): İletişim durumu (0=pasif, 1=aktif)
- `calibStatus` (1 byte): Kalibrasyon durumu (0=normal, 1=kalibrasyon_modu)

### Structure 2 (JointMessage) - 6 byte  
- `control_bits` (1 byte): Kontrol bitleri
- `j1p` (1 byte): Joint 1 pozisyon
- `j1s` (1 byte): Joint 1 hız
- `j2p` (1 byte): Joint 2 pozisyon
- `j2s` (1 byte): Joint 2 hız
- `ap` (1 byte): Airsoft güç

## Çalışma Mantığı

1. **Gelen Veri Tespiti**: Arduino, serial porttan gelen veriyi okur
2. **Paket Türü Belirleme**: 
   - 2 byte gelirse → Structure 1 (ControlMessage)
   - 6 byte gelirse → Structure 2 (JointMessage)
3. **Echo Response**: Gelen paketi aynı şekilde geri gönderir
4. **Özel Durum**: Eğer `calibStatus == 1` ise, 2 saniye bekler

## Kullanım

Bu kod PC'deki yeni UART communication sistemiyle test edilebilir:

```bash
# Terminal 1: UART communication node'unu başlat
rosrun gokhas_communication uart_com.py

# Terminal 2: Test mesajları gönder  
python3 /home/cyhunblr/bitirme_ws/test_separate_packets.py
```

## Önemli Notlar

- Arduino kodu, paket türünü otomatik olarak algılar
- Timeout mekanizması ile yarım kalan paketler temizlenir
- `calibStatus=1` durumunda 2 saniye sleep özelliği mevcuttur
- Her paket türü ayrı ayrı işlenir ve geri gönderilir
