# BabaV8 Uçuş Bilgisayarı

Saraç Roket Takımı tarafından ATmega2560-16AU tabanlı BabaV8 Pro Max kartı için geliştirilen uçuş yazılımıdır.

Kod; sensörlerin okunması, uçuş durumunun belirlenmesi, telemetri paketinin hazırlanması, microSD kaydı ve kurtarma sisteminin yönetilmesi görevlerini tek kart üzerinde yürütür.

## Donanım

- ATmega2560-16AU, harici 16 MHz kristal
- MPL3115A2 barometrik basınç sensörü
- BNO055 yönelim ve ivme sensörü
- Quectel L86-M33 GPS
- Ebyte E220-400T30S telemetri modülü
- microSD kart
- CH340C USB-UART dönüştürücü

## Seri Hatlar

| Hat | Görev | Baudrate |
|---|---|---:|
| Serial | CH340C / seri monitör | 115200 |
| Serial1 | Ebyte telemetri | 115200 |
| Serial2 | GPS | 9600 |
| Serial3 | RS232 / SIT-SUT | 115200 |

## Gerekli Kütüphaneler

- Adafruit MPL3115A2 Library
- Adafruit BNO055
- Adafruit Unified Sensor
- SdFat
- TinyGPSPlus

## Kullanım

1. `SaraçRoketTakımıBABAV8Algoritma.ino` ve `rocket_frame.h` dosyalarını aynı sketch klasöründe tutun.
2. Arduino IDE üzerinden kart olarak **Arduino Mega or Mega 2560** seçin.
3. Gerekli kütüphaneleri kurun.
4. Kodu derleyip BabaV8 kartına yükleyin.
5. Seri monitörü 115200 baud ile açın.

## Uyarı

Kurtarma çıkışları enerjili donanım bağlanmadan önce masaüstü test düzeneğinde doğrulanmalıdır. Uçuş parametreleri araç yapısına ve saha koşullarına göre kontrol edilmeden kullanılmamalıdır.

---

**Saraç Roket Takımı**  
Göğe Yerden Çıkıldığını Unutma
