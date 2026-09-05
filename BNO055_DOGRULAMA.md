# BabaV8 — güncel çalışma, tilt ve doğrulama notları

## DİKKAT: pyro artık aktif olabilir
Bu sürümde diagnostik mod, erken dönüş, BNO eksen görüntüleyici, VIS çıktısı
ve USB R referans komutu kaldırılmıştır. ANA_ALG, SIT_MOD ve SUT_MOD içinde
mevcut kurtarma koşulları sağlandığında P_CH=LOW, N_CH=HIGH uygulanır.
Sadece moda girmek veya state 3'e geçmek ateşleme değildir.

Tek paraşüt kurtarması: anlık irtifa kesin olarak DamValue=1300 m üzerindeyken
80° açı koşulu VEYA FallDam=4 m ile tanımlanan düşüş sayacının FallCounter=5'i
aşması kurtarmayı başlatır. 1300 m ve altında iki koşul da yeni ateşleme başlatmaz.
Açı tek başına yeterlidir; gerçek tepe noktasından önce de komut verebilir.
Eski apogee bayrağı artık bu algoritmik kurtarma kararını temsil eder.
1200 m ikinci paraşüt aşaması kaldırıldı; sürüklenme/alt irtifa durum bitleri
uyumluluk için daima sıfırdır. Ana paraşüt biti kurtarma komutuyla birlikte set edilir;
bu bit fiziksel paraşüt açılmasını doğrulamaz. Paket boyutu/sırası değişmedi.
SIT artık gerçek sensör verisiyle bu algoritmayı ve state güncellemesini de çalıştırır.
SUT, RS232 üzerinden gelen simülasyon değerlerini kullanır.

**Çıkış 5000 ms süreli tek darbedir:** süre dolduğunda ilk döngüde kapanır.
Tekrarlanan çağrılar süreyi uzatmaz; yeni darbe ancak bayrak sıfırlamasından sonra mümkündür.
Sıfırlama komutları devam eden darbeyi erken keser. Bloklayan delay kullanılmaz;
kapanma hassasiyeti ana döngü gecikmesine bağlıdır.
5 saniyenin donanıma uygunluğu, akım sınırı, bağımsız arming ve arıza güvenliği
donanımda doğrulanmadı. Ana algoritmanın mevcut örnekleme aralığı 1000 ms,
SIT'in 100 ms olduğundan örnek sayısına bağlı düşüş doğrulama gecikmeleri farklıdır.
Bunlar doğrulanmadan sürüm uçuşa hazır veya sertifikalı kabul edilmemelidir.

Karta yükleme ve fiziksel ateşleme testi yapılmadı. Masa testlerinde enerjik yükleri
ayırın; çıkışı uygun yapay yük ve ölçüm donanımıyla yetkin gözetim altında doğrulayın.

## SIT/SUT ve haberleşme
- Başlangıç/kalibrasyon mesajları korunur.
- Ardından USB Serial ve LoRa Serial1 aynı 60-byte binary paketi gönderir.
- Her iki UART 115200; telemetri aralığı 200 ms (5 Hz).
- USB akışında artık ANA/VIS/RAW/TELEM gibi sürekli metinler yoktur.
- GPS Serial2 ve RS232 Serial3 baudları, RS232 header/footer/checksum korunur.
- Serial3 komut okuması her normal döngüde çalışır; test modu engeli yoktur.
- Mevcut CMD_DURDUR komutu ANA_ALG'a döner ve bayrakları sıfırlar.
  Kalıcı disarm komutu değildir; ana algoritma sonraki verilerle tekrar tetikleyebilir.
- USB üzerinden arayüz Port 1'e bağlanmadan Arduino Serial Monitor'ü kapatın.
- Başlangıç referansı alınırken roket dik ve hareketsiz tutulmalıdır.

## State 3
Arayüzde 3 = Burnout Geçildi.
Uçuş algılanınca 1→2; mevcut MOTOR_SURE_MS=5000 dolunca 2→3;
kurtarma komutuyla 1/2/3→4 (Paraşüt Komutu Verildi);
5 saniyelik darbe tamamlanıp düşüş göstergesi mevcut olduğunda 4→5 (İniş Aşaması).
State 3 gerçek motor sönmesi ölçümü değil, kalkış algılamasından başlayan süre tahminidir.
Motor süresini gerçek motor verisiyle ayrıca doğrulayın.

## A–I: Eksen ve açı açıklaması
A. Eski eşleme 0x09/0x00 idi; ekran X←BNO Y, ekran Y←BNO Z, ekran Z←BNO X.
   Önceki X'in burun olduğu varsayımı kullanıcının masa gözlemiyle uyuşmadı.
B. Ekran X/Y/Z düzeni korunuyor. Donanım remap 0x24/0x00 (native);
   0x09/0x00 ekran dönüşümü yazılımda bir kez uygulanıyor. Modlara göre farklı
   sensör okuma yolu artık yok.
C. Kullanıcı gözlemine göre ekran Z boyuna eksendir. Gövde adlandırması
   X_R=Z_ekran, Y_R=X_ekran, Z_R=Y_ekran olarak ayrılır. Kesin pozitif yönler
   fiziksel işaretlerle hâlâ doğrulanmalıdır; eksenel dönüş gözlemi işareti kanıtlamaz.
D. BNO VECTOR_EULER x/y/z sırası Heading/Roll/Pitch'tir; bunlar vektör gibi permüte
   edilmez. Kod quaternion'dan referansa göre ZYX Euler hesaplar.
   angleDeltaX/Y/Z ve telemetrideki eski GyroX/Y/Z slotları derece taşır,
   açısal hız değildir. Paket alanlarının sırası ve işaretli değerleri korunmuştur.
E. Ekran X çevresindeki yatış tilt'i artırır; yön işareti fiziksel testle kontrol edilir.
F. Ekran Y çevresindeki yatış aynı şekilde tilt'i artırır.
G. Ekran Z çevresindeki eksenel dönüş Z değerini değiştirir, tilt'i tek başına artırmaz.
   Euler açıları ±90° pitch yakınında tekildir; tilt bu ayrışıma dayanmaz.
H. q_relative = conjugate(q_reference) * q_current; ekran bazına dönüştürülür.
   tilt = acos(clamp(1 - 2*(q_relative.x² + q_relative.y²), -1, 1)) × 180/π.
   Bu, başlangıç ve güncel ekran Z burun doğrultularının arasındaki 0–180° açıdır.
   Başlangıç dik değilse gerçek düşeyden değil başlangıç yönünden sapmayı ölçer.
I. Eksenel dönüş burun vektörünü değiştirmediği için ideal matematikte tilt sabittir.
   Başlangıç referansı q/-q işaret hizalamalı quaternion ortalamasıyla alınır;
   359/0 derece aritmetik ortalama sorunu yoktur.
   ANA/SIT açı eşiği tiltValid && tiltAngle>=80 kullanır.
   SUT quaternion taşımadığından eski X/Y açı yorumu ve filtreleri korunmuştur.

Geçersiz quaternion/referans halinde tilt NaN olur; açı koşulu sağlanmaz ve
mevcut binary yazıcı o snapshot'ı göndermez. Bu davranış tek başına sensör arızası
yönetimi değildir. Sensör sağlığı, kalibrasyon, manyetik parazit ve uçuş ivmeleri
için donanımlı doğrulama gerekir.

## J: Masa doğrulaması
1. Enerjik yükleri ayırın; önceki sürüm yedeklerini saklayın.
2. Dik ve hareketsiz başlangıç referansı alın. USB'de açılış mesajlarını,
   ardından arayüzde binary paketleri kontrol edin.
3. Ekran X ve Y yönlerinde ayrı ayrı 30/60/90° yatırın; tilt aynı açılara yaklaşmalı.
4. Dikeyken Z yönünde 90/180° eksenel döndürün; tilt yaklaşık 0 kalmalı.
5. 30° yatışta aynı eksenel dönüşü yapın; tilt yaklaşık 30 kalmalı.
6. RS232 SIT ve SUT başlatma/geçiş komutlarını, checksum reddini ve veri akışını test edin.
7. Yapay yükle, eşik altı/yükseliş verisinin ateşlemediğini; gerekli düşüş
   dizisinin çıkışı etkinleştirdiğini; reset komutunun mandalı temizlediğini doğrulayın.
8. 2→3→4→5 state akışını ve gerçek motor süresine uygunluğu kontrol edin.
9. Ateşleme darbe süresi/arming/arıza davranışı ve tüm uçuş koşulları ayrıca
   mühendislik doğrulamasından geçmeden enerjik uçuş testi yapmayın.

```text
                    +X_R = boyuna doğrultu (ekran Z)
                           BURUN
                             ↑
                           [UKB] ──→ +Y_R (ekran X)
                             ↓
                           KUYRUK
+Z_R (ekran Y): çizimin içine; X_R × Y_R = Z_R.
Pozitif burun ve radyal işaretleri fiziksel olarak doğrulanmalıdır.
```

## Doğrulama kapsamı
FlightFirmwareRegression.Tests.ps1 güncel INO'dan fonksiyonları her çalışmada
çıkarır; saat/sensör/seri port/dijital pinleri taklit ederek yazılım testleri yapar.
Gerçek Serial3 parser'ı, mod döngüsü, kurtarma hesabı, state geçişleri ve
paket yazıcı fonksiyonları kullanılır. Bu test gerçek UART/RF veya ateşleyiciyi ölçmez.
Yedekler bu klasörde *.backup-flight-* ve önceki *.backup-* adlarıyla durur.
