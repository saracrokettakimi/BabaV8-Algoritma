// Saraç Roket Takımı 2026 - BABA V8 Pro Max Algoritma
// TEKNOFEST Roket Yarışması A1 Lise Kategorisi
// Göğe Yerden Çıkıldığını Unutma 
// Author: Yusuf Yağız KOÇ /-/ "You do your best when you're at your lowest."



// Serial0  = CH340C (TypeC)         - Pin 0(RX0/PE0), 1(TX0/PE1)
// Serial1 = LoRa Telemetri (E220-400T30S)     - Pin 19(RX1/PD2), 18(TX1/PD3)
// Serial2 = GPS (Quectel L86-M33)             - Pin 17(RX2/PH0), 16(TX2/PH1)
// Serial3 = RS232                             - Pin 15(RX3/PJ0), 14(TX3/PJ1)
// I2C     = MPL3115A2(Barometrik İrtifa) & BNO055(Açı/İvme) Pin 20(SDA/PD1), 21(SCL/PD0)

//Kütüphaneler
#include <Adafruit_MPL3115A2.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SdFat.h>
#include <TinyGPSPlus.h> 
#include <avr/wdt.h>     
#include <stdlib.h>
#include "rocket_frame.h"

// PCB üzerindeki bağlantılar
#define BUZZER_PIN     5    // PE3 (BUZZ)
#define RED_LED_PIN    6    // PH3 (RGB_RED)
#define GREEN_LED_PIN  7    // PH4 (RGB_GREEN)
#define BLUE_LED_PIN   8    // PH5 (RGB_BLUE)
#define M1_PIN         36   // PC1 (M1_MCU)
#define M0_PIN         37   // PC0 (M0_MCU)
#define P_CH_PIN       2    // PE4 (P_CH)
#define N_CH_PIN       3    // PE5 (N_CH)
#define SD_CS_PIN      53   // PB0 (MICROSD)

//Uçuş Algoritması Atamaları
#define FallCounter    5 // Uçuşun Kurtarmasında etkili düşüş sayacı.
#define DamValue       1300.0f //Kurtarma algoritmasının çalışacağı min irtifa.
#define FallDam        4.0f // Düşüş sayacını başlatıcak min yükselik.
#define AltWhenFly     100.0f // Uçuşun algılanması için gerekli minimum irtifa.
#define ACI_ESIK       80.0f // Uçuşun kurtarmaya etkili açı eşiği.


// SIT-SUT Atamaları
#define MOTOR_SURE_MS  5000    // SUT için kalkıştan sonra motor bitti sayacağımız süre, ölçüm değil zaman hesabı
#define PROTO_HEADER    0xAA
#define PROTO_FOOTER1   0x0D
#define PROTO_FOOTER2   0x0A
#define CMD_SIT         0x20
#define CMD_SUT         0x22
#define CMD_DURDUR      0x24
#define KOMUT_PAKET_BOY 5

#define TELEM_HEADER    0xAB
#define TELEM_PAKET_BOY 36

// Millis ile sıra bekleyen işler, birbirinin veri akışını durdurmasın
#define SIT_INTERVAL_MS 100
#define SIT_BASLAMA_MS  1000
#define DURUM_INTERVAL_MS 100
#define ANA_ALG_INTERVAL_MS 1000
#define PYRO_PULSE_MS 5000UL
#define TELEMETRI_INTERVAL_MS 200   // Telemetri gönderme hızı
#define TELEMETRI_BUFFER_BOY 256
// Paket 60 byte, float metne dönüşmüyor direkt 4 byte gidiyor
const size_t TELEMETRI_BINARY_FRAME_SIZE = 60;
static_assert(sizeof(float) == 4, "Telemetry protocol requires 4-byte float");
#define RX_PARTIAL_TIMEOUT_MS 100
#define BARO_OSR_BITS MPL3115A2_CTRL_REG1_OS4

#define GPS_BAUD        9600

// SUT durum paketinde her olayın biti ayrı
#define BIT0_KALKIS          0x01
#define BIT1_MOTOR_SURE      0x02
#define BIT2_MIN_IRTIFA      0x04
#define BIT3_GOVDE_ACI_G     0x08
#define BIT4_IRTIFA_ALCAL    0x10
#define BIT5_SURUKLEME_PAR   0x20 // a1 kategorisi için yok
#define BIT6_IRTIFA_ALTI     0x40 // a1 kategorisi için yok
#define BIT7_ANA_PAR         0x80
#define DATA2_REZERVE        0x00


Adafruit_MPL3115A2 baro; // Barometre ataması
Adafruit_BNO055    bno = Adafruit_BNO055(55, 0x28); //Gyro sensör ataması
TinyGPSPlus        gps; // Gps atama
SdExFat            sd;
ExFile             dataFile;
bool               bnoHazir = false;

// Hangi moddayız burada tutuluyor, açılışta ana algoritmadayız
enum TestModu { BEKLEME, SIT_BEKLE, SIT_MOD, SUT_BEKLE, SUT_MOD, ANA_ALG };
TestModu aktifMod = ANA_ALG;

// RS232 tek seferde gelmek zorunda değil, yarım pakette kaldığımız yeri tutuyoruz
enum Rs232RxState { WAIT_HEADER, READ_COMMAND, READ_TELEMETRY };
Rs232RxState rxState = WAIT_HEADER;

// MPL bir irtifa bir basınç okuyacak, en son tamamlanan değerleri saklıyoruz
enum BarometreOrnekTuru { BARO_IRTIFA_ORNEGI, BARO_BASINC_ORNEGI };
BarometreOrnekTuru etkinBarometreOrnegi = BARO_IRTIFA_ORNEGI;
BarometreOrnekTuru siradakiBarometreOrnegi = BARO_IRTIFA_ORNEGI;
bool barometreHazir = false, barometreDonusumuSuruyor = false;
float sonBarometreIrtifa = 0.0f, sonBarometreBasinc = 0.0f;

// Yer istasyonuna gidecek son örnek, gönderme sırasında alanlar karışmasın
struct TelemetrySnapshot {
    float altitude;
    float gpsAltitude;
    float gpsLatitude;
    float gpsLongitude;
    float gyroX;
    float gyroY;
    float gyroZ;
    float accX;
    float accY;
    float accZ;
    int flightState;
    int satelliteCount;
    float totalAngle;
    unsigned long sampleTimestamp;
};


// BNO'dan tek okumada gelen yönelim, açı ve yerçekimi çıkarılmış ivmeler
struct BodyImuSample {
    RocketQuaternion orientation;
    float angleX;
    float angleY;
    float angleZ;
    float accelX;
    float accelY;
    float accelZ;
    bool valid;
};

TelemetrySnapshot telemetrySnapshot = {};
bool telemetrySnapshotHazir = false;

// Her işin son çalışma zamanı ayrı, biri çalışırken diğerinin süresi kaybolmasın
unsigned long sitBaslamaZamani = 0, sutBaslamaZamani = 0;
unsigned long sonSitGonderim = 0, sonDurumGonderim = 0, sonAnaAlgGonderim = 0;
unsigned long sonTelemetriOrnekleme = 0, sonTelemetriGonderim = 0;
unsigned long kalkisZamani = 0;

// 30 sn içinde sit sut verisi gelmezse ANA_ALG'a geçme kodu
#define OTOMATIK_ANA_ALG_TIMEOUT_MS 30000UL
unsigned long sonKomutZamani = 0;

// buzzer sweep
bool buzzerAktif = false;
unsigned long sonBuzzerDegisim = 0;
bool buzzerYuksekFaz = false;

// Uçuşta gerçekleşen olaylar, sıfırlama gelene kadar bayraklarda tutuluyor
bool kalkis_algilandi = false, motor_sure_doldu = false, min_irtifa_asild = false;
bool govde_aci_g_tamam = false, irtifa_alcalmaya = false, surukleme_par = false;
bool irtifa_alt_indi = false, ana_parasut = false;
bool donme_tamam = false, irtifa_tamam = false, kurtarma_aktif = false;
// Pyro bir kere başlasın, her döngüde baştan 5 saniye saymasın
bool pyroPulseStarted = false;
bool pyroPulseFinished = false;
unsigned long pyroPulseStartMs = 0;

float ilkirtifa = 0, irtifa = 0, apogeeAlt = 0, bottomAlt = 0;
int   FallingCheck = 0;
// Açılışta roketin duruşunu referans alıyoruz, sonraki açılar buna göre
RocketQuaternion imuReference = {1,0,0,0};
bool imuReferenceValid = false;
float angleDeltaX = 0, angleDeltaY = 0, angleDeltaZ = 0;
float tiltAngle = NAN;
bool tiltValid = false;
BodyImuSample sonImuOrnegi = {};
bool  apogee = false;
int   ucusDurumu = 1;

// Kalman hafızası, irtifa ve SUT açıları için ayrı ayrı hesap tutuluyor
float kalman_X = 0.0f, kalman_P = 1.0f, kalman_Q = 0.01f, kalman_R = 0.5f, kalman_K = 0.0f;
float kalman_aciX_X = 0.0f, kalman_aciX_P = 1.0f, kalman_aciX_Q = 0.01f, kalman_aciX_R = 0.5f, kalman_aciX_K = 0.0f;
float kalman_aciY_X = 0.0f, kalman_aciY_P = 1.0f, kalman_aciY_Q = 0.01f, kalman_aciY_R = 0.5f, kalman_aciY_K = 0.0f;

// SUT'ta sensör yerine test cihazından gelen değerleri
float sut_irtifa = 0, sut_basinc = 0;
float sut_ivmeX = 0, sut_ivmeY = 0, sut_ivmeZ = 0;
float sut_aciX = 0, sut_aciY = 0, sut_aciZ = 0;
bool  sutVeriHazir = false;

// Gelen byte'ları paket tamamlanana kadar bu tamponda biriktiriyoruzki paket kaybı en az olsun
uint8_t rxBuf[TELEM_PAKET_BOY], rxIdx = 0;
unsigned long sonRxByteZamani = 0;


// Fonksiyonları aşağıda yazıyoruz, burada isimlerini derleyiciye tanıtıyoruz
void komutVeVeriDinle();
void rxCercevesiBaslat(uint8_t header, unsigned long simdi);
void rxBeklemeyeDon();
void gecersizTelemetriSonekKurtar(unsigned long simdi);
bool komutPaketIsle(uint8_t* buf);
bool telemPaketIsle(uint8_t* buf);
bool checksumDogrula_Komut(uint8_t cmd, uint8_t cs);
bool checksumDogrula_Telem(uint8_t* buf);
void sit_calistir();
void sut_calistir();
void anaAlgoritma_calistir();
void algoritmaKontrol(float anlikIrtifa, float anlikAciX, float anlikAciY);
void calcOffset();
void barometreyiCalistir();
bool bnoRoketEksenleriniAyarla();
BodyImuSample bodyImuOku();
void readIMU();
float applyKalmanFilter(float measurement);
float applyKalmanFilterAngleX(float measurement);
float applyKalmanFilterAngleY(float measurement);
void durumBitleriniGuncelle();
void ucusDurumuGuncelle();
void durumPaketiGonder();
void bayraklariSifirla();
void telemPaketiGonder(float irtifaVal, float basincVal, float ivmeX, float ivmeY, float ivmeZ, float aciX, float aciY, float aciZ);
void floatYazBinary(uint8_t* buf, int& idx, float val);
float floatOku(uint8_t* buf, int idx);
uint8_t checksumHesapla(uint8_t* buf, int len);
const char* modToString(TestModu m);
void logToSD(float logIrtifa, float logApogee);
void gpsDinle();
void normalTelemetriOrnekle();
void telemetrySnapshotGuncelle(float altitude, float gyroX, float gyroY, float gyroZ,
                               float accX, float accY, float accZ, float totalAngle);
void telemetriGonderiminiCalistir();
bool yerIstasyonunaGonder(const TelemetrySnapshot& snapshot);
bool telemetriKarakterEkle(char* buffer, size_t kapasite, size_t& uzunluk, char deger);
bool telemetriMetinEkle(char* buffer, size_t kapasite, size_t& uzunluk, const char* deger);
bool telemetriFloatEkle(char* buffer, size_t kapasite, size_t& uzunluk, float deger, uint8_t hassasiyet);
bool telemetriIntEkle(char* buffer, size_t kapasite, size_t& uzunluk, long deger);
void buzzerAc(int freq = 2000);
void buzzerKapat();
void buzzerServisi();
void pyroGuvenliTut();


void buzzerAc(int freq) {
    // freq eski çağrılar için duruyor, burada iki notalı alarm başlatılıyor
    (void)freq; 
    buzzerAktif = true;
    buzzerYuksekFaz = false;
    sonBuzzerDegisim = millis();
    tone(BUZZER_PIN, 523); 
}
void buzzerKapat() {
    // Hem alarm bayrağını indir hem pindeki sesi kes
    buzzerAktif = false;
    noTone(BUZZER_PIN);
}

void buzzerServisi() {
    // 400ms'de bir nota değiştir, uçuş döngüsünü delay ile bekletme
    if (!buzzerAktif) return;
    constexpr unsigned long FASE_MS = 400UL;  
    constexpr uint16_t FREQ_LO = 523;         
    constexpr uint16_t FREQ_HI = 659;         
    if ((unsigned long)(millis() - sonBuzzerDegisim) >= FASE_MS) {
        sonBuzzerDegisim = millis();
        buzzerYuksekFaz = !buzzerYuksekFaz;
        tone(BUZZER_PIN, buzzerYuksekFaz ? FREQ_HI : FREQ_LO);
    }
}
void pyroGuvenliTut() {
    // pyro güvenliği için başlangıçta 2 hatta low
    digitalWrite(P_CH_PIN, LOW);
    digitalWrite(N_CH_PIN, LOW);
}
void pyroAtesle() {
    // pyro hattı 5 saniye boyunca açık kalacak
    const unsigned long now = millis();
    if (!pyroPulseStarted) {
        pyroPulseStarted = true;
        pyroPulseStartMs = now;
    }
    if ((unsigned long)(now - pyroPulseStartMs) >= PYRO_PULSE_MS) {
        // Süre dolduysa kapalı kal, yeniden başlamak için bayrak sıfırlanmalı
        pyroPulseFinished = true;
    }
    digitalWrite(P_CH_PIN, LOW);
    digitalWrite(N_CH_PIN, pyroPulseFinished ? LOW : HIGH);
}

// led aktivasyon
void rgbAyarla(bool r, bool g, bool b) {
    digitalWrite(RED_LED_PIN,   r ? HIGH : LOW);
    digitalWrite(GREEN_LED_PIN, g ? HIGH : LOW);
    digitalWrite(BLUE_LED_PIN,  b ? HIGH : LOW);
}
void rgbKapat() { rgbAyarla(false, false, false); }

// buzzer aktivasyon
void aktivasyonSesi() {
    // açılış melodisi
    const uint16_t notalar[]  = {523, 659, 784, 1047};
    const uint16_t sureler[]  = { 90,  90,  90,  180};
    for (uint8_t i = 0; i < 4; i++) {
        tone(BUZZER_PIN, notalar[i]);
        delay(sureler[i]);
        noTone(BUZZER_PIN);
        delay(30);
    }

    delay(80);
    tone(BUZZER_PIN, 1047); delay(60); noTone(BUZZER_PIN);
    delay(40);
    tone(BUZZER_PIN, 1047); delay(60); noTone(BUZZER_PIN);
}


void setup() {
    // Önce pyro hatlarını pasife al, sensörlere ondan sonra geç
    pinMode(P_CH_PIN, OUTPUT);
    pinMode(N_CH_PIN, OUTPUT);
    pyroGuvenliTut();

    pinMode(RED_LED_PIN,   OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN,  OUTPUT);
    rgbKapat();



    // ATmega2560 spi master koruması internetten aldık
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);

    pinMode(M1_PIN, OUTPUT); pinMode(M0_PIN, OUTPUT);
    digitalWrite(M0_PIN, LOW); digitalWrite(M1_PIN, LOW); // E220 transparent modda
    delay(100);

    pinMode(BUZZER_PIN, OUTPUT);
    buzzerKapat();

    // donanımsal seri portlar
    Serial.begin(115200);  // usb (UART0)
    Serial1.begin(115200); // lora (UART1)
    Serial2.begin(GPS_BAUD); // gps (UART2)
    Serial3.begin(115200); // rs232 (UART3)

    // I2C 400 khz başlatma
    Wire.begin();
    Wire.setClock(400000);

    Serial.println(F("Algoritma Ekibinden Selamlar Yoldaş"));

  

    barometreHazir = baro.begin();
    if (!barometreHazir) {
        // Sensör başlamadıysa hatasını yaz, LED gösterisi bundan bağımsız
        Serial.println(F("Hançer MPL nerede"));
    } else {
        
        baro.write8(MPL3115A2_CTRL_REG1, BARO_OSR_BITS | MPL3115A2_CTRL_REG1_ALT);
        baro.setSeaPressure(1013.26);
    }
    bnoHazir = bno.begin();
    if (!bnoHazir) {
        Serial.println(F("Miraç BNO055'i nereye koydun"));
    } else {
        bnoRoketEksenleriniAyarla();
    }



    if (!sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(4)))) {
        // Kart yoksa veya SPI konuşamıyorsa nedenini serial monitöre yaz
        Serial.print(F("HATA: MicroSD baslatilamadi! Hata kodu: "));
        sd.initErrorPrint(&Serial);
    } else {
        Serial.println(F("MicroSD init OK"));
        if (!dataFile.open("FLIGHT.CSV", O_WRITE | O_CREAT | O_APPEND)) {
            // SD'nin başlaması yetmez, kayıt dosyası da açılmalı
            Serial.println(F("HATA: FLIGHT.CSV acilamadi!"));
        } else {
            dataFile.println(F("Timestamp,Mode,Irtifa,Apogee,GPS_Lat,GPS_Lon,GPS_Alt,GPS_Sat,State"));
            dataFile.flush();
            Serial.println(F("MicroSD hazir. FLIGHT.CSV acildi."));
        }
    }

    // Şimdi ilk irtifayı ve açı referansını alıyoruz
    calcOffset();
    telemetrySnapshotGuncelle(0.0f, 0.0f, 0.0f, 0.0f,
                              0.0f, 0.0f, 0.0f, 0.0f);

    // Direkt RGB, sensör sağlam demek değil sadece açılış gösterisi
    rgbAyarla(true, false, false);
    delay(500);
    rgbAyarla(false, true, false);
    delay(500);
    rgbAyarla(false, false, true);
    delay(500);
    rgbKapat();

    // ATmega2560 watchdog timer
    wdt_enable(WDTO_2S);

    // timer sıfırlama
    sonKomutZamani     = millis();
    sonAnaAlgGonderim  = 0UL;
    sonTelemetriOrnekleme = 0UL;
    sonTelemetriGonderim  = 0UL;


    // Kontroller ve kalibrasyon bitti sesi, hepsi sağlam demek değil hata mesajlarına da bak
    aktivasyonSesi();  // buzzer aktivasyon sesi
    rgbKapat();

    Serial.println(F("============================================"));
    Serial.println(F("BABAV8 KOMUTADA"));
    Serial.println(F("MOD: ANA_ALG"));
    Serial.println(F("Alt?Basn?GyX?GyY?GyZ?AcX?AcY?AcZ?Dur?Sat?TopAci"));
    Serial.println(F("============================================"));
}


void loop() {
    // Döngü çalışıyor, watchdog'a haber ver
    wdt_reset();


    // Pyronun sadece belirli şartlartla  ana_alg, sit ve sut modunda açık olması
    if (kurtarma_aktif && (aktifMod == SIT_MOD || aktifMod == SUT_MOD || aktifMod == ANA_ALG)) {
        pyroAtesle();
    } else {
        pyroGuvenliTut();
    }

    // Her tur gelen GPS ve RS232 byte'larını topla, ölçüm ve alarm işlerini ilerlet
    gpsDinle(); 
    komutVeVeriDinle();
    barometreyiCalistir();
    buzzerServisi(); 

    // 30 sn içinde sit/sut komutu gelmezse geri ana algoritmaya geç
    if (aktifMod == BEKLEME &&
        (unsigned long)(millis() - sonKomutZamani) >= OTOMATIK_ANA_ALG_TIMEOUT_MS) {
        aktifMod = ANA_ALG;
    }

    // Aktif moda göre gerçek sensörleri veya SUT verisini işle
    switch (aktifMod) {
        case BEKLEME: delay(10); break;
        case SIT_BEKLE:
            if (millis() - sitBaslamaZamani >= SIT_BASLAMA_MS) {
                aktifMod = SIT_MOD;
                sonSitGonderim = 0;
            }
            break;
        case SIT_MOD: sit_calistir(); break;
        case SUT_BEKLE:
            if (millis() - sutBaslamaZamani >= SIT_BASLAMA_MS) {
                aktifMod = SUT_MOD;
                sonDurumGonderim = 0;
            }
            break;
        case SUT_MOD: sut_calistir(); break;
        case ANA_ALG: anaAlgoritma_calistir(); break;
        default: delay(10); break;
    }

    if (aktifMod == ANA_ALG) normalTelemetriOrnekle();
    telemetriGonderiminiCalistir();
}
// BNO055 eksen ayarlama
bool bnoRoketEksenleriniAyarla() {
    // Sensör kendi ekseninde kalsın, roket eksenine dönüşüm header'da bir kere yapılacak
    if (!bnoHazir) return false;
    bno.setAxisRemap(static_cast<Adafruit_BNO055::adafruit_bno055_axis_remap_config_t>(0x24));
    bno.setAxisSign(static_cast<Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t>(0x00));
    bno.setExtCrystalUse(true);
    return true;
}

BodyImuSample bodyImuOku() {
    // Okuma bozuksa geçerli işaretlemiyoruz, yanlış açı üretmesin
    BodyImuSample sample = {};
    sample.valid = false;
    if (!bnoHazir) return sample;

    const imu::Quaternion q = bno.getQuat();
    // Quaternion yönelimi tutuyor, normalize edip eksen hesabına hazırlıyoruz
    sample.orientation = {static_cast<float>(q.w()), static_cast<float>(q.x()),
                          static_cast<float>(q.y()), static_cast<float>(q.z())};
    if (!rocketNormalize(sample.orientation)) return sample;
    const imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    const float nativeAccel[3] = {static_cast<float>(accel.x()),
        static_cast<float>(accel.y()), static_cast<float>(accel.z())};
    float bodyAccel[3];
    rocketMapVector(nativeAccel,bodyAccel);
    // İvmeyi de açıyla aynı ekran eksenlerine koy
    sample.accelX = bodyAccel[0]; sample.accelY = bodyAccel[1]; sample.accelZ = bodyAccel[2];
    if (!isfinite(sample.accelX) || !isfinite(sample.accelY) || !isfinite(sample.accelZ)) return sample;
    RocketAngles angles;
    if (imuReferenceValid && rocketRelativeAngles(sample.orientation,imuReference,
                                                   true,angles)) {
        sample.angleX = angles.roll; sample.angleY = angles.pitch; sample.angleZ = angles.yaw;
    }

    sample.valid = true;
    return sample;
}

// kalibrasyon baba
void calcOffset() {
    Serial.println(F("Kalibrasyon Başlıyor"));
    float baro_sum = 0;
    RocketQuaternionMean referenceMean;
    imuReferenceValid = false;
    tiltValid = false;
    tiltAngle = NAN;
    for (int i = 0; i < 10; i++) {
        // Başlangıç için 10 örnek topla, bu sırada roket sabit durmalı
        const BodyImuSample sample = bodyImuOku();
        if (sample.valid) {
            referenceMean.add(sample.orientation);
        }
        if (barometreHazir) baro_sum += baro.getAltitude();
        delay(50);
    }
    // BNO055'in euler eksenlerinin birden - ye inmesini engelleyen kod 
    imuReferenceValid = referenceMean.count >= 8 && referenceMean.finish(imuReference);
    // En az 8 geçerli yönelim yoksa açı referansını kabul etmiyoruz
    if (!imuReferenceValid) Serial.println(F("IMU reference INVALID: no valid tilt; restart stationary."));
    ilkirtifa = barometreHazir ? (baro_sum / 10.0f) : 0.0f;
    sonBarometreIrtifa = ilkirtifa;
    kalman_X = ilkirtifa;
    kalman_aciX_X = 0.0f; kalman_aciY_X = 0.0f;
    bottomAlt = 0;
    Serial.println(F("Kalibrasyon bitti yoldaş"));
}

void barometreyiCalistir() {
    // MPL ölçümü bitirmediyse burada bekleme, sonraki döngüde tekrar bak
    if (!barometreHazir) return;

    if (barometreDonusumuSuruyor) {
        if (!baro.conversionComplete()) return;

        if (etkinBarometreOrnegi == BARO_IRTIFA_ORNEGI) {
            sonBarometreIrtifa = baro.getLastConversionResults(MPL3115A2_ALTITUDE);
        } else {
            sonBarometreBasinc = baro.getLastConversionResults(MPL3115A2_PRESSURE);
        }
        barometreDonusumuSuruyor = false;
    }

    etkinBarometreOrnegi = siradakiBarometreOrnegi;
    // Bir sonraki tek ölçümü başlat, irtifa ve basınç sırayla gelsin
    if (etkinBarometreOrnegi == BARO_IRTIFA_ORNEGI) {
        baro.setMode(MPL3115A2_ALTIMETER);
        siradakiBarometreOrnegi = BARO_BASINC_ORNEGI;
    } else {
        baro.setMode(MPL3115A2_BAROMETER);
        siradakiBarometreOrnegi = BARO_IRTIFA_ORNEGI;
    }
    baro.startOneShot();
    barometreDonusumuSuruyor = true;
}

float applyKalmanFilter(float measurement) {
    // Yeni irtifayı önceki tahminle birleştirip ani oynamayı azaltıyoruz. Tek katmanlı kalman filtresi
    kalman_P += kalman_Q;
    kalman_K = kalman_P / (kalman_P + kalman_R);
    kalman_X += kalman_K * (measurement - kalman_X);
    kalman_P = (1.0f - kalman_K) * kalman_P;
    return kalman_X;
}

float applyKalmanFilterAngleX(float measurement) {
    // SUT X açısına ayrı Kalman, gerçek BNO açısı burada filtrelenmiyor
    kalman_aciX_P += kalman_aciX_Q;
    kalman_aciX_K = kalman_aciX_P / (kalman_aciX_P + kalman_aciX_R);
    kalman_aciX_X += kalman_aciX_K * (measurement - kalman_aciX_X);
    kalman_aciX_P = (1.0f - kalman_aciX_K) * kalman_aciX_P;
    return kalman_aciX_X;
}

float applyKalmanFilterAngleY(float measurement) {
    // Aynı hesabın SUT Y açısı için olanı, X ile hafızası karışmasın
    kalman_aciY_P += kalman_aciY_Q;
    kalman_aciY_K = kalman_aciY_P / (kalman_aciY_P + kalman_aciY_R);
    kalman_aciY_X += kalman_aciY_K * (measurement - kalman_aciY_X);
    kalman_aciY_P = (1.0f - kalman_aciY_K) * kalman_aciY_P;
    return kalman_aciY_X;
}

void readIMU() {
    // Referansa göre X Y Z ve toplam eğimi hesapla, arayüz hazır değeri alacak
    sonImuOrnegi = bodyImuOku();
    RocketAngles angles;
    tiltValid = sonImuOrnegi.valid && imuReferenceValid &&
        rocketRelativeAngles(sonImuOrnegi.orientation,imuReference,true,angles);
    if (!tiltValid) {
        // Geçersiz eğimi sıfır sanmayalım, NAN ile geçersiz olduğu belli olsun
        angleDeltaX = angleDeltaY = angleDeltaZ = 0.0f;
        tiltAngle = NAN;
        return;
    }
    angleDeltaX = angles.roll; angleDeltaY = angles.pitch; angleDeltaZ = angles.yaw;
    tiltAngle = angles.tilt;
}

void algoritmaKontrol(float anlikIrtifa, float anlikAciX, float anlikAciY) {
    // ANA ve SIT gerçek eğime bakar, SUT kendisine gönderilen X veya Y açısına bakar
    const bool aciEsigiAsildi = (aktifMod == ANA_ALG || aktifMod == SIT_MOD)
        ? (tiltValid && tiltAngle >= ACI_ESIK)
        : (anlikAciX >= ACI_ESIK || anlikAciY >= ACI_ESIK);
    digitalWrite(RED_LED_PIN, aciEsigiAsildi ? HIGH : LOW);
    // yalnızca açı karşılanıyorsa açılacak

    if (anlikIrtifa > DamValue) {
        // 1300m ve altında yeni kurtarma yok, bu kapı iki koşul için de geçerli
        if (aciEsigiAsildi) {
            donme_tamam = true;
        }
        // Yeni tepe varsa kaydet, eşik altına düşmeye başlayınca sayacı artır
        if (anlikIrtifa > apogeeAlt) { apogeeAlt = anlikIrtifa; bottomAlt = anlikIrtifa - FallDam; }
        if (bottomAlt > anlikIrtifa) { bottomAlt = anlikIrtifa; FallingCheck++; }
        else if (anlikIrtifa > bottomAlt) { if (FallingCheck > 0) FallingCheck = 0; }
        if (FallingCheck > FallCounter) {
            irtifa_tamam = true;
        }
        // 1300m üzerinde açı veya FallingCheck > FallCounter (şu an 5) ise kurtarma komutu verilir
        // Açı tek başına yeterli, apogee bayrağı burada fiziksel tepe ölçümü değil kurtarma kararı

        if (donme_tamam || irtifa_tamam) {
            apogee = true; kurtarma_aktif = true;
            pyroAtesle(); // P_CH=LOW, N_CH=HIGH  kurtarma aktif
            if (!buzzerAktif) buzzerAc(); 
        }
    } else if (anlikIrtifa < 100.0f) { bottomAlt = anlikIrtifa; }
}

void anaAlgoritma_calistir() {
    // Ana uçuş hesabı, irtifadan kalkış yerinin yüksekliğini çıkarıyoruz
    unsigned long simdi = millis();
    if (simdi - sonAnaAlgGonderim < ANA_ALG_INTERVAL_MS) return;
    sonAnaAlgGonderim = simdi;

    float rawAltitude = sonBarometreIrtifa;
    float filteredAltitude = applyKalmanFilter(rawAltitude);
    irtifa = filteredAltitude - ilkirtifa;
    readIMU();
    algoritmaKontrol(irtifa, angleDeltaX, angleDeltaY);
    durumBitleriniGuncelle();
    ucusDurumuGuncelle();

    logToSD(irtifa, apogeeAlt);
}

void sut_calistir() {
    // Yeni test paketi varsa bir kere işle, aynı paketi tekrar tekrar sayma
    if (sutVeriHazir) {
        sutVeriHazir = false;
        // SUT irtifasına da kalman uygulama kısmı
        float filteredSutIrtifa = applyKalmanFilter(sut_irtifa);
        irtifa = filteredSutIrtifa;
        float filteredSutAciX = applyKalmanFilterAngleX(fabsf(sut_aciX));
        float filteredSutAciY = applyKalmanFilterAngleY(fabsf(sut_aciY));
        algoritmaKontrol(filteredSutIrtifa, filteredSutAciX, filteredSutAciY);
        durumBitleriniGuncelle();
        ucusDurumuGuncelle();
        float toplamAci = sqrt(filteredSutAciX * filteredSutAciX + filteredSutAciY * filteredSutAciY);
        telemetrySnapshotGuncelle(filteredSutIrtifa, filteredSutAciX, filteredSutAciY, sut_aciZ,
                                  sut_ivmeX, sut_ivmeY, sut_ivmeZ, toplamAci);
    }
    if (millis() - sonDurumGonderim >= DURUM_INTERVAL_MS) {
        // Test cihazına hangi koşulların gerçekleştiğini durum paketiyle bildir
        sonDurumGonderim = millis();
        durumPaketiGonder();
    }
}

void sit_calistir() {
    // Gerçek sensörlerle algoritmayı çalıştır ve SIT ölçümlerini RS232'ye gönder
    if (millis() - sonSitGonderim < SIT_INTERVAL_MS) return;
    sonSitGonderim = millis();

    float rawAltitude = sonBarometreIrtifa;
    float filteredAltitude = applyKalmanFilter(rawAltitude);
    irtifa = filteredAltitude - ilkirtifa;
    // MPL zaten hPa veriyor, tekrar 100'e bölmeyelim basınç 10 görünmesin
    float basinc = sonBarometreBasinc;
    readIMU();
    const float aciX = sonImuOrnegi.valid ? sonImuOrnegi.angleX : 0.0f;
    const float aciY = sonImuOrnegi.valid ? sonImuOrnegi.angleY : 0.0f;
    const float aciZ = sonImuOrnegi.valid ? sonImuOrnegi.angleZ : 0.0f;
    const float accelX = sonImuOrnegi.valid ? sonImuOrnegi.accelX : 0.0f;
    const float accelY = sonImuOrnegi.valid ? sonImuOrnegi.accelY : 0.0f;
    const float accelZ = sonImuOrnegi.valid ? sonImuOrnegi.accelZ : 0.0f;

    algoritmaKontrol(irtifa, angleDeltaX, angleDeltaY);
    durumBitleriniGuncelle();
    ucusDurumuGuncelle();

    telemPaketiGonder(irtifa, basinc, accelX, accelY, accelZ, aciX, aciY, aciZ);
    telemetrySnapshotGuncelle(irtifa, angleDeltaX, angleDeltaY, angleDeltaZ,
                              accelX, accelY, accelZ, tiltAngle);
}

void durumBitleriniGuncelle() {
    // Uçuş olaylarını bitlere hazırlıyoruz, paraşüt biti açıldı değil komut verildi demek
    if (irtifa > AltWhenFly && !kalkis_algilandi) { kalkis_algilandi = true; kalkisZamani = millis(); }
    if (kalkis_algilandi && !motor_sure_doldu &&
        (millis() - kalkisZamani >= MOTOR_SURE_MS)) { motor_sure_doldu = true; }
    if (irtifa > DamValue) min_irtifa_asild = true;
    if (donme_tamam) govde_aci_g_tamam = true;
    if (FallingCheck > 0) irtifa_alcalmaya = true;
    // SUT için A1 olduğumuz için daima pasif olmaya mahkum
    surukleme_par = false;
    irtifa_alt_indi = false;
    ana_parasut = kurtarma_aktif; 
}

void ucusDurumuGuncelle() {
    // 1 aktif, 2 uçuş, 3 süreye göre burnout, 4 kurtarma komutu, 5 iniş
    // Arayüz için stateler
    if (!apogee) {
        if (ucusDurumu == 1 && irtifa > AltWhenFly) ucusDurumu = 2;
        if (ucusDurumu == 2 && motor_sure_doldu) ucusDurumu = 3;
    } else {
        if (ucusDurumu < 4 && kurtarma_aktif) ucusDurumu = 4;
        if (ucusDurumu == 4 && pyroPulseFinished && irtifa_alcalmaya) {
            ucusDurumu = 5;
        }
    }
}

// 2 gün uğraştıran rs232 protokolü
void rxCercevesiBaslat(uint8_t header, unsigned long simdi) {
    // Yeni header geldi, paket türünü seçip ilk byte'ı kaydet
    rxState = (header == PROTO_HEADER) ? READ_COMMAND : READ_TELEMETRY;
    rxIdx = 0;
    rxBuf[rxIdx++] = header;
    sonRxByteZamani = simdi;
}

void rxBeklemeyeDon() {
    // Eski paketi bırak yeni başlangıç bekle
    rxState = WAIT_HEADER;
    rxIdx = 0;
}

void gecersizTelemetriSonekKurtar(unsigned long simdi) {
     /*
     Bozuk 36-byte aday içindeki en erken tamamlanabilir header soneğini koru.
     Böylece yeni bir telemetri başlangıcı, kendi binary payload'ındaki AA/AB
     Byte'ları yüzünden atlanmadan sonraki byte'larla tamamlanabilir. 
     */

    for (uint8_t i = 1; i < TELEM_PAKET_BOY; i++) {
        const uint8_t header = rxBuf[i];
        if (header != PROTO_HEADER && header != TELEM_HEADER) continue;

        const uint8_t hedefBoy = (header == PROTO_HEADER) ? KOMUT_PAKET_BOY : TELEM_PAKET_BOY;
        const uint8_t sonekBoy = TELEM_PAKET_BOY - i;
        if (sonekBoy > hedefBoy) continue;

        if (sonekBoy == hedefBoy) {
            // 36 byte adayın sonunda eksiksiz bir komut bulunabilir.
            if (header == PROTO_HEADER && komutPaketIsle(rxBuf + i)) {
                rxBeklemeyeDon();
                return;
            }
            continue;
        }

        memmove(rxBuf, rxBuf + i, sonekBoy);
        rxState = (header == PROTO_HEADER) ? READ_COMMAND : READ_TELEMETRY;
        rxIdx = sonekBoy;
        sonRxByteZamani = simdi;
        return;
    }

    rxBeklemeyeDon();
}

void komutVeVeriDinle() {
    // Yarım paket fazla beklediyse unut, yeni veriyle karışmasın
    unsigned long simdi = millis();
    if (rxState != WAIT_HEADER &&
        (unsigned long)(simdi - sonRxByteZamani) >= RX_PARTIAL_TIMEOUT_MS) {
        rxBeklemeyeDon();
    }

    while (Serial3.available()) {
        simdi = millis();
        if (rxState != WAIT_HEADER &&
            (unsigned long)(simdi - sonRxByteZamani) >= RX_PARTIAL_TIMEOUT_MS) {
            rxBeklemeyeDon();
        }

        uint8_t gelen = Serial3.read();

        
        if (rxState != READ_TELEMETRY &&
            // Binary telemetri içindeki AA AB değerlerini yeni header sanma
            (gelen == PROTO_HEADER || gelen == TELEM_HEADER)) {
            rxCercevesiBaslat(gelen, simdi);
            continue;
        }

        if (rxState == WAIT_HEADER) {
            // Eski tek byte SIT SUT başlatma komutları da hâlâ kabul ediliyor
            if (gelen == 0x03) {
                aktifMod = SIT_BEKLE;
                sitBaslamaZamani = simdi;
                sonKomutZamani = simdi; 
                bayraklariSifirla();
            } else if (gelen == 0x08) {
                aktifMod = SUT_BEKLE;
                sutBaslamaZamani = simdi;
                sonKomutZamani = simdi; 
                bayraklariSifirla();
            }

            continue;
        }

        rxBuf[rxIdx++] = gelen;
        sonRxByteZamani = simdi;

        const uint8_t hedefBoy = (rxState == READ_COMMAND) ? KOMUT_PAKET_BOY : TELEM_PAKET_BOY;
        if (rxIdx == hedefBoy) {
            if (rxState == READ_COMMAND) {
                komutPaketIsle(rxBuf);
                rxBeklemeyeDon();
            } else {
                if (telemPaketIsle(rxBuf)) {
                    sutVeriHazir = true;
                    rxBeklemeyeDon();
                } else {
                    gecersizTelemetriSonekKurtar(simdi);
                }
            }
        }
    }
}

bool komutPaketIsle(uint8_t* buf) {
    // Son byte'lar ve checksum doğruysa komutu uygula, değilse hiçbir moda geçme
    if (buf[3] != PROTO_FOOTER1 || buf[4] != PROTO_FOOTER2) return false;
    if (!checksumDogrula_Komut(buf[1], buf[2])) return false;
    switch (buf[1]) {
        case CMD_SIT: aktifMod = SIT_BEKLE; sitBaslamaZamani = millis(); sonKomutZamani = millis(); bayraklariSifirla(); break;
        case CMD_SUT: aktifMod = SUT_BEKLE; sutBaslamaZamani = millis(); sonKomutZamani = millis(); bayraklariSifirla(); break;
        case CMD_DURDUR: aktifMod = ANA_ALG; sonKomutZamani = millis(); bayraklariSifirla(); break;
        default: return false;
    }
    return true;
}

bool telemPaketIsle(uint8_t* buf) {
    // SUT ölçüm paketini aç, yanlış moddayken algoritmaya test verisi sokma
    if (buf[TELEM_PAKET_BOY - 2] != PROTO_FOOTER1 || buf[TELEM_PAKET_BOY - 1] != PROTO_FOOTER2) return false;
    if (!checksumDogrula_Telem(buf)) return false;
    if (aktifMod == SUT_MOD || aktifMod == SUT_BEKLE) {
        sut_irtifa = floatOku(buf, 1); sut_basinc = floatOku(buf, 5);
        sut_ivmeX = floatOku(buf, 9); sut_ivmeY = floatOku(buf, 13); sut_ivmeZ = floatOku(buf, 17);
        sut_aciX = floatOku(buf, 21); sut_aciY = floatOku(buf, 25); sut_aciZ = floatOku(buf, 29);
        return true;
    }
    return false;
}


uint8_t checksumHesapla(uint8_t* buf, int len) {
    // Byte'ları topla, toplamın son 8 bitini kontrol değeri olarak kullan
    uint8_t toplam = 0;
    for (int i = 0; i < len; i++) toplam = (uint8_t)(toplam + buf[i]);
    return toplam;
}

// dinamik checksum doğrulama çabaları
bool checksumDogrula_Komut(uint8_t cmd, uint8_t cs) {
    uint8_t dinamikCS = (PROTO_HEADER + cmd) & 0xFF;
    if (cs == dinamikCS) return true;

    switch(cmd) {
        case CMD_SIT: return (cs == 0x8C);
        case CMD_SUT: return (cs == 0x8E);
        case CMD_DURDUR: return (cs == 0x90);
        default: return false;
    }
}

bool checksumDogrula_Telem(uint8_t* buf) { 
    // Gönderilen kontrol toplamı bizim hesapla aynı mı
    return (buf[TELEM_PAKET_BOY - 3] == checksumHesapla(buf, TELEM_PAKET_BOY - 3)); 
}

void bayraklariSifirla() {
    // Yeni test için geçmişi temizle, devam eden pyro darbesi de burada kesilir
    // DURDUR sonrası ana moda dönülür
    pyroPulseStarted = false;
    pyroPulseFinished = false;
    pyroPulseStartMs = 0;
    donme_tamam = false; irtifa_tamam = false; kurtarma_aktif = false; apogee = false;
    kalkis_algilandi = false; motor_sure_doldu = false; min_irtifa_asild = false;
    govde_aci_g_tamam = false; irtifa_alcalmaya = false; surukleme_par = false;
    irtifa_alt_indi = false; ana_parasut = false;
    apogeeAlt = 0; bottomAlt = 0; FallingCheck = 0; irtifa = 0; sutVeriHazir = false;
    kalkisZamani = 0; ucusDurumu = 1;
    pyroGuvenliTut();
    digitalWrite(RED_LED_PIN, LOW);
    buzzerKapat();
    kalman_X = sonBarometreIrtifa; kalman_aciX_X = 0.0f; kalman_aciY_X = 0.0f;
}

void durumPaketiGonder() {
    // Olay bayraklarını tek byte'a sığdırıp 6 byte RS232 durum paketi yap
    uint8_t data1 = 0x00;
    if (kalkis_algilandi) data1 |= BIT0_KALKIS;
    if (motor_sure_doldu) data1 |= BIT1_MOTOR_SURE;
    if (min_irtifa_asild) data1 |= BIT2_MIN_IRTIFA;
    if (govde_aci_g_tamam) data1 |= BIT3_GOVDE_ACI_G;
    if (irtifa_alcalmaya) data1 |= BIT4_IRTIFA_ALCAL;
    if (surukleme_par) data1 |= BIT5_SURUKLEME_PAR;
    if (irtifa_alt_indi) data1 |= BIT6_IRTIFA_ALTI;
    if (ana_parasut) data1 |= BIT7_ANA_PAR;

    uint8_t paket[6] = {PROTO_HEADER, data1, DATA2_REZERVE, 0, PROTO_FOOTER1, PROTO_FOOTER2};
    paket[3] = checksumHesapla(paket, 3);
    Serial3.write(paket, 6);
}

void telemPaketiGonder(float irtifaVal, float basincVal, float ivmeX, float ivmeY, float ivmeZ, float aciX, float aciY, float aciZ) {
    // Bu paket yer istasyonunun 60 byte paketi değil, SIT için 36 byte big-endian paket
    uint8_t buf[TELEM_PAKET_BOY]; int idx = 0;
    buf[idx++] = TELEM_HEADER;
    floatYazBinary(buf, idx, irtifaVal); floatYazBinary(buf, idx, basincVal);
    floatYazBinary(buf, idx, ivmeX); floatYazBinary(buf, idx, ivmeY); floatYazBinary(buf, idx, ivmeZ);
    floatYazBinary(buf, idx, aciX); floatYazBinary(buf, idx, aciY); floatYazBinary(buf, idx, aciZ);
    uint8_t cs = checksumHesapla(buf, idx);
    buf[idx++] = cs; buf[idx++] = PROTO_FOOTER1; buf[idx++] = PROTO_FOOTER2;
    Serial3.write(buf, idx);
}

// Big-Endian'a göre floatları tersine encode etme kodu
void floatYazBinary(uint8_t* buf, int& idx, float val) {
    float yuv = roundf(val * 100.0f) / 100.0f;
    union { float f; uint8_t b[4]; } u; u.f = yuv;
    buf[idx++] = u.b[3]; // MSB
    buf[idx++] = u.b[2];
    buf[idx++] = u.b[1];
    buf[idx++] = u.b[0]; // LSB
}

// Big-Endian'a göre floatları tersine decode etme kodu
float floatOku(uint8_t* buf, int idx) {
    union { float f; uint8_t b[4]; } u;
    u.b[3] = buf[idx];     // MSB
    u.b[2] = buf[idx + 1];
    u.b[1] = buf[idx + 2];
    u.b[0] = buf[idx + 3]; // LSB
    return u.f;
}

// çerez gps kodu
void gpsDinle() {
    while (Serial2.available() > 0) {
        gps.encode(Serial2.read());
    }
}

// yer istasyonu 
void normalTelemetriOrnekle() {
    // Ana algoritmanın zamanından bağımsız son sensör değerlerini telemetriye hazırla
    unsigned long simdi = millis();
    if ((unsigned long)(simdi - sonTelemetriOrnekleme) < TELEMETRI_INTERVAL_MS) return;
    sonTelemetriOrnekleme = simdi;

    float sampleAltitude = sonBarometreIrtifa - ilkirtifa;
    readIMU();
    telemetrySnapshotGuncelle(sampleAltitude, angleDeltaX, angleDeltaY, angleDeltaZ,
                              sonImuOrnegi.accelX, sonImuOrnegi.accelY,
                              sonImuOrnegi.accelZ, tiltAngle);
}




void telemetrySnapshotGuncelle(float altitude, float sampleGyroX, float sampleGyroY, float sampleGyroZ,
                               float sampleAccX, float sampleAccY, float sampleAccZ, float totalAngle) {
    // Tek paketin alanlarını aynı yerde doldur, GPS geçersizse o alanlara sıfır koy
    TelemetrySnapshot yeniSnapshot;
    yeniSnapshot.altitude = altitude;
    yeniSnapshot.gpsAltitude = gps.altitude.isValid() ? gps.altitude.meters() : 0.0f;
    yeniSnapshot.gpsLatitude = gps.location.isValid() ? gps.location.lat() : 0.0f;
    yeniSnapshot.gpsLongitude = gps.location.isValid() ? gps.location.lng() : 0.0f;
    yeniSnapshot.gyroX = sampleGyroX;
    yeniSnapshot.gyroY = sampleGyroY;
    yeniSnapshot.gyroZ = sampleGyroZ;
    yeniSnapshot.accX = sampleAccX;
    yeniSnapshot.accY = sampleAccY;
    yeniSnapshot.accZ = sampleAccZ;
    yeniSnapshot.flightState = ucusDurumu;
    yeniSnapshot.satelliteCount = gps.satellites.isValid() ? gps.satellites.value() : 0;
    yeniSnapshot.totalAngle = totalAngle;
    yeniSnapshot.sampleTimestamp = millis();

    telemetrySnapshot = yeniSnapshot;
    telemetrySnapshotHazir = true;
}

void telemetriGonderiminiCalistir() {
    // Gönderme zamanı geldiyse son örneği yolla, başarılıysa zamanı güncelle
    if (!telemetrySnapshotHazir) return;
    unsigned long simdi = millis();
    if ((unsigned long)(simdi - sonTelemetriGonderim) < TELEMETRI_INTERVAL_MS) return;
    if (yerIstasyonunaGonder(telemetrySnapshot)) {
        sonTelemetriGonderim = millis();
    }
}

bool telemetriKarakterEkle(char* buffer, size_t kapasite, size_t& uzunluk, char deger) {
    // Eski metin paketi yardımcısı, sona karakter koyarken tamponu taşırma
    if (uzunluk + 1 >= kapasite) return false;
    buffer[uzunluk++] = deger;
    buffer[uzunluk] = '\0';
    return true;
}

bool telemetriMetinEkle(char* buffer, size_t kapasite, size_t& uzunluk, const char* deger) {
    // Metni karakter karakter ekleyen eski yardımcı, binary yolda kullanılmıyor
    while (*deger != '\0') {
        if (!telemetriKarakterEkle(buffer, kapasite, uzunluk, *deger++)) return false;
    }
    return true;
}

bool telemetriFloatEkle(char* buffer, size_t kapasite, size_t& uzunluk, float deger, uint8_t hassasiyet) {
    // Eski ASCII yolunda float'ı yazıya çevir, geçersiz sayıysa pakete ekleme
    if (isnan(deger) || isinf(deger)) return false;
    if (hassasiyet > 6) return false;


    char sayi[48];
    dtostrf(deger, 1, hassasiyet, sayi);
    size_t sayiUzunlugu = strlen(sayi);
    if (sayiUzunlugu >= sizeof(sayi)) return false;

    const char* ilkKarakter = sayi;
    while (*ilkKarakter == ' ') ilkKarakter++;
    if (*ilkKarakter == '\0') return false;
    return telemetriMetinEkle(buffer, kapasite, uzunluk, ilkKarakter);
}

bool telemetriIntEkle(char* buffer, size_t kapasite, size_t& uzunluk, long deger) {
    // Eski metin paketine tam sayı ekleme kısmı
    char sayi[12];
    ltoa(deger, sayi, 10);
    return telemetriMetinEkle(buffer, kapasite, uzunluk, sayi);
}

bool telemetriBinaryFloatYaz(uint8_t* frame, size_t offset, float value) {
    // Float'ı 4 byte olarak yerine kopyala, AVR tarafında little-endian gidiyor
    if (isnan(value) || isinf(value)) return false;
    memcpy(frame + offset, &value, sizeof(value));
    return true;
}

bool yerIstasyonunaGonder(const TelemetrySnapshot& snapshot) {
    // 14 tane soru işareti ayıracı var, sayılar yazı değil binary olacak
    uint8_t frame[TELEMETRI_BINARY_FRAME_SIZE] = {};
    const uint8_t ayiracOfsetleri[] = { 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 52, 54, 59 };
    for (uint8_t i = 0; i < sizeof(ayiracOfsetleri); i++) frame[ayiracOfsetleri[i]] = '?';

    bool gecerli = telemetriBinaryFloatYaz(frame, 1, snapshot.altitude) &&
        telemetriBinaryFloatYaz(frame, 6, snapshot.gpsAltitude) &&
        telemetriBinaryFloatYaz(frame, 11, snapshot.gpsLatitude) &&
        telemetriBinaryFloatYaz(frame, 16, snapshot.gpsLongitude) &&
        telemetriBinaryFloatYaz(frame, 21, snapshot.gyroX) &&
        telemetriBinaryFloatYaz(frame, 26, snapshot.gyroY) &&
        telemetriBinaryFloatYaz(frame, 31, snapshot.gyroZ) &&
        telemetriBinaryFloatYaz(frame, 36, snapshot.accX) &&
        telemetriBinaryFloatYaz(frame, 41, snapshot.accY) &&
        telemetriBinaryFloatYaz(frame, 46, snapshot.accZ) &&
        telemetriBinaryFloatYaz(frame, 55, snapshot.totalAngle);
    if (!gecerli) return false;

    frame[51] = (uint8_t)snapshot.flightState;
    // Durum ve uydu sayısı birer byte, diğer ölçümler float
    frame[53] = (uint8_t)snapshot.satelliteCount;
    const size_t usbWritten = Serial.write(frame, TELEMETRI_BINARY_FRAME_SIZE);
    const size_t radioWritten = Serial1.write(frame, TELEMETRI_BINARY_FRAME_SIZE);
    return usbWritten == TELEMETRI_BINARY_FRAME_SIZE &&
           radioWritten == TELEMETRI_BINARY_FRAME_SIZE;
}

// microsd log denemeleri
const char* modToString(TestModu m) {
    switch(m) { case BEKLEME: return "BEKLEME"; case SIT_BEKLE: return "SIT_BEKLE"; case SIT_MOD: return "SIT_MOD"; case SUT_BEKLE: return "SUT_BEKLE"; case SUT_MOD: return "SUT_MOD"; case ANA_ALG: return "ANA_ALG"; default: return "UNKNOWN"; }
}

void logToSD(float logIrtifa, float logApogee) {
    // Dosya açıksa ölçüm ve durum bitlerini CSV satırına yaz, değilse uçuşu bekletme
    if (!dataFile.isOpen()) return;
    uint8_t data1 = 0x00;
    if (kalkis_algilandi) data1 |= BIT0_KALKIS;
    if (motor_sure_doldu) data1 |= BIT1_MOTOR_SURE;
    if (min_irtifa_asild) data1 |= BIT2_MIN_IRTIFA;
    if (govde_aci_g_tamam) data1 |= BIT3_GOVDE_ACI_G;
    if (irtifa_alcalmaya) data1 |= BIT4_IRTIFA_ALCAL;
    if (surukleme_par) data1 |= BIT5_SURUKLEME_PAR;
    if (irtifa_alt_indi) data1 |= BIT6_IRTIFA_ALTI;
    if (ana_parasut) data1 |= BIT7_ANA_PAR;

    dataFile.print(millis()); dataFile.print(",");
    dataFile.print(modToString(aktifMod)); dataFile.print(",");
    dataFile.print(logIrtifa, 2); dataFile.print(",");
    dataFile.print(logApogee, 2); dataFile.print(",");
    dataFile.print(gps.location.isValid() ? gps.location.lat() : 0.0f, 6); dataFile.print(",");
    dataFile.print(gps.location.isValid() ? gps.location.lng() : 0.0f, 6); dataFile.print(",");
    dataFile.print(gps.altitude.isValid() ? gps.altitude.meters() : 0.0f, 1); dataFile.print(",");
    dataFile.print(gps.satellites.isValid() ? gps.satellites.value() : 0); dataFile.print(",");
    dataFile.print(data1, HEX); dataFile.println();
    
    static unsigned long sonFlush = 0;
    // Her satırda değil saniyede bir karttaki kaydı güncelle
    if (millis() - sonFlush >= 1000) { dataFile.flush(); sonFlush = millis(); }
}
