// ATmega2560 + custom PCB MicroSD hardware/filesystem diagnostic.
// This sketch never formats the card.  It only mounts exFAT and writes a
// small test file after SPI communication succeeds.

#include <SPI.h>
#include <SdFat.h>

constexpr uint8_t SD_CS_PIN = 53;
constexpr uint8_t SD_MISO_PIN = 50;
constexpr uint8_t SD_MOSI_PIN = 51;
constexpr uint8_t SD_SCK_PIN = 52;

SdExFat sd;
ExFile testFile;

static void printPinGuide() {
  Serial.println(F("Beklenen Mega2560 hatlari:"));
  Serial.println(F("  CS=53, MOSI=51, MISO=50, SCK=52"));
  Serial.println(F("  Kart VDD=3.3 V (kart takiliyken olcun)"));
}

static void sendIdleClocks() {
  // A card must see at least 74 clock pulses while CS is HIGH before CMD0.
  // This also helps recover from CS being low during the MCU power-up period.
  digitalWrite(SD_CS_PIN, HIGH);
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
  for (uint8_t i = 0; i < 10; ++i) {
    SPI.transfer(0xFF);
  }
  SPI.endTransaction();
}

void setup() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  Serial.begin(115200);
  const unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 3000UL) {}

  Serial.println(F("\n=== MicroSD tani testi (salt formatlamaz) ==="));
  printPinGuide();

  Serial.println(F("Karti takin ve testi baslatmak icin bir karakter gonderin."));
  Serial.println(F("Ilk deneme basarisizsa: gucu kesin, karti cikarin,"));
  Serial.println(F("karti takmadan acin, bu yazi geldikten sonra karti takin ve karakter gonderin."));
  while (!Serial.available()) {}
  while (Serial.available()) Serial.read();

  // Allow the 3.3 V rail and the card controller to settle completely.
  delay(500);
  SPI.begin();
  sendIdleClocks();

  Serial.println(F("Kart 1 MHz SPI ile baslatiliyor..."));
  if (!sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(1)))) {
    sd.initErrorPrint(&Serial);
    Serial.println(F("CMD0/0x1 ise dosya sistemi okunmadan once hata olmustur."));
    Serial.println(F("exFAT veya 512 GB kapasite bu hatanin nedeni degildir."));
    Serial.println(F("Once VDD, CS, SCK, MOSI, MISO ve soket lehimlerini olcun."));
    return;
  }

  Serial.print(F("SPI tamam; dosya sistemi: "));
  sd.printFatType(&Serial);
  Serial.println();

  if (!testFile.open("SD_TEST.TXT", O_WRITE | O_CREAT | O_APPEND)) {
    Serial.println(F("Kart haberlesiyor fakat SD_TEST.TXT acilamadi."));
    sd.errorPrint(&Serial);
    return;
  }

  testFile.println(F("ATmega2560 MicroSD test OK"));
  if (!testFile.sync()) {
    Serial.println(F("Dosya acildi fakat sync basarisiz."));
    sd.errorPrint(&Serial);
    testFile.close();
    return;
  }
  testFile.close();
  Serial.println(F("OK: SD_TEST.TXT yazildi. Donanim ve exFAT birlikte calisiyor."));
}

void loop() {}
