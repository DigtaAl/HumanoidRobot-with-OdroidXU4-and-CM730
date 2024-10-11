#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Interface
const int buttonPin1 = 9;  // Pin untuk tombol 1
const int buttonPin2 = 7;  // Pin untuk tombol 2
const int ledPinR1 = 2;    // Pin untuk LED 1
const int ledPinG1 = 4;    // Pin untuk LED 2
const int ledPinB1 = 6;    // Pin untuk LED 3
const int ledPinR2 = 8;    // Pin untuk LED 1
const int ledPinG2 = 10;   // Pin untuk LED 2
const int ledPinB2 = 3;    // Pin untuk LED 3
const int ledPinR3 = 14;   // Pin untuk LED 1
const int ledPinG3 = 12;   // Pin untuk LED 2
const int ledPinB3 = 11;   // Pin untuk LED 3
const int ledPinRX = 13;   // Pin untuk LED RX
const int ledPinTX = 5;    // Pin untuk LED TX

bool button1Pressed = false;  // Variabel untuk menyimpan status apakah tombol 1 ditekan
bool button1Sent = false;     // Variabel untuk melacak apakah data "1" sudah dikirim
bool button2Pressed = false;  // Variabel untuk menyimpan status apakah tombol 2 ditekan
bool button2Sent = false;     // Variabel untuk melacak apakah data "2" sudah dikirim
int zeroCount = 0;            // Counter untuk melacak berapa kali data "0" telah dikirim

// IMU
float arahValue, arahValue2, Arah2, lastArahValue2;
float Arah = 0; // Cyan 50 : Magen 230
int finalArahValue2;
bool check = false;
bool checkalib = false;

// Variabel untuk mencatat waktu stabilitas
unsigned long lastMovementTime = 0;
const unsigned long stabilityPeriod = 5000; // 2 detik

// Sensor Tegangan
int analogPin = A1; // Pin sensor tegangan
int value = 0;
const int buzzerPin = 24;
float Vmodul = 0.0;
float hasil = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;
float limitteg = 12.3;
unsigned long startTime = 0;
bool isBelowLimit = false;

#define BNO055_SAMPLERATE_DELAY_MS (50)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

void setup() {
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  bno.setExtCrystalUse(true);

  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(ledPinR1, OUTPUT);
  pinMode(ledPinG1, OUTPUT);
  pinMode(ledPinB1, OUTPUT);
  pinMode(ledPinR2, OUTPUT);
  pinMode(ledPinG2, OUTPUT);
  pinMode(ledPinB2, OUTPUT);
  pinMode(ledPinR3, OUTPUT);
  pinMode(ledPinG3, OUTPUT);
  pinMode(ledPinB3, OUTPUT);
  pinMode(ledPinRX, OUTPUT);
  pinMode(ledPinTX, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  // Set all LEDs to off initially
  digitalWrite(ledPinR1, HIGH);
  digitalWrite(ledPinG1, HIGH);
  digitalWrite(ledPinB1, HIGH);
  digitalWrite(ledPinR2, HIGH);
  digitalWrite(ledPinG2, HIGH);
  digitalWrite(ledPinB2, HIGH);
  digitalWrite(ledPinR3, HIGH);
  digitalWrite(ledPinG3, HIGH);
  digitalWrite(ledPinB3, HIGH);
  digitalWrite(ledPinRX, HIGH);
  digitalWrite(ledPinTX, HIGH);
}

void loop() {
  int mainOutput = 0;
  // Button
  // Membaca status tombol 1
  int buttonState1 = digitalRead(buttonPin1);
  if (buttonState1 == LOW && !button1Pressed) {
    // Tombol 1 baru saja ditekan
    button1Pressed = true;
    button1Sent = false;  // Setel ulang variabel agar data "1" bisa dikirim lagi
  } else if (buttonState1 == HIGH && button1Pressed) {
    // Tombol 1 telah dilepas
    button1Pressed = false;
  }

  // Membaca status tombol 2
  int buttonState2 = digitalRead(buttonPin2);
  if (buttonState2 == LOW && !button2Pressed) {
    // Tombol 2 baru saja ditekan
    button2Pressed = true;
    button2Sent = false;  // Setel ulang variabel agar data "2" bisa dikirim lagi
  } else if (buttonState2 == HIGH && button2Pressed) {
    // Tombol 2 telah dilepas
    button2Pressed = false;
  }

  if (button1Pressed && !button1Sent) {
    mainOutput = 1;
    button1Sent = true;  // Setel variabel agar data "1" tidak dikirim lagi sampai tombol ditekan kembali
    zeroCount = 10;      // Siapkan untuk mengirim data "0" sepuluh kali setelah mengirim data "1"
  }
  if (button2Pressed && !button2Sent) {
    mainOutput = 2;
    button2Sent = true;  // Setel variabel agar data "2" tidak dikirim lagi sampai tombol ditekan kembali
    zeroCount = 10;      // Siapkan untuk mengirim data "0" sepuluh kali setelah mengirim data "2"
  }

  if (mainOutput != 0) {
    Serial.print("B:");  // Mengirimkan identifier "B" untuk data button
    Serial.println(mainOutput);  // Mengirimkan nilai button ke mini PC
  }

  // Mengirim data "0" sepuluh kali setelah data "1" atau "2"
  if ((button1Sent || button2Sent) && zeroCount > 0) {
    Serial.print("B:");  // Mengirimkan identifier "B" untuk data button
    Serial.println(0);   // Mengirimkan nilai "0"
    zeroCount--;         // Kurangi counter untuk melacak berapa kali data "0" telah dikirim
  }

  // IMU
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  if (check == false) {
    if (mag < 2) {
      arahValue = euler.x();
    } else {
      check = true;
    }
    finalArahValue2 = (int)arahValue;
    Serial.println(finalArahValue2);
    digitalWrite(ledPinRX, HIGH);  // Matikan semua LED
    digitalWrite(ledPinTX, HIGH);
  } else {
    arahValue2 = euler.x();
    arahValue2 = arahValue2 - (Arah + Arah2);
    arahValue2 = fmod(arahValue2, 360);
    if (arahValue2 < 0) {
        arahValue2 += 360;
    }

    // Jika sensor diam (tidak berubah), lakukan penyesuaian menuju nol
    if (checkalib == false) {
      if (euler.x() != lastArahValue2) {
        lastMovementTime = millis(); // Reset waktu jika sensor bergerak
      }

      // Cek jika sensor stabil selama periode yang ditentukan
      if (millis() - lastMovementTime > stabilityPeriod) {
        Arah2 = arahValue2; // Sesuaikan nilai Arah
        checkalib = true;
      }
    }
    lastArahValue2 = euler.x();  // Update lastArahValue2 untuk iterasi berikutnya
    finalArahValue2 = (int)arahValue2;
    Serial.println(finalArahValue2);
    digitalWrite(ledPinRX, LOW);  // Nyala semua LED
    if(checkalib == true){
      digitalWrite(ledPinTX, LOW);
      }
  }

  // Cek apakah ada data yang diterima dari Odroid
  if (Serial.available() > 0) {
    char command = Serial.read();  // Baca perintah dari serial
    if (command == '1') {
      digitalWrite(ledPinR1, LOW);  // Nyalakan LED merah
      digitalWrite(ledPinG1, HIGH);
      digitalWrite(ledPinB1, HIGH);
      digitalWrite(ledPinR2, HIGH);  // Nyalakan LED merah
      digitalWrite(ledPinG2, HIGH);
      digitalWrite(ledPinB2, HIGH);
    } else if (command == '2') {
      digitalWrite(ledPinR1, HIGH);
      digitalWrite(ledPinG1, LOW);  // Nyalakan LED hijau
      digitalWrite(ledPinB1, HIGH);
      digitalWrite(ledPinR2, HIGH);  // Nyalakan LED merah
      digitalWrite(ledPinG2, HIGH);
      digitalWrite(ledPinB2, HIGH);
    } else if (command == '3') {
      digitalWrite(ledPinR1, HIGH);
      digitalWrite(ledPinG1, HIGH);
      digitalWrite(ledPinB1, LOW);  // Nyalakan LED biru
      digitalWrite(ledPinR2, HIGH);  // Nyalakan LED merah
      digitalWrite(ledPinG2, HIGH);
      digitalWrite(ledPinB2, HIGH);
    } else if (command == '4') {
      digitalWrite(ledPinR1, HIGH);
      digitalWrite(ledPinG1, HIGH);
      digitalWrite(ledPinB1, HIGH);  // Nyalakan LED biru
      digitalWrite(ledPinR2, LOW);  // Nyalakan LED merah
      digitalWrite(ledPinG2, HIGH);
      digitalWrite(ledPinB2, HIGH);
    } else if (command == '5') {
      digitalWrite(ledPinR1, HIGH);
      digitalWrite(ledPinG1, HIGH);
      digitalWrite(ledPinB1, HIGH);  // Nyalakan LED biru
      digitalWrite(ledPinR2, HIGH);  // Nyalakan LED merah
      digitalWrite(ledPinG2, LOW);
      digitalWrite(ledPinB2, HIGH);
    } else if (command == '6') {
      digitalWrite(ledPinR1, HIGH);
      digitalWrite(ledPinG1, HIGH);
      digitalWrite(ledPinB1, HIGH);  // Nyalakan LED biru
      digitalWrite(ledPinR2, HIGH);  // Nyalakan LED merah
      digitalWrite(ledPinG2, HIGH);
      digitalWrite(ledPinB2, LOW);
    } else if (command == '0') {
      digitalWrite(ledPinR1, HIGH);  // Matikan semua LED
      digitalWrite(ledPinG1, HIGH);
      digitalWrite(ledPinB1, HIGH);
      digitalWrite(ledPinR2, HIGH);  // Nyalakan LED merah
      digitalWrite(ledPinG2, HIGH);
      digitalWrite(ledPinB2, HIGH);
    }
  }

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
