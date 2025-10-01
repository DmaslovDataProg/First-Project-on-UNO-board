// Combined DHT11 Sensor + LCD Display
// TODO: implement the data storage in power off mode
// Date: 2025.09.29

#include <dht_nonblocking.h>
#include <LiquidCrystal.h>

#define DHT_SENSOR_TYPE DHT_TYPE_11
static const int DHT_SENSOR_PIN = 2;

DHT_nonblocking dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

// Initialize the LCD (RS, E, D4, D5, D6, D7)
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

// Function to measure temperature and humidity
static bool measure_environment(float *temperature, float *humidity) {
  static unsigned long measurement_timestamp = millis();

  // Measure once every 2 seconds (faster refresh than 5 sec)
  if (millis() - measurement_timestamp > 2000ul) {
    if (dht_sensor.measure(temperature, humidity) == true) {
      measurement_timestamp = millis();
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2); // 16 columns, 2 rows
  lcd.print("Initializing...");
  delay(2000);
  lcd.clear();
}

void loop() {
  float temperature;
  float humidity;

  if (measure_environment(&temperature, &humidity) == true) {
    // Print to Serial Monitor
    Serial.print("T = ");
    Serial.print(temperature, 1);
    Serial.print(" C, H = ");
    Serial.print(humidity, 1);
    Serial.println(" %");

    // Print to LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperature, 1);
    lcd.print((char)223); // degree symbol
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.print(humidity, 1);
    lcd.print(" %");
  }
}
