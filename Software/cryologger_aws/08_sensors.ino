// ----------------------------------------------------------------------------
// Adafruit DPS310 Precision Barometric Pressure Sensor
// ----------------------------------------------------------------------------
void configureDps310()
{
  // Enable power
  enable5V();

  DEBUG_PRINT("Info: Initializing DPS310...");

  if (dps310.begin_I2C())
  {
    dps310.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps310.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
    online.dps310 = true;
    DEBUG_PRINTLN("success!");
  }
  else
  {
    DEBUG_PRINTLN("failed!");
    online.dps310 = false;
  }
}

// Read DPS310
void readDps310()
{
  // Start the loop timer
  unsigned long loopStartTime = millis();

  // Initialize sensor(s)
  void readDps310();

  // Check if sensor initialized successfully
  if (online.dps310)
  {
    DEBUG_PRINT("Info: Reading DPS310...");

    sensors_event_t temp_event, pressure_event;

    // Read sensor until value is returned or timeout is exceeded
    while ((!dps310.temperatureAvailable() || !dps310.pressureAvailable()) && millis() - loopStartTime < 5000UL)
    {
      return;
    }

    // Record measurements
    dps310.getEvents(&temp_event, &pressure_event);
    float temperature = temp_event.temperature;
    float pressure = pressure_event.pressure;

    // Write data to union
    moSbdMessage.intTemperature = temperature * 100;
    moSbdMessage.intPressure = (pressure - 850) * 100;

    DEBUG_PRINTLN("done.");
  }
  else
  {
    DEBUG_PRINTLN("Warning: DPS310 offline!");
  }
  // Stop the loop timer
  timer.dps310 = millis() - loopStartTime;

  // Disable power
  disable5V();
}

// ----------------------------------------------------------------------------
// Adafruit LSM303AGR Accelerometer/Magnetomter
// ----------------------------------------------------------------------------
void configureLsm303()
{
  // Enable power
  enable5V();

  DEBUG_PRINT("Info: Initializing LSM303...");

  // Initialize LSM303 accelerometer
  if (lsm303.begin())
  {
    online.lsm303 = true;
    DEBUG_PRINTLN("success!");
  }
  else
  {
    DEBUG_PRINTLN("failed!");
    online.lsm303 = false;
  }
}

void readLsm303()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Initialize accelerometer
  configureLsm303();

  // Check if sensor initialized successfully
  if (online.lsm303)
  {
    DEBUG_PRINT("Info: Reading accelerometer...");

    // Read accelerometer data
    sensors_event_t accel;
    lsm303.getEvent(&accel);

    // Calculate pitch and roll
    float pitch = atan2(-accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 180 / PI;
    float roll = atan2(accel.acceleration.y, accel.acceleration.z) * 180 / PI;

    // Write data to union
    moSbdMessage.pitch = pitch * 100;
    moSbdMessage.roll = roll * 100;

    DEBUG_PRINTLN("done.");
  }
  else
  {
    DEBUG_PRINTLN("Warning: LSM303 offline!");
  }

  // Disable power to IMU
  disable5V();

  // Stop loop timer
  timer.lsm303 = millis() - loopStartTime;
}

// ----------------------------------------------------------------------------
// Vaisala HMP60 Temperature/Relative Humidity
// ----------------------------------------------------------------------------
void readHmp60()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Enable power
  enable12V();

  // Note: A startup delay of 4 s is recommended at 13.5 V. 2 s for 5 V
  myDelay(4000);

  // Perform analog readings
  float extTemperature = analogRead(A3);
  float extHumidity = analogRead(A4);

  // Convert analog reading to voltage
  extTemperature *= (3.3 / 4096.0);
  extHumidity *= (3.3 / 4096.0);

  // Map voltages to sensor ranges
  extTemperature = mapFloat(extTemperature, 0, 1240, -40, 60); // Map temperature from 0-1 V to -40-60°C
  extHumidity = mapFloat(extHumidity, 0, 1240, 0, 100);        // Map humidity 0-1 V to 0-100%

  // Add to statistics object
  extTemperatureStats.add(extTemperature);
  extHumidityStats.add(extHumidity);

  // Disable power
  //disable12V();

  // Stop loop timer
  timer.hmp60 = millis() - loopStartTime;
}

// ------------------------------------------------------------------------------------------------
// R.M. Young Wind Monitor 5103L (4-20 mA)
// ------------------------------------------------------------------------------------------------
void readAnemometer()
{
  unsigned int startTime = millis();

  // Enable power
  enable12V();

  // Measure wind speed
  // Calibration 6.250 x mA - 25
  windSpeed = analogRead(PIN_WIND_SPEED); // Raw analog wind speed value

  // Map wind speed to 0-100 m/s
  windSpeed = map(windSpeed, 746, 2978, 0, 100);

  // Calibration (22.5 x mA)-90
  // Output Signal 4 to 20 mA = 0 to 360°

  // Measure wind direction
  windDirection = analogRead(PIN_WIND_DIR); // Raw analog wind direction value

  // Map wind direction to 0-360°
  windDirection = map(windDirection, 746, 2978, 0, 360);

  // Disable power
  disable12V();

  // Correct for negative wind direction values
  if (windDirection > 360)
    windDirection -= 360;
  if (windDirection < 0)
    windDirection += 360;

  Serial.print(F("windSpeed: ")); Serial.println(windSpeed);
  Serial.print(F("windDirection: ")); Serial.println(windDirection);

  // Determine wind gust and direction
  if ((windSpeed > 0) && (windSpeed > windGust))
  {
    windGust = windSpeed;
    windGustDirection = windDirection;
  }

  // Calculate wind speed and direction vectors
  float windDirectionRadians = windDirection * DEG_TO_RAD;    // Convert wind direction from degrees to radians
  float vn1 = -1.0 * windSpeed * cos(windDirectionRadians);   // Magnitude of the north-south component (v) of the resultant vector mean wind
  float ve1 = -1.0 * windSpeed * sin(windDirectionRadians);   // Magnitude of the east-west component (u) of the resultant vector mean wind

  // Write data to union
  moSbdMessage.windGust = windGust * 100;
  moSbdMessage.windGustDirection = windGustDirection;

  // Add to wind statistics 1
  windSpeedStats.add(windSpeed);
  vnStats.add(vn1);
  veStats.add(ve1);

  // Stop loop timer
  //timer.Wm5103 = millis() - loopStartTime;

}

// Calculate resultant mean wind speed and direction vectors
// For more information see: http://www.webmet.com/met_monitoring/622.html
void windVectors()
{
  float rvWindDirection = atan2(veStats.average(), vnStats.average()); // Resultant mean wind direction
  rvWindDirection *= RAD_TO_DEG;  // Convert from radians to degrees

  if (rvWindDirection < 0)
  {
    rvWindDirection += 360;
  }

  float rvWindSpeed = sqrt(sq(veStats.average()) + sq(vnStats.average())); // Resultant mean wind speed

  if ((rvWindDirection == 0) && (rvWindSpeed != 0))
  {
    rvWindDirection = 360;
  }

  if (rvWindSpeed == 0)
  {
    rvWindDirection = 0;
  }

  // Wind direction "from" correction
  if (rvWindDirection < 180)
  {
    rvWindDirection += 180;
  }
  else if (rvWindDirection > 180)
  {
    rvWindDirection -= 180;
  }

  // Write data to union
  moSbdMessage.windSpeed = rvWindSpeed * 100;      // Resultant mean wind speed 1 (m/s)
  moSbdMessage.windDirection = rvWindDirection;    // Resultant mean wind direction 1 (°)
}