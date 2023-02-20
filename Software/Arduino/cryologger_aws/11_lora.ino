void LoRa_setup() 
{
  // manual reset
  digitalWrite(PIN_RFM95_RST, LOW);
  delay(100);
  digitalWrite(PIN_RFM95_RST, HIGH);
  delay(500);
  
  Serial.println("Feather LoRa RX Test!");
  if (!manager.init())
    Serial.println("LoRa manager init failed");
  Serial.println("LoRa radio init OK!");
  
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }

  rf95.setTxPower(23, false);
  Serial.println("Sleeping radio");
  rf95.sleep();
  Serial.println();
  packetnum = 0;  // packet counter, we increment per xmission

//  rf95.setSignalBandwidth(125000);
//  rf95.setCodingRate4(8);
//  rf95.setSpreadingFactor(12); // 4096
//  Serial.println("SUCCESS\nSet Config to: Bw = 125 kHz, Cr = 4/8, Sf = 4096 chips/symbol");
//  uint16_t loraAckTimeout = 1000;
//  manager.setTimeout(loraAckTimeout);
}

// Set-up to talk to LoRA
void talkToRadio(){
  digitalWrite(SD_Pin, HIGH);
  digitalWrite(RFM95_CS, LOW);
  delay(1);
}

// Send LoRa
void LoRa_send(){
  unsigned int loopStartTime = millis();
  
  talkToRadio();
  packetnum++;
  
  #if DEBUG
  Serial.println("Transmitting via LoRa"); // Send a message to rf95_server
  #endif

  // Send a message to manager_server
  if (manager.sendtoWait((uint8_t *)&tx_message, sizeof(tx_message), BASE_ADDRESS))
  {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAckTimeout(buf, &len, 4000, &from))
    {
      Serial.print("got reply from base station :");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
      loraRxFlag = true;
    }
    else
    {
      Serial.println("No reply, is the base station running?");
    }
  }
  else
    Serial.println("sendtoWait failed");

  Serial.print("RSSI = "); Serial.println(rf95.lastRssi());
  
  unsigned int loraLoopTime = millis() - loopStartTime;
  Serial.print("LoRa_send() function execution: "); Serial.print(loraLoopTime); Serial.println(F(" ms"));
}

// Function to listen to messages and write to datafile
void LoRa_receive(){

  // Volatile boolean for if a message is received
  volatile bool rxFlag = false;
  
  talkToRadio();

  if (manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      Serial.print("Got data from SnowBot #");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
      rxFlag = true; //Set rxFlag to true
      
      // Send a reply back to the originator client
      if (!manager.sendtoWait(rx_reply, sizeof(rx_reply), from))
      {
          Serial.println("sendtoWait failed");
          rxFlag = false; //Set rxFlag to false if sendtoWait fails
      }
    }
  }

  if (rxFlag) {
    // Add received buf into rx_message
    memcpy(&rx_message, buf, sizeof(rx_message)); // Copy received into rx_message
    
    // Write received rx message to the RockBlock buffer
    writeBuffer_rx();
  
    // Write received message to SD file
    talkToSD(); // Talk to SD card
  
    // Check if logging is enabled
    if (logFlag == true) {
      // Check that maximum file sample limit has not been exceeded
      if (samplesSaved >= samplesPerFile) {
      createLogFile();
      samplesSaved = 0;
      }
  
      // Write to microSD card
      if (file.open(fileName, O_APPEND | O_WRITE)) {
        samplesSaved++;   //  Increment sample count of current file
        file.print(rx_message.node);
        file.write(",");
        file.print(rx_message.unixtime);
        file.write(",");
        file.print(rx_message.voltage/(float)1000);
        file.write(",");
        file.print(rx_message.intTemperature/(float)100);
        file.write(",");
        file.print(rx_message.extTemperature/(float)100);
        file.write(",");
        file.print(rx_message.humidity/(float)100);
        file.write(",");
        file.print(rx_message.distMaxbotix_av);
        file.write(",");
        file.print(rx_message.distMaxbotix_std);
        file.write(",");
        file.print(rx_message.distMaxbotix_max);
        file.write(",");
        file.print(rx_message.distMaxbotix_min);
        file.write(",");
        file.println(rx_message.distMaxbotix_nan);
        writeTimestamps();
        file.close();
        
        #if DEBUG
        Serial.print(rx_message.node);
        Serial.print(",");
        Serial.print(rx_message.unixtime);
        Serial.print(",");
        Serial.print(rx_message.voltage/(float)1000);
        Serial.print(",");
        Serial.print(rx_message.intTemperature/(float)100);
        Serial.print(",");
        Serial.print(rx_message.extTemperature/(float)100);
        Serial.print(",");
        Serial.print(rx_message.humidity/(float)100);
        Serial.write(",");
        Serial.print(rx_message.distMaxbotix_av);
        Serial.write(",");
        Serial.print(rx_message.distMaxbotix_std);
        Serial.write(",");
        Serial.print(rx_message.distMaxbotix_max);
        Serial.write(",");
        Serial.print(rx_message.distMaxbotix_min);
        Serial.write(",");
        Serial.println(rx_message.distMaxbotix_nan);
        blinkLed(LED_PIN, 2, 100);
        #endif
      }
      else {
        Serial.println(F("Unable to open file"));
        logFlag = false;
      }
    }
  }
  
  // Clear data stored in rx_message
  memset(rx_message.bytes, 0x00, sizeof(rx_message));

  // Clear rxFlag
  rxFlag = false;
}

// Write received data from LoRa to RockBlock buffer
void writeBuffer_rx() {

  // Increment transmit counter
  transmitCounter++;
  
  // Concatenate current tx_message with existing tx_message(s) stored in transmit buffer
  memcpy(transmitBuffer + (sizeof(rx_message) * (transmitCounter + (retransmitCounter * transmitInterval) - 1)), rx_message.bytes, sizeof(rx_message)); // Copy rx_message to transmit buffer

  // Print data to Serial Monitor
//  printUnion();             // Print data stored in union
//  printUnionBinary();       // Print data stored in union in binary format
//  printTransmitBuffer();    // Print data stored transmit buffer array in binary format

}
