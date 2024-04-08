#include "LowPower.h"
#include <IRremote.hpp>

#define wakeUpPin 2
#define irRecPin 10
#define irSendPin 11
#define irRecGndPin 12
#define ledIndicatorPin 13

// pins from start to end will be the input buttons
#define startOfInputPins 2
#define endOfInputPins 7

#define holdTimeout 1000

// Storage for the recorded code
struct storedIRDataStruct {
    IRData receivedIRData;
    // extensions for sendRaw
    uint8_t rawCode[RAW_BUFFER_LENGTH]; // The durations if raw
    uint8_t rawCodeLength; // The length of the code
};

#define NO_LED_FEEDBACK_CODE      // saves 92 bytes program memory
#define EXCLUDE_EXOTIC_PROTOCOLS // saves around 650 bytes program memory if all other protocols are active


storedIRDataStruct *IrData = new storedIRDataStruct[endOfInputPins-startOfInputPins];

void wakeUp()
{
    // Just a handler for the pin interrupt.
}

void setup()
{
    Serial.begin(115200);
    pinMode(13,OUTPUT);
    pinMode(irSendPin, OUTPUT);
    pinMode(ledIndicatorPin,OUTPUT);
    pinMode(irRecPin,INPUT);
    pinMode(irRecGndPin,OUTPUT);
    digitalWrite(irRecGndPin,HIGH);    
    for(int pin = startOfInputPins; pin <= endOfInputPins; pin++){
      pinMode(pin,INPUT);
    }
    digitalWrite(ledIndicatorPin,LOW);
    IrReceiver.begin(irRecPin, ENABLE_LED_FEEDBACK, ledIndicatorPin);       

    IrSender.begin(irSendPin);
}

void loop() 
{
  Serial.println("Initialized and working");
  Serial.flush();

  // Allow wake up pin to trigger interrupt on low.
  attachInterrupt(0, wakeUp, LOW);
  // Enter power down state with ADC and BOD module disabled.
  // Wake up when wake up pin is low.
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
  // Disable external pin interrupt on wake up pin.
  detachInterrupt(0); 
  for(int pin = endOfInputPins; pin >= startOfInputPins; pin--){
    if(!digitalRead(pin)){
      Serial.print("Got pin: ");
      Serial.println(pin);
      if(checkHold(pin)){
        digitalWrite(irRecGndPin,LOW);  
        IrReceiver.start();     
        recForButton(pin);
        digitalWrite(irRecGndPin,HIGH);    
        IrReceiver.resume();
      }else{
        Serial.flush();     
        IrReceiver.stop();
        playbackButton(pin);
      }
      break;
    }
  }
}

bool checkHold(int pin){
  long startPress = millis();
  while(!digitalRead(pin)){
    if(millis()-startPress > holdTimeout){
      Serial.println("Button held");
      return true;
    }
  }
  Serial.println("Button not held");
  return false;
}

void recForButton(int pin){
  storedIRDataStruct *dataPointer = &IrData[pin-2];
  IrReceiver.start();
  long flashTracker = millis();
  bool flash = false;
  digitalWrite(13,HIGH);

  while(!IrReceiver.decode());
  digitalWrite(13,LOW);
  if (IrReceiver.decodedIRData.rawDataPtr->rawlen < 4) {
        Serial.print(F("Ignore data with rawlen="));
        Serial.println(IrReceiver.decodedIRData.rawDataPtr->rawlen);
        return;
  }
  if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
      Serial.println(F("Ignore repeat"));
      return;
  }
  if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_AUTO_REPEAT) {
      Serial.println(F("Ignore autorepeat"));
      return;
  }
  if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_PARITY_FAILED) {
      Serial.println(F("Ignore parity error"));
      return;
  }
  /*
    * Copy decoded data
    */
  dataPointer->receivedIRData = IrReceiver.decodedIRData;

  if (dataPointer->receivedIRData.protocol == UNKNOWN) {
      Serial.print(F("Received unknown code and store "));
      Serial.print(IrReceiver.decodedIRData.rawDataPtr->rawlen - 1);
      Serial.println(F(" timing entries as raw "));
      IrReceiver.printIRResultRawFormatted(&Serial, true); // Output the results in RAW format
      dataPointer->rawCodeLength = IrReceiver.decodedIRData.rawDataPtr->rawlen - 1;
      /*
        * Store the current raw data in a dedicated array for later usage
        */
      IrReceiver.compensateAndStoreIRResultInArray(dataPointer->rawCode);
  } else {
      IrReceiver.printIRResultShort(&Serial);
      IrReceiver.printIRSendUsage(&Serial);
      dataPointer->receivedIRData.flags = 0; // clear flags -esp. repeat- for later sending
      Serial.println();
  }
  Serial.flush();
}

void playbackButton(int pin){
  
  storedIRDataStruct *aIRDataToSend = &IrData[pin-2];
  if (aIRDataToSend->receivedIRData.protocol == UNKNOWN /* i.e. raw */) {
        // Assume 38 KHz
        IrSender.sendRaw(aIRDataToSend->rawCode, aIRDataToSend->rawCodeLength, 38);

        Serial.print(F("raw "));
        Serial.print(aIRDataToSend->rawCodeLength);
        Serial.println(F(" marks or spaces"));
    } else {

        /*
         * Use the write function, which does the switch for different protocols
         */
        IrSender.write(&aIRDataToSend->receivedIRData);
        printIRResultShort(&Serial, &aIRDataToSend->receivedIRData, false);
    }
}
