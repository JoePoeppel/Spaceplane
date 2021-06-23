#include <Wire.h>
#include <TinyGPS++.h>
#include <UbloxGPS.h>
#include <SPI.h>
#include <SFE_MicroOLED.h>
#include <SD.h>
#include <Servo.h>
#include <pitches.h>


// MICRO_OLED SETTINGS
#define PIN_RESET 9                                     
#define DC_JUMPER 1                                     

// SD CARD VARIABLES
#define chipSelect BUILTIN_SDCARD                     
File datalog;                                          
char filename[] = "SGP00.csv";                 
bool sdActive = false;                                  

// SERIAL DECLARATIONS
#define GPS_SERIAL Serial3
#define RFD900_Serial Serial1

// BAUD RATES
#define SERIAL_BAUD_RATE 9600                           
#define RFD900_BAUD_RATE 57600                          

// TEENSY 3.5 PIN CONNECTIONS
int SERVO = 23;
int LED1 = 24;
int LED2 = 25;
int BUZZER = 26;
int BUTTON1 = 27;
int BUTTON2 = 28;
int PULL_BEFORE_FLIGHT_PIN = 29;
int PULL_BEFORE_FLIGHT_PIN_2 = 1;
int LEDS_ACTIVE_PIN = 30;
int BUZZER_ACTIVE_PIN = 31;

// OBJECT DECLARATIONS
MicroOLED oled(PIN_RESET, DC_JUMPER);                                                             
Servo servo;
UbloxGPS gps(&GPS_SERIAL);

// GENERAL VARIABLES
String Header;
String Data;
String Spacer = ", ";
String Endline = "\n";
String Command;
bool Buzzer_Active = true;
bool LEDs_Active = true;
bool OLED_Active = true;
bool Flight_Active = false;
bool Header_Logged = false;
bool Not_Released = true;
bool Skip_Setup = false;
bool Emergency_Release = false;
float Flight_Start_Time;
float Current_Altitude;
float Previous_Altitude = 800;
float Altitude_Buffer = 200; ////////////////////////////////////
float Previous_Altitude_2 = 0;
int Descent_Counter = 0;
int Servo_Release_Mode = 0;
int RFD_Log_Count = 1;
int RFD_Log_Count_Separator = 11;
int Timer = 0;
int Engage_Check_Descent = 0; ////////////////////////////////////////////////
int Display = 0;


// SPEED DERIVATION VARIABLES
float Altitude_New;
float Altitude_Old = 0;
float Time_New;
float Time_Old = 0;
float Vertical_Speed;
float Horizontal_Speed;

float Phi;
float Lambda;
float Lat_1 = 0;
float Lon_1 = 0;
float Lat_2 = 0;
float Lon_2 = 0;
int lat_degree;
float lat_minute;
int lon_degree;
float lon_minute;

float a;
float c;
float R = 3958.8;
float Distance;



// GPS VARIABLES
bool GPS_Airborne_Active = false;
float GPS_Altitude;
float GPS_Latitude;
float GPS_Longitude;
int GPS_Satellites = 0;
int GPS_Fixes = 0;
String GPS_Fix = "No fix!";

// SERVO VARIABLES
bool Servo_Moved = false;
int Servo_Open_Position = 180;
int Servo_Closed_Position = 2;
int Servo_Hold_Position = 140;



// *********** USER INPUT VARIABLES *********** //
int Release_Altitude = 7000;
int Release_Time = 60 * 60;  // Minutes x 60 seconds/min


// SETUP FUNCTION
void PREFLIGHT();                                
void OLED_SETUP();
void SERVO_SETUP();
void SD_SETUP();
void TEST_TELEMETRY_SEND();
void TEST_TELEMETRY_RECEIVE();
void GPS_SETUP();
void HOLD();

// LOOP FUNCTIONS
void FLIGHT();
void UPDATE_GPS();
void UPDATE_RELEASE();
void CHECK_UNEXPECTED_DESCENT();
void UPDATE_SPEEDS();
void UPDATE_DATA();
void UPDATE_SD(String text);
void UPDATE_RFD900(String text, int x);
void UPDATE_DISPLAY();
void CHECK_COMMAND();

// EXTRA USER FUNCTIONS
void UPDATE_OLED();
void CHECK_LEDS_ACTIVE();
void CHECK_BUZZER_ACTIVE();
bool CHECK_FLIGHT_ACTIVE();
void BLINK_LED(int x);
void BLINK_LEDS(int x);
void BLINK_LEDS_ERROR();
void UPDATE_BUZZER_INITIATING(int x);
void UPDATE_BUZZER_SUCCESS();
void UPDATE_BUZZER_ERROR();
void UPDATE_BUZZER_PREFLIGHT(int x);
void UPDATE_BUZZER_STARTING_FLIGHT();
void UPDATE_BUZZER_FLIGHT(int x);
void SKIP_SETUP();
void CHECK_RESET();

// COMMANDS
void PREFLIGHT_COMMAND();
void FLIGHT_COMMAND();
void SERVO_COMMAND();
void RELEASE_COMMAND();
void BUZZER_COMMAND();
void LED_COMMAND();
void OLED_COMMAND();




void setup() {
    
    PREFLIGHT();

}



void loop() {

    FLIGHT();

}





// PREFLIGHT PROCEDURE & FUNCTIONS
void PREFLIGHT() {
  
    Serial.begin(SERIAL_BAUD_RATE);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(PULL_BEFORE_FLIGHT_PIN, INPUT_PULLUP);
    pinMode(PULL_BEFORE_FLIGHT_PIN_2, INPUT_PULLUP);
    pinMode(LEDS_ACTIVE_PIN, INPUT_PULLUP);
    pinMode(BUZZER_ACTIVE_PIN, INPUT_PULLUP);
    pinMode(BUTTON1, INPUT);
    pinMode(BUTTON2, INPUT_PULLDOWN);
    servo.attach(SERVO);
    
    BLINK_LED(1);
    BLINK_LED(2);
    BLINK_LEDS(2);
    //UPDATE_BUZZER_INITIATING(4);

    CHECK_LEDS_ACTIVE();
    CHECK_BUZZER_ACTIVE();
    
    OLED_SETUP();
    delay(10);
    RFD900_SETUP();
    delay(10);
    SD_SETUP();
    delay(10);
    SKIP_SETUP();
    if (Skip_Setup == false) {
      SERVO_SETUP();
      delay(10);
      //TEST_TELEMETRY_SEND();
      delay(10);
      //TEST_TELEMETRY_RECEIVE();
      delay(10);
    }
    GPS_SETUP();
    delay(10);
    UPDATE_BUZZER_SUCCESS();
    HOLD();
}


void OLED_SETUP() {
    Serial.print("Setting up OLED...");
    Wire.begin();
    oled.begin();
    oled.clear(ALL);
    oled.display();
    oled.clear(PAGE);
    randomSeed(analogRead(A0)+ analogRead(A1));
    Serial.println(" complete!");
    UPDATE_OLED("OLED\nonline!");
    UPDATE_BUZZER_SUCCESS();
    BLINK_LEDS(2);
    delay(200);
    UPDATE_OLED("Spaceplanebooting\nup!");
    delay(500);
}


void RFD900_SETUP() {
    Serial.print("Setting up RFD900...");
    UPDATE_OLED("Setting upRFD900");
    RFD900_Serial.begin(RFD900_BAUD_RATE);
    delay(500);
    Serial.println(" complete!");
    UPDATE_OLED("RFD900\nonline!");
    UPDATE_BUZZER_SUCCESS();
    BLINK_LEDS(2);
}


void SD_SETUP(){
  pinMode(chipSelect, OUTPUT);
  Serial.print("Setting up SD card... ");
  if(!SD.begin(chipSelect)) {
    Serial.println("SD card failed, or not present");
    UPDATE_OLED("SD Card\nfailed\nor\nnot\npresent!");
    UPDATE_BUZZER_ERROR();
    delay(200); 
  }
  else {
    Serial.print("        card initialized! \nCreating File...             ");
    for (byte i = 0; i<100; i++) {
      filename[3] = '0' + i/10;
      filename[4] = '0' + i%10;
      if(!SD.exists(filename)) {
        datalog = SD.open(filename, FILE_WRITE);
        sdActive = true;
        BLINK_LEDS(2);
        UPDATE_BUZZER_SUCCESS();
        Serial.println("complete!");
        Serial.println("Logging to: " + String(filename));
        UPDATE_OLED("SD\nonline!");
        delay(200);
        UPDATE_OLED("Logging:\n\n" + String(filename));
        delay(200);
        break;}
      }

      if(!sdActive) {
        Serial.println("No available file names; clear SD card to enable logging");
        UPDATE_OLED("Warning:\n\nSD Card\nfull!\n\nClear ASAP");
        UPDATE_BUZZER_ERROR();
        BLINK_LEDS_ERROR();
        delay(200);
        }
    }
}

void SERVO_SETUP() {
  
    Serial.println("Beginning servo attachment process...");
    UPDATE_OLED("Beginning\nservo\nattachmentprocess...");
    delay(200);
    Serial.println("Push button 1 to open/close servo - push button 2 to move on with program...");
    digitalWrite(LED1, HIGH);
    while(1) {
     UPDATE_OLED("Button 1:\nopen/close\nButton 2:\nmove on");
        if(digitalRead(BUTTON1) == HIGH && Servo_Moved == false) {
            BLINK_LED(2);
            servo.write(Servo_Open_Position);
            Servo_Moved = true;
            Serial.println("Servo open!");
            UPDATE_RFD900("Servo open!\n", 0);
            UPDATE_OLED("Servo\nopen!");
            UPDATE_BUZZER_INITIATING(2);
            delay(1000);
            BLINK_LED(2);
        }

        else if (digitalRead(BUTTON1) == HIGH && Servo_Moved == true) {
            BLINK_LED(2);
            servo.write(Servo_Hold_Position);
            for (int i = Servo_Hold_Position; i>Servo_Closed_Position; i = i-2) {
              UPDATE_OLED(String(i));
              servo.write(i);
              delay(250);
              if (i == Servo_Hold_Position) { 
                while(1) {
                  UPDATE_OLED("Press\nbutton 1\nto\ncontinue\nclosing...");
                  if (digitalRead(BUTTON1) == HIGH) {break;}
                }
              }
            }
            servo.write(Servo_Closed_Position);
            Servo_Moved = false;
            Serial.println("Servo closed!");
            UPDATE_RFD900("Servo closed!\n", 0);
            UPDATE_OLED("Servo\nclosed!");
            UPDATE_BUZZER_INITIATING(2);
            delay(1000);
            BLINK_LED(2);
        }

        if (digitalRead(BUTTON2) == HIGH) {
            Serial.println("Servo setup complete!");
            UPDATE_RFD900("Servo setup complete!\n", 0);
            UPDATE_OLED("Servo\nsetup\ncomplete!");
            UPDATE_BUZZER_SUCCESS();
            BLINK_LEDS(2);
            digitalWrite(LED1, LOW);
            delay(200);
            break;
        }
    }
}


void TEST_TELEMETRY_SEND() {
    digitalWrite(LED1, HIGH);
    Serial.println("Testing send telemetry...");
    UPDATE_OLED("Testing\nsend\ntelemetry");
    while(1) {
        if (digitalRead(BUTTON1) == HIGH) {
            Serial.println("TESTING TELEMETRY!");
            UPDATE_RFD900("TESTING TELEMTRY!\n", 0);
            UPDATE_BUZZER_INITIATING(2);
            delay(1000);
            BLINK_LED(2);
        }

        if (digitalRead(BUTTON2) == HIGH) {
            BLINK_LEDS(2);
            digitalWrite(LED1, LOW);
            UPDATE_BUZZER_SUCCESS();
            UPDATE_RFD900("exit", 0);
            UPDATE_OLED("Testing\nsend\ntelemetry\ncomplete!");
            Serial.println("Testing send telemetry complete!");
            delay(200);
            break;
        }
    }
}


void TEST_TELEMETRY_RECEIVE() {
    digitalWrite(LED1, HIGH);
    Serial.println("Testing receive telemetry...");
    UPDATE_OLED("Testing\nreceive\ntelemetry");
    while(1) {
        String Command;
        String Clear_Buffer;

        if(Serial.available() > 0) {
            Command = Serial.readString();
            Serial.println("Command: " + Command);
            UPDATE_OLED("Command:\n" + Command);
            BLINK_LED(2);
            UPDATE_BUZZER_INITIATING(2);
            delay(1000);
            UPDATE_OLED("");
            if(Command == "exit") {break;};
            if(RFD900_Serial.available() > 0) {Clear_Buffer = Serial.readString();}
        }

        if(RFD900_Serial.available() > 0) {
            Command = RFD900_Serial.readString();
            Serial.println("Command: " + Command);
            UPDATE_OLED("Command:\n" + Command);
            BLINK_LED(2);
            UPDATE_BUZZER_INITIATING(2);
            delay(1000);
            UPDATE_OLED("");
            if(Command == "exit") {break;};
            if(Serial.available() > 0) {Clear_Buffer = Serial.readString();}
        }

        if (digitalRead(BUTTON2) == HIGH) {
            BLINK_LEDS(2);
            digitalWrite(LED1, LOW);
            UPDATE_BUZZER_SUCCESS();
            UPDATE_OLED("Testing\nreceive\ntelemetry\ncomplete!");
            Serial.println("Testing receive telemetry complete!");
            delay(200);
            break;
        }
        delay(100);
    }
}

void GPS_SETUP() {

  float Current_Time = millis();
  
    Serial.println("Setting up GPS...");
    UPDATE_RFD900("Setting up GPS...\n", 0);
    UPDATE_OLED("Setting\nup GPS");
    GPS_SERIAL.begin(UBLOX_BAUD);
    gps.init();
    int i = 0;
    while (i < 50) {
        i++;
        if (gps.setAirborne()) {
            Serial.println("Airborne mode successfully set!");
            UPDATE_RFD900("Airborne mode successfully set!\n", 0);
            UPDATE_OLED("Airborne\nmode set!");
            UPDATE_BUZZER_INITIATING(2);
            BLINK_LEDS(2);
            GPS_Airborne_Active = true;
            break;
        }
        if (i == 50) {
            Serial.println("Failed to set to airborne mode.");
            UPDATE_RFD900("Failed to set to airborne mode.\n", 0);
            UPDATE_OLED("Failed\nto set\nair mode.");
            UPDATE_BUZZER_ERROR();
            BLINK_LEDS_ERROR();
        }
    }
    
    Serial.println("Waiting for GPS fixes...");
    UPDATE_RFD900("Waiting for GPS fixes...\n", 0);
    UPDATE_OLED("Waiting\nfor GPS\nfixes...");
    
    while(gps.getSats() < 6){
      if (millis() - Current_Time > 10000) {break;}
      gps.update();
        if(GPS_Fixes != gps.getSats()) {
            Serial.println("GPS Fixes: " + String(gps.getSats()));
            UPDATE_RFD900("GPS Fixes: " + String(gps.getSats()) + "\n", 0);
            UPDATE_OLED("GPS Fixes:\n" + String(gps.getSats()));
            BLINK_LED(1);
            GPS_Fixes = gps.getSats();
        }

        if(digitalRead(BUTTON2) == HIGH) {
            Serial.println("GPS Fixes: " + String(gps.getSats()) + "\nForced to move on with program via button!");
            UPDATE_RFD900("GPS Fixes: " + String(gps.getSats()) + "\nForced to move on with program via button!\n", 0);
            UPDATE_OLED("GPS Fixes:" + String(gps.getSats()));
            BLINK_LED(1);
            break;
        }
       delay(10);
      }
    

    Serial.println("GPS setup complete!");
    UPDATE_RFD900("GPS setup complete!\n", 0);
    UPDATE_OLED("GPS setup\ncomplete!\n\nFixes: " + String(gps.getSats()));
    UPDATE_BUZZER_SUCCESS();
    BLINK_LEDS(2);
}

void HOLD() {
    int i = 0;
    while(1) {
      
        gps.update();

        if (millis() - Timer > 500) {
          CHECK_COMMAND();
          CHECK_LEDS_ACTIVE();
          CHECK_BUZZER_ACTIVE();
          BLINK_LED(2);
          UPDATE_BUZZER_PREFLIGHT(2);
          Timer = millis();
  
          if(i < 1) {
              Serial.println("Wating to start flight...");
              UPDATE_RFD900("Waiting to start flight...\n", 0);
              i++;
          }

          UPDATE_OLED("Waiting\nto start\nflight...\n\nFixes: " + String(gps.getSats()));
  
          if(CHECK_FLIGHT_ACTIVE() == true) {
              Serial.println("Flight starting!");
              UPDATE_RFD900("Flight starting!\n", 0);
              UPDATE_OLED("Flight\nstarting!");
              Serial.println("\n\nRelease Altitude: " + String(Release_Altitude) + " feet");
              Serial.println("Release Time: " + String(Release_Time) + " seconds\n");
              UPDATE_BUZZER_STARTING_FLIGHT();
              BLINK_LEDS(3);
              Flight_Start_Time = millis();
              break;
          }
          Timer = millis();
        }
        
        delay(10);
    }
}


bool CHECK_FLIGHT_ACTIVE() {
    if (digitalRead(PULL_BEFORE_FLIGHT_PIN) == LOW || digitalRead(PULL_BEFORE_FLIGHT_PIN_2) == LOW ) {
        Flight_Active = false;
        return false;
    }

    else {
        Flight_Active = true;
        return true;
    }
}






// FLIGHT PROCEDURE & FUNCTIONS
void FLIGHT() {

    gps.update();
    
    if (millis() - Timer > 500) { 
      CHECK_LEDS_ACTIVE();
      CHECK_BUZZER_ACTIVE();
      CHECK_RESET();
      UPDATE_GPS();
      CHECK_UNEXPECTED_DESCENT();
      UPDATE_RELEASE();
      UPDATE_DATA();
      UPDATE_SD(Data);
     //if (RFD_Log_Count_Separator > 10) {UPDATE_RFD900(Data, 1); RFD_Log_Count++; RFD_Log_Count_Separator = 0;};
      UPDATE_DISPLAY();
      CHECK_COMMAND();
      UPDATE_BUZZER_FLIGHT(1);
      BLINK_LED(1);
      //RFD_Log_Count_Separator++;
      Timer = millis();
    }
    delay(7);
}


void UPDATE_GPS() {
    
    GPS_Altitude = gps.getAlt_feet();
    GPS_Longitude = gps.getLon();
    GPS_Latitude = gps.getLat();
    GPS_Satellites = gps.getSats();

    if (gps.getFixAge() > 2000) {GPS_Fix = "No fix!";}
    else {GPS_Fix = "Fix!"; BLINK_LED(2);}
}


void UPDATE_RELEASE () {

    if (GPS_Altitude - Previous_Altitude < Altitude_Buffer) {
    
      if ((GPS_Altitude >= Release_Altitude || (millis() - Flight_Start_Time)/1000 >= Release_Time) && Not_Released == true) {
      
          Not_Released = false;
          if (GPS_Altitude >= Release_Altitude) {Servo_Release_Mode = 1;}
          else {Servo_Release_Mode = 2;}
  
          servo.write(Servo_Open_Position);
          UPDATE_RFD900("Servo released due to Mode " + String(Servo_Release_Mode) + "! We will see how this goes...\n", 0);
          UPDATE_OLED("Servo\nreleased!");
          Serial.println("\nServo released due to Mode " + String(Servo_Release_Mode) + "! We will see how this goes...\n");
          delay(1000);
          servo.write(Servo_Hold_Position);
          delay(250);
          servo.write(Servo_Open_Position);
      }
    }
    Previous_Altitude = GPS_Altitude;
}


void CHECK_UNEXPECTED_DESCENT() {
    
    Current_Altitude = GPS_Altitude;

  if (Current_Altitude > Engage_Check_Descent) {
    if (Current_Altitude <= Previous_Altitude_2) {
        Descent_Counter++;
    }

    else if (Current_Altitude > Previous_Altitude_2) {
        Descent_Counter = 0;
    }
    
    Previous_Altitude_2 = Current_Altitude;
    
    if (Descent_Counter > 20 && Emergency_Release == false) {
        servo.write(Servo_Open_Position);
        UPDATE_RFD900("Unexpected descent detected! Releasing servo!\n", 0);
        Serial.println("Unexpected descent detected! Releasing servo!");
        UPDATE_OLED("Unexpected\ndescent\ndetected!\nReleasing\nservo!");
        UPDATE_BUZZER_ERROR();
        Servo_Release_Mode = 4;
        Emergency_Release = true;
    }
  }
}


void UPDATE_SPEEDS() {

    // Vertical Speed
    Altitude_New = GPS_Altitude;
    Time_New = millis()/1000.0;
    Vertical_Speed = (Altitude_New - Altitude_Old) / (Time_New - Time_Old)  * 3600.0 / 5280.0;

    // Horizontal Speed
    Phi = GPS_Latitude;
    Lambda = GPS_Longitude;
  
    int lat_degree = Phi;
    float lat_minute = Phi - lat_degree;

    int lon_degree = Lambda;
    float lon_minute = Lambda - lon_degree;

    Lat_1 = (lat_degree + lat_minute*100.0/60.0)*PI/180.0;
    Lon_1 = (lon_degree + lon_minute*100.0/60.0)*PI/180.0;

    Distance = 2*asin( sqrt( pow(sin((Lat_1 - Lat_2)/2), 2) + cos(Lat_1)*cos(Lat_2)*pow(sin((Lon_1 - Lon_2)/2),2)  ) ) *180.0 * 60.0 / PI * 1852.0 * 3.28084;
    
    Horizontal_Speed = Distance / (Time_New - Time_Old) * 3600.0 / 5280.0;

    // For Next Iteration
    Altitude_Old = Altitude_New;
    Time_Old = Time_New;
    Lat_2 = Lat_1;
    Lon_2 = Lon_1;
}


void UPDATE_DATA() {
  
    UPDATE_SPEEDS();
    
    if (Header_Logged == false) {
        Data = "Time(s), Lat(dg), Long(dg), Alt(ft), Vertical Speed(mph), Horizontal Speed(mph), GPS Satellites, GPS Fix, Servo Release Mode";
        Header_Logged = true;
    }

    else {
        Data = String((millis() - Flight_Start_Time)/1000) + Spacer + String(GPS_Latitude) + Spacer + String(GPS_Longitude) + Spacer + String(GPS_Altitude) + Spacer + String(Vertical_Speed) + Spacer + String(Horizontal_Speed) + Spacer + String(GPS_Satellites) + Spacer + GPS_Fix + Spacer  + String(Servo_Release_Mode);
    }
}


void UPDATE_SD(String text) {
    datalog = SD.open(filename, FILE_WRITE);
    datalog.println(text);
    datalog.close();
    Serial.println(text);
}


void UPDATE_RFD900(String text, int x) {

  if (x == 0) { text = text; }
  else {text = "Data, " + text;}
  
  RFD900_Serial.print(text);

}


void UPDATE_DISPLAY() {
 
  String display;
  if(digitalRead(BUTTON1) == HIGH) { 
    
    Display++; 
    if (Display > 3)  Display = 0;

  }
      
    switch(Display) {
      case 0: display = "Time:" + String(int((millis() - Flight_Start_Time)/1000)) + "\nLat:" + String(GPS_Latitude) + "\nLon:" + String(GPS_Longitude) + "Alt:" + String(GPS_Altitude) + "\nSats: " + String(GPS_Satellites); break;
      case 1: display = "R_Alt:\n" + String(Release_Altitude) + "\n\nR_Time:\n" + String(Release_Time); break;
      case 2: display = "VelocitiesVertical\n" + String(Vertical_Speed) + "\nHorizontal" + String(Horizontal_Speed); break;
      case 3: display = GPS_Fix + "\n\nRFD:" + String(RFD_Log_Count) + "\n\nServo: " + String(Servo_Release_Mode); break;
    }
 
  UPDATE_OLED(display);
}


void CHECK_COMMAND() {
    String Clear_Buffer;

    if (RFD900_Serial.available() > 0) {
        Command = RFD900_Serial.readString();
        if     (Command == "Preflight") {PREFLIGHT_COMMAND();}
        else if(Command == "Flight")    {FLIGHT_COMMAND();}
        else if(Command == "Servo")     {SERVO_COMMAND();}
        else if(Command == "Release")   {RELEASE_COMMAND();}
        else if(Command == "Buzzer")    {BUZZER_COMMAND();}
        else if(Command == "Leds")      {LED_COMMAND();}
        else if(Command == "Oled")      {OLED_COMMAND();}

        if(Serial.available() > 0) {Clear_Buffer = Serial.readString();}
    }

    if(Serial.available() > 0) {
        Command = Serial.readString();
        if     (Command == "Preflight") {PREFLIGHT_COMMAND();}
        else if(Command == "Flight")    {FLIGHT_COMMAND();}
        else if(Command == "Servo")     {SERVO_COMMAND();}
        else if(Command == "Release")   {RELEASE_COMMAND();}
        else if(Command == "Buzzer")    {BUZZER_COMMAND();}
        else if(Command == "Leds")      {LED_COMMAND();}
        else if(Command == "Oled")      {OLED_COMMAND();}

        if(RFD900_Serial.available() > 0) {Clear_Buffer = Serial.readString();}
    }
}






// ADDITIONAL FUNCTIONS
void UPDATE_OLED(String text) {
    if(OLED_Active == true) {
    oled.clear(PAGE);
    oled.setFontType(0);
    oled.setCursor(0,0);
    oled.print(text);
    oled.display();
    }

    else {oled.clear(PAGE);}
}


void CHECK_LEDS_ACTIVE() {
    if (digitalRead(LEDS_ACTIVE_PIN) == LOW) {
        LEDs_Active = true;
    }

    else {
       LEDs_Active = false;
    }
}


void CHECK_BUZZER_ACTIVE() {
    if (digitalRead(BUZZER_ACTIVE_PIN) == LOW) {
        Buzzer_Active = true;
    }

    else {
        Buzzer_Active = false;
    }
}


void BLINK_LED(int x) {

    if (LEDs_Active == true) {
    if (x == 1) {
      digitalWrite(LED1, HIGH);
      delay(10);
      digitalWrite(LED1, LOW);
    }
  
    if (x == 2) {
      digitalWrite(LED2, HIGH);
      delay(10);
      digitalWrite(LED2, LOW);
    }
  
    if(x != 1 && x != 2) {
      {Serial.println("Error: 'x' value inputted into function 'blinkLED(x)' not valid! - - - Please use x = 1 or x = 2"); UPDATE_OLED("Error:\nblinkLED\nUse valid\nx input!");}
      while(1){}
    }
  }
}


void BLINK_LEDS(int x) {
    if (LEDs_Active == true) {
    for (int i = 0; i<x; i++) {
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
      delay(10);
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      delay(10);
    }
  }
}


void BLINK_LEDS_ERROR() {
    if (LEDs_Active == true) {
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, HIGH);
        delay(5000);
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, LOW);
    }
}


void UPDATE_BUZZER_INITIATING(int x) {
    if (Buzzer_Active == true) {
    for (int i = 0; i < x; i++) {
        tone(BUZZER,NOTE_D7);
        delay(250);
        noTone(BUZZER);
        delay(250);
    }    
    }  
}


void UPDATE_BUZZER_SUCCESS() {

    if (Buzzer_Active == true) {
    int NOTE_SUSTAIN = 50;
    for(int i = 0; i < 1; i++)
        {
          tone(BUZZER, NOTE_A5);
          delay(NOTE_SUSTAIN);
          tone(BUZZER,NOTE_B5);
          delay(NOTE_SUSTAIN);
          tone(BUZZER,NOTE_C5);
          delay(NOTE_SUSTAIN);
          tone(BUZZER,NOTE_B5);
          delay(NOTE_SUSTAIN);
          tone(BUZZER,NOTE_C5);
          delay(NOTE_SUSTAIN);
          tone(BUZZER,NOTE_D5);
          delay(NOTE_SUSTAIN);
          tone(BUZZER,NOTE_C5);
          delay(NOTE_SUSTAIN);
          tone(BUZZER,NOTE_D5);
          delay(NOTE_SUSTAIN);
          tone(BUZZER,NOTE_E5);
          delay(NOTE_SUSTAIN);
          tone(BUZZER,NOTE_D5);
          delay(NOTE_SUSTAIN);
          tone(BUZZER,NOTE_E5);
          delay(NOTE_SUSTAIN);
          tone(BUZZER,NOTE_E5);
          delay(NOTE_SUSTAIN);
        }
        noTone(BUZZER);
    }
}


void UPDATE_BUZZER_ERROR() {
    if (Buzzer_Active == true) {
        tone(BUZZER,NOTE_G4);
        delay(250);
        tone(BUZZER,NOTE_C4);
        delay(500);
        noTone(BUZZER);
    }
}


void UPDATE_BUZZER_PREFLIGHT(int x) {
    if (Buzzer_Active == true) {
        for (int i = 0; i < x; i++) {
            tone(BUZZER,NOTE_D5);
            delay(20);
            noTone(BUZZER);
            delay(10);
        }   
        
    }
}


void UPDATE_BUZZER_STARTING_FLIGHT() {
    if (Buzzer_Active == true) {
        tone(BUZZER, NOTE_E5);
        delay(100);
        tone(BUZZER, NOTE_D5);
        delay(100);
        tone(BUZZER, NOTE_C5);
        delay(100);
        tone(BUZZER, NOTE_G5);
        delay(100);
    }
}


void UPDATE_BUZZER_FLIGHT(int x) {
    if (Buzzer_Active == true) {
        for (int i = 0; i < x; i++) {
            tone(BUZZER,NOTE_C5);
            delay(15);
            noTone(BUZZER);
        }  
    }
}


void CHECK_LANDING() {
    // Not complete
}

void SKIP_SETUP() {
  float Current_Time = millis();
  Serial.println("Would you like to skip servo/telemtry setup? Press button 1 for no - press button 2 for yes:");
  UPDATE_OLED("Skip\nsetup?\n1 = no\n2 = yes");
  while(1) {
  if (millis() - Current_Time > 5000) {Skip_Setup = true; Serial.println("Skipping setup"); UPDATE_OLED("Skipping\nsetup"); break;}
    if (digitalRead(BUTTON1) == HIGH) {
      Serial.println("Moving on to setup.");
      UPDATE_OLED("Moving\non to\nsetup.");
      delay(1000);
      break;
    }
  
    else if (digitalRead(BUTTON2) == HIGH) {
      Skip_Setup = true;
      Serial.println("Skipping setup.");
      UPDATE_OLED("Skipping\nsetup.");
      delay(500);
      break;
    }
  }
}

void CHECK_RESET() {
  if(CHECK_FLIGHT_ACTIVE() == false) {
    
    Serial.println("Reseting unit...");
    UPDATE_OLED("Reseting\nunit!");
    servo.write(Servo_Open_Position);
    Header_Logged = false;
    Not_Released = true;
    Servo_Release_Mode = 0;
    GPS_Altitude = 0;
    PREFLIGHT();
  }
}



// COMMANDS
void PREFLIGHT_COMMAND() {
    UPDATE_RFD900("\nPreflight command!\n\n", 0);
    Serial.println("\nPreflight command!\n");
    UPDATE_OLED("Preflight\ncommand!");
    servo.write(Servo_Open_Position);
    Header_Logged = false;
    Not_Released = true;
    Servo_Release_Mode = 0;
    PREFLIGHT();
}


void FLIGHT_COMMAND() {
    UPDATE_RFD900("\nFlight command!\n\n", 0);
    Serial.println("\nFlight command!\n");
    UPDATE_OLED("Flight\ncommand!");
    Header_Logged = false;
    Not_Released = true;
    Servo_Release_Mode = 0;
    RFD_Log_Count = 1;
    RFD_Log_Count_Separator = 11;
    Flight_Start_Time = millis();
    loop();
}


void SERVO_COMMAND() {
    if (Servo_Moved == false) {
        servo.write(Servo_Open_Position);
        UPDATE_RFD900("\nServo opened via command!\n\n", 0);
        Serial.println("\nServo opened via command!\n");
        UPDATE_OLED("Servo\nopened\nvia\ncommand!");
        Servo_Moved = true;
    }

    else if (Servo_Moved == true) {
        servo.write(Servo_Closed_Position);
        UPDATE_RFD900("\nServo closed via command!\n", 0);
        Serial.println("\nServo closed via command!\n");
        UPDATE_OLED("Servo\nclosed\nvia\ncommand!");
        Servo_Moved = false;
    }
}


void RELEASE_COMMAND() {
    
    if(Not_Released == true) {
        servo.write(Servo_Open_Position);
        UPDATE_RFD900("\nServo released via command! We will see how this goes...\n\n", 0);
        Serial.println("\nServo released via command! We will see how this goes...\n");
        UPDATE_OLED("Servo\nreleased!\nvia\ncommand!");
        Servo_Moved = true;
        Not_Released = false;
        Servo_Release_Mode = 3;
    }

    else if(Not_Released == false) {
        UPDATE_RFD900("\nServo already released!\n\n", 0);
        Serial.println("\nServo already released!\n");
        UPDATE_OLED("Servo\nalready\nreleased!");
    }
}


void BUZZER_COMMAND() {

    if (Buzzer_Active == true) {
        Buzzer_Active = false;
        UPDATE_RFD900("\nBuzzer turned off via command!\n\n", 0);
        Serial.println("\nBuzzer turned off via command!\n");
        UPDATE_OLED("Buzzer\nturned\noff!");
        
    }
    else if(Buzzer_Active == false) {
        Buzzer_Active = true;
        UPDATE_RFD900("\nBuzzer turned on via command!\n\n", 0);
        Serial.println("\nBuzzer turned on via command!\n");
        UPDATE_OLED("Buzzer\nturned\non!");
    }

}


void LED_COMMAND() {

    if (LEDs_Active == true) {
        LEDs_Active = false;
        UPDATE_RFD900("\nLEDs turned off via command!\n\n", 0);
        Serial.println("\nLEDs turned off via command!\n");
        UPDATE_OLED("LEDs\nturned\noff!");
    }
    else if(LEDs_Active == false) {
        LEDs_Active = true;
        UPDATE_RFD900("\nLEDs turned on via command!\n\n", 0);
        Serial.println("\nLEDs turned on via command!\n");
        UPDATE_OLED("LEDs\nturned\non!");
    }
}


void OLED_COMMAND() {
    if(OLED_Active == true) {
        UPDATE_OLED("");
        OLED_Active = false;
        UPDATE_RFD900("\nOLED turned off via command!\n\n", 0);
        Serial.println("\nOLED turned off via command!\n");
    }

    else if (OLED_Active == false) {
        OLED_Active = true;
        UPDATE_RFD900("\nOLED turned on via command!\n\n", 0);
        Serial.println("\nOLED turned on via command!\n");
    }
}
