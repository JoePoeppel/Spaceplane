#include <Wire.h>
#include <TinyGPS++.h>
#include <UbloxGPS.h>
#include <SPI.h>
#include <SFE_MicroOLED.h>
#include <SD.h>
#include <Servo.h>
#include <pitches.h>

// MICRO_OLED SETTINGS
#define PIN_RESET 9                                     // The SFE_MicroOLED.h library assumes a reset pin is necessary.The Qwiic OLED has RST hard-wired, so pick an arbitrary IO pin that is not being used.
#define DC_JUMPER 1                                     // The DC_JUMPER is the I2C Address Select jumper. Set to 1 if the jumper is open (default)

// SD CARD VARIABLES
#define chipSelect BUILTIN_SDCARD                       // Using built in chipselect on Teensy 3.5
File datalog;                                           // File object to be opened, written to, and closed
char filename[] = "Spaceplane00.csv";                   // File name as will be seen on SD card -- can have maximum of 99 files on SD card ('GLM00' -> 'GLM99')
bool sdActive = false;                                  // Boolean to check if there are any available filenames left to be used 

// SERIAL DECLARATIONS
#define GPS_SERIAL Serial3
#define RFD900_Serial Serial1

// BAUD RATES
#define SERIAL_BAUD_RATE 9600                           // Baud rate of serial monitor 
#define RFD900_BAUD_RATE 57600                          // Baud rate of RFD900 Radio

// TEENSY 3.5 PIN CONNECTIONS
int SERVO = 23;
int LED1 = 24;
int LED2 = 25;
int BUZZER = 26;
int BUTTON1 = 27;
int BUTTON2 = 28;
int PULL_BEFORE_FLIGHT_PIN = 29;
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
bool Flight_Active = false;
float Flight_Start_Time;
bool Header_Logged = false;
float Current_Altitude;
float Previous_Altitude = 0;
int Descent_Counter = 0;
int Servo_Release_Mode = 0;
bool Not_Released = true;
String Command;
bool Buzzer_Active = true;
bool LEDs_Active = true;
bool OLED_Active = true;

// SPEED DERIVATION VARIABLES
float Altitude_New;
float Altitude_Old = 0;
float Time_New;
float Time_Old = 0;
float Vertical_Speed;
float Horizontal_Speed;
float Total_Speed;
float Phi_1;
float Phi_2 = 0;;
float Delta_Phi;
float Lambda_1;
float Lambda_2 = 0;
float Delta_Lambda;
float a;
float c;
float R = 3958.8;
float Distance;

// OLED VARIABLES


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
int Servo_Open_Position = 0;
int Servo_Closed_Position = 180;



// *********** USER INPUT VARIABLES *********** //
int Burn_Altitude = 10000;
int Burn_Time = 40 * 60;  // Minutes x 60 seconds/min



// SETUP FUNCTION
void PREFLIGHT();                                
void OLED_SETUP();
void GPS_SETUP();
void SERVO_SETUP();
void SD_SETUP();
void HOLD();

// LOOP FUNCTIONS
void FLIGHT();
void UPDATE_GPS();
void UPDATE_RELEASE();
void CHECK_UNEXPECTED_DESCENT();
void UPDATE_SPEEDS();
void UPDATE_DATA();
void UPDATE_SD(String text);
void UPDATE_RFD900(String text);
void UPDATE_DISPLAY();
void CHECK_COMMAND();

// EXTRA USER FUNCTIONS
void UPDATE_OLED();
void CHECK_LEDS_ACTVE();
void CHECK_BUZZER_ACTIVE();
bool CHECK_FLIGHT_ACTIVE();
void BLINK_LED(int x);
void BLINK_LEDS(int x);
void BLINK_LEDS_ERROR();
void UPDATE_BUZZER_INITIATING(int x);
void UPDATE_BUZZER_SUCCESS();
void UPDATE_BUZZER_ERROR();
void UPDATE_BUZZER_PREFLIGHT(int x);
void UPDATE_BUZZER_FLIGHT(int x);

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

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(PULL_BEFORE_FLIGHT_PIN, INPUT_PULLUP);
    pinMode(LEDS_ACTIVE_PIN, INPUT_PULLUP);
    pinMode(BUZZER, INPUT_PULLUP);
    pinMode(BUTTON1, INPUT);
    pinMode(BUTTON2, INPUT);

    BLINK_LED(1);
    BLINK_LED(2);
    BLINK_LEDS(2);
    UPDATE_BUZZER_INITIATING(4);

    Serial.begin(SERIAL_BAUD_RATE);

    OLED_SETUP();
    delay(1000);
    RFD900_SETUP();
    delay(1000);
    GPS_SETUP();
    delay(1000);
    SERVO_SETUP();
    delay(1000);
    SD_SETUP();
    delay(1000);
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
    UPDATE_BUZZER_INITIATING(1);
    BLINK_LEDS(2);
    delay(1500);
    UPDATE_OLED("Spaceplane\ninitiating...");
    delay(1500);
}


void RFD900_SETUP() {
    Serial.print("Setting up RFD900...");
    UPDATE_OLED("Setting up\nRFD900");
    RFD900_Serial.begin(RFD900_BAUD_RATE);
    delay(2000);
    Serial.println(" complete!");
    UPDATE_OLED("RFD900\nonline!");
    UPDATE_BUZZER_INITIATING(1);
    BLINK_LEDS(2);
}


void GPS_SETUP() {
    GPS_SERIAL.begin(UBLOX_BAUD);
    gps.init();
    int i = 0;
    while (i < 50) {
        i++;
        if (gps.setAirborne()) {
            Serial.println("Airborne mode successfully set!");
            UPDATE_RFD900("Airborne mode successfully set!");
            UPDATE_OLED("Airborne\nmode set!");
            UPDATE_BUZZER_INITIATING(1);
            BLINK_LEDS(2);
            GPS_Airborne_Active = true;
            break;
        }
        if (i == 50) {
            Serial.println("Failed to set to airborne mode.");
            UPDATE_RFD900("Failed to set to airborne mode.");
            UPDATE_OLED("Failed to\nset air mode.");
            UPDATE_BUZZER_ERROR();
            BLINK_LEDS_ERROR();
        }
    }
    
    Serial.println("Waiting for GPS fixes...");
    UPDATE_RFD900("Waiting for GPS fixes...");
    while(gps.getSats() < 4){

        if(GPS_Fixes != gps.getSats()) {}
            Serial.println("GPS Fixes: " + String(gps.getSats()));
            UPDATE_RFD900("GPS Fixes: " + String(gps.getSats()));
            UPDATE_OLED("GPS Fixes: " + String(gps.getSats()));
            BLINK_LED(1);
            GPS_Fixes = gps.getSats();
    }

    Serial.println("At least 4 GPS fixes established!\nGPS setup complete!");
    UPDATE_RFD900("At leastt 4 GPS fixes established!\nGPS setup complete!");
    UPDATE_OLED("GPS setup\ncomplete\n\n Fixes: " + String(gps.getSats()));
    UPDATE_BUZZER_INITIATING(1);
    BLINK_LEDS(2);
}


void SERVO_SETUP() {
    servo.attach(SERVO);
    Serial.println("Beginning servo attachment process...");
    UPDATE_OLED("Beginning\nservo\nattachment\nprocess...");
    while(1) {
        if(digitalRead(BUTTON1) == HIGH && Servo_Moved == false) {
            BLINK_LED(1);
            servo.write(Servo_Open_Position);
            Servo_Moved = true;
            Serial.println("Servo open!");
            UPDATE_OLED("Servo open!");
            UPDATE_BUZZER_INITIATING(2);
            BLINK_LED(2);
            delay(2000);
        }

        else if (digitalRead(BUTTON1) == HIGH && Servo_Moved == true) {
            BLINK_LED(1);
            servo.write(Servo_Closed_Position);
            Servo_Moved = false;
            Serial.println("Servo closed!");
            UPDATE_OLED("Servo closed!");
            UPDATE_BUZZER_INITIATING(2);
            BLINK_LED(2);
            delay(2000);
        }

        if (digitalRead(BUTTON2) == HIGH) {
            Serial.println("Servo setup complete!");
            UPDATE_OLED("Servo\nsetup\ncomplete!");
            UPDATE_BUZZER_INITIATING(3);
            BLINK_LEDS(2);
            delay(2000);
            break;
        }
        delay(1000);
    }
}


void SD_SETUP(){
  pinMode(chipSelect, OUTPUT);

  if(!SD.begin(chipSelect)) {
    Serial.println("SD card failed, or not present");
    UPDATE_OLED("SD Card\nfailed\nor\nnot\npresent");
    UPDATE_BUZZER_ERROR();
    delay(1000); 
  }
  else {
    Serial.print("        card initialized! \nCreating File...             ");
    for (byte i = 0; i<100; i++) {
      filename[10] = '0' + i/10;
      filename[11] = '0' + i%10;
      if(!SD.exists(filename)) {
        datalog = SD.open(filename, FILE_WRITE);
        sdActive = true;
        BLINK_LEDS(2);
        UPDATE_BUZZER_INITIATING(1);
        Serial.println("complete!");
        Serial.println("Logging to: " + String(filename));
        UPDATE_OLED("SD\nonline!");
        delay(500);
        UPDATE_OLED("Logging:\n\n" + String(filename));
        delay(500);
        break;}
      }

      if(!sdActive) {
        Serial.println("No available file names; clear SD card to enable logging");
        UPDATE_OLED("Warning:\n\nSD Card\nfull!\n\nClear ASAP");
        UPDATE_BUZZER_ERROR();
        BLINK_LEDS_ERROR();
        delay(500);
        }
    }
}


void HOLD() {
    int i = 0;
    while(1) {
        if(i < 1) {
            Serial.println("Wating to start flight...");
            UPDATE_RFD900("Waiting to start flight...");
            UPDATE_OLED("Waiting\nto start\flight...");
            i++;
        }

        if(CHECK_FLIGHT_ACTIVE() == true) {
            Serial.println("Flight starting!");
            UPDATE_RFD900("Flight starting!");
            UPDATE_OLED("Flight\nstarting!");
            UPDATE_BUZZER_FLIGHT(3);
            BLINK_LEDS(3);
            Flight_Start_Time = millis();
            break;
        }
        CHECK_COMMAND();
        CHECK_LEDS_ACTVE();
        CHECK_BUZZER_ACTIVE();
        BLINK_LED(2);
        UPDATE_BUZZER_PREFLIGHT(2);
        delay(1000);
    }
}


bool CHECK_FLIGHT_ACTIVE() {
    if (digitalRead(PULL_BEFORE_FLIGHT_PIN) == LOW) {
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
    UPDATE_GPS();
    UPDATE_RELEASE();
    CHECK_UNEXPECTED_DESCENT();
    UPDATE_DATA();
    UPDATE_SD(Data);
    UPDATE_RFD900(Data);
    UPDATE_DISPLAY();
    CHECK_COMMAND();
    UPDATE_BUZZER_FLIGHT(1);
    BLINK_LED(1);
    delay(5000);
}


void UPDATE_GPS() {
    gps.update();
    GPS_Altitude = gps.getAlt_feet();
    GPS_Longitude = gps.getLon();
    GPS_Latitude = gps.getLat();
    GPS_Satellites = gps.getSats();

    if (gps.getFixAge() > 2000) {GPS_Fix = "No fix!,";  digitalWrite(LED2, HIGH); delay(150); digitalWrite(LED2, LOW);}
    else {GPS_Fix = "Fix!,"; BLINK_LED(2); delay(50);BLINK_LED(2);}
}


void UPDATE_RELEASE () {

    if ((GPS_Altitude >= Burn_Altitude || (millis()/1000) - (Flight_Start_Time/1000) >= Burn_Time) && Not_Released == true) {
        
        Not_Released = false;
        if (GPS_Altitude >= Burn_Altitude) {Servo_Release_Mode = 1;}
        else {Servo_Release_Mode = 2;}

        servo.write(Servo_Open_Position);
        UPDATE_RFD900("Servo released! We will see how this goes...");
        UPDATE_OLED("Servo\nreleased!");
        Serial.println("Servo released! We will see how this goes...");
        
    }
}


void CHECK_UNEXPECTED_DESCENT() {
    
    Current_Altitude = GPS_Altitude;

    if (Current_Altitude < Previous_Altitude) {
        Descent_Counter++;
    }

    else if (Current_Altitude > Previous_Altitude) {
        Descent_Counter = 0;
    }

    if (Descent_Counter > 5) {
        servo.write(Servo_Open_Position);
        UPDATE_RFD900("Unexpected descent detected! Releasing servo!");
        Serial.println("Unexpected descent detected! Releasing servo!");
    }


    Previous_Altitude = Current_Altitude;
}


void UPDATE_SPEEDS() {

    // Vertical Speed
    Altitude_New = GPS_Altitude;
    Time_New = millis()/1000.0;
    Vertical_Speed = (Altitude_New - Altitude_Old) / (Time_New - Time_Old)  * 3600.0 / 5280.0;

    // Horizontal Speed
    Phi_1 = GPS_Latitude;
    Delta_Phi = Phi_1 - Phi_2;

    Lambda_1 = GPS_Longitude;
    Delta_Lambda = Lambda_1 - Lambda_2;

    // Haversine Formula for Horizontal Speed
    a = sin(Delta_Phi/2.0) * sin(Delta_Phi/2.0) + cos(Phi_1) * cos(Phi_2) * sin(Delta_Lambda/2.0) * sin(Delta_Lambda/2.0);
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    Distance = R*c;

    Horizontal_Speed = Distance / (Time_New - Time_Old);

    // Total Speed
    Total_Speed = sqrt(pow(Vertical_Speed,2) + pow(Horizontal_Speed,2));

    // For Next Iteration
    Altitude_Old = Altitude_New;
    Time_Old = Time_New;
    Phi_2 = Phi_1;
    Lambda_2 = Lambda_1;
}


void UPDATE_DATA() {
    
    if (Header_Logged == false) {
        Data = "Time(s), Lat(dg), Long(dg), Alt(ft), Vertical Speed(mph), Horizontal Speed(mph), Total Speed(mph), GPS Satellites, GPS Fix, Servo Release Mode";
        Header_Logged = true;
    }

    else {
        Data = String((millis() - Flight_Start_Time)/1000) + Spacer + String(GPS_Latitude) + Spacer + String(GPS_Longitude) + Spacer + String(GPS_Altitude) + Spacer + String(Vertical_Speed) + Spacer + String(Horizontal_Speed) + Spacer + String(Total_Speed) + Spacer + String(GPS_Satellites) + Spacer + GPS_Fix + Spacer + String(Servo_Release_Mode);
    }
}


void UPDATE_SD(String text) {
    datalog = SD.open(filename, FILE_WRITE);
    datalog.println(text);
    datalog.close();
}


void UPDATE_RFD900(String text) {

    RFD900_Serial.println(text);
    Serial.println(text);

}


void UPDATE_DISPLAY() {
    String display = "Time: " + String(millis()/1000) + "\nLat: " + String(GPS_Latitude) + "\nLong: " + String(GPS_Longitude) + "\nAlt: " + String(GPS_Altitude) + "\nSats: " + String(GPS_Satellites);
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


void CHECK_LEDS_ACTVE() {
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
      delay(50);
      digitalWrite(LED1, LOW);
    }
  
    if (x == 2) {
      digitalWrite(LED2, HIGH);
      delay(50);
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
      delay(50);
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      delay(50);
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
    for(int i = 0; i < 2; i++)
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
            delay(250);
            noTone(BUZZER);
            delay(250);
        }   
        
    }
}


void UPDATE_BUZZER_FLIGHT(int x) {
    if (Buzzer_Active == true) {
        for (int i = 0; i < x; i++) {
            tone(BUZZER,NOTE_C5);
            delay(250);
            tone(BUZZER,NOTE_C6);
            delay(250);
            noTone(BUZZER);
        }  
    }
}


void CHECK_LANDING() {
    // Not complete
}




// COMMANDS
void PREFLIGHT_COMMAND() {
    UPDATE_RFD900("Preflight command!");
    Serial.println("Preflight command!");
    UPDATE_OLED("Preflight\ncommand!");
    PREFLIGHT();
}


void FLIGHT_COMMAND() {
    UPDATE_RFD900("Flight command!");
    Serial.println("Flight command!");
    UPDATE_OLED("Flight\ncommand!");
    loop();
}


void SERVO_COMMAND() {
    if (Servo_Moved == false) {
        servo.write(Servo_Open_Position);
        UPDATE_RFD900("Servo opened via command!");
        Serial.println("Servo opened via command!");
        UPDATE_OLED("Servo\nopened\nvia\ncommand!");
        Servo_Moved = true;
    }

    else if (Servo_Moved == true) {
        servo.write(Servo_Closed_Position);
        UPDATE_RFD900("Servo closed via command!");
        Serial.println("Servo closed via command!");
        UPDATE_OLED("Servo\nclosed\nvia\ncommand!");
        Servo_Moved = false;
    }
}


void RELEASE_COMMAND() {
    
    if(Not_Released == true) {
        servo.write(Servo_Open_Position);
        UPDATE_RFD900("Servo released via command! We will see how this goes...");
        Serial.println("Servo released via command! We will see how this goes...");
        UPDATE_OLED("Servo\nreleased!\nvia\ncommand!");
        Not_Released = false;
        Servo_Release_Mode = 3;
    }

    else if(Not_Released == false) {
        UPDATE_RFD900("Servo already released!");
        Serial.println("Servo already released!");
        UPDATE_OLED("Servo\nalready\nreleased!");
    }
}


void BUZZER_COMMAND() {

    if (Buzzer_Active == true) {
        Buzzer_Active = false;
        UPDATE_RFD900("Buzzer turned off via command!");
        Serial.println("Buzzer turned off via command!");
        UPDATE_OLED("Buzzer\nturned\noff!");
        
    }
    else if(Buzzer_Active == false) {
        Buzzer_Active = true;
        UPDATE_RFD900("Buzzer turned on via command!");
        Serial.println("Buzzer turned on via command!");
        UPDATE_OLED("Buzzer\nturned\non!");
    }

}


void LED_COMMAND() {

    if (LEDs_Active == true) {
        LEDs_Active = false;
        UPDATE_RFD900("LEDs turned off via command!");
        Serial.println("LEDs turned off via command!");
        UPDATE_OLED("LEDs\nturned\noff!");
    }
    else if(LEDs_Active == false) {
        LEDs_Active = true;
        UPDATE_RFD900("LEDs turned on via command!");
        Serial.println("LEDs turned on via command!");
        UPDATE_OLED("LEDs\nturned\non!");
    }
}


void OLED_COMMAND() {
    if(OLED_Active == true) {
        OLED_Active = false;
        UPDATE_RFD900("OLED turned off via command!");
        Serial.println("OLED turned off via command!");

    }

    else if (OLED_Active == false) {
        OLED_Active = true;
        UPDATE_RFD900("OLED turned on via command!");
        Serial.println("OLED turned on via command!");
        UPDATE_OLED("OLED\nturned\non!");

    }
}




