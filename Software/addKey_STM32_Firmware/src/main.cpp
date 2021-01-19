
#define isInDebug 1

#include <Arduino.h>
#include <SPI.h> //SD and MFRC522
#include <Keyboard.h>
#include <Mouse.h>
#include <main.h>








// Needed for Shift Register
#define NUMBER_OF_SHIFT_CHIPS   3 // How many shift register chips are daisy-chained.
#define DATA_WIDTH              NUMBER_OF_SHIFT_CHIPS * 8

#define ploadPin                PA8  // SH_LD
#define dataPin                 PA9 // Q7
#define clockPin                PA10 //CLK

// Needed for Joystick
#define XAxisPin PA0    
#define YAxisPin PA1


///FOR  VERSION > 2.1 this will be Not PA4 anymore
#define SD_SS PA4//10 
#define RFID_SS PB4
#define RFID_RST PB3

#define RFID_ON 1

HardwareSerial debug(USART2);

int joyValX;
int joyValY;
int MouseMoveX;
int MouseMoveY;
int JoyOffX=0;
int JoyOffY=0;
int JoyDeadZone=60;
int Release;

int MouseSteps=24;
int threshold = 1;
int center = MouseSteps / 2;

char lastfilename[32]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

unsigned int keysPressed=0;
uint32_t elapsedTime;


#include <SD.h>
File myFile;

// RFID
#if (RFID_ON) 
  #include <MFRC522.h>
  MFRC522 mfrc522(RFID_SS, RFID_RST);   // SS,RST 
  MFRC522::StatusCode status; //variable to get card status
  uint8_t pageAddr = 0x01;
  byte RFIDbuffer[64]={0,0,0,0};
  byte RFIDbuffer_size = sizeof(RFIDbuffer);
#endif



#define numOfMaxKeyDefinitions 64
#define numPressedAtOnce 4
byte ReadInputBuffer[numPressedAtOnce]={0,0,0,0};
int actNumOfKeyDefinitions=0;
unsigned long keyDelay[numOfMaxKeyDefinitions];
byte keyBoard1[numOfMaxKeyDefinitions*(numPressedAtOnce+3)]=
   {
     
     1,0,0,65,0,25
     ,2,0,0,66,0,25
     ,3,0,0,67,0,25
     ,4,0,0,68,0,25
     ,5,0,0,69,0,25
     ,6,0,0,70,0,25
     ,7,0,0,71,0,25
     ,8,0,0,72,0,25
     ,9,0,0,97,0,25
     ,10,0,0,98,0,25
     ,11,0,0,99,0,25
     ,12,0,0,100,0,25
     ,13,0,0,101,0,25
     ,14,0,0,102,0,25
     ,15,0,0,103,0,25
     ,18,0,0,104,0,25
     ,19,0,0,107,0,0
     ,20,0,0,108,0,0
     ,21,0,0,109,0,0
     ,22,0,0,105,0,0
     ,23,0,0,106,0,0
     ,50,0,0,170,0,0
     ,52,0,0,172,0,0
     ,53,0,0,173,0,0
     ,54,0,0,174,0,0
     ,55,0,0,175,0,0 
     ,56,0,0,176,0,0
     ,57,0,0,177,0,0
     ,58,0,0,178,0,0
     ,59,0,0,179,0,0
     ,60,0,0,180,0,0
     ,61,0,0,181,0,0

    

  };
//Key1+key2+Key3+F1+F2+Delay




void setup() 
{
    
    pinMode(ploadPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(dataPin, INPUT);
    digitalWrite(clockPin, LOW);
    digitalWrite(ploadPin, HIGH);

    
    #ifdef isInDebug
    debug.begin(9600);
    delay(500);
    while (!debug){delay(10);}
    debug.println("STAeeeeRT!!!");
    #endif




    //SDCARD
    // Open serial communications and wait for port to open:
  if (!SD.begin(SD_SS)) {
    debug.println("No SD FOUND!");
    while (1);
  }
#ifdef isInDebug
  debug.println("REadFile");
#endif

readKeyFile("default.key");



#if (RFID_ON) 
    //MFRC RFID Setup
    mfrc522.PCD_Init(RFID_SS, RFID_RST);        // Init MFRC522 card
#endif

elapsedTime=millis();

    Keyboard.begin();
    Mouse.begin();

//get joyoffsets
JoyOffY = 512-analogRead(YAxisPin);
JoyOffX = 512-analogRead(XAxisPin);
  
#ifdef isInDebug
debug.println("Init Done!");
#endif

}

void loop() 
{
   unsigned int orgKeys=0;
   int tPos;

 char filename[32]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
 
#if (RFID_ON)
  if (millis()>elapsedTime+1000)
  {
    elapsedTime=millis();
  //Check if RFID changed and setup new one
  //***************************************
    // Look for new cards
    if ( mfrc522.PICC_IsNewCardPresent()) 
    {
      if (  mfrc522.PICC_ReadCardSerial())
      {
        // Read data ***************************************************
            RFIDbuffer_size= sizeof(RFIDbuffer);
            status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(7, RFIDbuffer,  &RFIDbuffer_size);
            if (status != MFRC522::STATUS_OK) 
            {
              #ifdef isInDebug
                debug.print(F("MIFARE_Read() failed: "));
                debug.println(mfrc522.GetStatusCodeName(status));
              #endif              
            }else{
              int dr=0;
              while (dr<16 && RFIDbuffer[dr]!=0xfe)
              {
                filename[dr]=char(RFIDbuffer[dr]);
                dr++;
              }
              if (strcmp(filename,lastfilename))
              {
                #ifdef isInDebug
                  debug.print("Would read: ");
                  debug.println(filename);
                #endif

                readKeyFile(filename);
                memcpy(lastfilename,filename,32);

                
              }else{
                #ifdef isInDebug
                  debug.println("No Change!");
                  debug.println(filename);
                  debug.println(lastfilename);
                  debug.println();
                #endif
              }
            }              
      }
    } 

  }
#endif 
 
 
 //Get all data from the shift register
  readKeys();
   //bringe die bits in die reihehnfolge wie sie auf dem keyboard sind
  for (int x=0; x<8; x++)
  {
    bitWrite(orgKeys,x,bitRead(keysPressed,x+16));
    bitWrite(orgKeys,x+8,bitRead(keysPressed,x+8));
    bitWrite(orgKeys,x+16,bitRead(keysPressed,x));
  }
  


//We have the rotary encoder on PIN 18 and PIN19 and will interpret Left as PIN 18=0 and Right as PIN19=0
//So we have a left right turn equal to a button press

int a=rotaryEncoder(!bitRead(orgKeys,18),!bitRead(orgKeys,19));
if (a==-1){
  bitWrite(orgKeys,18,0);
  bitWrite(orgKeys,19,1);
}else if (a==1){
  bitWrite(orgKeys,18,1);
  bitWrite(orgKeys,19,0);
}else{
  bitWrite(orgKeys,18,1);
  bitWrite(orgKeys,19,1);
 }




  //For Debug Check which keys are pressed
    #ifdef isInDebug
/*
    for(int i = 0; i < DATA_WIDTH; i++)
    {
        if(( orgKeys>> i) & 1)
        {}
        else
        {
            debug.println(String(i));
        }

    }
*/
    #endif




 byte readByte=0; 
 ReadInputBuffer[0]=0;
 ReadInputBuffer[1]=0;
 ReadInputBuffer[2]=0;
 ReadInputBuffer[3]=0;
  //Now go through all Inputs and check if some are pressed. If so Write into ReadInputBuffer. Max of "numPressedAtOnce" Keys at once are allowed
  for (int tKeyCounter=0;tKeyCounter<24;tKeyCounter++)
  {
    if (!bitRead(orgKeys,tKeyCounter))
    {
      ReadInputBuffer[readByte]=tKeyCounter+1;   //+1 because our keys should start at 1 because 0 means nothing pressed
      readByte++;
      if (readByte>=numPressedAtOnce+1)return;
    }
  }

  if (readByte==4)
  {
    #ifdef isInDebug
      debug.print(ReadInputBuffer[0]);
      debug.print("-");
      debug.print(ReadInputBuffer[1]);
      debug.print("-");
      debug.print(ReadInputBuffer[2]);
      debug.print("-");
      debug.print(ReadInputBuffer[3]);
      debug.println();
      readKeyFile(String(ReadInputBuffer[0])+".key");
      delay(500);
    #endif

  }


  //Check if JoyMovement
  joyValX=readJoyAxis(XAxisPin);
  joyValY=readJoyAxis(YAxisPin);
  
  if (joyValX || joyValY){
    ReadInputBuffer[readByte]=50;
    readByte++;
  }


  int stdKey=0;
  int shiftKey=0;
  int Release=1;
  if (ReadInputBuffer[0]!=0)//Key has been pressed
  {
    //Check if we have an hit in the Keyboard Matrix
    for (int ArrayCounter=0;ArrayCounter<actNumOfKeyDefinitions;ArrayCounter++)
    {
      tPos=ArrayCounter*6;
      if (keyBoard1[tPos]==ReadInputBuffer[0] && keyBoard1[tPos+1]==ReadInputBuffer[1] && keyBoard1[tPos+2]==ReadInputBuffer[2])
      {
          #ifdef isInDebug
           debug.print(ReadInputBuffer[0]);
           debug.print("-");
           debug.print(ReadInputBuffer[1]);
           debug.print("-");
           debug.print(ReadInputBuffer[2]);
           debug.print(" /// ");
           debug.print(keyBoard1[tPos+3]);
           debug.print(" / ");
           debug.println(keyBoard1[tPos+4]);
          #endif

        if (keyDelay[ArrayCounter]<millis())
          {

            keyDelay[ArrayCounter]=millis()+keyBoard1[tPos+5]*10;//Set the next time this key is allowed to be pressed
            //Check for special shift key
            shiftKey=keyBoard1[tPos+3];
            if (shiftKey)
            {  
              doShiftKey(shiftKey);
            }
                    
            //Normal Key            
            stdKey=keyBoard1[tPos+4];
            if (stdKey!=170)
            {
            Keyboard.press(stdKey);
            #ifdef isInDebug
              debug.println(keyBoard1[tPos+4]);
            #endif

            MouseMoveX=0;
            MouseMoveY=0;
            
            }else{
              
            MouseMoveX=joyValX;
            MouseMoveY=joyValY;
            #ifdef isInDebug
              debug.print(MouseMoveY);
              debug.print("-");
              debug.println(MouseMoveX);
            #endif

            }
          }
          if (MouseMoveX||MouseMoveY)
          {
            Mouse.move(MouseMoveX,MouseMoveY);   
          }    
         if(Release){Keyboard.releaseAll();}         
      }
    }
  }else{
    stopMouse();
  }
   
}



void doShiftKey(int keyVal)
{
  
  switch (keyVal)
  {
    case 1: // Shift
      Keyboard.press(KEY_LEFT_SHIFT);
      break;
    case 2: // ALT
      Keyboard.press(KEY_LEFT_ALT);
      break;
    case 3: // CTRL
      Keyboard.press(KEY_LEFT_CTRL);
      break;
    case 4: // Shift + Alt
      Keyboard.press(KEY_LEFT_SHIFT);
      Keyboard.press(KEY_LEFT_ALT);
      break;
    case 5: // Shift + CTRL
      Keyboard.press(KEY_LEFT_SHIFT);
      Keyboard.press(KEY_LEFT_CTRL);
      break;
    case 6: // ALT + CTRL
      Keyboard.press(KEY_LEFT_ALT);
      Keyboard.press(KEY_LEFT_CTRL);
      break;
    case 7: // Shift + CTRL +ALT
      Keyboard.press(KEY_LEFT_SHIFT);
      Keyboard.press(KEY_LEFT_ALT);
      Keyboard.press(KEY_LEFT_CTRL);
      break;
    case 20: // LEFT Mouse Button
      if (!Mouse.isPressed(MOUSE_LEFT)) {
        Mouse.press(MOUSE_LEFT);
      }
      break;
    case 21: // RIGHT Mouse Button
      if (!Mouse.isPressed(MOUSE_RIGHT)) {
        Mouse.press(MOUSE_RIGHT);
      }
    case 22: // MIDDLE Mouse Button
      if (!Mouse.isPressed(MOUSE_MIDDLE)) {
        Mouse.press(MOUSE_MIDDLE);
      }
      break;
    case 23: // LEFT Mouse Button + SHIFT
      Keyboard.press(KEY_LEFT_SHIFT);
      if (!Mouse.isPressed(MOUSE_LEFT)) {
        Mouse.press(MOUSE_LEFT);
      }
      Release=0;
      break;
    case 24: // RIGHT Mouse Button + SHIFT
      Keyboard.press(KEY_LEFT_SHIFT);
      if (!Mouse.isPressed(MOUSE_RIGHT)) {
        Mouse.press(MOUSE_RIGHT);
      }
      Release=0;
    case 25: // MIDDLE Mouse Button + SHIFT
      Keyboard.press(KEY_LEFT_SHIFT);
      if (!Mouse.isPressed(MOUSE_MIDDLE)) {
        Mouse.press(MOUSE_MIDDLE);
      }
      Release=0;
      break;
  }
}

void stopMouse()
{
  if (Mouse.isPressed(MOUSE_LEFT)) 
  {
    Mouse.release(MOUSE_LEFT);
  }
  if (Mouse.isPressed(MOUSE_RIGHT)) 
  {
    Mouse.release(MOUSE_RIGHT);
  }
  if (Mouse.isPressed(MOUSE_MIDDLE)) 
  {
    Mouse.release(MOUSE_MIDDLE);
  }
}


void readKeys()
{
   unsigned int bitVal;
   keysPressed=0;


    digitalWrite(ploadPin, LOW);
    delayMicroseconds(10);
    digitalWrite(ploadPin, HIGH);

    /* Loop to read each bit value from the serial out line
     * of the SN74HC165N.
    */
    for(int i = 0; i < DATA_WIDTH; i++)
    {
        bitVal = digitalRead(dataPin);
        keysPressed |= (bitVal << ((DATA_WIDTH-1) - i));
        digitalWrite(clockPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(clockPin, LOW);
    }
}


//Taken from the Arduino JoystickMouseControl Example
int readJoyAxis(int thisAxis) {
  int Offs=0;
  // read the analog input:
  if (thisAxis==XAxisPin)
  {
    Offs=JoyOffX;
    #ifdef isInDebug
      //debug.print("X:");
    #endif
  }else{
    Offs=JoyOffY;
    #ifdef isInDebug
      //debug.print("Y:");
    #endif
  }
  int reading = analogRead(thisAxis)+Offs;
  
  if (reading>(512-JoyDeadZone) && reading<(512+JoyDeadZone)) reading=512;
  // map the reading from the analog input range to the output range:
  int mapreading = map(reading, 0, 1023, -MouseSteps, MouseSteps);
  #ifdef isInDebug 
    //debug.print(reading);
    //debug.print("--");
    //debug.println(mapreading);
  #endif

  return mapreading;
  // if the output reading is outside from the rest position threshold, use it:
  /*
  int distance = mapreading - center;

  if (abs(distance) < distance) {
    distance = 0;
  }

  // return the distance for this axis:
  return distance;
  */
}

void readKeyFile(String filename)
{
  #ifdef isInDebug
    debug.print("Try to find ");
    debug.println(filename);
  #endif  
  myFile = SD.open(filename);
  int tCount=0;
  if (myFile) {

    // read from the file until there's nothing else in it:
    while (myFile.available()) 
    {
      keyBoard1[tCount]=myFile.read();
      //debug.print(keyBoard1[tCount],DEC);
      //debug.print(" - ");
      tCount++;
      if (!(tCount%6))debug.println();
    }
    actNumOfKeyDefinitions=tCount/6;
    //debug.print(actNumOfKeyDefinitions);
    //debug.println("key definitions read");
    // close the file:
    myFile.close();
    #ifdef isInDebug
    debug.print(filename);
    debug.println(" Loaded!");
    #endif

  } else {
    // if the file didn't open, print an error:
    #ifdef isInDebug
    debug.println("File not found!");
    #endif
  }
}


int8_t rotaryEncoder(bool a, bool b) {
  int delta = 0;
  enum {STATE_LOCKED, STATE_TURN_RIGHT_START, STATE_TURN_RIGHT_MIDDLE, STATE_TURN_RIGHT_END, STATE_TURN_LEFT_START, STATE_TURN_LEFT_MIDDLE, STATE_TURN_LEFT_END, STATE_UNDECIDED};
  static uint8_t encoderState = STATE_LOCKED;
  //bool a = !digitalRead(ENCODER_A_PIN);
  //bool b = !digitalRead(ENCODER_B_PIN);
  //bool s = !digitalRead(SWITCH_PIN);
  //static bool switchState = s;
  switch (encoderState) {
    case STATE_LOCKED:
      if (a && b) {
        encoderState = STATE_UNDECIDED;
      }
      else if (!a && b) {
        encoderState = STATE_TURN_LEFT_START;
      }
      else if (a && !b) {
        encoderState = STATE_TURN_RIGHT_START;
      }
      else {
        encoderState = STATE_LOCKED;
      };
      break;
    case STATE_TURN_RIGHT_START:
      if (a && b) {
        encoderState = STATE_TURN_RIGHT_MIDDLE;
      }
      else if (!a && b) {
        encoderState = STATE_TURN_RIGHT_END;
      }
      else if (a && !b) {
        encoderState = STATE_TURN_RIGHT_START;
      }
      else {
        encoderState = STATE_LOCKED;
      };
      break;
    case STATE_TURN_RIGHT_MIDDLE:
    case STATE_TURN_RIGHT_END:
      if (a && b) {
        encoderState = STATE_TURN_RIGHT_MIDDLE;
      }
      else if (!a && b) {
        encoderState = STATE_TURN_RIGHT_END;
      }
      else if (a && !b) {
        encoderState = STATE_TURN_RIGHT_START;
      }
      else {
        encoderState = STATE_LOCKED;
        delta = -1;
      };
      break;
    case STATE_TURN_LEFT_START:
      if (a && b) {
        encoderState = STATE_TURN_LEFT_MIDDLE;
      }
      else if (!a && b) {
        encoderState = STATE_TURN_LEFT_START;
      }
      else if (a && !b) {
        encoderState = STATE_TURN_LEFT_END;
      }
      else {
        encoderState = STATE_LOCKED;
      };
      break;
    case STATE_TURN_LEFT_MIDDLE:
    case STATE_TURN_LEFT_END:
      if (a && b) {
        encoderState = STATE_TURN_LEFT_MIDDLE;
      }
      else if (!a && b) {
        encoderState = STATE_TURN_LEFT_START;
      }
      else if (a && !b) {
        encoderState = STATE_TURN_LEFT_END;
      }
      else {
        encoderState = STATE_LOCKED;
        delta = 1;
      };
      break;
    case STATE_UNDECIDED:
      if (a && b) {
        encoderState = STATE_UNDECIDED;
      }
      else if (!a && b) {
        encoderState = STATE_TURN_RIGHT_END;
      }
      else if (a && !b) {
        encoderState = STATE_TURN_LEFT_END;
      }
      else {
        encoderState = STATE_LOCKED;
      };
      break;
  }
/*
  uint32_t current_time = millis();
  static uint32_t switch_time = 0;
  const uint32_t bounce_time = 30;
  bool back = false;
  if (current_time - switch_time >= bounce_time) {
    if (switchState != s) {
      switch_time = current_time;
      back = s;
      switchState = s;
    }
  }
  return back;
  */
  return delta;
}