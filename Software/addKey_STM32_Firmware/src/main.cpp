//This code is far away from being perfect
//I am still debugging and didn't optimezed anything


#define isInDebug 1
#define useRFID 0
#define useJoystick 0
#define useRotary 0
#define useSD 0

#include <Arduino.h>
#include <SPI.h> //SD and MFRC522
#include <Keyboard.h>
#include <Mouse.h>
#include <main.h>

// Needed for Shift Register
#define NUMBER_OF_SHIFT_CHIPS   3 // How many shift register chips are daisy-chained.
#define DATA_WIDTH              NUMBER_OF_SHIFT_CHIPS * 8

#define ploadPin                PA8   // SH_LD
#define dataPin                 PA9   // Q7
#define clockPin                PA10  //CLK

///FOR  VERSION < 2.2 this will be PA4 
#define SD_SS PA4

#if (useRFID)
  #define RFID_SS PB4
  #define RFID_RST PB3
#endif


HardwareSerial debug(USART2);

#if (useJoystick)
  #define XAxisPin PA0    
  #define YAxisPin PA1

  int joyValX;
  int joyValY;
  int JoyOffX=0;
  int JoyOffY=0;
  int JoyDeadZone=60;
#endif

int MouseMoveX;
int MouseMoveY;
int Release;

int MouseSteps=24;
int threshold = 1;
int center = MouseSteps / 2;

char lastfilename[32]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

unsigned int keysPressed=0;
uint32_t elapsedTime;

#if (useSD)
  #include <SD.h>
  File myFile;
#endif
// RFID
#if (useRFID) 
  #include <MFRC522.h>
  MFRC522 mfrc522(RFID_SS, RFID_RST);   // SS,RST 
  MFRC522::StatusCode status; //variable to get card status
  uint8_t pageAddr = 0x01;
  byte RFIDbuffer[64]={0,0,0,0};
  byte RFIDbuffer_size = sizeof(RFIDbuffer);
#endif



#define numOfMaxKeyDefinitions 128
#define numPressedAtOnce 4
byte ReadInputBuffer[numPressedAtOnce]={0,0,0,0};

unsigned long keyDelay[numOfMaxKeyDefinitions];
unsigned long keyIsPressed[numOfMaxKeyDefinitions];

/*
The key definition has the following struct:
Key1,key2,Key3,F1,F2,Delay
everything needs to be ordered by Key1 then Key2 then Key3
if not it will not work!!!!!
*/
byte keyBoard1[numOfMaxKeyDefinitions*(numPressedAtOnce+3)]={
     1, 18, 0, 3, 122, 1
    ,1, 0, 0, 3, 99, 1
    ,2, 18, 0, 3, 121, 1
    ,2, 0, 0, 3, 120, 1
    ,3, 0, 0, 3, 118, 1
    ,5, 0, 0, 0, 177, 1
    ,6, 0, 0, 0, 97, 0
    ,7, 0, 0, 0, 98, 10
    ,8, 0, 0, 0, 99, 1
    ,9, 0, 0, 0, 100, 2

  };
int actNumOfKeyDefinitions=10;




//****************************************************
void setup() 
{

  
   
 

  pinMode(ploadPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);
  digitalWrite(clockPin, LOW);
  digitalWrite(ploadPin, HIGH);

  
  #if(isInDebug)
    debug.begin(9600);
    delay(500);
    while (!debug){delay(10);}
    debug.println("Start!!!");
  #endif

  //SDCARD
  #if (useSD)
    if (!SD.begin(SD_SS)) 
    {
      debug.println("No SD FOUND!");
    } 
    #ifdef isInDebug
      debug.println("REadFile");
    #endif
    readKeyFile("default.key");
  #endif

  #if (useRFID) 
      //MFRC RFID Setup
      mfrc522.PCD_Init(RFID_SS, RFID_RST);        // Init MFRC522 card
  #endif

  elapsedTime=millis();
  Keyboard.begin();
  Mouse.begin();

  //get joyoffsets
  #if (useJoystick)
    JoyOffY = 512-analogRead(YAxisPin);
    JoyOffX = 512-analogRead(XAxisPin);
  #endif

  #ifdef isInDebug
    debug.println("Init Done!");
  #endif

}

//****************************************************
void loop() 
{
   unsigned int orgKeys=0;
   int tPos;

 
  #if (useRFID)
  //char filename[32]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
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

  //orgKeys=keysPressed;
  //OK hier hab ich es etwas verbockt, normalerwiese sollte die reihenfolge genau wie die Tasten sein da ich aber seit meinem 1. Prototype die tasten verändert habe
  //und die reihenfolge aber nicht beachtet habe habe ich nun zwei möglichkeiten die reihenfolge hier zu ändern oder in der Software zum einstellen des Tastenlayouts.
  //Fürs erste mach ich es hier!
  //bringe die bits in die reihehnfolge wie sie auf dem keyboard sind
  for (int x=0; x<8; x++)
  {
    bitWrite(orgKeys,x,bitRead(keysPressed,x+16));
    bitWrite(orgKeys,x+8,bitRead(keysPressed,x+8));
    bitWrite(orgKeys,x+16,bitRead(keysPressed,x));
  }
  

  //So we have a left right turn equal to a button press
  #if (useRotery)
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


  //IF we hit the three buttons on the right and an additional one it will load a file with the number of the additional key
  #if (useSD)
    if (readByte==4)
    {
      if (ReadInputBuffer[1]==16 && ReadInputBuffer[2]==17 && ReadInputBuffer[1]==18)
      {
        readKeyFile(String(ReadInputBuffer[0])+".key");
        delay(500);
      }
    }
  #endif

  #if (useJoystick)
    //Check if JoyMovement
    joyValX=readJoyAxis(XAxisPin);
    joyValY=readJoyAxis(YAxisPin);
    
    if (joyValX || joyValY){
      ReadInputBuffer[readByte]=50;
      readByte++;
    }
  #endif

  int Release=1;
  if (ReadInputBuffer[0]!=0)//Key has been pressed
  {
    //Compare the Input with our Keyboard array
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

          
          if (keyBoard1[tPos+5]==0)
          {
            doSomething(tPos);
            Release=1;
            
          }
          else if (keyBoard1[tPos+5]==1)
          {
            if (!keyIsPressed[ArrayCounter])
            {
              keyIsPressed[ArrayCounter]=1;
              doSomething(tPos);
            }
          }else if (keyBoard1[tPos+5]==2)
          {
              keyIsPressed[ArrayCounter]=1;
          }

          else if (keyBoard1[tPos+5]>=10)
          {
            if (keyDelay[ArrayCounter]<millis())
            {
              keyDelay[ArrayCounter]=millis()+keyBoard1[tPos+5]*10;//Set the next time this key is allowed to be pressed
              //Here we zuweisen all functions
              doSomething(tPos);
              Release=1;
            }
          }

          if (MouseMoveX||MouseMoveY)
          {
            Mouse.move(MouseMoveX,MouseMoveY);   
          }    
          
          if(Release){Keyboard.releaseAll();
        }      
      }   
      
    }
  }else{
    stopMouse();
    //Nothing is pressed ... so check if there was a onRelase Key and  remove all flags
    for (int ArrayCounter=0;ArrayCounter<actNumOfKeyDefinitions;ArrayCounter++)
    {
      tPos=ArrayCounter*6;
      if (keyBoard1[tPos+5]==2 && keyIsPressed[ArrayCounter]==1)
      {
        doSomething(tPos);
      }
      keyIsPressed[ArrayCounter]=0;
    }
  }
}



void doSomething(int tPos){
  int stdKey=0;
  int shiftKey=0;

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
    
    #if (useJoystick)
      MouseMoveX=joyValX;
      MouseMoveY=joyValY;
      #ifdef isInDebug
        debug.print(MouseMoveY);
        debug.print("-");
        debug.println(MouseMoveX);
      #endif
    #endif
  }
}

//****************************************************
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

//****************************************************
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

//****************************************************
void readKeys()
{
  unsigned int bitVal;
  keysPressed=0;
  
  digitalWrite(ploadPin, LOW);
  delayMicroseconds(10);
  digitalWrite(ploadPin, HIGH);

  //Loop through Shift Register
  for(int i = 0; i < DATA_WIDTH; i++)
  {
      bitVal = digitalRead(dataPin);
      keysPressed |= (bitVal << ((DATA_WIDTH-1) - i));
      digitalWrite(clockPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(clockPin, LOW);
  }
}


//****************************************************
//Taken from the Arduino JoystickMouseControl Example
#if (useJoystick)
  int readJoyAxis(int thisAxis) 
  {
    int Offs=0;
    // read the analog input:
    if (thisAxis==XAxisPin)
    {
      Offs=JoyOffX;
    }else{
      Offs=JoyOffY;
    }
    int reading = analogRead(thisAxis)+Offs;
    
    if (reading>(512-JoyDeadZone) && reading<(512+JoyDeadZone)) reading=512;
    // map the reading from the analog input range to the output range:
    int mapreading = map(reading, 0, 1023, -MouseSteps, MouseSteps);
    return mapreading;
  }
#endif

//****************************************************
#if (useSD)
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
        tCount++;
        if (!(tCount%6))debug.println();
      }
      actNumOfKeyDefinitions=tCount/6;
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
#endif

//****************************************************
#if (useRotary)
  // Take from https://forum.arduino.cc/index.php?topic=701098.0
  int8_t rotaryEncoder(bool a, bool b) {
  int delta = 0;
  enum {STATE_LOCKED, STATE_TURN_RIGHT_START, STATE_TURN_RIGHT_MIDDLE, STATE_TURN_RIGHT_END, STATE_TURN_LEFT_START, STATE_TURN_LEFT_MIDDLE, STATE_TURN_LEFT_END, STATE_UNDECIDED};
  static uint8_t encoderState = STATE_LOCKED;
  switch (encoderState) 
    {
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
    return delta;
  }
#endif