/* Melbus CDCHGR Emulator
   Program that emulates the MELBUS communication from a CD-changer (CD-CHGR) in a Volvo V70 (HU-xxxx) to enable AUX-input through the 8-pin DIN-contact.
   This setup is using an Arduino Nano 5v clone

   The HU enables the CD-CHGR in its source-menue after a successful initialization procedure is accomplished.
   The HU will remove the CD-CHGR everytime the car starts if it wont get an response from CD-CHGR (second init-procedure).

   Karl Hagström 2015-11-04
   mod by S. Zeller 2016-03-14
   mod by V.Gijsen for bluetooth integration based on OVC3868

   This project went realy smooth thanks to these sources:
   http://volvo.wot.lv/wiki/doku.php?id=melbus
   https://github.com/festlv/screen-control/blob/master/Screen_control/melbus.cpp
   http://forums.swedespeed.com/showthread.php?50450-VW-Phatbox-to-Volvo-Transplant-(How-To)&highlight=phatbox

   pulse train width=120us (15us per clock cycle), high phase between two pulse trains is 540us-600us
*/

#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

#define OVC_SETPARING "AT#CB\r\n"
#define OVC_LASTPARING "AT#CC\r\n"
#define OVC_PLAY_NEXT "AT#MD\r\n"
#define OVC_PLAY_PREVIOUS "AT#ME\r\n"
#define OVC_PLAY_PAUSE "AT#MA\r\n"
#define OVC_PLAY_STOP "AT#MC\r\n"
#define OVC_RESET "AT#CZ\r\n"
#define OVC_PLAY_FR "AT#MR\r\n"
#define OVC_PLAY_FF "AT#MS\r\n"
#define OVC_PLAY_CANCEL_FAST "AT#MT\r\n"
#define OVC_SWITCH_DEVICES "AT#MZ\r\n"
enum cmds {
  NONE,
  PLAY,
  PAUSE,
  NEXT,
  PREVIOUS,
  STOP,
  FASTFORWARD,
  FASTREVERSE,
  PAIR,
  RESET,
  SWITCH_DEV
};

volatile cmds nextCmd = NONE;

#define SERDBG 1
const uint8_t MELBUS_CLOCKBIT_INT = 1; //interrupt numer (INT1) on DDR3
const uint8_t MELBUS_CLOCKBIT = 3; //Pin D3 - CLK
const uint8_t MELBUS_DATA = 2; //Pin D2  - Data
const uint8_t MELBUS_BUSY = 4; //Pin D4  - Busy

volatile uint8_t melbus_ReceivedByte = 0;
volatile uint8_t melbus_CharBytes = 0;
volatile uint8_t melbus_OutByte = 0xFF;
volatile uint8_t melbus_LastReadByte[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile uint8_t melbus_SendBuffer[9] = {0x00, 0x02, 0x00, 0x01, 0x80, 0x01, 0xC7, 0x0A, 0x02};
volatile uint8_t melbus_SendCnt = 0;
volatile uint8_t melbus_DiscBuffer[6] = {0x00, 0xFC, 0xFF, 0x4A, 0xFC, 0xFF};
volatile uint8_t melbus_DiscCnt = 0;
volatile uint8_t melbus_Bitposition = 0x80;

volatile bool InitialSequence_ext = false;
volatile bool ByteIsRead = false;
volatile bool sending_byte = false;
volatile bool melbus_MasterRequested = false;
volatile bool melbus_MasterRequestAccepted = false;

volatile bool testbool = false;
volatile bool AllowInterruptRead = true;
volatile int incomingByte = 0;   // for incoming serial data

//Startup seequence
void setup() {

  mySerial.begin(115200);
  //reset ovc module
  // mySerial.print("AT#CZ\r\n");


  //Data is deafult input high
  pinMode(MELBUS_DATA, INPUT_PULLUP);

  //Activate interrupt on clock pin (INT1, D3)
  attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, FALLING);
  //Set Clockpin-interrupt to input
  pinMode(MELBUS_CLOCKBIT, INPUT_PULLUP);
#ifdef SERDBG
  //Initiate serial communication to debug via serial-usb (arduino)
  Serial.begin(230400);
  Serial.println("Initiating contact with Melbus: hit <space> for extra dbg info");
#endif
  //Call function that tells HU that we want to register a new device
  melbus_Init_CDCHRG();
}

//Main loop
void loop() {

  if (mySerial.available()) {      // If anything comes in Serial (USB),
    Serial.write(mySerial.read());   // read it and send it out Serial1 (pins 0 & 1)
  }

  //handled sheduled commands
  switch (nextCmd) {
#define CMD(_X_, _Y_) case _X_: mySerial.print(_Y_); nextCmd=NONE; break;
#define CMDFAST(_X_, _Y_) case _X_: mySerial.print(_Y_); delay(1500); mySerial.print(OVC_PLAY_CANCEL_FAST); nextCmd=NONE; break;
    case NONE:
      break;
      CMD (PLAY, OVC_PLAY_PAUSE);
      CMD (STOP, OVC_PLAY_STOP);
      CMD (NEXT, OVC_PLAY_NEXT);
      CMD (PREVIOUS, OVC_PLAY_PREVIOUS);
      CMD (RESET, OVC_RESET);
      CMD (SWITCH_DEV, OVC_SWITCH_DEVICES);
      CMDFAST (FASTREVERSE, OVC_PLAY_FR);
      CMDFAST (FASTFORWARD, OVC_PLAY_FF);


      // CMD (FASTFORWARD, OVC_PLAY_FF);
  }

  /*
    if (Serial1.available()) {     // If anything comes in Serial1 (pins 0 & 1)
      Serial.write(Serial1.read());   // read it and send it out Serial (USB)
    }
  */
  //Waiting for the clock interrupt to trigger 8 times to read one byte before evaluating the data
#ifdef SERDBG
  if (ByteIsRead) {
    //Reset bool to enable reading of next byte
    ByteIsRead = false;




    if (incomingByte == ' ') {
      if (melbus_LastReadByte[11] == 0x0 && (melbus_LastReadByte[10] == 0x4A || melbus_LastReadByte[10] == 0x4C || melbus_LastReadByte[10] == 0x4E) && melbus_LastReadByte[9] == 0xEC && melbus_LastReadByte[8] == 0x57 && melbus_LastReadByte[7] == 0x57 && melbus_LastReadByte[6] == 0x49 && melbus_LastReadByte[5] == 0x52 && melbus_LastReadByte[4] == 0xAF && melbus_LastReadByte[3] == 0xE0 && melbus_LastReadByte[2] == 0x0)
      {
        melbus_CharBytes = 8; //print RDS station name
      }

      if (melbus_LastReadByte[1] == 0x0 && melbus_LastReadByte[0] == 0x4A)
      {
        Serial.println("\n LCD is master: (no CD init)");
      }
      else if (melbus_LastReadByte[1] == 0x0 && melbus_LastReadByte[0] == 0x4C)
      {
        Serial.println("\n LCD is master: (???)");
      }
      else if (melbus_LastReadByte[1] == 0x0 && melbus_LastReadByte[0] == 0x4E)
      {
        Serial.println("\n LCD is master: (with CD init)");
      }
      else if (melbus_LastReadByte[1] == 0x80 && melbus_LastReadByte[0] == 0x4E)
      {
        Serial.println("\n ???");
      }
      else if (melbus_LastReadByte[1] == 0xE8 && melbus_LastReadByte[0] == 0x4E)
      {
        Serial.println("\n ???");
      }
      else if (melbus_LastReadByte[1] == 0xF9 && melbus_LastReadByte[0] == 0x49)
      {
        Serial.println("\n HU  is master: ");
      }
      else if (melbus_LastReadByte[1] == 0x80 && melbus_LastReadByte[0] == 0x49)
      {
        Serial.println("\n HU  is master: ");
      }
      else if (melbus_LastReadByte[1] == 0xE8 && melbus_LastReadByte[0] == 0x49)
      {
        Serial.println("\n HU  is master: ");
      }
      else if (melbus_LastReadByte[1] == 0xE9 && melbus_LastReadByte[0] == 0x4B)
      {
        Serial.println("\n HU  is master: -> CDC");
      }
      else if (melbus_LastReadByte[1] == 0x81 && melbus_LastReadByte[0] == 0x4B)
      {
        Serial.println("\n HU  is master: -> CDP");
      }
      else if (melbus_LastReadByte[1] == 0xF9 && melbus_LastReadByte[0] == 0x4E)
      {
        Serial.println("\n HU  is master: ");
      }
      else if (melbus_LastReadByte[1] == 0x50 && melbus_LastReadByte[0] == 0x4E)
      {
        Serial.println("\n HU  is master: ");
      }
      else if (melbus_LastReadByte[1] == 0x50 && melbus_LastReadByte[0] == 0x4C)
      {
        Serial.println("\n HU  is master: ");
      }
      else if (melbus_LastReadByte[1] == 0x50 && melbus_LastReadByte[0] == 0x4A)
      {
        Serial.println("\n HU  is master: ");
      }
      else if (melbus_LastReadByte[1] == 0xF8 && melbus_LastReadByte[0] == 0x4C)
      {
        Serial.println("\n HU  is master: ");
      }

      if (melbus_CharBytes) {
        Serial.write(melbus_LastReadByte[1]);
        melbus_CharBytes--;
      } else
      {
        Serial.print(melbus_LastReadByte[1], HEX);
        Serial.write(' ');
      }
    }










  }
#endif



  //If BUSYPIN is HIGH => HU is in between transmissions
  if (digitalRead(MELBUS_BUSY) == HIGH)
  {
    //Make sure we are in sync when reading the bits by resetting the clock reader

#ifdef SERDBG
    if (melbus_Bitposition != 0x80) {
      Serial.println(melbus_Bitposition, HEX);
      Serial.println("\n not in sync! ");
    }
#endif
    if (incomingByte != 'k') {
      melbus_Bitposition = 0x80;
      melbus_OutByte = 0xFF;
      melbus_SendCnt = 0;
      melbus_DiscCnt = 0;
      DDRD &= ~(1 << MELBUS_DATA);
      PORTD |= (1 << MELBUS_DATA);
    }

  }
#ifdef SERDBG
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();


  }

  if (incomingByte != 0) {
    if (incomingByte == 's') {
      nextCmd = PAIR;
      Serial.println("s");
      incomingByte = 0;
    }

    if (incomingByte == 'n') {
      nextCmd = NEXT;
      Serial.println("n");
      incomingByte = 0;
    }

    if (incomingByte == 'b') {
      nextCmd = PREVIOUS;
      Serial.println("b");
      incomingByte = 0;
    }

    if (incomingByte == 'p') {
      nextCmd = PLAY;
      incomingByte = 0;
      Serial.println("p");

    }


    if (incomingByte == '1') {
      nextCmd = FASTFORWARD;
      incomingByte = 0;
      Serial.println("1");
    }
    if (incomingByte == '2') {
      nextCmd = FASTREVERSE;
      incomingByte = 0;
      Serial.println("2");
    }

    if (incomingByte == 'r') {
      nextCmd = RESET;
      Serial.println("r");
      incomingByte = 0;
    }

    if (incomingByte == 'q') {
      nextCmd = SWITCH_DEV;
      Serial.println("r");
      incomingByte = 0;
    }




    if (incomingByte == 'i') {
      melbus_Init_CDCHRG();
      Serial.println("\n forced init: ");
      incomingByte = 0;

    }

  }
#endif
  if ((melbus_Bitposition == 0x80) && (PIND & (1 << MELBUS_CLOCKBIT)))
  {
    delayMicroseconds(7);
    DDRD &= ~(1 << MELBUS_DATA);
    PORTD |= (1 << MELBUS_DATA);
  }

}

//Notify HU that we want to trigger the first initiate procedure to add a new device (CD-CHGR) by pulling BUSY line low for 1s
void melbus_Init_CDCHRG() {
  //Disabel interrupt on INT1 quicker then: detachInterrupt(MELBUS_CLOCKBIT_INT);
  EIMSK &= ~(1 << INT1);

  // Wait untill Busy-line goes high (not busy) before we pull BUSY low to request init
  while (digitalRead(MELBUS_BUSY) == LOW) {}
  delayMicroseconds(10);

  pinMode(MELBUS_BUSY, OUTPUT);
  digitalWrite(MELBUS_BUSY, LOW);
  delay(1200);
  digitalWrite(MELBUS_BUSY, HIGH);
  pinMode(MELBUS_BUSY, INPUT_PULLUP);

  //Enable interrupt on INT1, quicker then: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
  EIMSK |= (1 << INT1);
}




//Global external interrupt that triggers when clock pin goes high after it has been low for a short time => time to read datapin
void MELBUS_CLOCK_INTERRUPT() {

  //Read status of Datapin and set status of current bit in recv_byte

  if (melbus_OutByte & melbus_Bitposition) {
    DDRD &= (~(1 << MELBUS_DATA));
    PORTD |= (1 << MELBUS_DATA);
  }
  //if bit [i] is "0" - make databpin low
  else {
    PORTD &= (~(1 << MELBUS_DATA));
    DDRD |= (1 << MELBUS_DATA);
  }


  if (PIND & (1 << MELBUS_DATA)) {
    melbus_ReceivedByte |= melbus_Bitposition; //set bit nr [melbus_Bitposition] to "1"
  }
  else {
    melbus_ReceivedByte &= ~melbus_Bitposition; //set bit nr [melbus_Bitposition] to "0"
  }


  //if all the bits in the byte are read:
  if (melbus_Bitposition == 0x01) {

    //Move every lastreadbyte one step down the array to keep track of former bytes
    for (int i = 11; i > 0; i--) {
      melbus_LastReadByte[i] = melbus_LastReadByte[i - 1];
    }

    if (melbus_OutByte != 0xFF) {
      melbus_LastReadByte[0] = melbus_OutByte;
      melbus_OutByte = 0xFF;
    } else {

      //Insert the newly read byte into first position of array
      melbus_LastReadByte[0] = melbus_ReceivedByte;
    }
    //set bool to true to evaluate the bytes in main loop
    ByteIsRead = true;


    //Reset bitcount to first bit in byte
    melbus_Bitposition = 0x80;
    if (melbus_LastReadByte[2] == 0x07 && (melbus_LastReadByte[1] == 0x1A || melbus_LastReadByte[1] == 0x4A) && melbus_LastReadByte[0] == 0xEE)
    {
      InitialSequence_ext = true;
    }
    else if (melbus_LastReadByte[2] == 0x0 && (melbus_LastReadByte[1] == 0x1C || melbus_LastReadByte[1] == 0x4C) && melbus_LastReadByte[0] == 0xED)
    {
      InitialSequence_ext = true;
    }
    else if ((melbus_LastReadByte[0] == 0xE8 || melbus_LastReadByte[0] == 0xE9) && InitialSequence_ext == true) {
      InitialSequence_ext = false;

      //Returning the expected byte to the HU, to confirm that the CD-CHGR is present (0xEE)! see "ID Response"-table here http://volvo.wot.lv/wiki/doku.php?id=melbus
      melbus_OutByte = 0xEE;
    }
    else if ((melbus_LastReadByte[2] == 0xE8 || melbus_LastReadByte[2] == 0xE9) && (melbus_LastReadByte[1] == 0x1E || melbus_LastReadByte[1] == 0x4E) && melbus_LastReadByte[0] == 0xEF)
    {
      // CartInfo
      melbus_DiscCnt = 6;
#ifdef SERDBG
      Serial.println("CardInfo");
#endif
    }
    else if ((melbus_LastReadByte[2] == 0xE8 || melbus_LastReadByte[2] == 0xE9) && (melbus_LastReadByte[1] == 0x19 || melbus_LastReadByte[1] == 0x49) && melbus_LastReadByte[0] == 0x22)
    {
      // Powerdown
      melbus_OutByte = 0x00; // respond to powerdown;
      melbus_SendBuffer[1] = 0x02; // STOP
      melbus_SendBuffer[8] = 0x02; // STOP
      nextCmd = STOP;
#ifdef SERDBG
      Serial.println("powerdown");
#endif
    }
    else if ((melbus_LastReadByte[2] == 0xE8 || melbus_LastReadByte[2] == 0xE9) && (melbus_LastReadByte[1] == 0x19 || melbus_LastReadByte[1] == 0x49) && melbus_LastReadByte[0] == 0x52)
    {
      // RND
#ifdef SERDBG
      nextCmd = PLAY;
      Serial.println("rnd");
#endif
    }
    else if ((melbus_LastReadByte[2] == 0xE8 || melbus_LastReadByte[2] == 0xE9) && (melbus_LastReadByte[1] == 0x19 || melbus_LastReadByte[1] == 0x49) && melbus_LastReadByte[0] == 0x29)
    {
      // FF
#ifdef SERDBG
      nextCmd = FASTFORWARD;
      Serial.println("FF");
#endif
    }
    else if ((melbus_LastReadByte[2] == 0xE8 || melbus_LastReadByte[2] == 0xE9) && (melbus_LastReadByte[1] == 0x19 || melbus_LastReadByte[1] == 0x49) && melbus_LastReadByte[0] == 0x2F)
    {
      // FR
      melbus_OutByte = 0x00; // respond to start;
      melbus_SendBuffer[1] = 0x08; // START
      melbus_SendBuffer[8] = 0x08; // START
      nextCmd = FASTREVERSE;
#ifdef SERDBG
      Serial.println("FR");
#endif
    }
    else if ((melbus_LastReadByte[3] == 0xE8 || melbus_LastReadByte[3] == 0xE9) && (melbus_LastReadByte[2] == 0x1A || melbus_LastReadByte[2] == 0x4A) && melbus_LastReadByte[1] == 0x50 && melbus_LastReadByte[0] == 0x01)
    {
      // D-
      melbus_SendBuffer[3]--;
      melbus_SendBuffer[5] = 0x01;
#ifdef SERDBG
      Serial.println("D-");
#endif
      nextCmd = SWITCH_DEV;
    }
    else if ((melbus_LastReadByte[3] == 0xE8 || melbus_LastReadByte[3] == 0xE9) && (melbus_LastReadByte[2] == 0x1A || melbus_LastReadByte[2] == 0x4A) && melbus_LastReadByte[1] == 0x50 && melbus_LastReadByte[0] == 0x41)
    {
      // D+
      melbus_SendBuffer[3]++;
      melbus_SendBuffer[5] = 0x01;
      nextCmd = SWITCH_DEV;

#ifdef SERDBG
      Serial.println("D+");
#endif
    }
    else if ((melbus_LastReadByte[4] == 0xE8 || melbus_LastReadByte[4] == 0xE9) && (melbus_LastReadByte[3] == 0x1B || melbus_LastReadByte[3] == 0x4B) && melbus_LastReadByte[2] == 0x2D && melbus_LastReadByte[1] == 0x00 && melbus_LastReadByte[0] == 0x01)
    {
      // T-
      melbus_SendBuffer[5]--;
      nextCmd = PREVIOUS;
#ifdef SERDBG
      Serial.println("T-");
#endif
    }
    else if ((melbus_LastReadByte[4] == 0xE8 || melbus_LastReadByte[4] == 0xE9) && (melbus_LastReadByte[3] == 0x1B || melbus_LastReadByte[3] == 0x4B) && melbus_LastReadByte[2] == 0x2D && melbus_LastReadByte[1] == 0x40 && melbus_LastReadByte[0] == 0x01)
    {
      // T+
      melbus_SendBuffer[5]++;
      nextCmd = NEXT;
#ifdef SERDBG
      Serial.println("T+");
#endif
    }
    else if ((melbus_LastReadByte[4] == 0xE8 || melbus_LastReadByte[4] == 0xE9) && (melbus_LastReadByte[3] == 0x1B || melbus_LastReadByte[3] == 0x4B) && melbus_LastReadByte[2] == 0xE0  && melbus_LastReadByte[1] == 0x01 && melbus_LastReadByte[0] == 0x08 ) {
      // Playinfo
      melbus_SendCnt = 9;
#ifdef SERDBG
      Serial.println("pInfo");
#endif
    }
    if (melbus_SendCnt) {
      melbus_OutByte = melbus_SendBuffer[9 - melbus_SendCnt];
      melbus_SendCnt--;
    } else if (melbus_DiscCnt) {
      melbus_OutByte = melbus_DiscBuffer[6 - melbus_DiscCnt];
      melbus_DiscCnt--;
    }
  } else {
    //set bitnumber to address of next bit in byte
    melbus_Bitposition >>= 1;
  }
  EIFR |= (1 << INTF1);
}


