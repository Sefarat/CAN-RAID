// demo: CAN-BUS Shield, receive data with check mode
// send data coming to fast, such as less than 10ms, you can use this way
// loovee, 2014-6-13


#include <SPI.h>
#include "mcp_can.h"
const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN);

#define VICID  0x5
#define OTHID 0x25
//Enter ID you want to Bus Off
#define NS 10
#define PRECID VICID-1
#define CLUTID VICID+1
#define PERIOD 10369







//Enter Periodicity of ID you want to Bus off in us. collsison happens at the first vicid with most periods, happens at the second with 500000, 150000 and other periods
#define STHRESH 900
#define STHRESH2 500//800 for 5, 1000 for 10 and 9 and 30 (Even 1100 works). 1100for 60,90,120 ms 150 doesn't work at all, 160, 190, 210 works but collision happens at second frame, 250 doesnt work
#define CLN 17
#define N    5
#define BITS 22
#define SOF 3

byte tbuf1, tbuf2, tbuf0;
//unsigned long overflows=0;
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10


volatile unsigned long readings[N];
volatile unsigned long ids[N + 1];
volatile unsigned long elevens[N];
volatile int i = 0;
volatile unsigned long id;
volatile bool sof = 0;
volatile bool receive = 0;
bool beg = 0;
volatile unsigned long diff = 0;
volatile unsigned long eleven = 0;
#include "functions.h"
inline void resetCANController() {
  CAN.mcp2515_reset();
  CAN.mcp2515_configRate(CAN_500KBPS, MCP_16MHz);
  //  CAN.mcp2515_initCANBuffers();
  //  CAN.mcp2515_setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);
  CAN.mcp2515_modifyRegister(MCP_RXB0CTRL,
                             MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
                             MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK);
  CAN.mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
                             MCP_RXB_RX_STDEXT);
  CAN.mcp2515_modifyRegister(MCP_TXB0CTRL, 3, 3);
  CAN.mcp2515_modifyRegister(MCP_TXB1CTRL, 3, 2);
  CAN.mcp2515_modifyRegister(MCP_TXB2CTRL, 3, 1);
  CAN.mcp2515_setCANCTRL_Mode(MODE_NORMAL);
}
void setup()
{
  Serial.begin(115200);

  while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  //CAN.mcp2515_setCANCTRL_Mode(MODE_CONFIG);
  //CAN.mcp2515_setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF | MCP_TX2IF | MCP_TX1IF | MCP_TX0IF);
  //CAN.mcp2515_setCANCTRL_Mode(MODE_NORMAL);

  delay(100);
  //attachInterrupt(digitalPinToInterrupt(SOF), SOF_ISR, RISING);
  //attachInterrupt(0, MCP2515_ISR, FALLING);
  sof = 0;


  Serial.println("CAN BUS Shield init ok!");
}

int sentclut = 0;
unsigned int canId;
bool cont = 0;
long tim=0;
bool Yass = 1;
int phas = 0;
int number = 0;
unsigned long average = 0;
void loop()
{

    delay(10);
    sendInParticularBuf(VICID,0,  stmp, 8);
//    sendInParticularBuf(CAN.padID(VICID), stmp,1, 8);
    delay(10);
    sendInParticularBuf(VICID, stmp, 1);
//    sendInParticularBuf(CAN.padID(VICID),1, stmp,1, 8);
    delay(10);
    sendInParticularBuf(VICID, stmp, 1);
//    sendInParticularBuf(CAN.padID(VICID),1, stmp,1, 8);
    sendInParticularBuf(OTHID , stmp, 0); 
//    sendInParticularBuf(CAN.padID(0x25),0, stmp,1, 8);
    delay(10);
    sendInParticularBuf(VICID, stmp, 1);
//    sendInParticularBuf(CAN.padID(VICID),1, stmp, 1,8);
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
