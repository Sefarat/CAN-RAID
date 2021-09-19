#define freq 16 //MHz
#define overflowTime (65536/freq) //us, 2^16/freq
#define INST 120
byte txbuf;
int trials=0;
int smalltrials=0;
unsigned long overflows=0;
unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char stmp2[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff};
byte msgLen = 8;
unsigned char flagRecv = 0;
int sofar=0;
int phase=0;
unsigned long timo=0;
unsigned char len = 0;
unsigned char buf[8];

int ECUCOUNT=0;
int smallid, bigid;
unsigned long smallperiod, bigperiod;
int counts,countb;
int order=0;
int ordere=0;

void starttimer(){
  overflows=0;
  TCNT1=0; 
  TCCR1B = 1;     
}

void starttimer2(unsigned int val){
  TCCR1B = 0;
  
  
  TCNT1=val; 
  TCCR1B |= (1 << CS11); 
  
}

void stoptimer(){
  overflows=0; 
  TCCR1B = 0;     
}
  
ISR(TIMER1_OVF_vect){  
  overflows++;  
}
inline void setupTimer(){
  TCCR1A = 0;
  TCCR1C = 0;
  TCCR1B=0;
  TIMSK1 = _BV(TOIE1);
}

inline void setupprescaleTimer(){
  TCCR1A = 0;
  TCCR1C = 0;
  TCCR1B |= (1 << CS11);
  TIMSK1 = _BV(TOIE1);
}
inline unsigned long getElapsedTime(){ //in us
  unsigned long currOverflows = overflows; 
  unsigned int currTimer = TCNT1; 
  return (currOverflows*overflowTime + currTimer/freq);
}

inline void enableOneShot(){
  CAN.mcp2515_modifyRegister(MCP_CANCTRL, MODE_ONESHOT, MODE_ONESHOT);
}

inline void disableOneShot(){
  CAN.mcp2515_modifyRegister(MCP_CANCTRL, MODE_ONESHOT, 0);
}
inline byte readREC(bool sleep = false){
  //if (sleep) waitTillElapsedDuration(1000, getElapsedTime());
  return CAN.mcp2515_readRegister(MCP_REC);
}
inline byte readTEC(bool sleep = false){
  //if (sleep) waitTillElapsedDuration(1000, getElapsedTime());
  return CAN.mcp2515_readRegister(MCP_TEC);
}
inline byte sendInAnyBuf(unsigned long id, unsigned char * buf){
  return CAN.sendMsgBuf(id, 0, msgLen, buf, false);
}
inline void resetcounters(){
 while(readTEC()>0){ 
  CAN.mcp2515_setCANCTRL_Mode(MODE_CONFIG);
  CAN.mcp2515_configRate(CAN_500KBPS, MCP_16MHz);
  //CAN.mcp2515_setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF | MCP_TX2IF | MCP_TX1IF | MCP_TX0IF);
  CAN.mcp2515_setCANCTRL_Mode(MODE_NORMAL);
}
}
char str[20];
struct message {
  int id;
  byte ECU=0;
  unsigned long lasttime;
  byte observations;
  unsigned long average;
  bool done;
};
void swap(struct message* xp, struct message* yp) { 
    //Serial.println("Herr");
    struct message temp = *xp; 
    *xp = *yp; 
    *yp = temp; 
}

void selectionSort(message arr[], int n, int choice) 
{ 
    int i, j, min_idx; 
  
    // One by one move boundary of unsorted subarray 
    if (choice==0){
      for (i = 0; i < n - 1; i++) { 
          min_idx = i; 
          for (j = i + 1; j < n; j++){ 
            //Serial.println(j);
              if (arr[j].id < arr[min_idx].id) {
                  min_idx = j; 
                  //Serial.println("Yo");
              }
          }
          // Swap the found minimum element 
          // with the first element 
          swap(&arr[min_idx], &arr[i]); 
      }
    } 
    else if (choice==1){
          //Serial.println("Her");
          for (i = 0; i < n - 1; i++) { 
            //Serial.println(i);
            min_idx = i; 
            for (j = i + 1; j < n; j++){ 
              //Serial.println(j);
                if (arr[j].average < arr[min_idx].average){
                    
                    min_idx = j; 
                    //Serial.println("Yo");
                    }
            }
            swap(&arr[min_idx], &arr[i]); 
      }
    }
} 
int binarySearch(message arr[], int l, int r, int x) 
{ 
    while (l <= r) { 
        int m = l + (r - l) / 2; 
        if (arr[m].id == x) 
            return m; 
        if (arr[m].id < x) 
            l = m + 1; 
        else
            r = m - 1; 
    }  
    return -1; 
} 
volatile int activateSof=0;
//volatile int activateSof2=0;
volatile int activateEnd=0;
volatile int els=0;
volatile int sofs=0;
volatile unsigned long remaining=0;
void SOF_ISR()
{
      cli();
      if(activateSof==1){
        
        stoptimer();
        
        diff=TCNT1;
        while(TCNT1!=0) {
          TCNT1=0;
          __builtin_avr_delay_cycles(1);}
        //CAN.mcp2515_setRegister(MCP_CANINTF, 0);
        //diff=diff/2;
        
        //CAN.mcp2515_setRegister(MCP_CANINTF, 0);
        els=els +(diff/44);
         
        remaining= (INST-els) * 44;  
        
        
      }
      else if(activateSof==2){
        stoptimer();
        diff=getElapsedTime()+12; //12 if reading buffer
      }
      else{
        
      }
    sof=1;
    sei();
}
void MCP2515_ISR()
{
    receive=1;
    if(activateEnd==1){
      starttimer2(29);
    }
    
}
inline byte sendInParticularBuf(unsigned long id, unsigned char * buf, byte bufferIdx, byte msgLen=8){
  return CAN.trySendMsgBuf(id, 0, 0, msgLen, buf, bufferIdx);
}


inline long readMsg() {
  CAN.readMsgBuf(&len, buf);
  return CAN.getCanId();
}
inline void ReadRBuffer(){
  while (CAN_MSGAVAIL == CAN.checkReceive()) 
  {
      CAN.readMsgBuf(&len, buf);
      
  }
}
inline unsigned long waitNewMsg(int victimID){
//  while (CAN_MSGAVAIL != CAN.checkReceive());
  bool received = false;
  volatile unsigned long id, t;
  while (!received){
     
    //if (TCNT1>=remaining) return;
    
    t=remaining;
    while (CAN_MSGAVAIL == CAN.checkReceive()){
      CAN.mcp2515_read_canMsg( MCP_READ_RX0, &id, NULL, NULL, &len, buf);
      
      if (id==victimID) {
        received=true;
        break;
      }
    }
    
    //CAN.mcp2515_setRegister(MCP_CANINTF, 0);
    
  }
}

inline unsigned long waitNewMsg2(int victimID){
//  while (CAN_MSGAVAIL != CAN.checkReceive());
  bool received = false;
  volatile unsigned long id;
  receive=0;
  sof=0;
  volatile unsigned long t=0;
  volatile unsigned long r=remaining;
   while(t<r){
      
      if(sof==1){
        sof=0;
        CAN.mcp2515_setRegister(MCP_CANINTF, 0);
        //CAN.mcp2515_read_canMsg( MCP_READ_RX0, &id, NULL, NULL, &len, buf);
        //__builtin_avr_delay_cycles(1);
      }    
     cli();
     t=TCNT1; 
     r=remaining;
     sei();   
   }
   
   return t;
}
inline unsigned long waitNewMsg3(int victimID){
//  while (CAN_MSGAVAIL != CAN.checkReceive());
  bool received = false;
  //volatile unsigned long id;
  //receive=0;
  unsigned long t=0;
   while(TCNT1<remaining){
      t=TCNT1;
      if(receive){
       receive=0;
        CAN.mcp2515_read_canMsg( MCP_READ_RX0, &id, NULL, NULL, &len, buf);
      }
      
   }
   return t;
}




inline void ErrorPassive(int Victim,unsigned char  stmp[] ){
    
    sendInParticularBuf(Victim-1, stmp, 2);
    sendInParticularBuf(Victim-1, stmp, 1); 
    sendInParticularBuf(Victim-1, stmp,0);
    //CAN.mcp2515_write_canMsg(1, Victim, 0, 0, 0, stmp);
    //while(CAN.mcp2515_isTXBufFree(&txbuf, 2)!=CAN_OK);

    //Beginning From Here, This is added for testing, remove if not working//
    while(CAN.mcp2515_isTXBufFree(&txbuf, 1)!=CAN_OK);
    sendInParticularBuf(Victim-1, stmp, 2);
    sendInParticularBuf(Victim-1, stmp, 1); 
    sendInParticularBuf(Victim-1, stmp,0);
    //Until Here
    
    while(CAN.mcp2515_isTXBufFree(&txbuf, 1)!=CAN_OK);
    //while(sendInParticularBuf(Victim, stmp,2,0)!=CAN_OK);
    sendInParticularBuf(Victim, stmp, 2,0);
    //CAN.mcp2515_write_canMsg(2, Victim, 0, 0, 0, stmp);
    //while(CAN.mcp2515_isTXBufFree(&txbuf, 1)!=CAN_OK);
    //while(CAN.mcp2515_isTXBufFree(&txbuf, 1)!=CAN_OK);
   //sendInParticularBuf(Victim+1, stmp, 1);
    for (int i=0; i<6; i++){
      sendInAnyBuf(Victim+1, stmp); 
    } 
}
/* old ErrorPassive
 * inline void ErrorPassive(int Victim,unsigned char  stmp[] ){
    sendInParticularBuf(Victim-1, stmp, 0); 
    sendInParticularBuf(Victim-1, stmp, 1);
    while(CAN.mcp2515_isTXBufFree(&txbuf, 0)!=CAN_OK);
    sendInParticularBuf(Victim-1, stmp, 0);
    while(CAN.mcp2515_isTXBufFree(&txbuf, 1)!=CAN_OK);
    sendInParticularBuf(Victim, stmp, 2);
    sendInParticularBuf(Victim+1, stmp, 1);
    for (int i=0; i<6; i++){
      sendInAnyBuf(Victim+1, stmp); 
    } 
}
*/

inline void busOff(int Victim,unsigned char  stmp[] ){

    activateSof=0;
    sendInParticularBuf(0, stmp, 2);
    sendInParticularBuf(0, stmp, 1); 
    sendInParticularBuf(0, stmp,0);
    while(CAN.mcp2515_isTXBufFree(&txbuf, 1)!=CAN_OK);
   
    
    sendInParticularBuf(0, stmp, 2);
    sendInParticularBuf(0, stmp, 1); 
    sendInParticularBuf(0, stmp,0);
 

    while(CAN.mcp2515_isTXBufFree(&txbuf, 1)!=CAN_OK);
    sendInParticularBuf(Victim, stmp, 2,2);
    
    for (int i=0; i<CLN; i++){
      while(sendInAnyBuf(CLUTID, stmp)!=CAN_OK); 
    } 
    
    starttimer();
    activateSof=1;
    activateEnd=1;  
      
    while(CAN.mcp2515_isTXBufFree(&txbuf, 2)!=CAN_OK);
    CAN.mcp2515_setRegister(MCP_CANINTF, 0);
    els=0;
    /*
    activateSof=1;
    activateEnd=1;
    */
    while(CAN.mcp2515_isTXBufFree(&txbuf, 0)!=CAN_OK);
        //starttimer();
        //CAN.mcp2515_setRegister(MCP_CANINTF, 0);
        //sofs=0;
        //activateSof=1;
        //activateEnd=1;


    
    //_delay_us(3000);
    //disableOneShot();
}

inline void setfilters(int id){
    int filt0=CAN.mcp2515_readRegister(MCP_RXB0CTRL);
    int filt1=CAN.mcp2515_readRegister(MCP_RXB1CTRL);
    CAN.mcp2515_modifyRegister(MCP_RXB0CTRL,filt0,0); 
    CAN.mcp2515_modifyRegister(MCP_RXB1CTRL,filt1,0);
    unsigned long mask=0x7ff<<18;
    unsigned long value=id<<18;
    CAN.init_Mask(0, 0, mask);                        
    CAN.init_Mask(1, 0, mask);
    //CAN.init_Mask(0, 0, 0x1FFC0000); 
    CAN.init_Filt(0, 0, value); 
    CAN.init_Filt(1, 0, value); 
    //CAN.init_Mask(1, 0, 0x1FFFFFFF); 
    CAN.init_Filt(2, 0, value); 
    CAN.init_Filt(3, 0, value);
    CAN.init_Filt(4, 0, value);
    CAN.init_Filt(5, 0, value);
}
inline int CheckifPassive(int Victim,int tec,unsigned char  stmp[]){
    //byte txbuf;
    
    sendInParticularBuf(Victim-1, stmp, 2);
    sendInParticularBuf(Victim-1, stmp, 1); 
    sendInParticularBuf(Victim-1, stmp,0);
    
    //Beginning From Here, This is added for testing, remove if not working//
    while(CAN.mcp2515_isTXBufFree(&txbuf, 1)!=CAN_OK);
    sendInParticularBuf(Victim-1, stmp, 2);
    sendInParticularBuf(Victim-1, stmp, 1); 
    sendInParticularBuf(Victim-1, stmp,0);
    //Until Here
    
    while(CAN.mcp2515_isTXBufFree(&txbuf, 1)!=CAN_OK);
    sendInParticularBuf(Victim, stmp2,2);
    sendInParticularBuf(0xFF, stmp,1);
    sendInParticularBuf(0xFF, stmp,0);
    while((CAN.mcp2515_isTXBufFree(&txbuf, 2)!=CAN_OK)&&(readTEC()<=8));
    //while(CAN.mcp2515_isTXBufFree(&txbuf, 2)!=CAN_OK);
    enableOneShot();
    _delay_us(6000);
    int tec2=readTEC();
    disableOneShot();
    Serial.println("***************************");
    Serial.print("Before: ");
    Serial.print(tec);
    Serial.print(" After: ");
    Serial.println(tec2);
    if ((tec2>=16)&&(tec2<50)) {
      return 0;
      //Serial.println("Different ECUs");
    }
    else if ((tec2>0)&&(tec2<=8)){
      //Serial.println("Same ECUs");
      return 1;
    }
    else{
      //Serial.println("Missed");
      return -1;
    }  
}
/* Old CheckifPassive
inline int CheckifPassive(int Victim,int tec,unsigned char  stmp[]){
    byte txbuf;
    sendInParticularBuf(Victim-1, stmp, 0); 
    sendInParticularBuf(Victim-1, stmp, 1);
    while(CAN.mcp2515_isTXBufFree(&txbuf, 0)!=CAN_OK);
    //(id, 0, 0, msgLen, buf, bufferIdx);
    CAN.mcp2515_write_canMsg(0, Victim-1, 0, 0, 8, stmp);
    //sendInParticularBuf(Victim-1, stmp, 0);
    while(CAN.mcp2515_isTXBufFree(&txbuf, 1)!=CAN_OK);
    sendInParticularBuf(Victim, stmp2, 2);
    //sendInParticularBuf(Victim, stmp2, 2);
    
    while((CAN.mcp2515_isTXBufFree(&txbuf, 2)!=CAN_OK)&&(readTEC()<=8));
    //while(CAN.mcp2515_isTXBufFree(&txbuf, 2)!=CAN_OK);
    enableOneShot();
    _delay_us(6000);
    int tec2=readTEC();
    disableOneShot();
    Serial.println("***************************");
    Serial.println(tec2);
    if (tec2==24) {
      return 0;
      //Serial.println("Different ECUs");
    }
    else if ((tec2>0)&&(tec2<16)){
      //Serial.println("Same ECUs");
      return 1;
    }
    else{
      //Serial.println("Missed");
      return -1;
    }  
}
*/
/*
inline bool CheckifPassive(int Victim,int tec,unsigned char  stmp[]){
    byte txbuf;
    sendInParticularBuf(Victim-1, stmp, 0); 
    sendInParticularBuf(Victim-1, stmp, 1);
    while(CAN.mcp2515_isTXBufFree(&txbuf, 0)!=CAN_OK);
    sendInParticularBuf(Victim-1, stmp, 0);
    while(CAN.mcp2515_isTXBufFree(&txbuf, 1)!=CAN_OK);
    sendInParticularBuf(Victim, stmp, 2,0);
    sendInParticularBuf(Victim, stmp, 1,0);
    while(CAN.mcp2515_isTXBufFree(&txbuf, 2)!=CAN_OK);
    //sendInParticularBuf(Victim+1, stmp, 1);
    for (int i=0; i<0; i++){
      sendInAnyBuf(Victim+1, stmp); 
    } 
    while(CAN.mcp2515_isTXBufFree(&txbuf, 2)!=CAN_OK);
    _delay_us(6000);
    int tec2=readTEC();
    if (tec2>tec) {
      return 0;
    }
    else {
      return 1;
    }
}
*/
