#define freq 16 //MHz
#define overflowTime (65536/freq) //us, 2^16/freq

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
int Remainingids=NIDS;
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

void stoptimer(){
  overflows=0; 
  TCCR1B = 0;     
}
  
ISR(TIMER1_OVF_vect){  
  overflows++;  
}

inline unsigned long getElapsedTime(){ //in us
  unsigned long currOverflows = overflows; 
  unsigned int currTimer = TCNT1; 
  return (currOverflows*overflowTime + currTimer/freq);
}

inline void enableOneShot(){
  CAN.mcp2515_modifyRegister(MCP_CANCTRL, MODE_ONESHOT, MODE_ONESHOT);
}
inline void resetcounters(){
  while(CAN.mcp2515_setCANCTRL_Mode(MODE_CONFIG)!=MCP2515_OK);
  while(CAN.mcp2515_configRate(CAN_500KBPS, MCP_16MHz)!=MCP2515_OK);
  while(CAN.mcp2515_setCANCTRL_Mode(MODE_NORMAL)!=MCP2515_OK);
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

inline byte sendInParticularBuf(unsigned long id, unsigned char * buf, byte bufferIdx, byte msgLen=8){
  return CAN.trySendMsgBuf(id, 0, 0, msgLen, buf, bufferIdx);
}
struct message list[NIDS];
void MCP2515_ISR()
{
    flagRecv = 1;
    timo=micros();
}
void MCP2515_ISR2()
{  
     starttimer();
     flagRecv = 1;  
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
inline void waitNewMsg(int victimID){
//  while (CAN_MSGAVAIL != CAN.checkReceive());
  bool received = false;
  while (!received){
    while (CAN_MSGAVAIL == CAN.checkReceive()){
      if (CAN.unpadID(readMsg())==victimID) {
        received=true;
        break;
      }
    }
  }
}
void readIDs(){
  unsigned int canId;
  while (CAN_MSGAVAIL == CAN.checkReceive()) 
  {           
   canId = CAN.unpadID(CAN.getCanId()); 
   //Serial.println(canId);
   CAN.readMsgBuf(&len, buf);      
  }
  bool exist=0;
  for (int i=0; i<=sofar;i++){
    if (canId==list[i].id) exist=1;
  }
  if (exist==0) {
    list[sofar].id=canId;
    list[sofar].lasttime=0;
    list[sofar].observations=0;
    list[sofar].average=0;
    list[sofar].done=0;
    sofar=sofar+1;
    if (sofar>=NIDS){
      phase=GETPERIODS;
      Serial.println("All Ids Observed");
      selectionSort(list, sofar, 0);
      for (int j=0; j<sofar; j++){
        Serial.println(list[j].id,HEX);
      }
    }
  }
}
void getPeriods(){
detachInterrupt(0);
for (int i=0; i<NIDS; i++){
  Serial.print("****************************** ID: ");
  Serial.println(list[i].id, HEX);
  unsigned long Observations[MAXOBS];
  unsigned int victim=list[i].id;
  unsigned long last, current, average;
  ReadRBuffer();
  last=0;
  average=0;
  waitNewMsg(list[i].id);
  starttimer();
  for (int j=0; j<MAXOBS; j++){
    waitNewMsg(list[i].id);
    current=getElapsedTime();
    Observations[j]=current-last;
    last=current; 
    
  }

  Serial.println("Observations: ");
  for (int j=0; j<MAXOBS; j++){
    Serial.println(Observations[j]);   
    average=((average*j)+ Observations[j])/(j+1);                     
  }
  list[i].average=average;
  Serial.print("Average: ");
  Serial.println(average);       
}

phase=SORTIDS;
}



inline void ErrorPassive(int Victim,unsigned char  stmp[] ){
    
    sendInParticularBuf(Victim-1, stmp, 2);
    sendInParticularBuf(Victim-1, stmp, 1); 
    sendInParticularBuf(Victim-1, stmp,0);
    //CAN.mcp2515_write_canMsg(1, Victim, 0, 0, 0, stmp);
    //while(CAN.mcp2515_isTXBufFree(&txbuf, 2)!=CAN_OK);
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
    //CAN.mcp2515_write_canMsg(1, Victim, 0, 0, 0, stmp);
    //while(CAN.mcp2515_isTXBufFree(&txbuf, 2)!=CAN_OK);
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
inline void AssignSmallID(){
    trials=0;
    smalltrials=0;
    if (Remainingids>0){
    while ((list[counts].ECU!=0)&&(counts<NIDS)){
      counts++;
    }
    if (counts<NIDS){
      smallid=list[counts].id;
      smallperiod=list[counts].average;
      ECUCOUNT++;
      Remainingids--; 
      list[counts].ECU=ECUCOUNT;
      Serial.println();
      Serial.println();
      Serial.println("***************************************");
      Serial.println("***************************************");
      Serial.println("***************************************");
      Serial.print("Enumerating IDs Associated With ID: ");
      Serial.println(smallid=list[counts].id,HEX);
      Serial.println("***************************************");
      Serial.println("***************************************");
      Serial.println("***************************************");
      Serial.println();
      return;
      }
    }
    counts==NIDS;
}
inline void AssignBigID(){
  smalltrials=(smalltrials%MODULE);
  
  trials=0;
  if (Remainingids>0){
  while ((list[countb].ECU!=0)&&(countb>=0)){
    countb--;
  }
  if(countb>=0){
    bigid=list[countb].id;
    bigperiod=list[countb].average; 
  }
  return;
  }
  countb=-1;  
}
inline void PrintAndExit(){
  for (int all=1; all <=ECUCOUNT;all++){
    Serial.print("***************ECU: ");
    Serial.print(all);
    Serial.println("***************");
    for (int id=0; id< NIDS; id++){
      if (list[id].ECU==all) Serial.println(list[id].id, HEX);
    }
  }
  Serial.println("Unidentified IDs: ");
    for (int id=0; id< NIDS; id++){
      if (list[id].ECU<=0) Serial.println(list[id].id, HEX);
    }
 Serial.println("Everything: ");
    for (int id=0; id< NIDS; id++){
     Serial.print(list[id].id, HEX);
     Serial.print(": ");
     Serial.println((int)list[id].ECU);
    }
  phase=8; //check number
}
