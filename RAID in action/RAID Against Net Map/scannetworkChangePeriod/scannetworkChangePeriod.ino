#include <mcp_can.h>
#include <SPI.h>
#define NIDS 6
#define MAXOBS 15
#define TRIALS 24
#define SMALLTRIALS 36
#define MODULE 12 
#define STHRESH 900 //try 1150
#define STHRESH2 900//try 1150
#define STHRESHA 1150
#define STHRESHB 400
#define STHRESHC 1400
#define STHRESHD 1800

#define REST 2500 //change this as you wish
#define OBSERVATION 0
#define GETPERIODS 1
#define SORTIDS 2
#define ADDECU 3
#define ERRORPASSIVE 4 
#define CHECKPASSIVE 5
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin
#include "functions.h"

int threshe=STHRESH;
int thresht=STHRESH2;

void setup()
{   
    TCCR1A = 0;
    TCCR1C = 0;
    TIMSK1 = _BV(TOIE1);
    Serial.begin(115200);

    while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");

    attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt
}
bool done=0;
void loop()
{   
    if(flagRecv) { 
        flagRecv = 0;                   // clear flag
        if(phase==OBSERVATION){
          readIDs();
        }
        else if (phase==GETPERIODS){
          getPeriods();
        }

    }

    else if (phase==SORTIDS){
      detachInterrupt(0);     
      ReadRBuffer();
      selectionSort(list, NIDS, 1); //sort list by average period
      for (int r=0; r<NIDS; r++){
        Serial.print(list[r].id,HEX);
        Serial.print("   average interval:");
        Serial.print(list[r].average);
        Serial.println();
      } 
      //attachInterrupt(0, MCP2515_ISR2, FALLING);
      phase=ADDECU;
      //flagRecv=1;
      Serial.println("3 set"); 
   }  
   
  else if (phase>=ADDECU){
    while (!done){   
      if (phase==ADDECU){
          detachInterrupt(0); 
          ReadRBuffer();
          counts=0;
          countb=NIDS-1;
          if (Remainingids>0){
            AssignSmallID();
            AssignBigID(); 
            if(countb==-1) list[counts].ECU=0;        
            if(countb<=counts){
              PrintAndExit();
            }
            else{
              phase=ERRORPASSIVE;
              threshe=STHRESH;
            }
           }
          else{
            PrintAndExit();
            done=1;
            }
        }
        else if(phase==ERRORPASSIVE){
            Serial.print("Passive Threshold: ");
            Serial.println(threshe);
            ReadRBuffer();
            waitNewMsg(bigid);
            waitNewMsg(bigid);
            starttimer();
            if (trials==12){
              unsigned long observed1,observed2;
              waitNewMsg(bigid);
              observed1=getElapsedTime();
              waitNewMsg(bigid);
              observed2=getElapsedTime();
              observed2=observed2-observed1;
              Serial.println("+++++++++++++++++++++++++++++Big ID++++++++++++++++++++++++++++");
              Serial.print("First Observed Period: ");
              Serial.print(observed1);
              Serial.print(",   Second Observed Period: ");
              Serial.print(observed2);
              Serial.print(", Record Period: ");
              Serial.println(list[countb].average);
              trials++;
              if (trials>TRIALS){
                  Serial.print("Skipping ID: ");
                  Serial.println(bigid,HEX);
                  list[countb].ECU=0;
                  countb--;
                  AssignBigID();
                  if (countb==-1) phase=ADDECU;
              }
            }
            else{
            int tec1=readTEC();
            unsigned long t=getElapsedTime();
            unsigned long k=bigperiod-threshe;
            while(t<k){
              Serial.print(""); //There is a bug that requires this to stay here. Fix when free.
              t=getElapsedTime();
            }
            ErrorPassive(bigid, stmp);
            stoptimer();
            _delay_us(1000);
            tec1=readTEC();
            resetcounters();
            if (tec1>=119) {
              phase=CHECKPASSIVE;
              //(smallid);
              Serial.print("ID: ");
              Serial.print(bigid,HEX);
              Serial.println(" is now Passive");

            }else { //if failed to error passive ECU
              Serial.print("ID: ");
              Serial.print(bigid,HEX);
              Serial.print(" Not Passived, TEC:  ");
              Serial.println(tec1);
              trials++;
              
              if (trials<2)
              {
                Serial.print("Waiting For:  ");
                Serial.print(REST);
                Serial.println(" ms");
                delay(REST);
                
              }
              else if(trials==2){
                Serial.print("Big Period in s:  ");
                Serial.println(bigperiod/1000000.0);
                Serial.print("Waiting For:  ");
                Serial.print(REST);
                Serial.println(" ms");
                delay(REST);
                /*
                Serial.print("Waiting For:  ");
                Serial.println(bigperiod*0.254);
                delay(bigperiod*0.254);
                */
              }
              else if((trials>2)&&(trials<TRIALS)){
                Serial.println("Not Passived, Retrying with no wait");
                if ((trials%6)==0) threshe=STHRESH;
                else if ((trials%6)==1) threshe=STHRESHA;
                else if ((trials%6)==2) threshe=STHRESHB;
                else if ((trials%6)==3) threshe=STHRESHC;
                else if ((trials%6)==4) threshe=STHRESHD;
                else if ((trials%6)==5) threshe=0;
              }
              else{
                Serial.print("Skipping ID: ");
                Serial.println(bigid,HEX);
                list[countb].ECU=0;
                countb--;
                AssignBigID();
                if (countb==-1) phase=ADDECU;
              }
              resetcounters();
              //delay(bigperiod*0.254);
            }
           }
         }


         
        else if(phase==CHECKPASSIVE){
          //thresht=STHRESH2;
          if (smalltrials>=0){
            if (((smalltrials%MODULE)>=0)&& ((smalltrials%MODULE)<7))thresht=STHRESH2;

            
            else if ((smalltrials%MODULE)==7) thresht=STHRESHA;
            
            else if ((smalltrials%MODULE)==8) thresht=STHRESHB;
       
            else if ((smalltrials%MODULE)==9) thresht=STHRESHC;

            else if ((smalltrials%MODULE)==10) thresht=STHRESHD;

            else if ((smalltrials%MODULE)==11) thresht=0;

          }
          Serial.print("Test Threshold: ");
          Serial.println(thresht);
          ReadRBuffer();
          waitNewMsg(smallid);
          
          waitNewMsg(smallid);
          starttimer();
          if (smalltrials!=12){
            int tec1=readTEC();
            unsigned long k=smallperiod-thresht;
            while(getElapsedTime()<k);
            int result=CheckifPassive(smallid,tec1,stmp);
            stoptimer();
            resetcounters();
            if (result==0){ //not passive (not same ECU)
              if(countb>0) {
                  countb--;
                  AssignBigID();
                  phase=ERRORPASSIVE;
                  threshe=STHRESH; 
                  //(bigid);
              }
              if(countb<=counts){
                  phase=ADDECU;
                  //Print something phase 3?
              }else{
                  phase=ERRORPASSIVE;
                  threshe=STHRESH;
                  //(bigid);
                }
                delay(REST);
              
            }
            else if (result==1){ //passive (same ECU)
              list[countb].ECU=ECUCOUNT;
              Remainingids--;
                if(countb>0) {
                    countb--;
                    AssignBigID();
                    phase=ERRORPASSIVE;
                    threshe=STHRESH; 
                }
                if(countb<=counts){
                    phase=ADDECU;
                    //Print something phase 3?
                }else{
                    phase=ERRORPASSIVE;
                    threshe=STHRESH;
                    
                  }
              delay(REST);
              //delay(smallperiod*0.205);
            }
            else{
              phase=ERRORPASSIVE;
              smalltrials++;
              if (smalltrials<SMALLTRIALS){
                threshe=STHRESH;
                delay(REST);
                }
                else{
                //Serial.println("Undeterministic Output. skipping");
                
                if(countb>0) {
                    countb--;
                    AssignBigID();
                    phase=ERRORPASSIVE;
                    threshe=STHRESH; 
                    
                }
                if(countb<=counts){
                    phase=ADDECU;
                    //Print something phase 3?
                }else{
                    phase=ERRORPASSIVE;
                    threshe=STHRESH;
                    
                  }
                  delay(REST);
              }
            } 
          }
          else{
            unsigned long observed1,observed2;
            waitNewMsg(smallid);
            observed1=getElapsedTime();
            waitNewMsg(smallid);
            observed2=getElapsedTime();
            observed2=observed2-observed1;
            Serial.println("+++++++++++++++++++++++++++++Small ID++++++++++++++++++++++++++++");
            Serial.print("First Observed Period: ");
            Serial.print(observed1);
            Serial.print(",   Second Observed Period: ");
            Serial.print(observed2);
            Serial.print(", Record Period: ");
            Serial.println(list[counts].average);
            smalltrials++;
            if (smalltrials>SMALLTRIALS){
                if(countb>0) {
                    countb--;
                    AssignBigID();
                    phase=ERRORPASSIVE;
                    threshe=STHRESH; 
                    
                }
                if(countb<=counts){
                    phase=ADDECU;
                    //Print something phase 3?
                }else{
                    phase=ERRORPASSIVE;
                    threshe=STHRESH;
                    
                  }
            }
            
            
          }
        }
      }
  }
  }
//855-800-8053
