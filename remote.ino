#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#define TuningYaw 1
#define TuningPitch 4
#define TuningRoll 7

#define Tune TuningYaw

RF24 radio(9,10);
const uint64_t pipe = 0xE8E8F0F0E1LL;

struct inputdata
{
 int Rx,Ry,Rb;
 int Lx,Ly,Lb;
 int Throttle;
 int Toggle;
} inpd;

const int datasize = 13;
int8_t data[datasize];
float floatolddata[datasize];
float floatdata[datasize];

void updateinpd()
{
  digitalWrite(6,LOW);digitalWrite(7,LOW);digitalWrite(8,LOW);delay(15);
  inpd.Throttle = analogRead(A0)*0.2499;
  digitalWrite(6,LOW);digitalWrite(7,LOW);digitalWrite(8,HIGH);delay(15);
  inpd.Rx = 32-analogRead(A0)*0.06;inpd.Rx = inpd.Rx>=-2&&inpd.Rx<=2?0:inpd.Rx;
  digitalWrite(6,LOW);digitalWrite(7,HIGH);digitalWrite(8,LOW);delay(15);
  inpd.Lb = analogRead(A0)>0;
  digitalWrite(6,LOW);digitalWrite(7,HIGH);digitalWrite(8,HIGH);delay(15);
  inpd.Ry = analogRead(A0)*0.06-30;inpd.Ry = inpd.Ry>=-2&&inpd.Ry<=2?0:inpd.Ry;
  digitalWrite(6,HIGH);digitalWrite(7,LOW);digitalWrite(8,LOW);delay(15);
  inpd.Rb = analogRead(A0)>0;
  digitalWrite(6,HIGH);digitalWrite(7,LOW);digitalWrite(8,HIGH);delay(15);
  inpd.Lx = 32-analogRead(A0)*0.06;inpd.Lx = inpd.Lx>=-2&&inpd.Lx<=2?0:inpd.Lx;
  digitalWrite(6,HIGH);digitalWrite(7,HIGH);digitalWrite(8,LOW);delay(15);
  inpd.Toggle = analogRead(A0)>0;
  digitalWrite(6,HIGH);digitalWrite(7,HIGH);digitalWrite(8,HIGH);delay(15);
  inpd.Ly = analogRead(A0)*0.06-33;inpd.Ly = inpd.Ly>=-2&&inpd.Ly<=2?0:inpd.Ly;
}

float getnumb()
{
  float ret = 0;
  int neg = 0;
  char c;
  if(Serial.available()>0)
  {
    c = Serial.read();
    if( c == '-' )
    {
      neg = 1;
      c = Serial.read();
    }
  }

  while( c!='.' && Serial.available()>0 )
  {
    ret = ret * 10 + ( c - '0' );
    c = Serial.read();
  }

  if( c=='.' && Serial.available()>0)
  {
    c = Serial.read();
  }
  
  float p = 0.1;

  while(c!='n' && Serial.available()>0)
  {
    ret += p * (c-'0');
    p=p/10;
    c = Serial.read();
  }

  return neg ? -ret : ret;
}

void setup()
{
  pinMode(6,OUTPUT);pinMode(7,OUTPUT);pinMode(8,OUTPUT);pinMode(A0,INPUT);
  Serial.begin(2000000);
  radio.begin();
  radio.openWritingPipe(pipe);
  radio.printDetails();
}

char c;
float Temp;
unsigned int a,b;

void loop()
{
  updateinpd();
  
  floatdata[0] = ( (99 * inpd.Toggle)   +   (01 * floatolddata[0]) ) / 100;
  floatdata[1] = ( (80 * ((float)((unsigned int)(inpd.Throttle & 0x00ff))) ) +   (20 * floatolddata[1]) ) / 100;
  floatdata[2] = ( (85 * inpd.Rx)       +   (15  * floatolddata[2]) ) / 100;
  floatdata[3] = ( (85 * inpd.Ry)       +   (15  * floatolddata[3]) ) / 100;
  floatdata[4] = ( (99 * inpd.Rb)       +   (01  * floatolddata[4]) ) / 100;
  floatdata[5] = ( (85 * inpd.Lx)       +   (15  * floatolddata[5]) ) / 100;
  floatdata[6] = ( (85 * inpd.Ly)       +   (15  * floatolddata[6]) ) / 100;
  floatdata[7] = ( (99 * inpd.Lb)       +   (01  * floatolddata[7]) ) / 100;

  data[0] = ((int) floatdata[0] );
  data[1] = ((unsigned int) floatdata[1] );
  data[2] = ((int) floatdata[2] );
  data[3] = ((int) floatdata[3] );
  data[4] = ((int) floatdata[4] );
  data[5] = ((int) floatdata[5] );
  data[6] = ((int) floatdata[6] );
  data[7] = ((int) floatdata[7] );

  floatolddata[0] = floatdata[0];
  floatolddata[1] = floatdata[1];
  floatolddata[2] = floatdata[2];
  floatolddata[3] = floatdata[3];
  floatolddata[4] = floatdata[4];
  floatolddata[5] = floatdata[5];
  floatolddata[6] = floatdata[6];
  floatolddata[7] = floatdata[7];
  if( Serial.available() > 0 )
  {
     c = Serial.read();
     if(c!='r')
     {
      Temp = getnumb();
     }
    // 
     a = Temp;
     b = (Temp * 1000.0 - a * 1000.0);
    
    data[9]=  (a & 0xff00)>>8; 
    data[10]= (a & 0x00ff)>>0;
    data[11]= (b & 0xff00)>>8;
    data[12]= (b & 0x00ff)>>0;
    switch(c)
    {
      case 'p':
      {
        data[8] = Tune + 0;
        break;
      }
      case 'i':
      {
        data[8] = Tune + 1;
        break;
      }
      case 'd':
      {
        data[8] = Tune + 2;
        break;
      }
      case 'r':
      {
        Serial.print( ((int) data[0] ) );Serial.print("\t");
        Serial.print( ((unsigned int) (data[1] & 0x00ff) ) );Serial.print("\t");
        Serial.print( ((int) data[2] ) );Serial.print("\t");
        Serial.print( ((int) data[3] ) );Serial.print("\t");
        Serial.print( ((int) data[4] ) );Serial.print("\t");
        Serial.print( ((int) data[5] ) );Serial.print("\t");
        Serial.print( ((int) data[6] ) );Serial.print("\t");
        Serial.print( ((int) data[7] ) );Serial.print("\n");
        Serial.print(  data[8] );Serial.print(" \t ");
        Serial.print(  Temp  );Serial.print(" \t ");
        Serial.print( a );Serial.print(" \t ");
        Serial.print( b );Serial.print("\n");
        break;
      }
    }
  }
  else
  {
    data[8] = 10;
  }

  if( radio.write( data, datasize ) )
  {
     //Serial.println("ok");
  }
  else
  {
     //Serial.println("failed");
  }
  
}
