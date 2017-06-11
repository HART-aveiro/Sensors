/* Vrl é o byte que deve ser enviado para a plataforma, do lado deles eles devem fazer a seguinte conta:
    Rs =((Vc-Vrl)*3440500)/(9830*Vrl-(Vc-Vrl)*350);
    logppm = -1.4635*log10(Rs/Ro)+2.0282;
    ppm = pow(10,logppm);

    Vc=5,Ro=500

    O codigo de exemplo faz uso de um timer a 1 ms, visto que o codigo de integracao usa 2 ms:
      baseTime = 15000;

*/

#include <TimerOne.h>
#define heatingTime 2 //2*baseTime
#define readingTime 3 //3*readingTime
#define baseTime 30000
#define togglePIN 2
#define readPIN 3

#define P1 9830
#define P2 350
#define Vc 5
#define Ro 500
unsigned int counter;
char mark;
char isReading;

byte Vrl;
float Rs;
float logppm;
float ppm;

void setup() {
  Serial.begin(115200);
  mark = 0;
  pinMode(togglePIN,OUTPUT);
  digitalWrite(togglePIN,HIGH);
  Timer1.initialize(1000); // Inicializa o Timer1 
  Timer1.attachInterrupt(callback1); // Configura a funÃ§Ã£o callback() como a funÃ§Ã£o para ser chamada a cada interrupÃ§Ã£o do Timer1
}

void callback1(){
  counter++;

  if (counter == baseTime){
    mark++;
    counter = 0;
  }
  
  if (mark == heatingTime && isReading == 0){
    digitalWrite(togglePIN,LOW);
    isReading = 1;
    mark = 0;
  }

  if (mark == readingTime && isReading == 1){
    digitalWrite(togglePIN,HIGH);
    isReading = 0;
    mark = 0;
  }
  
  
  if(isReading == 1 && mark == 2 && counter == 28000){
    Vrl = (byte) analogRead(readPIN)*0.48876; // 0.48876=5.0/1023*10;
  }
  
}




void loop() {
  
}

