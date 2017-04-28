#include <TimerOne.h>
#define heatingTime 2 //2*baseTime
#define readingTime 3 //3*readingTime
#define baseTime 30000
#define togglePIN 2
#define readPIN 3
unsigned int counter;
char mark;
char isReading;

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
  Serial.println((float)analogRead(readPIN)*5/1023);
  
}




void loop() {
  
}

