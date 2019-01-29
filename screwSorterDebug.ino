#include <Servo.h> 
#define POSMINA 40
#define POSMAXA 190
#define POSMINB 45
#define POSMAXB 145
#define POSMINZ 45
#define POSMAXZ 132
#define MAXSPEED 160 //60 deg / 0.160 sec = 375
#define MAXACC 300    //reach maxspeed in one sec
#define COILPIN 6 //pin 5 and 6 have 980 dutycycle, others 490 Hz,
#define LITEPIN 12
#define MAXSPEEDZ 160
#define MAXACCZ 300
#define TOLERANCE 1
#define TOLERANCEZ 1

Servo myservo_a, myservo_b, myservo_z;
float t1,t2,t3,t4,t5,t6;//transition time
float speed_a,speed_b, speed_z,speedmax_a,speedmax_b, speedmax_z,acc_a = MAXACC,acc_b = MAXACC, acc_z=MAXACCZ;//max speed a speed b acceleration a and b
int phase=0, coil=0;//phase 1: acceleration, 2: constant speed, 3: desceleration, 0: stop
float pos_a=90, pos_b=90, pos_z=POSMINZ, newpos_a=90, newpos_b=90, newpos_z=90, oldpos_a=90, oldpos_b=90, oldpos_z=POSMINZ, dtot_a=0, dtot_b=0, dtot_z=0, signe_a=1, signe_b=1, signe_z=1;
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false , cas=1, cas_z=1, lite=0;  // whether the string is complete
const unsigned long dt=5;                   //time step in milisecond
unsigned long previousMillis = 0, startTime = 0, currentMillis=0;
boolean ok=0;

void setup() {
  // put your setup code here, to run once:
  myservo_a.attach(10);
  myservo_b.attach(9);//the big one
  myservo_z.attach(11);//z axix
  // initialize serial:
  Serial.begin(9600);
  // reserve 20 bytes for the inputString:
  inputString.reserve(20);
  pinMode(COILPIN, OUTPUT);
  pinMode(LITEPIN, OUTPUT);
  digitalWrite(LITEPIN,1);
}

void loop() {
  serialEvent();
  if (stringComplete) {
    //Serial.println(inputString);
    newpos_a=(float)inputString.toInt();
    inputString.remove(0,inputString.indexOf(" ")+1);
    newpos_b=(float)inputString.toInt();
    inputString.remove(0,inputString.indexOf(" ")+1);
    newpos_z=(float)inputString.toInt();
    inputString.remove(0,inputString.indexOf(" ")+1);
    coil=inputString.toInt();
    inputString.remove(0,inputString.indexOf(" ")+1);
    lite=(bool)inputString.toInt();
    analogWrite(COILPIN, coil);
    digitalWrite(LITEPIN, !lite);
    
    if(newpos_a > POSMAXA){
      newpos_a = POSMAXA;
    }
    if(newpos_a < POSMINA){
      newpos_a = POSMINA;
    }
    if(newpos_b > POSMAXB){
      newpos_b = POSMAXB;
    }
    if(newpos_b < POSMINB){
      newpos_b = POSMINB;
    }
    if(newpos_z > POSMAXZ){
      newpos_z = POSMAXZ;
    }
    if(newpos_z < POSMINZ){
      newpos_z = POSMINZ;
    }
    
    if(newpos_a < oldpos_a){
      signe_a = -1;
    }
    else{
      signe_a = 1;
    }
    if(newpos_b < oldpos_b){
      signe_b = -1;
    }
    else{
      signe_b = 1;
    }
    if(newpos_z < oldpos_z){
      signe_z = -1;
    }
    else{
      signe_z = 1;
    }
    //Serial.println(newpos_a);
    //Serial.println(newpos_b);
    // clear the string:
    inputString = "";
    stringComplete = false;
    dtot_a = newpos_a - oldpos_a;
    dtot_b = newpos_b - oldpos_b;
    dtot_z = newpos_z - oldpos_z;
    speedmax_a = MAXSPEED * signe_a;
    speedmax_b = MAXSPEED * signe_b;
    speedmax_z = MAXSPEEDZ * signe_z;
    acc_a = MAXACC * signe_a;
    acc_b = MAXACC * signe_b;
    acc_z = MAXACCZ * signe_z;
    speed_a = 0;
    speed_b = 0;
    speed_z = 0;
    pos_a = oldpos_a;
    pos_b = oldpos_b;
    pos_z = oldpos_z;
    t1 = (float)MAXSPEED / (float)MAXACC;
    
    if(abs(dtot_a) > abs(dtot_b)){
      if((float)MAXSPEED * t1 < abs(dtot_a)){
        //Serial.println("te1");
        cas=1;                                      //cas1: acc, const speed,  desceler
        t2 = abs(dtot_a / MAXSPEED);
        t3 = t2 + t1;
        speedmax_b = dtot_b * abs(speedmax_a / dtot_a);   //division by 0 impossible
        acc_b = speedmax_b / t1;                          //division by 0 impossible
      }
      else{   //acc, descel
        //Serial.println("te2");
        cas=0;
        speedmax_a = signe_a * sqrt(abs(dtot_a) * MAXACC);
        t1 = dtot_a / speedmax_a; //signs are going together -> always positive
        speedmax_b = dtot_b / t1; //t1 is non null in this case
        acc_b = speedmax_b / t1;
        t2=t1;//not used
        t3 = 2 * t1;
      }
    }
    else{                               
      if((float)MAXSPEED * t1 < abs(dtot_b)){  //profil trapeze
        //Serial.println("te3");
        cas=1;//acc, const speed,  desceler
        t2 = abs(dtot_b / MAXSPEED);
        t3 = t2 + t1;
        speedmax_a = dtot_a * abs(speedmax_b / dtot_b);//no division by 0
        acc_a = speedmax_a / t1;
      }
      else{   //triangle, 
        //Serial.println("te4");
        cas=0;
        speedmax_b = signe_b * sqrt(abs(dtot_b) * MAXACC);
        if(abs(dtot_b) > TOLERANCE){    
          t1 = dtot_b / speedmax_b;     //signs are going together -> always positive
          speedmax_a = dtot_a / t1;
          acc_a = speedmax_a / t1;
          t2=t1;//not used
          t3 = 2 * t1;
        }
        else{           //takes the case dtot_a = dtot_b = 0
          t1 = 0;
          speedmax_a = 0;
          acc_a = 0;
          t2 = 0;
          t3 = 0;
        }
      }
    }
    
    
    if(((float)MAXSPEEDZ * (float)MAXSPEEDZ / (float)MAXACCZ) < abs(dtot_z)){  // profile trapeze
      cas_z = 1;
      t4 = t3 + (float)MAXSPEEDZ / (float)MAXACCZ;
      t5 = t4 + abs(dtot_z / (float)MAXSPEEDZ);
      t6 = t5 + t4;
      //Serial.println("te5");
    }
    else{ //profil triangle: acc, descc
      //Serial.println("te6");
      cas_z = 0;
      speedmax_z = signe_z * sqrt(abs(dtot_z) * MAXACCZ);
      if(abs(dtot_z) > TOLERANCEZ){
        t4 = t3 + dtot_z /speedmax_z;
        t5 = t4; //not used
        t6 = t4 + dtot_z /speedmax_z;
      }
      else{
        t4 = t3;
        t5 = t3;
        t6 = t3;
      }
    }
    
//    Serial.println(t1);
//    Serial.println(t2-t1);
//    Serial.println(t3-t2);
//    Serial.println(t4-t3);
//    Serial.println(t5-t4);
//    Serial.println(t6-t5);

    phase = 1;
    startTime=millis();
  }
  
  currentMillis = millis();
  if (currentMillis - previousMillis >= dt) {
    previousMillis = currentMillis;
    if(phase==1){
      pos_a = pos_a + acc_a * (float)dt * (float)dt / 2000000 + speed_a * (float)dt / 1000;
      speed_a = speed_a + acc_a * (float)dt / 1000;
      //Serial.print(pos_a);
      //Serial.print(pos_a);
      pos_b = pos_b + acc_b * (float)dt * (float)dt / 2000000 + speed_b * (float)dt / 1000;
      speed_b = speed_b + acc_b * (float)dt / 1000;
      //Serial.print(speed_a);
      if(currentMillis - startTime > (unsigned long)(t1 * 1000)){
        if(cas){
          phase = 2;
          //Serial.println("phase2");
        }
        else{
          phase = 3;
          //Serial.println("phase3");
        }
        
      }
    }
    
    if(phase==2){
      pos_a = pos_a + speedmax_a * (float)dt / 1000;
      //Serial.print(pos_a);
      pos_b = pos_b + speedmax_b * (float)dt / 1000;
      //Serial.print(speed_a);
      if(currentMillis - startTime > (unsigned long)(t2 * 1000)){
        phase = 3;
        //Serial.println("phase3");
      }
    }
    
    if(phase==3){
      speed_a = speed_a - acc_a * (float)dt / 1000; 
      pos_a = pos_a + acc_a * (float)dt * (float)dt / 2000000 + speed_a * (float)dt / 1000;
            //is is reversed from acceleration
      
      //Serial.print(pos_a);
      speed_b = speed_b - acc_b * (float)dt / 1000;
      pos_b = pos_b + acc_b * (float)dt * (float)dt / 2000000 + speed_b * (float)dt / 1000;
      
      
      //Serial.print(speed_a);
      if(currentMillis - startTime > (unsigned long)(t3 * 1000)){ //what is the best condition to stop?
        phase=4;
        //Serial.println(phase);
      }
    }
    if(phase==4){ //z move
      pos_z = pos_z + acc_z * (float)dt * (float)dt / 2000000 + speed_z * (float)dt / 1000;
      speed_z = speed_z + acc_z * (float)dt / 1000;
      
      if(currentMillis - startTime > (unsigned long)(t4 * 1000)){
        if(cas_z){
          phase = 5;
          //Serial.println("phase2");
          //Serial.println(phase);
        }
        else{
          phase = 6;
          //Serial.println("phase3");
          //Serial.println(phase);
        }
      }
    }
    if(phase == 5){
      pos_z = pos_z + speedmax_z * (float)dt / 1000;
      
      if(currentMillis - startTime > (unsigned long)(t5 * 1000)){
        phase = 6;
        //Serial.println("phase3");
        //Serial.println(phase);
      }
    }
    if(phase == 6){
      speed_z = speed_z - acc_z * (float)dt / 1000;
      pos_z = pos_z + acc_z * (float)dt * (float)dt / 2000000 + speed_z * (float)dt / 1000;
      
      
      //Serial.println(pos_z);
      if(currentMillis - startTime > (unsigned long)(t6 * 1000)){
        phase = 0;
        pos_a = newpos_a;
        pos_b = newpos_b;
        pos_z = newpos_z;
        //Serial.println(pos_a);
        //Serial.println(pos_b);
        oldpos_a = newpos_a;
        oldpos_b = newpos_b;
        oldpos_z = newpos_z;
        //Serial.println("phase0");
        //Serial.println("ok");
        ok=1;
      }
    }
    
    
    
      //Serial.println(speed_a);
      //Serial.println(speed_b);
    
    //Serial.println(acc_a);
    //Serial.println(pos_z);
    if(pos_a > POSMAXA){
      pos_a = POSMAXA;
    }
    if(pos_a < POSMINA){
      pos_a = POSMINA;
    }
    if(pos_b > POSMAXB){
      pos_b = POSMAXB;
    }
    if(pos_b < POSMINB){
      pos_b = POSMINB;
    }
    if(pos_z > POSMAXZ){
      pos_z = POSMAXZ;
    }
    if(pos_z < POSMINZ){
      pos_z = POSMINZ;
    }
    myservo_a.write((int)pos_a);
    myservo_b.write((int)pos_b);
    myservo_z.write((int)pos_z);
  }
  if(ok){
      Serial.println("ok");
      ok=0;
    }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
