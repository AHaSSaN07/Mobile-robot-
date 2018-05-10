  #include <math.h>

  const int IN1 = 4;
  const int IN2 = 5;
  const int TS = 100;
  const int IN3 = 6;
  const int IN4 = 7;
  const float P_controller = 2.5;
  const float L = 13.5e-2;
  const int enableLeftWheel = 9;
  const int enableRightWheel = 10;
  const float V = 0.2;
  const int Encoder1 = 2;
  const int Encoder2 = 3;
  float R = 3.2e-2;
  float C = 15; //dc gain for motors 
  const int ticks_per_rev = 8;
  float RPML = 0;
  float RPMR = 0;
  volatile float theta_goal = 0;
  volatile float error = 0;
  volatile unsigned int TickR = 0;
  volatile unsigned int TickL = 0;
  float vr = 0;
  float vl = 0;
  float V_R = 0;
  float V_L = 0;
  float omega = 0;
  float D_R = 0, D_L = 0, D_C;
  float x = 0, y = 0, theta = 0;
  float goals[][2] = {{0.25*2,0}, {0.5*2, 0.5*2},{.5*2,.75*2}};
  int number_of_goals = sizeof(goals)/sizeof(goals[0]);
  int current_goal = 0;

  //          Removables
  float lastError=0;
  const float D_controller = 0;

  float clip(float value, float floor, float ceil){
  if (value>ceil) return ceil;
  if (value<floor) return floor;
  return value;
  }

  void moveRightWheel(int PWM){
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(enableRightWheel, PWM);
  }
  
  void moveLeftWheel(int PWM){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  analogWrite(enableLeftWheel, PWM);
  }


  void control_action()
  {
  error = theta_goal - theta;
  //clip(error, -PI/4, PI/4);
  omega = P_controller * error;
  omega = clip(omega, -P_controller*PI/2, P_controller*PI/2);
  //clip(omega, -PI/4, PI/4);
  }

float smoother(float old, float New, float threshold){
  if ((New-old)>threshold) return old+threshold;
  //if ((old-New)>threshold) return old-threshold;
  return New;
}


  void kinematics()
  {
    
    
    float VR = clip((V + omega * L / 2) / (R * C) * 30 / PI, 0, 5); // voltage for motors 
    float VL = clip((V - omega * L / 2) / (R * C) * 30 / PI, 0, 5);
    V_R = smoother(V_R, VR, 1.5);
    V_L = smoother(V_L, VL, 1.5);
  }

  void odemetry_model()
  {
  D_R = (2 * PI * R * TickR) / ticks_per_rev;
  D_L = (2 * PI * R * TickL) / ticks_per_rev;
  D_C = (D_R + D_L) / 2;
  x += (D_C * cos(theta));
  y += (D_C * sin(theta));
  theta += ((D_R - D_L) / L);
/*  if (theta>180) theta-=360;
  if (theta<-180) theta+=360;*/
  }



  void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(enableLeftWheel, OUTPUT);
  pinMode(enableRightWheel, OUTPUT);

  pinMode(Encoder1, INPUT);
  digitalWrite(Encoder1, HIGH);       // turn on pull-up resistor
  pinMode(Encoder2, INPUT);
  digitalWrite(Encoder2, HIGH);       // turn on pull-up resistor


  attachInterrupt(digitalPinToInterrupt(Encoder1), leftMotorInterrupt, RISING);  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(digitalPinToInterrupt(Encoder2), rightMotorInterrupt, RISING);  // encoder pin on interrupt 0 - pin 2
  Serial.begin(9600);
  }
  /*
  void loop() {
  int voltageModifier = 255/5;
  int voltage = 5;
  analogWrite(enableLeftWheel,voltage*voltageModifier);
  analogWrite(enableRightWheel,voltage*voltageModifier);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  RPML= (TickL*60)/(float)ticks_per_rev;
  RPMR= (TickR*60)/(float)ticks_per_rev;
  float CL = RPML/voltage;
  float CR = RPMR/voltage;
  if (C>0) C=(C+CR)/2;
  else C = CR;
  if (C>0) C=(C+CL)/2;
  else C = CL;
  Serial.println (voltage, DEC);
  Serial.println (RPMR, DEC);
  Serial.println (RPML, DEC);
  Serial.println (CR, DEC);
  Serial.println (CL, DEC);
  

  

  TickL = 0;
  TickR = 0;
  delay(TS);
  }
  
*/


void loop(){
  if (abs(x-goals[current_goal][0])<=0.1 && abs(y-goals[current_goal][1])<=0.1){
    if (current_goal<number_of_goals-1){
      current_goal++;
      analogWrite(enableLeftWheel, 0);
      analogWrite(enableRightWheel, 0);
     delay(500);
    //TickL = 0;
    //TickR = 0;
    Serial.println("==============================================");
      Serial.println("==============================================");
    }
    else {
      analogWrite(enableLeftWheel, 0);
      analogWrite(enableRightWheel, 0);      
      
    }
  }else{
    theta_goal = atan2(goals[current_goal][1]-y, goals[current_goal][0]-x); //theta_goal = atan(y1-y/x2-x) (rad)
  control_action();
  //error = theta_goal - theta (rad)    
  //omega = P_controller * error (rad);

  int voltageModifier = 255/5;

  kinematics();
  //Vr = V + omega
  moveRightWheel(V_R*voltageModifier);
  moveLeftWheel(V_L*voltageModifier);
  

  delay(TS);
  odemetry_model();


  Serial.println("theta_goal");
  Serial.println (theta_goal, DEC);
  Serial.println("V_R");
  Serial.println (V_R, DEC);
  Serial.println("V_l");
  Serial.println (V_L, DEC);
  Serial.println("omega");
  Serial.println (omega, DEC);
  Serial.println("theta");
  Serial.println (theta, DEC);
  Serial.println("x");
  Serial.println (x, DEC);
  Serial.println("y");
  Serial.println(y, DEC);
  Serial.println("left ticks");
  Serial.println(TickL, DEC);
  Serial.println("right ticks");
  Serial.println(TickR, DEC);
  
  TickL = 0;
  TickR = 0;
  }

}

void rightMotorInterrupt() {
TickR++;
}

void leftMotorInterrupt() {
TickL++;
}


