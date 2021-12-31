#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#define M_SPEED 100 // The default motor speed
#define r_C1 2
#define r_C2 13
#define l_C1 3
#define l_C2 12
#define CPR 3374 // The number of counts from the encoder per rotation
// Conversion from counts to angles radian -> angle = (pos/374) * (2*PI)
#define ISC '<' // Input Start char
#define IEC '>' //Input End Char
#define OSC '(' // Output Start char
#define OEC ')' // Output End Char
#define MAX_IN_BUFFER 80
bool started=true;
bool ended=true;
char inData[MAX_IN_BUFFER];

typedef struct{
  float right;
  float left;
} wheel_vels;
wheel_vels goalVels;
wheel_vels currentVels;

// right connections
int enA = 9;
int in1 = 8;
int in2 = 7;
// left connections
int enB = 5;
int in3 = 6;
int in4 = 4;
volatile int r_pos_i = 0;
volatile int l_pos_i = 0;
int r_pos = 0;
int l_pos = 0;
int r_lastPos= 0;
int l_lastPos= 0;

long lastTime = 0;
int r_dir = 1;
int l_dir = 1;
float r_e_d = 0.; // Differential error for the right wheel
float l_e_d = 0.; // Differential error for the left wheel
float r_e_i = 0.; // Integral error for the right wheel
float l_e_i = 0.; // Integral error for the left wheel

float kp = 1.0;
float ki = 1.0;
float kd = 1.0;
float r_i_term = 0;
float l_i_term = 0;
float r_last_e = 0; // To calculate derivative error
float l_last_e = 0; // To calculate derivative error
float r_currentPWM = 0.;
float l_currentPWM = 0.;

const int filter_width = 10; 
unsigned long last_control_time = 0;
float control_frequency = 10.0;
unsigned long control_time_gap = 0;
byte index = 0;
bool dataAvailable = false;
// TODO: Add a timer function to print the velocity graphs. Both for current velocity and goal velocity
void setup() {
  Serial.begin(115200);
  // This line will change the frequecy of the timer itself. The default prescaler is 64. This changes it to 8.
  // This means the 
  //TCCR0B = TCCR0B & B11111000 | B00000010; 
  // Set all the motor control pins to outputs
  pinMode(r_C1,INPUT);
  pinMode(r_C2,INPUT);
  pinMode(l_C1,INPUT);
  pinMode(l_C2,INPUT);
  attachInterrupt(digitalPinToInterrupt(r_C1),readRightEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(l_C1),readLeftEncoder,RISING);
  l_pos = 0;
  r_pos = 0;
  l_dir = 1;
  r_dir = 1;

  started=false;
  ended=false;
  index=0;
  pinMode(enB,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  pinMode(enA,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  //analogWrite(enB, 90);

  // Initial set of pid values
  kp = 10.0;
  ki = 0.01;
  kd = 0.3;
  
  control_time_gap = (unsigned long)(1000000.0/control_frequency);
  goalVels.right = 0.0;
  goalVels.left = 0.0;
  currentVels.right = 0.0;
  currentVels.left = 0.0;
  delay(1000);
//  analogWrite(enB, 255);
//  analogWrite(enA, 50);

}

void loop() {
  float command[4];
  while(Serial.available() > 0)
  {
    char inChar = Serial.read();
    if(inChar == ISC)
    {
      index = 0;
      inData[index] = '\0';
      started = true;
      ended = false;
    }
    else if(inChar == IEC)
    {
      ended = true;
      break;
    }
    else if(started && !ended)
    {
      if(index < MAX_IN_BUFFER)
      {
        inData[index] = inChar;
        index++;
        inData[index] = '\0';
      }
    }
  }
  // We are here either because all pending serial
  // data has been read OR because an end of
  // packet marker arrived. Which is it?
  if(started && ended)
  {
    char* val = strtok(inData, ",");
    if(val != NULL)
    {
      goalVels.right = atof(val);
    } 
    
    val = strtok (NULL,",");
    if(val != NULL )
    {
      goalVels.left = atof(val);
    }
    Serial.print(OSC);
    Serial.print(currentVels.right);
    Serial.print(",");
    Serial.print(currentVels.left);
    Serial.print(",");
    Serial.print(r_pos);
    Serial.print(",");
    Serial.print(l_pos);
    Serial.println(OEC);
  // Reset for the next packet
  started = false;
  ended = false;
  index = 0;
  inData[index] = '\0';
  }
  
  if(micros() - last_control_time > control_time_gap)
  {
  unsigned long currentTime = micros();
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      r_pos = r_pos_i;
      l_pos = l_pos_i;
    }
    float deltaT = (currentTime - lastTime)/(1000000.0);

    float r_dtheta = ((r_pos - r_lastPos)/(float)CPR) * (2*PI); // The angle the motor shaft moved through since the last time we checked
    float l_dtheta = ((l_pos - l_lastPos)/(float)CPR) * (2*PI); // The angle the motor shaft moved through since the last time we checked
    currentVels.right = (float)r_dtheta/deltaT; // The current velocity of the wheel
    currentVels.left = (float)l_dtheta/deltaT; // The current velocity of the wheel


    r_lastPos = r_pos;
    l_lastPos = l_pos;
    lastTime = currentTime;
    

      last_control_time = micros();
      float r_e = goalVels.right - currentVels.right;
      float l_e = goalVels.left - currentVels.left;
      r_i_term += r_e;
      l_i_term += l_e;
      float r_d_term = r_e - r_last_e;
      float l_d_term = l_e - l_last_e;
      r_last_e = r_e;
      l_last_e = l_e;

      float right_control = r_e * kp + r_i_term * ki + kd * r_d_term;
      float left_control  = l_e * kp + l_i_term * ki + kd * l_d_term;


      r_currentPWM += right_control;
      

      l_currentPWM += left_control;

      if(r_currentPWM > 255)
      {
        r_currentPWM = 255;
      }
      else if(r_currentPWM < -255)
      {
        r_currentPWM = -255;
      }
      if(l_currentPWM > 255)
      {
        l_currentPWM = 255;
      }
      else if(l_currentPWM < -255)
      {
        l_currentPWM = -255;
      }


      setMotorDirection(r_currentPWM, 'r', r_dir, goalVels.right);
      setMotorDirection(l_currentPWM, 'l', l_dir, goalVels.left);
      // TODO: Have different right and left goal velocities
      if(abs(goalVels.right) < 0.001)
      {
        r_currentPWM = 0;
      }
      if(abs(goalVels.left) < 0.001)
      {
        l_currentPWM = 0;
      }
      analogWrite(enA, (int)abs(r_currentPWM));
      analogWrite(enB, (int)abs(l_currentPWM));

      last_control_time = micros();
    }
      
    delay(1);
}

void setMotorDirection(float pwm, char side, int& dir, float goalVel)
{
  int i1, i2;
  if(side == 'r')
  {
    
    i1 = in1;
    i2 = in2;
  }
  else if(side == 'l')
  {
    i1 = in3;
    i2 = in4;
  }

  if(goalVel > 0 && pwm > 0)
  {
    digitalWrite(i1, HIGH);
    digitalWrite(i2, LOW);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      dir = 1;
    }
  }
  else if(goalVel < 0 && pwm < 0)
  {
    digitalWrite(i1, LOW);
    digitalWrite(i2, HIGH);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      dir = -1;
    }
  }
  return;
}

int kickstartMotors()
{
  int kickstartPWM = 200;
  analogWrite(enB, kickstartPWM);
  analogWrite(enA, kickstartPWM);
  r_currentPWM = kickstartPWM;
  l_currentPWM = kickstartPWM;
  delay(200);
  //return; // This should work similarly to loop 
}
void readRightEncoder(){
    r_pos_i += r_dir;
}
void readLeftEncoder(){
    l_pos_i += l_dir;
}
