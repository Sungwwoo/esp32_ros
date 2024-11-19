#include <WiFi.h>
#include <WiFiClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ros.h>
#include <std_msgs/Int64MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#define PI 3.141592
// Indecies for motors and encoders
#define RIGHT 1
#define LEFT 0

#define MOTOR_CONTROL_FREQ  60  // Hz
#define VELOCITY_CALC_FREQ  60  // Hz
#define INFO_PUB_FREQ       60  // Hz

/* Pin Info

// MOTORS
IN1 - 23; IN2 - 19; IN3 - 18; IN4 - 5;

// ENCODERS: Encoder value increases when the robot is moving forward
R_Encoder_A - 34; R_Encoder_B - 35;
L_Encoder_A -  4; L_Encoder_B - 15;

*/


// WIFI AP

// Bandal
// const char* ssid = "Sungwoo";
// const char* password = "1432850781";
// IPAddress server(192, 168, 0, 53); // rosmaster uri

// HRI
// const char* ssid = "HRI_01B";
// const char* password = "auto2957";
// IPAddress server(192, 168, 0, 38); // rosmaster uri


// HOTSPOT
const char* ssid = "SWS23";
const char* password = "sungwooo";
IPAddress server(192, 168, 97, 144); // rosmaster uri

const uint16_t serverPort = 11411; // 11411 is default port of rosserial tcp

/***************** ROS ******************/
ros::NodeHandle nh;

// headers
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];
char joint_state_header_frame_id[30];


// Encoder values
std_msgs::Int64MultiArray encoder_msg;
ros::Publisher pub_enc("encoder", &encoder_msg);

int64_t encoder_data[2] = {0, 0};
int64_t last_count_diff[2] = {0, 0};
double motor_rad[2] = {0.0, 0.0};
void publishEncoders(void);

// cmd_vel
geometry_msgs::Twist cmd_vel_msg;
float cmd_lin_vel, cmd_ang_vel;
double target_vel[2] = {0.0, 0.0};
double current_motor_vel[2] = {0.0, 0.0};
int motor_pwm[2] = {0, 0};

// cmd_vel callback
void cbCmdVel(const geometry_msgs::Twist& cmd_vel_msg);

void calcVelocities(void);
void controlMotors(void);

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", cbCmdVel);

// odometry
// Odom
nav_msgs::Odometry odom;
ros::Publisher pub_odom("odom", &odom);
void calcOdom(void);
void updateOdom(void);
void publishOdom(void);

// tf
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;
void updateTFPrefix(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);

// joint state
sensor_msgs::JointState joint_states;
ros::Publisher pub_joint_states("joint_states", &joint_states);
void updateJointStates(void);
void publishJointStates(void);


float odom_pose[3];
double odom_vel[3];
static uint32_t times[4]; // 0: motor control, 1: Info publish, ...
uint32_t prev_encoder_time;
uint32_t current_encoder_time;
uint32_t timediff;

/********************* MOTORS ***************************/
// Set Index 1 as Right, 0 as Left.

// Directly using PWM for IN1 and IN2 for each motor
typedef struct { int in1; int in2;} Motor;  // Motor Setting  
volatile Motor Mot[2] = {{23,19}, {5,18}};  // Chech your motor wiring and change pin munbers for IN1 & IN2.


/********************* ENCODERS ***************************/
typedef struct { int pinA; int pinB; int pos; int del;} Encoder; // Encoder Setting   
// Foward -> Increase, Backward -> Decrease
volatile Encoder Enc[2] = {{4,15,0,0},{35,34,0,0}};  // Swap A & B if reverse count
/* auxiliary debouncing function                             */
void debounce(int del) { // can't use delay in the ISR
    for (int k=0;k<del;k++) k = k +0.0 +0.0 -0.0 +3.0 -3.0;
}


// Wheel Informations
double wheel_radius = 0.0325; // m
int64_t one_cycle_count = 2340;
double wheel_seperation = 0.136; // m

double count2rad = (2 * PI) / one_cycle_count ;
double rad2count = one_cycle_count / (2 * PI);
int meter_to_pulse(double meter){
  return meter / (wheel_radius * count2rad);
}

int64_t lastEn0pos, lastEn1pos;

/********************** ENCODER ISRs *********************************/
/* Interrupt Service Routine(ISR): change on pin A for Encoder 0  */
void IRAM_ATTR isrPinAEn0(){
  int64_t drB = digitalRead(Enc[0].pinB); /* read pin B right away */
  debounce(Enc[0].del);   /* possibly wait before reading pin A, then read it */
  int64_t drA = digitalRead(Enc[0].pinA);
  /* this updates the counter                                */
  if (drA == HIGH) {   /* low->high on A? */ 
    if (drB == LOW) Enc[0].pos++;  /* clockwise: increment by checking pin B */  
    else Enc[0].pos--;  /* going counterclockwise: decrement  */
  } else {                       /* must be high to low on A */
    if (drB == HIGH) Enc[0].pos++;  /* clockwise: increment by checking pin B */
    else Enc[0].pos--;  /* going counterclockwise: decrement  */
  } /* end counter update                                    */
} /* end ISR pin A Encoder 0                                 */

/* Interrupt Service Routine: change on pin B for Encoder 0  */
void IRAM_ATTR isrPinBEn0(){ 
  int64_t drA = digitalRead(Enc[0].pinA); /* read pin A right away */
  debounce(Enc[0].del);   /* possibly wait before reading pin B, then read it */
  int64_t drB = digitalRead(Enc[0].pinB);
if (drB == HIGH) {   /* low->high on B? */
    if (drA == HIGH) Enc[0].pos++;  /* clockwise: increment check pin A */
    else Enc[0].pos--;  /* going counterclockwise: decrement  */
  } else {                       /* must be high to low on B */
    if (drA == LOW) Enc[0].pos++;  /* clockwise: increment check pin A */
    else Enc[0].pos--;  /* going counterclockwise: decrement  */
  } /* end counter update */
} /* end ISR pin B Encoder 0  */

void IRAM_ATTR isrPinAEn1(){   /* ISR: change on pin A for Encoder 1  */
  int64_t drB = digitalRead(Enc[1].pinB); /* read pin B right away */ 
  debounce(Enc[1].del);   /* possibly wait before reading pin A, then read it */
  int64_t drA = digitalRead(Enc[1].pinA);
if (drA == HIGH) {   /* low->high on A? */
    if (drB == LOW) Enc[1].pos++;  /* going clockwise: increment         */
    else Enc[1].pos--;  /* going counterclockwise: decrement  */
  } else { /* must be high to low on A                       */
    if (drB == HIGH) Enc[1].pos++;  /* going clockwise: increment         */
    else Enc[1].pos--;  /* going counterclockwise: decrement  */   
  } /* end counter update                                    */
} /* end ISR pin A Encoder 1                                 */

void IRAM_ATTR isrPinBEn1(){    /* ISR: change on pin B for Encoder 1  */
  int64_t drA = digitalRead(Enc[1].pinA); /* read pin A right away */
  debounce(Enc[1].del);   /* possibly wait before reading pin B, then read it */
  int64_t drB = digitalRead(Enc[1].pinB);
if (drB == HIGH) {   /* low->high on B? */
    if (drA == HIGH) Enc[1].pos++;  /* going clockwise: increment         */
    else Enc[1].pos--;  /* going counterclockwise: decrement  */ 
  } else { /* must be high to low on B                       */
    if (drA == LOW) Enc[1].pos++;  /* going clockwise: increment         */
    else Enc[1].pos--;  /* going counterclockwise: decrement  */  
  } /* end counter update                                    */
} /* end ISR pin B Encoder 1 */

/********************* PWM Properties ***************************/
const int freq = 30000;
const int pwm_in1 = 0; 
const int pwm_in2 = 1; 
const int pwm_in3 = 2; 
const int pwm_in4 = 3;
const int resolution = 8;
int dutyCycle = 200;


void carStop() {
  ledcWrite(pwm_in1, 0);
  ledcWrite(pwm_in2, 0);
  ledcWrite(pwm_in3, 0);
  ledcWrite(pwm_in4, 0);
}


int duty = 215;  // Speed in duty : 200 is moving, 255 is maximum. 
char ch = 0; // for incoming serial data



void setup(void) {


  // Motor Setup
  // sets the pins as outputs:
  pinMode(Mot[RIGHT].in1, OUTPUT);     pinMode(Mot[RIGHT].in2, OUTPUT);
  pinMode(Mot[LEFT].in1, OUTPUT);     pinMode(Mot[LEFT].in2, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwm_in1, freq, resolution);     ledcSetup(pwm_in2, freq, resolution);
  ledcSetup(pwm_in3, freq, resolution);     ledcSetup(pwm_in4, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(Mot[RIGHT].in1, pwm_in1);     ledcAttachPin(Mot[RIGHT].in2, pwm_in2);
  ledcAttachPin(Mot[LEFT].in1, pwm_in3);     ledcAttachPin(Mot[LEFT].in2, pwm_in4);
  carStop();

  Serial.begin(115200);     Serial.println("");
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");


  // Wait for connection
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }


  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


  
  // Encoder pin setup
  pinMode(Enc[RIGHT].pinA, INPUT);
  pinMode(Enc[RIGHT].pinB, INPUT);
  pinMode(Enc[LEFT].pinA, INPUT);
  pinMode(Enc[LEFT].pinB, INPUT);
  // turn on pullup resistors
  digitalWrite(Enc[RIGHT].pinA, HIGH);
  digitalWrite(Enc[RIGHT].pinB, HIGH);
  digitalWrite(Enc[LEFT].pinA, HIGH);
  digitalWrite(Enc[LEFT].pinB, HIGH);
  // encoder pin on interrupts
  attachInterrupt(Enc[RIGHT].pinA, isrPinAEn1, CHANGE);
  attachInterrupt(Enc[RIGHT].pinB, isrPinBEn1, CHANGE);
  attachInterrupt(Enc[LEFT].pinA, isrPinAEn0, CHANGE);
  attachInterrupt(Enc[LEFT].pinB, isrPinBEn0, CHANGE);

  // ROS Initialize

  Serial.println("Initializing ROS");


  // Connect to master
  
  Serial.println("Connecting to ROS master...");
  nh.getHardware()->setConnection(server, serverPort);

  Serial.println("Connected!");

  // Node initialize
  delay(500);
  Serial.println("Initializing rosserial node...");
  nh.initNode();

  // Publisher advertise
  Serial.println("Advertising publishers...");
  nh.advertise(pub_odom);
  nh.advertise(pub_enc);
  nh.advertise(pub_joint_states);

  tf_broadcaster.init(nh);


  // Subscriber
  Serial.println("Setting up subscribers...");
  nh.subscribe(sub_cmd_vel);

  
  Serial.println("Initializing msgs...");

  encoder_msg.data_length = 2;
  encoder_data[RIGHT] = 0;
  encoder_data[LEFT] = 0;
  encoder_msg.data = encoder_data;

  updateTFPrefix();

  // Initialize odom
  for (int index = 0; index < 3; index++){
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;

  joint_states.name_length = 2;
  joint_states.position_length = 2;
  joint_states.velocity_length = 2;
  joint_states.effort_length   = 2;

  static char *joint_states_name[] = {(char*)"joint_right_wheel", (char*)"joint_left_wheel"};
  joint_states.header.frame_id = joint_state_header_frame_id;

  joint_states.name = joint_states_name;        

  float temp[] = {0.0, 0.0};
  joint_states.position = temp;
  joint_states.velocity = temp;
  joint_states.effort = temp;


  Serial.println("ESP32 Initialized");
  prev_encoder_time = millis();

  lastEn0pos = 0; lastEn1pos = 0;
}

void loop(void) {  
  if (!nh.connected()) {
    // Serial.println("connection failed. Retrying...");
    delay(500);
  }

  else  {
    uint32_t t = millis();
    if ((t - times[2]) >= (1000 / MOTOR_CONTROL_FREQ)){
      controlMotors();
      times[2] = t;
    }
    if ((t - times[0]) >= (1000 / VELOCITY_CALC_FREQ)){
      calcVelocities();
      times[0] = t;
    }

    if ((t - times[1]) >= (1000 / INFO_PUB_FREQ)){
      publishEncoders();
      publishOdom();
      publishJointStates();
      times[1] = t;
    }

  }
  nh.spinOnce();
}

/****** Functions ******/
// cmd_vel callback
void cbCmdVel(const geometry_msgs::Twist& cmd_vel_msg){
  cmd_lin_vel = cmd_vel_msg.linear.x;
  cmd_ang_vel = cmd_vel_msg.angular.z;
}


void publishEncoders(){
  

  encoder_data[RIGHT] = Enc[RIGHT].pos;
  encoder_data[LEFT] = Enc[LEFT].pos;
  Serial.print("Encoders: "); Serial.print(Enc[LEFT].pos); Serial.print(" | "); Serial.println(Enc[RIGHT].pos); 
  encoder_msg.data = encoder_data;

  pub_enc.publish(&encoder_msg);
}

void calcVelocities(){
  target_vel[LEFT] = cmd_lin_vel + (cmd_ang_vel * wheel_seperation/2);
  target_vel[RIGHT] = cmd_lin_vel - (cmd_ang_vel * wheel_seperation/2);

  // Right motor velocity


  // Use Fixed Velocities 

  if (target_vel[RIGHT] > 0.03) motor_pwm[RIGHT] = 250;
  else if (target_vel[RIGHT] > 0) motor_pwm[RIGHT] = 150;
  else if (target_vel[RIGHT] < -0.03) motor_pwm[RIGHT] = -250;
  else if (target_vel[RIGHT] < 0) motor_pwm[RIGHT] = -150;
  else motor_pwm[RIGHT] = 0;

  if (target_vel[LEFT] > 0.03) motor_pwm[LEFT] = 250;
  else if (target_vel[LEFT] > 0) motor_pwm[LEFT] = 150;
  else if (target_vel[LEFT] < -0.03) motor_pwm[LEFT] = -250;
  else if (target_vel[LEFT] < 0) motor_pwm[LEFT] = -150;
  else motor_pwm[LEFT] = 0;

  // for(int i = 0; i<2; i++)
  //   if (target_vel[i] > 0.6) target_vel[i] = 0.6;
  //   else if (target_vel[i] < -0.6) target_vel[i] = -0.6;
  //   else if ((target_vel[i] > 0.0) && (target_vel[i] < 0.13)) target_vel[i] = 0.13;
  //   else if ((target_vel[i] < 0.0) && (target_vel[i] > -0.13)) target_vel[i] = -0.13;
  //   else target_vel[i] = 0.0;


  Serial.println();
  Serial.print("Target Velocity: "); Serial.print(target_vel[LEFT]); Serial.print(" m/s | ");
  Serial.print(target_vel[RIGHT]);Serial.println(" m/s");

  Serial.print("Current Velocity: ");
  Serial.print(current_motor_vel[LEFT]); Serial.print(" m/s | "); 
  Serial.print(current_motor_vel[RIGHT]); Serial.println(" m/s");

  int err_p = 50;
  // motor_pwm[LEFT] += (int)(err_p *(target_vel[LEFT] - current_motor_vel[LEFT]));
  // motor_pwm[RIGHT] += (int)(err_p *(target_vel[RIGHT] - current_motor_vel[RIGHT]));

  for(int i = 0; i<2; i++){  
    // Max Velocity: 0.60 m/s at 250
    // Min Velocity: 0.18 m/s at 130
    if (motor_pwm[i] > 255) motor_pwm[i] = 255;
    else if (motor_pwm[i] < -255) motor_pwm[i] = -255;
  }
}

void controlMotors(){

  
  Serial.print("PWM command: ");
  Serial.print(motor_pwm[LEFT]); Serial.print(" | "); 
  Serial.print(motor_pwm[RIGHT]); Serial.println("");
  if (motor_pwm[RIGHT] > 0){
    ledcWrite(pwm_in1, motor_pwm[RIGHT]);
    ledcWrite(pwm_in2, 0);
  }
  else if (motor_pwm[RIGHT] < 0){
    ledcWrite(pwm_in1, 0);
    ledcWrite(pwm_in2, -motor_pwm[RIGHT]);
  }
  else{
    ledcWrite(pwm_in1, 0);
    ledcWrite(pwm_in2, 0);
  }

  if (motor_pwm[LEFT] > 0){
    ledcWrite(pwm_in3, motor_pwm[LEFT]);
    ledcWrite(pwm_in4, 0);
  }
  else if (motor_pwm[LEFT] < 0){
    ledcWrite(pwm_in3, 0);
    ledcWrite(pwm_in4, -motor_pwm[LEFT]);
  }
  else{
    ledcWrite(pwm_in3, 0);
    ledcWrite(pwm_in4, 0);
  }
}

void calcOdom(){
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  current_encoder_time = millis();

  timediff = current_encoder_time - prev_encoder_time;
  int64_t current_en_pos = encoder_data[RIGHT];
  last_count_diff[RIGHT] = current_en_pos - lastEn1pos;
  lastEn1pos = current_en_pos;
  
  current_en_pos = encoder_data[LEFT];
  last_count_diff[LEFT] = current_en_pos - lastEn0pos;
  lastEn0pos = current_en_pos;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = (double)timediff / 1000.0;

  wheel_l = count2rad * (double)last_count_diff[LEFT];
  wheel_r = count2rad * (double)last_count_diff[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s = wheel_radius * (wheel_r + wheel_l) / 2.0;
  delta_theta = wheel_radius * (wheel_r - wheel_l) / wheel_seperation;  
  

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;
  // compute odometric instantaneouse velocity

  v = delta_s / step_time;
  w = delta_theta / step_time;


  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  // rad
  motor_rad[LEFT] += wheel_l;
  motor_rad[RIGHT] += wheel_r;

  // m/s
  current_motor_vel[LEFT]  = wheel_radius * wheel_l / step_time;
  current_motor_vel[RIGHT] = wheel_radius * wheel_r / step_time; 
  prev_encoder_time = current_encoder_time;
}

void updateOdom(){
  calcOdom();
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id  = odom_child_frame_id;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}

void publishOdom(){
  
  updateOdom();
  
  odom.header.stamp =  nh.now();
  pub_odom.publish(&odom);

  updateTF(odom_tf);
  odom_tf.header.stamp =  nh.now();
  tf_broadcaster.sendTransform(odom_tf);

}

void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

void updateTFPrefix(){
  sprintf(odom_header_frame_id, "odom");
  sprintf(odom_child_frame_id, "base_link");  
  sprintf(joint_state_header_frame_id, "base_link");
}

void updateJointStates(){
  
  static float joint_states_pos[2] = {0.0, 0.0};
  static float joint_states_vel[2] = {0.0, 0.0};
  static float joint_states_eff[2] = {0.0, 0.0};

  joint_states_pos[LEFT]  = (float)motor_rad[LEFT];
  joint_states_pos[RIGHT] = (float)motor_rad[RIGHT];

  joint_states_vel[LEFT]  = (float)current_motor_vel[LEFT];
  joint_states_vel[RIGHT] = (float)current_motor_vel[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
  joint_states.effort = joint_states_eff;
}

void publishJointStates(){
  updateJointStates();
  joint_states.header.stamp = nh.now();
  pub_joint_states.publish(&joint_states);

}
