#include <WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <ESP32_Servo.h>

#define ESC_CAL_DELAY  2900  // Calibration delay (milisecond)
#define ESC_STOP_PULSE  500

Servo betaTL, betaTR, betaFL, betaFR, betaBL, betaBR;  // create servo object to control a servo

int escPinTL = 19;
int escPinTR = 21;

int escPinFL = 22;
int escPinFR = 23;

int escPinBL = 27;
int escPinBR = 12;

//global variables
int liftThrust = 1100, BLThrust=0, BRThrust=0, FLThrust=0, FRThrust = 0;

const char* ssid     = "hovercraft";
const char* password = "1234abcd";
IPAddress server(192,168,1,191);
const uint16_t serverPort = 11411;


void cmd_thrustCallback( const geometry_msgs::Pose& motorcomm)
{
  liftThrust = constrain(motorcomm.position.z,1000,2000);
  BRThrust = constrain(motorcomm.position.x,1000,2000);
  BLThrust = constrain(motorcomm.position.y,1000,2000);
  FRThrust = constrain(motorcomm.orientation.x,1000,2000);
  FLThrust = constrain(motorcomm.orientation.y,1000,2000);
}

ros::NodeHandle nh;
geometry_msgs::PoseStamped feedback_mesg;
ros::Publisher hovstatus("hoverstats", &feedback_mesg, queue_size=100);
ros::Subscriber<geometry_msgs::Pose> hovsub("/cmd_thu", &cmd_thrustCallback);


void stophover(){
  liftThrust = 1000, BLThrust=0, BRThrust=0, FLThrust=0, FRThrust = 0;
  betaTL.writeMicroseconds(ESC_STOP_PULSE);
  betaTR.writeMicroseconds(ESC_STOP_PULSE);
  betaFL.writeMicroseconds(ESC_STOP_PULSE);
  betaFR.writeMicroseconds(ESC_STOP_PULSE);
  betaBL.writeMicroseconds(ESC_STOP_PULSE);
  betaBR.writeMicroseconds(ESC_STOP_PULSE);
}


void setup() {
  pinMode(13, OUTPUT);  
  betaTL.attach(escPinTL);   
  betaTR.attach(escPinTR);                            
  betaFL.attach(escPinFL);                            
  betaFR.attach(escPinFR);                          
  betaBL.attach(escPinBL);                           
  betaBR.attach(escPinBR);                          
  
  betaTL.writeMicroseconds(1000);
  betaTR.writeMicroseconds(1000);
  betaFL.writeMicroseconds(1000);
  betaFR.writeMicroseconds(1000);
  betaBL.writeMicroseconds(1000);
  betaBR.writeMicroseconds(1000);
  
  delay(5000);
  
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  //Serial.print("ROS IP check= ");
  //Serial.println(nh.getHardware()->getLocalIP());

  //Init to be topics
  nh.advertise(hovstatus);
  nh.subscribe(hovsub);
  delay(1000); //extremely important delay.. 

  betaFL.writeMicroseconds(1000);
  betaFR.writeMicroseconds(1000);
  betaBL.writeMicroseconds(1000);
  betaBR.writeMicroseconds(1000);
}


void loop()
{
  if (nh.connected()) {
    digitalWrite(13, HIGH); 
    feedback_mesg.header.stamp = nh.now();
    feedback_mesg.pose.position.x = BRThrust;
    feedback_mesg.pose.position.y = BLThrust;
    feedback_mesg.pose.position.z = liftThrust;
    feedback_mesg.pose.orientation.x = FRThrust;
    feedback_mesg.pose.orientation.y = FLThrust;
    hovstatus.publish(&feedback_mesg);
    betaTL.writeMicroseconds(liftThrust);
    betaTR.writeMicroseconds(liftThrust);
    betaFL.writeMicroseconds(FLThrust);
    betaFR.writeMicroseconds(FRThrust);
    betaBL.writeMicroseconds(BLThrust);
    betaBR.writeMicroseconds(BRThrust); 
  } 
  else {
    digitalWrite(13, LOW); 
    
    Serial.println("Not Connected");
    stophover();
  }
  nh.spinOnce();
  delay(1);
}
