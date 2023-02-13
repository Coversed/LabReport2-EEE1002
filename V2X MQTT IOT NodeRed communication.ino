//****************************************//
//* ESP32 MQTT EEEBot Template           *//
//* Modified from Rui Santos'            *//
//* https://randomnerdtutorials.com      *//
//*                                      *//
//* ESP32 Code                           *//
//*                                      *//
//* UoN 2022 - ND                        *//
//****************************************//
const int trigPin = "REPLACE_WITH_TRIGGER_PIN"; //CHANGE ME
const int echoPin = "REPLACE_WITH_ECHO_PIN"; //CHANGE ME
int distance = 0;
#include <WiFi.h> // WIFI Library
#include <PubSubClient.h> //PubSubClient Library
#include <Wire.h> // Wire Library
#include "mpu6500.h" //MPU 6500 library
bfs::Mpu6500 imu; // MPU 6500 Object
float temperature,xacc,yacc,zacc; //Variables temperature and acceleration in each axis
const int ledPin = "REPLACE_WITH_LED_PIN"; //CHANGE ME
#define I2C_SLAVE_ADDR 0x04 // Arduino slave address 04
const char* ssid = "REPLACE_WITH_YOUR_SSID";  //CHANGE ME
const char* password = "REPLACE_WITH_YOUR_PASSWORD";  //CHANGE ME                  
const char* mqtt_server = "YOUR_MQTT_BROKER_IP_ADDRESS";  //CHANGE ME
WiFiClient espClient; // Define WiFi and ESP clients
PubSubClient client(espClient); //Allow for MQTT messaging
long lastMsg = 0; //Holds last message
char msg[50]; //Holds current message to a maximum of 50 letters/numbers 
int leftMotor_speed, rightMotor_speed, servoAngle; //Variables left and right motor speed aswell as servoAngle


void setup() {
  while(!Serial) {}
  Wire.begin();
  Wire.setClock(400000); //Set clock to 400000Hz
  imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM); // Connect wire to MPU6500
  Serial.begin(9600); // 9600 baud, can be any
  setup_wifi();
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(ledPin, OUTPUT);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Transmit_to_arduino(0, 0, 90);

}

void Transmit_to_arduino(int leftMotor_speed, int rightMotor_speed, int servoAngle)
{
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(leftMotor_speed & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));   // first byte of y, containing bits 16 to 9
  Wire.write((byte)(rightMotor_speed & 0x000000FF));          // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(servoAngle & 0x000000FF));
  Wire.endTransmission();   // stop transmitting
}


void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to "); // Prints to monitor to indicate the function is running
  Serial.println(ssid); //Returns ssid to user on the monitor to ensure the right one was entered

  WiFi.begin(ssid, password); // Attempts to connect to the host using the set password

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("."); // Prints to monitor to indicate not connected yet
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP()); // Displays the local IP in this case the Broker IP for validation
}


void callback(char* topic, byte* message, unsigned int length) { // Takes in topic, the message byte by byte and the length of the message
  Serial.print("\nMessage arrived on topic: ");
  Serial.print(topic); // Validates input and displays the respective topic
  Serial.print(". Message: ");
  String messageTemp;
  int message1;
 
  for (int i = 0; i < length; i++) // for each byte in the message until the end
  {
    Serial.print((char)message[i]); // Print the message
    messageTemp += (char)message[i]; //Temporarily holds the whole message
  }
  Serial.println();

  // Add your subscribed topics here i.e. statements to control GPIOs with MQTT
  // --
  if (String(topic) == "esp32/Goutput") //If the topic is Goutput, "George Output"
  {
    Serial.print("changing output to ");
    if(messageTemp == "on"){ // If the message is on
      Serial.println("on");
      digitalWrite(ledPin, HIGH); //Turn the ledPin on
    }
    else if (messageTemp == "off"){ // If the message is off
      Serial.println("off");
      digitalWrite(ledPin, LOW); // Turn the ledPin off
    }
  }
  if (String(topic) == "esp32/GServo"){ //If the topic is Gservo, "George Servo"
    Serial.print("\nchanging angle to ");
    Serial.print(servoAngle);
    servoAngle = messageTemp.toInt(); //Convert the string message into an integer 
    Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle); //Transmit the change in servoAngle to the arduino keeping left and right motor speed the same

  }
  if (String(topic) == "esp32/Gmotor"){ //If the topic is Gmotor, "George Motor"
    Serial.print("/nchanging speed to ");
    Serial.print(rightMotor_speed);
    int newspeed = messageTemp.toInt(); //Converts the message into an integer
    leftMotor_speed = newspeed; //Sets the left motor speed to the message
    rightMotor_speed = newspeed; //Sets the right motor speed to the message
    Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle); //Transmit the change in motor speed to the arduino keeping servo angle the same
  }
}

 

  // --


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("\nAttempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Subscribe to MQTT Output Topics
      client.subscribe("esp32/Goutput");
      client.subscribe("esp32/GServo");
      client.subscribe("esp32/Gmotor");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;

    Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
    if (imu.Read()) {
      xacc = imu.accel_x_mps2();
      yacc = imu.accel_y_mps2();
      zacc = imu.accel_z_mps2();
      temperature = imu.die_temp_c();
      client.publish("esp32/Gtemperature", temperature);
      client.publish("esp32/Gxacc", xacc);
      client.publish("esp32/Gyacc", yacc);
      client.publish("esp32/Gzacc", zacc);
      //Serial.print(xacc);
      //Serial.print("\t");
      //Serial.print(yacc);
      //Serial.print("\t");
      //Serial.print(zacc);
      //Serial.print("\t");
      //Serial.print(temperature);
      //Serial.print("\n");
  }
  }
}