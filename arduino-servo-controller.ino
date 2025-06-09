#include <stdio.h>
#include <math.h>
#include <ESP32Servo.h>

#define PI 3.14159265358979323846
#define servo0Pin 27
#define servo1Pin 13
#define servo2Pin 12
#define servo3Pin 14
#define servo4Pin 26

typedef struct {
    double angle0;
    double angle1;
    double angle2;
    double angle3;
} JointAngles;

double convertToDegrees(double angle) {
    return angle * 180.0 / PI;
}

JointAngles inverseKinematics(double x, double y, double z) {
    JointAngles result;
    double angleJoint0 = atan2(y, x);
    double l = sqrt(x * x + y * y);
    double link1Length = 8.975;
    double link2Length = 9.025;
    double link3Length = 14.5;
    double theNewTargetBaseDistance = l - link3Length;
    double max_reach = link1Length + link2Length;
    double angleForHeight = atan2(z, theNewTargetBaseDistance);
    double distanceToTarget = sqrt(theNewTargetBaseDistance * theNewTargetBaseDistance + z * z);
    
    if (distanceToTarget > max_reach) {
        Serial.print("Warning: Target point (L=");
        Serial.print(l, 1);
        Serial.print(", Z=");
        Serial.print(z, 1);
        Serial.println(") is unreachable!");
        result.angle0 = 90.0;
        result.angle1 = 90.0;
        result.angle2 = 180.0;
        result.angle3 = 180.0;
        return result;
    }
    
    double cos_arg = (link1Length * link1Length + distanceToTarget * distanceToTarget - 
                     link2Length * link2Length) / (2 * link1Length * distanceToTarget);
    
    if (cos_arg > 1.0) cos_arg = 1.0;
    if (cos_arg < -1.0) cos_arg = -1.0;
    
    double halfAngleOfJoint1 = acos(cos_arg);
    double angleJoint1 = halfAngleOfJoint1 + angleForHeight;
    
    double cos_arg2 = (link1Length * link1Length + link2Length * link2Length - 
                      distanceToTarget * distanceToTarget) / (2 * link1Length * link2Length);
    
    if (cos_arg2 > 1.0) cos_arg2 = 1.0;
    if (cos_arg2 < -1.0) cos_arg2 = -1.0;
    
    double angleJoint2 = PI - acos(cos_arg2);
    double angleJoint3 = angleJoint2 - angleJoint1;
    
    result.angle0 = (int)constrain(convertToDegrees(angleJoint0) + 90, 0, 180);
    result.angle1 = (int)constrain(convertToDegrees(angleJoint1), 0, 180);
    result.angle2 = (int)constrain(convertToDegrees(angleJoint2) + 90, 0, 180);
    result.angle3 = (int)constrain(convertToDegrees(angleJoint3) + 90, 0, 180);

        
    Serial.print("Applied angles: ");
    Serial.print(result.angle0);
    Serial.print("°, ");
    Serial.print(result.angle1);
    Serial.print("°, ");
    Serial.print(result.angle2);
    Serial.print("°, ");
    Serial.print(result.angle3);
    Serial.println("°");
    
    return result;
}

void moveProgressively(Servo& servo, int targPos){
    int currPos = servo.read();

    while(currPos < targPos){
        currPos += 2;
        servo.write(currPos);
        delay(25);
    }
    while(currPos > targPos){
        currPos -= 2;
        servo.write(currPos);
        delay(25);
    }
}

Servo servo0, servo1, servo2, servo3, servo4;
JointAngles result;


void targetPosPick(double x, double y, double z){
    JointAngles result = inverseKinematics(x, y, z);
    Serial.println("Heading to target");

    servo4.write(110);
    delay(500);
    moveProgressively(servo0, result.angle0);
    delay(500);
    moveProgressively(servo2, result.angle2);
    delay(500);
    moveProgressively(servo1, result.angle1);
    delay(500);
    moveProgressively(servo3, result.angle3);
    delay(1000);
    servo4.write(60);
}

void targetPosPlace(double x, double y, double z){
    JointAngles result = inverseKinematics(x, y, z);
    Serial.println("Heading to target");

    delay(500);
    moveProgressively(servo0, result.angle0);
    delay(500);
    moveProgressively(servo2, result.angle2);
    delay(500);
    moveProgressively(servo1, result.angle1);
    delay(500);
    moveProgressively(servo3, result.angle3);
    delay(1000);
    servo4.write(110);
}

void initPos(){
    Serial.println("Resetting all servos");
    moveProgressively(servo3, 180);
    delay(500);
    moveProgressively(servo2, 180);
    delay(500);
    moveProgressively(servo1, 100);
    delay(500);    
    moveProgressively(servo0, 90);
    delay(500);
}
//int lastAngle0 = 90;
//int lastAngle1 = 100;
//int lastAngle2 = 180;
//int lastAngle3 = 180;

void setup(){
    Serial.begin(115200);

    servo0.attach(servo0Pin);
    servo1.attach(servo1Pin);
    servo2.attach(servo2Pin);
    servo3.attach(servo3Pin);
    servo4.attach(servo4Pin);

    Serial.println("Initializing position");

    //moveProgressively(servo3, servo3.read(), 180);
    //delay(1000);
    //moveProgressively(servo2, servo2.read(), 180);
    //delay(1000);
    //moveProgressively(servo1, servo1.read(), 90);
    //delay(1000);    
    //moveProgressively(servo0, servo0.read(), 90);

    servo3.write(180);
    delay(500);
    servo2.write(180);
    delay(500);
    servo1.write(100);
    delay(500);
    servo0.write(90);
    servo4.write(110);
    
    Serial.println("Enter x and y and z values (format: x,y,z): ");
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        // Parse the comma-separated values
        int firstComma = input.indexOf(',');
        int secondComma = input.indexOf(',', firstComma + 1);
        
        if (firstComma > 0 && secondComma > firstComma) {
            double x = input.substring(0, firstComma).toDouble();
            double y = input.substring(firstComma + 1, secondComma).toDouble();
            double z = input.substring(secondComma + 1).toDouble();
            
            Serial.print("Received values - x: ");
            Serial.print(x);
            Serial.print(", y: ");
            Serial.print(y);
            Serial.print(", z: ");
            Serial.println(z);
            
            // Reset Command
            if (x == 0 && y == 0 && z == 0) {
                Serial.println("Resetting all servos");
                moveProgressively(servo3, 180);
                delay(500);
                moveProgressively(servo2, 180);
                delay(500);
                moveProgressively(servo1, 100);
                delay(500);    
                moveProgressively(servo0, 90);
                delay(500);
                
            } else {
                targetPosPick(x, y, z);
                delay(500);
                initPos();
                delay(500);
                targetPosPlace(25, 15, -1);
                delay(500);
                initPos();
            }
        } else {
            Serial.println("Invalid format. Use x,y,z (e.g. 10,20,13)");
        }
    }
}

/*void loop(){
    if (Serial.available() > 0) {
        String inputStr = Serial.readStringUntil('\n');
        inputStr.trim();

        int commaIndex1 = inputStr.indexOf(',');
        int commaIndex2 = inputStr.indexOf(',', commaIndex1 + 1);
        
        if (commaIndex1 > 0 && commaIndex2 > commaIndex1) {
          double x = inputStr.substring(0, commaIndex1).toDouble();
          double y = inputStr.substring(commaIndex1 + 1, commaIndex2).toDouble();
          double z = inputStr.substring(commaIndex2 + 1).toDouble();

          Serial.print("Received values - x: ");
          Serial.print(x);
          Serial.print(", y: ");
          Serial.print(y);
          Serial.print(", z: ");
          Serial.println(z);

          result = inverseKinematics(x, y, z);

          moveProgressively(servo0, servo0.read(), result.angle0);
          delay(1000);
          moveProgressively(servo2, servo2.read(), result.angle2);
          delay(1000);
          moveProgressively(servo1, servo1.read(), result.angle1);
          delay(1000);
          moveProgressively(servo3, servo3.read(), result.angle3);
        }
        else {
            Serial.print("Wrong Input");
        }
    }

    else {
        servo0.write(servo0.read());
        servo1.write(servo1.read());
        servo2.write(servo2.read());
        servo3.write(servo3.read());
    }
    delay(100);
}*/
