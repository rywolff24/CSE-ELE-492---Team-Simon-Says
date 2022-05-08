#include <Servo.h>
String  inByte;
int angles[7];
int counter = 0;
Servo leftElbow; 
Servo leftShoulder;
Servo leftShoulderRotate;
Servo rightElbow; 
Servo rightShoulder;
Servo rightShoulderRotate;
Servo head;
void setup (void)
{
  Serial.begin (15200);
  leftElbow.attach(2);
  leftShoulder.attach(9);
  leftShoulderRotate.attach(6);
  rightShoulderRotate.attach(3);
  rightElbow.attach(4);
  rightShoulder.attach(5);


  head.attach(13);
  
 
}

void loop () {
  if(Serial.available())  // if data available in serial port
    { 
    delay(50);
    inByte = Serial.readStringUntil('\n');
    Serial.println(inByte);
     // Split the string into substrings
    while (inByte.length() > 0)
    {
      int index = inByte.indexOf(',');
      if (index == -1) // No comma found
      {
        angles[counter++] = inByte.toInt();
        break;
      }
      else
      {
        angles[counter++] = inByte.substring(0, index).toInt();
        inByte = inByte.substring(index+1);
      }
    }
    
    leftElbow.write(angles[0]);     // move servo
    leftShoulder.write(angles[1]);     // move servo
    leftShoulderRotate.write(angles[2]);     // move servo
    rightElbow.write(angles[3]);     // move servo
    rightShoulder.write(angles[4]); 
    rightShoulderRotate.write(angles[5]); // move servo
    head.write(angles[6]);
    
    counter = 0;
    }
    
}
