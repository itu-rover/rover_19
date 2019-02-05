int sensor1 = A0;
int sensorValue1 = 0;

int sensor2 = A1;
int sensorValue2 = 0;

int sensor3 = A2;
int sensorValue3 = 0;

int sensor4 = A3;
int sensorValue4 = 0;

int sensor5 = A4;
int sensorValue5 = 0;


void setup() {
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
    
  Serial.begin(115200);
}

void loop() {
  // read the value from the sensor:
  String finalString = "";
  
  sensorValue1 = analogRead(sensor1);
  sensorValue2 = analogRead(sensor2);
  sensorValue3 = analogRead(sensor3);
  sensorValue4 = analogRead(sensor4);
  sensorValue5 = analogRead(sensor5);

  sensorValue1 = map(sensorValue1,0,1008,90,-90);
  sensorValue2 = map(sensorValue2,460,675,84,15);
  sensorValue3 = map(sensorValue3,710,920,104,55);
  sensorValue4 = map(sensorValue4,1008,20,-90,180)+15;
  sensorValue5 = map(sensorValue5,870,550,90,0);
  
  
  /*Serial.print(processValue(sensorValue1));
  Serial.print(" ");
  Serial.print(processValue(sensorValue2));
  Serial.print(" ");
  Serial.print(processValue(sensorValue3));
  Serial.print(" ");
  Serial.print(processValue(sensorValue4));
  Serial.print(" ");
  Serial.println(processValue(sensorValue5));*/

  finalString = "S"+processValue(sensorValue1)+processValue(sensorValue2)+processValue(sensorValue3)+processValue(sensorValue4)+processValue(sensorValue5)+"0000F\r\n";
  //finalString = "S"+processValue(sensorValue1)+processValue(sensorValue2)+"  " + String(sensorValue3)+"  " +processValue(sensorValue4)+processValue(sensorValue5)+"0000F\r\n";
  //Serial.print(finalString);
  Serial.println(finalString);
  delay(40);
}
String processValue(int value){
  String valueString = "0000";
  if (value < 0)
  {
    if (value > -10)                      // -9 , -1
    {
      valueString = "000"+String(abs(value));
    }
    else if((value <= -10) && (value > -100))    // -99,-10
    {
      valueString = "00" + String(abs(value));
    }
    else                                        // -999,-11
    {
      valueString = "0" + String(abs(value));
    }
  }
  else if (value > 0)
  {
    if(value < 10)                            //1, 9
    {
      valueString = "100" + String(value);
    }
    else if((value >= 10) && (value < 100))   //10, 99
    {
      valueString = "10" + String(value);
    }
    else
    {
      valueString = "1" + String(value);
    }
  }
  else
  {
    valueString = "0000";
  }
 return valueString; 
}

