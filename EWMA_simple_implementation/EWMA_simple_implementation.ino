
#define ALPHA 0.28
#define LED RED_LED

int Result ;
int Result1 ;
float Current ;
float Voltage ;
unsigned int i = 0, j = 0;
float yesterdayPower[24];
float todayPower[24];
float prediction[24];


int sensorPin = A10;    // select the input pin for the potentiometer

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(LED, OUTPUT); 
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
 
}

void loop() {
  // put your main code here, to run repeatedly: 
  read_adc_and_predict();
  delay(500);
}

void read_adc_and_predict ()
{
    Result = analogRead(sensorPin);
    
    Current = ( Result /4096.)*100;
    Result1 = ADC12MEM1 ;
    
    Serial.println(Result1);
    Voltage = ( Result1 /4096.)*5;

    todayPower[i]= Voltage * Current;
      Serial.println("---------------------------");
      Serial.print("prediction, heure ");
      Serial.print(" ");
      Serial.print(i);
      Serial.print(" = ");
      Serial.print(todayPower [i]);
      Serial.println("W");

    if(i <24)
    {
        i ++;
    }
    else
    {
        for (j = 0; j < 24; j ++)
        {
            yesterdayPower[j] = todayPower[j];
        }
        i = 0;
    }

    //Dans le cas ou on veut réaliser une prédiction au jour 0 on prendra la dernière du jour précédent.
    if(i == 0){
      Serial.print("prediction, heure ");
      Serial.print(" ");
      Serial.print(i);
      Serial.print(" = ");
      Serial.print(prediction [i]);
      Serial.println("W");
      
      prediction[i] = ALPHA * yesterdayPower[i]+(1 - ALPHA )* todayPower[24]; // simpleEWMA prediction
    }else{
      Serial.print("prediction, heure ");
      Serial.print(" ");
      Serial.print(i);
      Serial.print(" = ");
      Serial.print(prediction [i]);
      Serial.println("W");
      
      prediction[i] = ALPHA * yesterdayPower[i]+(1 - ALPHA )* todayPower[i-1]; // simpleEWMA prediction    
    }
    Serial.println("---------------------------");
    Serial.println("");
    
}
