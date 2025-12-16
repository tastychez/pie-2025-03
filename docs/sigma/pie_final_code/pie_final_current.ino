/*
Plugboard Code - Single Keypress, simple version
Prints the pressed key once and waits for release before detecting next key

PINS 20 and 21 CANNOT be used as input pins??
*/

char alphabet[] = {'Q', 'A', 'P', 'W', 'S', 'Y', 'E', 'D', 'X', 'R', 'F', 'C', 'T', 'G', 'V', 'Z', 'H', 'B', 'U', 'J', 'N', 'I', 'K', 'M', 'O', 'L'};
bool keyPressed = false; // tracks if a key is being held
int corresponding_pin;
int current_ignored[] = {-1,-1};
int previous_ignored[] = {-1,-1}; 


void setup() {
  Serial.begin(9600);

  //Setting all pins to input from 28 to 53 (these are keypresses) we skip 20 and 21 because those pins are unusable
  for (int i = 28; i <= 53; i++) {
    pinMode(i, INPUT_PULLUP);
  }
  for (int i = 2; i <= 19; i++) {
    pinMode(i, INPUT);
  }
  for (int i = 21; i <= 27; i++) {
    pinMode(i, INPUT);
  }

  //Special Pins (replacement for 20 and 21) Setting these to input.
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);

  Serial.println("-----"); //Bar for being able to read new line. (Mostly for debugging)
}

void loop() {
  
  //Loop through pins 28 and 53 to see 
  for (int i = 28; i <= 53; i++) {
    if (digitalRead(i) == LOW) {
       Serial.print("Pin ");
       Serial.println(i);
      // Serial.println(" pressed");
      // Serial.println(alphabet[i-28]);

      delay(10);
      pinMode(i-26,OUTPUT);
      delay(10);
      digitalWrite(i-26,HIGH);
      Serial.print("Pin ");
       Serial.print(i-26);
       Serial.println(" is high");

      corresponding_pin = i-26;
      delay(10);

      
      current_ignored[0] = i; 

      //If the corresponding pin is 20 or 21, set A0 or A1 high 
      if(corresponding_pin ==20)
      {
        pinMode(A0,OUTPUT);
        digitalWrite(A0,HIGH);
        Serial.println("A0 is high actually");
      }
      if(corresponding_pin ==21)
      {
        pinMode(A1,OUTPUT);
        digitalWrite(A1,HIGH);
        Serial.println("A1 is high actually");
      }
      break;//Assign ignored pairing to avoid multiple keypresses
    }
  }

  //NO 20 or 21
  for (int i = 2; i<=19; i++)
  {
    if(digitalRead(i) ==HIGH && i!=corresponding_pin)
    {
      current_ignored[1] = i;
      Serial.println(alphabet[i-2]);
      Serial.println(digitalRead(i));
      Serial.println(" swapped");
    }
  }

  if(digitalRead(A0) == HIGH && 20!=corresponding_pin)
  {
     current_ignored[1] = 20;
      Serial.println(alphabet[20-2]);
      Serial.println(digitalRead(A0));
      Serial.println(" swapped");
    
  }
  if(digitalRead(A1) == HIGH && 21!=corresponding_pin)
  {
     current_ignored[1] = 21;
      Serial.println(alphabet[21-2]);
      Serial.println(digitalRead(A1));
      Serial.println(" swapped");
    
  }

  for (int i = 22; i<=27; i++)
  {
    if(digitalRead(i) ==HIGH && i!=corresponding_pin)
    {
       Serial.print(i);
       Serial.println(" - Swapped Pin");

      current_ignored[1] = i;
      
      //Serial.println(previous_ignored[0]);

      // Serial.println("Current");
      // Serial.print(current_ignored[0]);
      // Serial.print(" ");
      // Serial.println(current_ignored[1]);
  
      // Serial.println("Previous");
      // Serial.print(previous_ignored[0]);
      // Serial.print(" ");
      // Serial.println(previous_ignored[1]);

     Serial.println(alphabet[i-2]);
      Serial.println(digitalRead(i));
      // if(previous_ignored[0] != current_ignored[0] && previous_ignored[1] != current_ignored[1])
      // {
        
      // }  

    }
  }
  delay(20);

  pinMode(corresponding_pin,INPUT);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);

  // Serial.println("Previous");
  //previous_ignored[0] = current_ignored[0];
  //previous_ignored[1] = current_ignored[1];
  // Serial.println("Current");
  //current_ignored[0] = -1;
  //current_ignored[1] = -1;

}


