/*
  Analog Input
  HIL for IKarus IMU

 */

int Analog_pin_0 = A0;    
int Analog_pin_1 = A1;
int Analog_pin_2 = A2;    

int i;

int Analog_val_1,Analog_val_2;
// Construccion para conversion datos
union u_tag {
    byte b[4];
    float fval;
  } u;



void setup() {
  Serial.begin(9600); 
  
}

void loop() {
  // read the value from the sensor:
  analogRead(Analog_pin_0);
  Analog_val_1 = analogRead(Analog_pin_1); 
  Analog_val_2 = analogRead(Analog_pin_2); 

  

  
//  //Protocolo de comunicacion. Solo se pueden TRANSMITIR por el USB a Simulink BYTES (8 bits)
//  // y con un protocolo que se puede especificar. 
//    Serial.write('A'); //Inicio paquete
//    u.fval=Analog_val_1;
//    for (i=0;i<=3;i++){Serial.write(u.b[i] );}
//    u.fval=Analog_val_1;
//    for (i=0;i<=3;i++){Serial.write(u.b[i] );}  
//
//    Serial.println();  //Fin de paquete
//    
//    
    
    
   //Protocolo de comunicacion. 

    Serial.println(Analog_val_1,DEC);
    Serial.println(Analog_val_2 ,DEC);  

    //Serial.println();  //Fin de paquete
    delay(200);
}
