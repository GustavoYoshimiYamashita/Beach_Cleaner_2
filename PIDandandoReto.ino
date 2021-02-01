//incluindo as bibliotecas necessarias
#include <MPU6050_tockn.h>
#include <Wire.h>

// DEFINIÇÕES
#define MPU6050_ADDR         0x68 // ENDEREÇO QUANDO O PINO AD0 ESTIVER LIGADO AO GND

#define DEBUG
// INSTANCIANDO OBJETOS
MPU6050 mpu6050(Wire);

// DECLARAÇÃO DE VARIÁVEIS  
float anguloX;
float anguloY;
float anguloZ;


// Entrada dos motores
#define pinMot1A 9
#define pinMot2A 10
#define pinMot1B 5
#define pinMot2B 6
double propA;
double propB;

float    diferenca,                            
           kp = 1.0,                              
           ki = 0.001, //0.0005                            
           kd = 16.0, // 10.0                              
           proporcional,                          
           integral,                              
           derivativo,                            
           PID,                                  
           ideal_value = 0,                    
           ultimaMedida;

void setup() {
  Serial.begin(9600);
  pinMode(pinMot1A, OUTPUT);
  pinMode(pinMot1B, OUTPUT);
  pinMode(pinMot2A, OUTPUT);
  pinMode(pinMot2B, OUTPUT);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false);// MUDAR PARA "true" SE QUISER VISUALIZAR INFORMAÇÕES DE CALIBRAÇÃO NO MONITOR SERIAL
  
}

void loop() {

   
  mpu6050.update();


  // GUARDA NA MEMÓRIA OS VALORES ENVIADOS PELO GIROSCOPIO
  anguloZ = mpu6050.getAngleZ();
  
  diferenca = anguloZ - ideal_value;

  proporcional = diferenca * kp;

  integral += diferenca * ki;
 
  derivativo = (anguloZ - ultimaMedida) * kd;

  ultimaMedida = anguloZ;

  PID = proporcional + integral + derivativo;

  Serial.println(anguloZ);
  int motor2 = 0;


  if(PID < -1)
  {
      PID = map(PID, 0, -30, 25, 255);
      if(PID > 255) PID = 255;
      //motor2 = map(PID, 25, 255, 0, 100);
      //if (motor2 > 0) motor2 = 0;
      analogWrite(pinMot2A, 0);
      analogWrite(pinMot2B, 0);
      //esquerda
      analogWrite(pinMot1A, 0);
      //direita
      analogWrite(pinMot1B, PID);
  }

  else if(PID > 1)
  {
    PID = map(PID, 0, 30, 25, 255);
    if(PID > 255) PID = 255;
    //motor2 = map(PID, 25, 255, 0, 100);
    //if (motor2 > 0) motor2 = 0;
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0);  
    //esquerda   
    analogWrite(pinMot1A, PID); 
    //direita                
    analogWrite(pinMot1B, 0);
  }
  else
  {
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0); 
    //esquerda    
    analogWrite(pinMot1A, 0);
    //direita                 
    analogWrite(pinMot1B, 0);
  }
  

}
