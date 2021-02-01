//70 pulsos = 45cm
//angulo negativo - direita
//angulo positivo - esquerda

/*
 *  VARIÁVEIS PARA MAPEAMENTO FLOOD FILL
 */
//direcoes
int oeste = 4;
int leste = 6;
int sul = 2; 
int norte = 8;
//parede
int pd = -1;
//caminho livre
int cl = 0;
//labirinto
int mapa[7][7] = {{pd,pd,pd,pd,pd,pd,pd},
                  {pd,cl,cl,cl,pd,pd,pd},
                  {pd,cl,pd,cl,pd,pd,pd},
                  {pd,cl,pd,cl,pd,pd,pd},
                  {pd,cl,pd,cl,pd,pd,pd},
                  {pd,cl,pd,cl,pd,pd,pd},
                  {pd,pd,pd,pd,pd,pd,pd},};


/*          
 *  VARIÁVEIS PARA O MOTOR
 */
//velocidade motor1
int           rpm;
//veloidade motor2
int           rpm2;
//leitura de pulsos do primeiro encoder
volatile long pulsos;
//leitura de pulsos do segundo encoder
volatile long pulsos2;
unsigned long timeold;
unsigned long timeold2;
//somador de pulsos par cm
int pulsosParaCm;
//definicao da quantidade de pulsos para dar uma volta
unsigned int pulsos_por_volta = 20;
float rad;
double propA;
double propB;
// Entrada dos motores
#define pinMot1A 6
#define pinMot2A 7
#define pinMot1B 8
#define pinMot2B 9

//Definindo LEDS

#define led1 4
#define led2 5

/*
 * Variaveis temporarias 
 */

int validador;

/*
 *  VARIÁVEIS PARA O SENSOR INFRAVERMELHO
 */
//Pinos do sensor infravermelho 
#define pinoSharp1 A0          
#define pinoSharp2 A1
#define pinoSharp3 A2
double valorVoltsSharp1;
double valorVoltsSharp2;
double valorVoltsSharp3;
double distanciaSharp1;
double distanciaSharp2;
double distanciaSharp3;
double distancia1;
double distancia2;
double distancia3;
double potencia1, potencia2;
double A = 118.67443195;
double t = 0.53449301;
double y0 = 8.76683547;
double exp1;

//VARIAVÉL PARA LOOP OPCIONAL
bool loop1 = true;

/*
 *  VARIAVEIS PARA O PID
 */
//variaveis do PID
float    diferenca,                            
           kp = 2.0,                              
           ki = 0.0,                             
           kd = 0.0,                              
           proporcional,                          
           integral,                              
           derivativo,                            
           PID,                                  
           ideal_value = 0,                  
           ultimaMedida;

float    diferencaN,                            
           kpN = 0.8,                              
           kiN = 0.0,                             
           kdN = 0.2,                              
           proporcionalN,                          
           integralN,                              
           derivativoN,                            
           PIDN,                                  
           ideal_valueN = 15,                    
           ultimaMedidaN;

float    diferencaL,                            
           kpL = 0.5,                              
           kiL = 0.0,                             
           kdL = 0.0,                              
           proporcionalL,                          
           integralL,                              
           derivativoL,                            
           PIDL,                                  
           ideal_valueL = 0,                    
           ultimaMedidaL;

float direita, esquerda;

//CLASSE MICROMOUSE
class Micromouse{
  public:
  //O - 4 /L - 6/ S - 2/ N - 8 
  int direcao;
  int posL;
  int posC;
  int distanciaPercorrida;

  void setPosL(int posL){
    this->posL = posL;
  }
  int getPosL(){
    return this->posL;
  }
  void setPosC(){
    this->posC = posC;
  }
  int getPosC(){
    return this->posC;
  }
  
};


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

int motor2 = 0;

float kpAngulo = 2.3;
float kiAnguloDireita = 0.05;
float kiAnguloEsquerda = 0.05;

float erroDireita;
float erroEsquerda;

float propAnguloDireita;
float propAnguloEsquerda;
float integralAnguloDireita = 0;
float integralAnguloEsquerda = 0;
float PIDdireita;
float PIDesquerda;


void contador()
{
  pulsos = pulsos + 1;

}
void contador2()
{
  pulsos2 = pulsos2 + 1;
}

unsigned long controleTempo;

uint32_t print_timer;


int ladoObstaculo;

#define velocidade 100

  Micromouse micromouse;


void atualizarVariaveis(){

  /*
   *  AQUI DENTRO TODAS AS VARIÁVEIS QUE FUNCIONAM DENTRO DE ALGUM LOOP SÃO ATUALIZADAS
   *  LEITURA DO GIROSCÓPIO
   *  LEITURA SENSOR INFRAVERMELHO
   *  PID
   *  ESSA FUNÇÃO DEVE ESTAR DENTRO DE TODAS FUNÇÕES QUE TRABALHAM COM MOVIMENTO OU LEITURA,
   *  CASO VOCE ESTIVER TRABALHANDO NO VOID LOOP ESSA FUNÇÃO DEVE SER COLOCADA!
   */
  //Atualizando o giroscpio
  mpu6050.update();


  // GUARDA NA MEMÓRIA OS VALORES ENVIADOS PELO GIROSCOPIO
  anguloX = mpu6050.getAngleX();
  anguloY = mpu6050.getAngleY();
  anguloZ = mpu6050.getAngleZ();
  
  //Atualizando variaveis do PID
  diferenca = anguloZ - ideal_value;

  proporcional = diferenca * kp;

  integral += diferenca * ki;

  derivativo = (ultimaMedida - anguloZ) * kd;

  ultimaMedida = anguloZ;

  PID = proporcional + integral + derivativo;


  erroDireita = (anguloZ - direita);
  erroEsquerda = (anguloZ - esquerda);

  propAnguloDireita = erroDireita * kpAngulo;
  propAnguloEsquerda = erroEsquerda * kpAngulo;
  integralAnguloDireita += erroDireita * kiAnguloDireita;
  integralAnguloEsquerda += erroEsquerda * kiAnguloEsquerda;
  
  PIDdireita = propAnguloDireita + integralAnguloDireita;
  PIDesquerda = propAnguloEsquerda + integralAnguloEsquerda;


  diferencaN = distancia2 - ideal_valueN;

  proporcionalN = diferencaN * kpN;

  integralN += diferencaN * kiN;

  derivativoN = (ultimaMedidaN - anguloZ) * kdN;

  ultimaMedidaN = anguloZ;

  PIDN = proporcionalN + integralN + derivativoN;

  

  diferencaL = (distancia3 - distancia1) - ideal_valueL;

  proporcionalL = diferencaL * kpL;

  integralL += diferencaL * kiL;

  derivativoL = (ultimaMedidaL - anguloZ) * kdL;

  ultimaMedidaL = anguloZ;

  PIDL = proporcionalL + integralL + derivativoL;
  


}

void manterDistanciaObstaculoPID(){
  atualizarVariaveis();
  leituraSensorInfravermelho();
  Serial.println(PIDN);
  if(PIDN > 0){
  PIDN = map(PIDN,  0, 10, 0, 70);            //normaliza para PWM de 8 bits
      if(PIDN > 70) PIDN = 70;
//Andando reto*/
  if(PID < -5)
    {
      PID = map(PID, 0, -30, 25, 100);
      if(PID > 100) PID = 100;
      motor2 = map(PID, 25, 100, 0, 60);
      if (motor2 > 60) motor2 = 60;
      analogWrite(pinMot2A, 0);
      analogWrite(pinMot2B, 0);
      //esquerda
      analogWrite(pinMot1A, motor2);
      //direita
      analogWrite(pinMot1B, PID);
    }

  else if(PID > 5)
  {
    PID = map(PID, 0, 30, 25, 100);
    if(PID > 100) PID = 100;
    motor2 = map(PID, 25, 100, 0, 60);
    if (motor2 > 60) motor2 = 60;
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0);  
    //esquerda   
    analogWrite(pinMot1A, PID); 
    //direita                
    analogWrite(pinMot1B, motor2);
  }
  else
  {
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0); 
    //esquerda    
    analogWrite(pinMot1A, PIDN);
    //direita                 
    analogWrite(pinMot1B, PIDN);
  }
//Fim andando reto
}
else if(PIDN < 0){
  Serial.print("PIDN: ");
  Serial.println(PIDN);
  atualizarVariaveis();
    //Andando reto
  if(PID < -10)
    {
      PID = map(PID, 0, -30, 0, 70);
      if(PID > 70) PID = 70;
      motor2 = map(PID, 0, 70, 0, 40);
      if (motor2 > 40) motor2 = 40;
      analogWrite(pinMot2A, PID);
      analogWrite(pinMot2B, motor2);
      //esquerda
      analogWrite(pinMot1A, 0);
      //direita
      analogWrite(pinMot1B, 0);
    }

  else if(PID > 10)
  {
    PID = map(PID, 0, 30, 0, 70);
    if(PID > 70) PID = 70;
    motor2 = map(PID, 0, 70, 0, 40);
    if (motor2 > 40) motor2 = 40;
    analogWrite(pinMot2A, motor2);
    analogWrite(pinMot2B, PID);  
    //esquerda   
    analogWrite(pinMot1A, 0); 
    //direita                
    analogWrite(pinMot1B, 0);
  }
  else
  {
    PIDN = map(PIDN,  0, -5, 30, 100);            //normaliza para PWM de 8 bits
      if(PIDN > 70) PIDN = 70;
    analogWrite(pinMot2A, PIDN);
    analogWrite(pinMot2B, PIDN); 
    //esquerda    
    analogWrite(pinMot1A, 0);
    //direita                 
    analogWrite(pinMot1B, 0);
  }
}
}

void zerarPulsos(){
  /*
   *  ESSA FUNCAO ZERA OS PULSOS DO ENCODER
   */
    pulsos = 0;
    pulsos2 = 0;
}

void desligarMotores(){
  /*
   *  ESSA FUNCAO DESLIGA TODOS OS MOTORES, COLOQUE ELA FORA DE ALGUM LOOP QUE ENVOLVA MOTORES,
   *  ASSIM QUE ACABAR A MOVIMENTAÇÃO, OS MOTORES DEVEM SER DESLIGADOS
   */
  analogWrite(pinMot1A, 0);             
   analogWrite(pinMot1B, 0);
    analogWrite(pinMot2A, 0);             
    analogWrite(pinMot2B, 0);
}

void irParaFrente(){
  /*
   *  ESSA FUNCAO TRABALHA JUNTO COM O PID, ELA SEMPRE ANDA PARA FRENTE. A FRENTE NÃO É FIXA, ELE 
   *  SE MODIFICA SEMPRE QUE O ROBÔ VIRA PARA DIREITA OU PARA ESQUERDA!!! CASO O ROBÔ VIRE PARA DIREITA
   *  SEM SER DE PROPÓSITO A ROTA É CORRIGIDA!
   */
  atualizarVariaveis();
  Serial.println(diferenca);
  atualizarVariaveis();
  //Andando reto
  if(PID < -5)
    {
      PID = map(PID, 0, -30, 25, 100);
      if(PID > 100) PID = 100;
      motor2 = map(PID, 25, 100, 0, 60);
      if (motor2 > 60) motor2 = 60;
      analogWrite(pinMot2A, 0);
      analogWrite(pinMot2B, 0);
      //esquerda
      analogWrite(pinMot1A, motor2);
      //direita
      analogWrite(pinMot1B, PID);
    }

  else if(PID > 5)
  {
    PID = map(PID, 0, 30, 25, 100);
    if(PID > 100) PID = 100;
    motor2 = map(PID, 25, 100, 0, 60);
    if (motor2 > 60) motor2 = 60;
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0);  
    //esquerda   
    analogWrite(pinMot1A, PID); 
    //direita                
    analogWrite(pinMot1B, motor2);
  }
  else
  {
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0); 
    //esquerda    
    analogWrite(pinMot1A, 100);
    //direita                 
    analogWrite(pinMot1B, 100);
  } 
}

void irParaFrenteDistancia(int pulsosParaCmRecebido){
  /*
   *  ESSA FUNCAO TRABALHA JUNTO COM O PID, ELA SEMPRE ANDA PARA FRENTE. A FRENTE NÃO É FIXA, ELE 
   *  SE MODIFICA SEMPRE QUE O ROBÔ VIRA PARA DIREITA OU PARA ESQUERDA!!! CASO O ROBÔ VIRE PARA DIREITA
   *  SEM SER DE PROPÓSITO A ROTA É CORRIGIDA!
   *  NESSA FUNÇÃO É ESCOLHIDO QUANTOS PULSOS O ROBÔ DEVE SE MOVIMENTAR
   */
  zerarPulsos();
  while((pulsos2)/1 < pulsosParaCmRecebido){
    atualizarVariaveis();
  Serial.println(diferenca);
  atualizarVariaveis();
  //Andando reto
  if(PID < -5)
    {
      PID = map(PID, 0, -30, 25, 100);
      if(PID > 100) PID = 100;
      motor2 = map(PID, 25, 100, 0, 60);
      if (motor2 > 60) motor2 = 60;
      analogWrite(pinMot2A, 0);
      analogWrite(pinMot2B, 0);
      //esquerda
      analogWrite(pinMot1A, motor2);
      //direita
      analogWrite(pinMot1B, PID);
    }

  else if(PID > 5)
  {
    PID = map(PID, 0, 30, 25, 100);
    if(PID > 100) PID = 100;
    motor2 = map(PID, 25, 100, 0, 60);
    if (motor2 > 60) motor2 = 60;
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0);  
    //esquerda   
    analogWrite(pinMot1A, PID); 
    //direita                
    analogWrite(pinMot1B, motor2);
  }
  else
  {
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0); 
    //esquerda    
    analogWrite(pinMot1A, 100);
    //direita                 
    analogWrite(pinMot1B, 100);
  }
  }
  desligarMotores();  
}

void irParaFrentePIDLateral(){
  /*
   *  ESSA FUNCAO TRABALHA JUNTO COM O PID, ELA SEMPRE ANDA PARA FRENTE. A FRENTE NÃO É FIXA, ELE 
   *  SE MODIFICA SEMPRE QUE O ROBÔ VIRA PARA DIREITA OU PARA ESQUERDA!!! CASO O ROBÔ VIRE PARA DIREITA
   *  SEM SER DE PROPÓSITO A ROTA É CORRIGIDA!
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  Serial.println(diferenca);
  atualizarVariaveis();
  //Andando reto
  if(PIDL < -5)
    {
      PIDL = map(PIDL, 0, -30, 25, 100);
      if(PIDL > 100) PIDL = 100;
      motor2 = map(PIDL, 25, 100, 0, 60);
      if (motor2 > 60) motor2 = 60;
      analogWrite(pinMot2A, 0);
      analogWrite(pinMot2B, 0);
      //esquerda
      analogWrite(pinMot1A, motor2);
      //direita
      analogWrite(pinMot1B, PIDL);
    }

  else if(PIDL > 5)
  {
    PIDL = map(PIDL, 0, 30, 25, 100);
    if(PID > 100) PIDL = 100;
    motor2 = map(PIDL, 25, 100, 0, 60);
    if (motor2 > 60) motor2 = 60;
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0);  
    //esquerda   
    analogWrite(pinMot1A, PIDL); 
    //direita                
    analogWrite(pinMot1B, motor2);
  }
  else
  {
    analogWrite(pinMot2A, 0);
    analogWrite(pinMot2B, 0); 
    //esquerda    
    analogWrite(pinMot1A, 100);
    //direita                 
    analogWrite(pinMot1B, 100);
  } 
}

void irParaTras(){
  /*
   *  ESSA FUNCAO TRABALHA JUNTO COM O PID, ELA SEMPRE ANDA PARA TRÁS. A "TRÁS" NÃO É FIXA, ELE 
   *  SE MODIFICA SEMPRE QUE O ROBÔ VIRA PARA DIREITA OU PARA ESQUERDA!!! CASO O ROBÔ VIRE PARA DIREITA
   *  SEM SER DE PROPÓSITO A ROTA É CORRIGIDA!
   */
  atualizarVariaveis();
    //Andando reto
  if(PID < -10)
    {
      PID = map(PID, 0, -30, 60, 80);
      if(PID > 80) PID = 80;
      motor2 = map(PID, 60, 80, 0, 40);
      if (motor2 > 40) motor2 = 40;
      analogWrite(pinMot2A, PID);
      analogWrite(pinMot2B, motor2);
      //esquerda
      analogWrite(pinMot1A, 0);
      //direita
      analogWrite(pinMot1B, 0);
    }

  else if(PID > 10)
  {
    PID = map(PID, 0, 30, 60, 80);
    if(PID > 80) PID = 80;
    motor2 = map(PID, 60, 80, 0, 40);
    if (motor2 > 40) motor2 = 40;
    analogWrite(pinMot2A, motor2);
    analogWrite(pinMot2B, PID);  
    //esquerda   
    analogWrite(pinMot1A, 0); 
    //direita                
    analogWrite(pinMot1B, 0);
  }
  else
  {
    analogWrite(pinMot2A, 40);
    analogWrite(pinMot2B, 40); 
    //esquerda    
    analogWrite(pinMot1A, 0);
    //direita                 
    analogWrite(pinMot1B, 0);
  }
}

void virarParaDireita(){
  /*
   *  NESSA FUNCAO O ROBÔ VIRA 90 GRAUS PARA DIREITA!!!
   */
  direita = anguloZ - 90;
  ideal_value = direita;
  while(anguloZ > direita + 12){
    atualizarVariaveis();
    Serial.println(PIDdireita);
    PIDdireita = map(PIDdireita,0, 90, 32, 60);
    if(PIDdireita > 60) PIDdireita = 60;
    analogWrite(pinMot1A, PIDdireita);
    analogWrite(pinMot2B, 0);
  }
  analogWrite(pinMot1A, 0);             
  analogWrite(pinMot1B, 0);
  analogWrite(pinMot2A, 0);             
  analogWrite(pinMot2B, 0);
  zerarPulsos();
}

void virarParaEsquerda(){
  /*
   *  NESSA FUNÇÃO O ROBÔ VIRA 90 GRAUS PARA ESQUERDA!!!
   */
  esquerda = anguloZ + 90;
  ideal_value = esquerda;
  while(anguloZ < esquerda - 5){
    atualizarVariaveis();
    Serial.println(PIDesquerda);
    PIDesquerda = map(PIDesquerda, 90, 0, 32, 60);
    if(PIDesquerda > 60) PIDesquerda = 60;
    analogWrite(pinMot2A, PIDesquerda);
    analogWrite(pinMot1B, 0);
  }
  analogWrite(pinMot1A, 0);             
  analogWrite(pinMot1B, 0);
  analogWrite(pinMot2A, 0);             
  analogWrite(pinMot2B, 0);
  zerarPulsos();
}



void leituraSensorInfravermelho(){

  /*
   * AQUI SÃO ATUALIZADAS AS LEITURAS DOS SENSORES INFRAVERMELHOS!!! ALÉM DISSO É FEITO O CÁLCULO DE INTERPOLAÇÃO DA DISTÂNCIA
   */
  
   //Obtenção do valor em Volts através da formula ( valorAnalógico * (5/1023) )
   valorVoltsSharp1 = analogRead(pinoSharp1) * 0.0048828125;
   // Obtenção do valor em Volts através da formula ( valorAnalógico * (5/1023) )
   valorVoltsSharp2 = analogRead(pinoSharp2) * 0.0048828125; 
   // Obtenção do valor em Volts através da formula ( valorAnalógico * (5/1023) )
   valorVoltsSharp3 = analogRead(pinoSharp3) * 0.0048828125; 
   
   //media das distancias do sensor 1
   for(int i=0;i<20;i++){
    // Obtenção do valor em Volts através da formula ( valorAnalógico * (5/1023) )
    double valorVoltsSharp1 = analogRead(pinoSharp1) * 0.0048828125; 
    // Formula para cálculo da distância levando em consideração o valor em volts
    exp1 = (-1*valorVoltsSharp1)/t;
    distanciaSharp1 = y0 + A*exp(exp1);
    if(distanciaSharp1 > 80) distanciaSharp1 = 80;
    if(distanciaSharp1 < 0) distanciaSharp1 = 0;
    distancia1 = (distancia1 + distanciaSharp1);
   }
   distancia1 = distancia1/20;
   
   //media das distancias do sensor 2
   for(int i=0;i<20;i++){
    // Obtenção do valor em Volts através da formula ( valorAnalógico * (5/1023) )
    double valorVoltsSharp2 = analogRead(pinoSharp2) * 0.0048828125; 
    // Formula para cálculo da distância levando em consideração o valor em volts
    exp1 = (-1*valorVoltsSharp2)/t;
    distanciaSharp2 = y0 + A*exp(exp1);
    if(distanciaSharp2 > 80) distanciaSharp2 = 80;
    if(distanciaSharp2 < 0) distanciaSharp2 = 0;
    distancia2 = (distancia2 + distanciaSharp2);
   }
   distancia2 = distancia2/20;
   
   ///media das distancias do sensor 3
   for(int i=0;i<20;i++){
    // Obtenção do valor em Volts através da formula ( valorAnalógico * (5/1023) )
    double valorVoltsSharp3 = analogRead(pinoSharp3) * 0.0048828125; 
    // Formula para cálculo da distância levando em consideração o valor em volts
    exp1 = (-1*valorVoltsSharp3)/t;
    distanciaSharp3 = y0 + A*exp(exp1);
    if(distanciaSharp3 > 80) distanciaSharp3 = 80;
    if(distanciaSharp3 < 0) distanciaSharp3 = 0;
    distancia3 = (distancia3 + distanciaSharp3);
   }
   distancia3 = distancia3/20;

   float distancia1 = 0;
   float distancia2 = 0;
   float distancia3 = 0;
}

bool frenteCaminhoLivre(){
  /*
   *  CASO O ROBÔ ESTEJA EM UM CORREDOR (PAREDE DOS DOIS LADOS E FRENTE LIVRE) É RETORNADO O VALOR TRUE
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  if(((distancia1 - distancia3) >= -10 && (distancia1 - distancia3) <= 10) && distancia2 >= 40){
    return 1;
  }
  else{
    return 0;
  }
}

bool direitaCaminhoLivre(){
  /*
   *  CASO APENAS O LADO DIREITO DO ROBÔ POSSUA CAMINHO LIVRE
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  if((distancia1 < 23) && (distancia2 < 35 ) && (distancia3 >= 30)){
    return 1;
  }
  else{
    return 0;
  }
}

bool esquerdaCaminhoLivre(){
  atualizarVariaveis();
  leituraSensorInfravermelho();
  if((distancia1 >= 30) && (distancia2 < 35) && (distancia3 < 23)){
    return 1;
  }
  else{
    return 0;
  }
}


bool menorLado(){
  /*
   *  É VERIFICADO QUAL SENSOR DE DISTANCIA ENTRE DIREITA E ESQUERDA ESTÁ LENDO UMA DISTÂNCIA MENOR
   */
  leituraSensorInfravermelho();
  atualizarVariaveis();
  //direita maior
  if(distancia1 > distancia3){
    return 1;
  }
  //esquerda maior ou igual
  else{
    return 0;
  }
}

void corrigirCentro(){
  /*
   *  ESSA FUNÇÃO FORÇA O ROBÔ A MANTER UMA DISTÂNCIA DEFINIDA DO OBSTÁCULO
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  validador = 0;
  while(validador <= 50){
    if(distancia2 > 16){
    atualizarVariaveis();
    leituraSensorInfravermelho();
  while(distancia2 > 16){
    atualizarVariaveis();
    leituraSensorInfravermelho();
    manterDistanciaObstaculoPID();
  }
  desligarMotores();
  zerarPulsos();
  } else
  if(distancia2 < 14){
    leituraSensorInfravermelho();
    manterDistanciaObstaculoPID();
    while(distancia2 < 14){
      atualizarVariaveis();
    leituraSensorInfravermelho();
    manterDistanciaObstaculoPID();
    }
    desligarMotores();
    zerarPulsos();
  }
  validador += 1;
  }
  
}
  

void imprimirLeituraInfra(){
  /*
   *  IMPRIMI AS LEITURAS DOS 3 SENSORES INFRAVERMELHOS
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  Serial.print("d1 ");
  Serial.print(distancia1);
  Serial.print(" d2 ");
  Serial.print(distancia2);
  Serial.print(" d3 ");
  Serial.println(distancia3);
}

void imprimirLeituraAnguloZ(){
  /*
   *  IMPRIMI A LEITURA DO ANGULO DO GIROSCÓPIO
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  Serial.print("angulo Z ");
  Serial.println(anguloZ);
}

void imprimirEncoders(){
  /*
   *  IMPRIMI O VALOR DOS ENCODERS
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  Serial.print("pulso 1 ");
  Serial.print(pulsos);
  Serial.print(" pulso 2 ");
  Serial.println(pulsos2);
}

bool detectouObstaculo(){
  /*
   *  CASO TENHA ALGUM OBSTÁCULO NA FRENTE DENTRO DE UMA DISTÂNCIA FIXA É RETORNADO TRUE
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  if(distancia2 <=20){
    desligarMotores();
    return true;
  }
  else{
    return false;
  }
}

void virarParaMenorLadoObstaculo(){
  /*
   *  UTILIZA A FUNÇÃO menorLado() PARA VERIFICAR O MENOR LADO E ENTÃO VIRAR PARA ESSE MENOR LADO,
   *  O PROPÓSITO DESSA FUNÇÃO É ESCOLHER O LADO NO QUAL O OBSTÁCULO É MENOR PARA DESVIAR
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  if(menorLado() == 1){
    virarParaEsquerda();
    ladoObstaculo = 1;
  }
  else{
    virarParaDireita();
    ladoObstaculo = 2;
  }
  zerarPulsos();
}

void lendoTamanhoObstaculo(){
  /*
   *  V1.0 ESSA FUNÇÃO POR ENQUANTO APENAS ANDA ATÉ O OBSTÁCULO SUMIR
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  if(ladoObstaculo == 1){
    while(distancia3 <= 25){
      atualizarVariaveis();
      leituraSensorInfravermelho();
      irParaFrente();
      pulsosParaCm = ((pulsos + pulsos2)/2);
    }
    desligarMotores();
  }else
  if(ladoObstaculo == 2){
    while(distancia1 <= 25){
      atualizarVariaveis();
      leituraSensorInfravermelho();
      irParaFrente();
      pulsosParaCm = ((pulsos + pulsos2)/2);
    }
  }
  else{
    Serial.println("erro 'lendoTamanhoObstaculo'");
  }
}

void lendoTamanhoObstaculoPT2() {
  /*
   *  V1.0 ESSA FUNÇÃO POR ENQUANTO APENAS ANDA ATÉ O OBSTÁCULO SUMIR
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  if(ladoObstaculo == 1){
    while(distancia3 <= 25){
      atualizarVariaveis();
      leituraSensorInfravermelho();
      irParaFrente();
    }
    desligarMotores();
  }else
  if(ladoObstaculo == 2){
    while(distancia1 <= 25){
      atualizarVariaveis();
      leituraSensorInfravermelho();
      irParaFrente();
    }
  }
  else{
    Serial.println("erro 'lendoTamanhoObstaculo'");
  }
  desligarMotores();
  irParaFrenteDistancia(35);
}

void pegarDistanciaObstaculo(){
  /*
   *  ESSA FUNÇÃO É UTILIZADA PARA O ROBÔ PEGAR CERTA DISTÂNCIA DO OBSTÁCULO, OU SEJA, SE AFASTAR
   */
  atualizarVariaveis();
  leituraSensorInfravermelho();
  while(distancia2 < 17){
    atualizarVariaveis();
    leituraSensorInfravermelho();
    irParaTras();
  }
  desligarMotores();
  zerarPulsos();
}

void virandoMelhorLadoContorno(){
  /*
   *  ESSA FUNÇÃO JUNTA VÁRIAS FUNÇÕES PARA CONSEGUIR DESVIAR DO OBSTÁCULO!!!
   */
  while(loop1){
    atualizarVariaveis();
    leituraSensorInfravermelho();
    if(detectouObstaculo()){
      desligarMotores();
      pegarDistanciaObstaculo();
      virarParaMenorLadoObstaculo();
      lendoTamanhoObstaculo();
      desligarMotores();
      andarDistanciaRestanteParaVirar();
      virandoLadoContrarioObstaculo();
      irParaFrenteDistancia(35);
      lendoTamanhoObstaculoPT2();
      virandoLadoContrarioObstaculo();
      irParaFrenteDistancia(pulsosParaCm * 1.5 + 35);
      virandoLadoOriginalObstaculo();
      loop1 = false;
    }
    else{
      irParaFrente();
    }
  }
  zerarPulsos();
}

void virandoLadoContrarioObstaculo(){
  /*
   *  O ROBÔ VIRA PARA O LADO CONTRÁRIO NO QUAL ELE VIROU PARA DESVIAR DO OBSTÁCULO... POR EXEMPLO
   *  CASO O ROBÔ PERCEBA O OBSTÁCULO ELE DEVE ENTÃO DESVIAR, AO FAZER SUA LEITURA ELE PERCEBE QUE
   *  O MENOR LADO É DIREITA, LOGO ELE VIRA PARA DIREITA, MAS, PARA RETORNAR PARA A VOLTA ELE DEVE
   *  POSTERIORMENTE VIRAR PARA A ESQUERDA, JÁ QUE PARA DESVIAR ELE VIROU PARA DIREITA. POR ISSO
   *  O LADO "CONTRÁRIO"!!!
   */
  if(ladoObstaculo == 1){
    virarParaDireita();
    desligarMotores();
  }
  else{
    virarParaEsquerda();
    desligarMotores();
  }
}

void virandoLadoOriginalObstaculo(){
  if(ladoObstaculo == 1){
    virarParaEsquerda();
    desligarMotores();
  }
  else{
    virarParaDireita();
    desligarMotores();
  }
}

void andarDistanciaRestanteParaVirar(){
  /*
   *  VALOR PADRAO PARA O ROBÔ CONSEGUIR ANDAR UM POUCO PARA LONGE DO OBSTÁCULO PARA CONSEGUIR DESVIAR
   *  LEVANDO EM CONSIDERAÇÃO SEU TAMANHO
   */
  zerarPulsos();
  atualizarVariaveis();
  leituraSensorInfravermelho();
  while((pulsos + pulsos2)/2 < 45){
    imprimirEncoders();
    atualizarVariaveis();
    leituraSensorInfravermelho();
    irParaFrente();
  }
  desligarMotores();
  zerarPulsos();
}

void andarUmaCelula(){
  zerarPulsos();
  irParaFrenteDistancia(57);
}

void zerarPulsosParaCm(){
  pulsosParaCm = 0;
}

void floodFill(){
  
  atualizarVariaveis();
  leituraSensorInfravermelho();
  if(frenteCaminhoLivre()){
    digitalWrite(led1, HIGH);
    andarUmaCelula();
    digitalWrite(led1, LOW);
    atualizarVariaveis();
    leituraSensorInfravermelho();
  }else
  if(direitaCaminhoLivre()){
    digitalWrite(led1, HIGH);
    virarParaDireita();
    andarUmaCelula();
    digitalWrite(led1, LOW);
    atualizarVariaveis();
    leituraSensorInfravermelho();
  }else
  if(esquerdaCaminhoLivre()){
    digitalWrite(led1, HIGH);
    virarParaEsquerda();
    andarUmaCelula();
    digitalWrite(led1, LOW);
    andarUmaCelula();
    atualizarVariaveis();
    leituraSensorInfravermelho();
  }
  else{
    desligarMotores();
    atualizarVariaveis();
    leituraSensorInfravermelho();
  }
}

void setup() {
  
  attachInterrupt(digitalPinToInterrupt(3), contador, RISING);
  attachInterrupt(digitalPinToInterrupt(2), contador2, RISING);
  pulsos  = 0;
  rpm     = 0;
  rpm2     = 0;
  timeold = 0;
  timeold2 = 0;
  pinMode(led1, OUTPUT);
  digitalWrite(led1, LOW);
  pinMode(led2, OUTPUT);
  digitalWrite(led1, LOW);
  Serial.begin(9600);
  pinMode(pinMot1A, OUTPUT);
  pinMode(pinMot1B, OUTPUT);
  pinMode(pinMot2A, OUTPUT);
  pinMode(pinMot2B, OUTPUT);


  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false);// MUDAR PARA "true" SE QUISER VISUALIZAR INFORMAÇÕES DE CALIBRAÇÃO NO MONITOR SERIAL
  
  #ifdef DEBUG
  Serial.println("Fim Setup");
  #endif  

  loop1 = true;

 virandoMelhorLadoContorno();
 andarUmaCelula();
 digitalWrite(led2, HIGH);

}

// led1 amarelo
// led2 verde

void loop() {

  /*  
   *   NÃO TEM PROBLEMA DEIXAR O VOID LOOP VAZIO!!!
   */

  
}
