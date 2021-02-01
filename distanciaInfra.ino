//Pinos do sensor infravermelho        
#define pinoSharp2 A0

double potencia1, potencia2;
double A = 118.67443195;
double t = 0.53449301;
double y0 = 8.76683547;
double exp1;

double distancia1;
double distanciaSharp1;

double valorVoltsSharp1;

void setup() {
  Serial.begin(9600);             // Inicialização da comunicação serial ( Caso queira ver a distância em cm )
}
void loop() {
   /*// Obtenção do valor em Volts através da formula ( valorAnalógico * (5/1023) )
   valorVoltsSharp1 = analogRead(pinoSharp2) * 0.0048828125;*/
   distancia1 = 0;
   //media das distancias do sensor 1
   for(int i=0;i<=20;i++){
    // Obtenção do valor em Volts através da formula ( valorAnalógico * (5/1023) )
    double valorVoltsSharp1 = analogRead(pinoSharp2) * 0.0048828125; 
    // Formula para cálculo da distância levando em consideração o valor em volts
    exp1 = (-1*valorVoltsSharp1)/t;
    distanciaSharp1 = y0 + A*exp(exp1);
    if(distanciaSharp1 > 80) distanciaSharp1 = 80;
    if(distanciaSharp1 < 0) distanciaSharp1 = 0;
    distancia1 = (distancia1 + distanciaSharp1);
   }
   distancia1 = distancia1/21;
    


   Serial.print(" distancia: ");
   Serial.print(distancia1);
   Serial.println(" cm");
   //delay(1000);

   
}
