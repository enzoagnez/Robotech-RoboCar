#include "defines.h"
#include "motor_pid.h"  //Motor A é Direita e Motor B Esquerda
#include "sensoresIR.h" // Supondo que o nome do seu arquivo .h seja esse

//readSensorEsq() / readSensorDir() / readSensorCen() == lê a linha (0 branco, 1 preto)
//setVelocidade("DIREÇÃO", VELOCIDADE) == define a velocidade

void setup() {
  Serial.begin(115200);
  pinMode(ledVermelho, OUTPUT);
  pinMode(ledAmarelo, OUTPUT);
  pinMode(ledVerde, OUTPUT);
  delay(100);
  inicializarSensores();
  delay(1000);
  inicializarMotores(); 
}

void loop() {
   //Lê os três sensores
   int esq = readSensorEsq();
   int cen = readSensorCen();
   int dir = readSensorDir();

    digitalWrite(ledVermelho, dir);
    digitalWrite(ledAmarelo, cen);
    digitalWrite(ledVerde, esq);

  // // // Aplica lógica de decisão nos motores
   if (esq == 1 && dir == 0) {
     setVelocidade("D", 10);
     setVelocidade("E", -10);
   } if (esq == 0 && dir == 1) {
     setVelocidade("D", -10);
     setVelocidade("E", 10);
   } if (esq == 0 && dir == 0) {
    setVelocidade("D", 15);
    setVelocidade("E", 15);

    } //else //{
  // //   // Situação fora do previsto (ex: os dois sensores fora da linha)
  //    setVelocidade("D", -40);
  //    setVelocidade("E", -40);
  //  }  
  // // Monitor Serial
  Serial.print("Esq: "); Serial.print(esq);
  Serial.print(" | Cen: "); Serial.print(cen);
  Serial.print(" | Dir: "); Serial.println(dir);

}