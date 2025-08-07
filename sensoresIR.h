#pragma once
#include <Arduino.h>

// === Pinos dos sensores ===
#define SENSOR_ESQ_PIN 18
#define SENSOR_CEN_PIN 9
#define SENSOR_DIR_PIN 8
#define LED_SENSORES_PIN 10

#define ledAmarelo 37
#define ledVerde 36
#define ledVermelho 38

#define botao1 47

// === Estados internos ===
int stateEsq = 0;
int stateCen = 0;
int stateDir = 0;

// === Calibração dinâmica ===
int valSensorBrancoEsq = 0;
int valSensorBrancoCen = 0;
int valSensorBrancoDir = 0;
int valSensorPretoEsq = 0;
int valSensorPretoCen = 0;
int valSensorPretoDir = 0;
int offsetEsq = 100;
int offsetCen = 100;
int offsetDir = 100;
int limiarSensor = 0;
const int valSensorMargem = 100;

// === Leitura bruta com cancelamento de luz ambiente ===
int readRawSensor(int sensorPin, int ledPin) {
    const int numLeituras = 5;
    int somaLeituras = 0;

    for (int i = 0; i < numLeituras; i++) {
        digitalWrite(ledPin, HIGH);
        delayMicroseconds(200);
        int leituraIR = analogRead(sensorPin);

        digitalWrite(ledPin, LOW);
        delayMicroseconds(200);
        int leituraAmb = analogRead(sensorPin);

        somaLeituras += (leituraAmb - leituraIR); // Ambiente - IR
        delay(1);
    }

    return somaLeituras / numLeituras;
}
// === Inicialização dos sensores e calibração ===
void inicializarSensores() {
    pinMode(SENSOR_ESQ_PIN, INPUT);
    pinMode(SENSOR_CEN_PIN, INPUT);
    pinMode(SENSOR_DIR_PIN, INPUT);
    pinMode(LED_SENSORES_PIN, OUTPUT);
	pinMode(botao1, INPUT_PULLUP);
    digitalWrite(LED_SENSORES_PIN, LOW);

    delay(300); // Tempo para estabilizar
	
	
	bool leituraBotao = digitalRead(botao1);
	while(leituraBotao == HIGH){
		digitalWrite(ledVerde, HIGH);
		delay(100);
		digitalWrite(ledVerde, LOW);
		delay(100);
		leituraBotao = digitalRead(botao1);
	}
	
	int leituraEsq = readRawSensor(SENSOR_ESQ_PIN, LED_SENSORES_PIN);
  int leituraCen = readRawSensor(SENSOR_CEN_PIN, LED_SENSORES_PIN);
	int leituraDir = readRawSensor(SENSOR_DIR_PIN, LED_SENSORES_PIN);
	
	//valor de leitura dos sensores para o branco
	valSensorBrancoCen = leituraCen;
	
	//valor de leitura dos sensores para o preto
	valSensorPretoEsq = leituraEsq - offsetEsq;
	
	delay(1000);
	
	leituraBotao = digitalRead(botao1);
	while(leituraBotao == HIGH){
		digitalWrite(ledAmarelo, HIGH);
		delay(100);
		digitalWrite(ledAmarelo, LOW);
		delay(100);
		leituraBotao = digitalRead(botao1);
	}
	
	leituraEsq = readRawSensor(SENSOR_ESQ_PIN, LED_SENSORES_PIN);
  leituraDir = readRawSensor(SENSOR_DIR_PIN, LED_SENSORES_PIN);
  leituraCen = readRawSensor(SENSOR_CEN_PIN, LED_SENSORES_PIN);
	
	// Valor de leitura dos sensores para o branco
    valSensorBrancoEsq = leituraEsq;
    valSensorBrancoDir = leituraDir;
	
	//valor de leitura dos sensores para o preto
    valSensorPretoCen = leituraCen - offsetCen;
	
	delay(1000);
	
	leituraBotao = digitalRead(botao1);
	while(leituraBotao == HIGH){
		digitalWrite(ledVermelho, HIGH);
		delay(100);
		digitalWrite(ledVermelho, LOW);
		delay(100);
		leituraBotao = digitalRead(botao1);
	}
	

  leituraEsq = readRawSensor(SENSOR_ESQ_PIN, LED_SENSORES_PIN);
  leituraCen = readRawSensor(SENSOR_CEN_PIN, LED_SENSORES_PIN);
  leituraDir = readRawSensor(SENSOR_DIR_PIN, LED_SENSORES_PIN);
    
	
	//Valor de leitura dos sensores para o preto
    valSensorPretoDir = leituraDir - offsetDir;	
	delay(1000);

  offsetEsq = (valSensorBrancoEsq - valSensorPretoEsq) / 3;
  offsetCen = (valSensorBrancoCen - valSensorPretoCen) / 3;
  offsetDir = (valSensorBrancoDir - valSensorPretoDir) / 3;
    
  Serial.println("----- VALORES DOS SENSORES -----");
  
  Serial.print("valSensorPretoEsq: ");
  Serial.println(valSensorPretoEsq);
  
  Serial.print("valSensorBrancoEsq: ");
  Serial.println(valSensorBrancoEsq);
  
  Serial.print("valSensorPretoCen: ");
  Serial.println(valSensorPretoCen);
  
  Serial.print("valSensorBrancoCen: ");
  Serial.println(valSensorBrancoCen);
  
  Serial.print("valSensorPretoDir: ");
  Serial.println(valSensorPretoDir);
  
  Serial.print("valSensorBrancoDir: ");
  Serial.println(valSensorBrancoDir);

  Serial.println("----- VALORES DOS OFFSETS -----");

  Serial.print("offsetEsq: ");
  Serial.println(offsetEsq);
  
  Serial.print("offsetCen: ");
  Serial.println(offsetCen);
  
  Serial.print("offsetDir: ");
  Serial.println(offsetDir);

    // Feedback opcional
    /*Serial.println("=== Calibração dos Sensores ===");
    Serial.print("Branco (avg): "); Serial.println(valSensorBranco);
    Serial.print("Preto (cen): "); Serial.println(valSensorPreto);
    Serial.print("Limiar: "); Serial.println(limiarSensor);
    Serial.println("================================"); */
}

// === Conversão com zona morta ===
int convertToDigital(int rawValue, int* currentState, int valPreto, int valBranco, int offset) {
    if (rawValue < (valPreto + offset)) {
        *currentState = 1; // preto
    } else if (rawValue > (valBranco - offset)) {
        *currentState = 0; // branco
    }
    return *currentState; // mantém valor se na zona morta
}

// === Leituras digitais ===
int readSensorEsq() {
    int raw = readRawSensor(SENSOR_ESQ_PIN, LED_SENSORES_PIN);
    Serial.print("Sensor esquerdo: ");
    Serial.println(raw);
    return convertToDigital(raw, &stateEsq, valSensorPretoEsq, valSensorBrancoEsq, offsetEsq);
}

int readSensorCen() {
    int raw = readRawSensor(SENSOR_CEN_PIN, LED_SENSORES_PIN);
    Serial.print("Sensor centro: ");
    Serial.println(raw);
    return convertToDigital(raw, &stateCen, valSensorPretoCen, valSensorBrancoCen, offsetCen);
}

int readSensorDir() {
    int raw = readRawSensor(SENSOR_DIR_PIN, LED_SENSORES_PIN);
    Serial.print("Sensor direito: ");
    Serial.println(raw);
    return convertToDigital(raw, &stateDir, valSensorPretoDir, valSensorBrancoDir, offsetDir);
}
