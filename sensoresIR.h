#pragma once
#include <Arduino.h>

//Pinos dos sensores
#define SENSOR_ESQ_PIN 18
#define SENSOR_CEN_PIN 9
#define SENSOR_DIR_PIN 8
#define LED_SENSORES_PIN 10

//Estado atual dos sensores
int stateEsq = 0;
int stateCen = 0;
int stateDir = 0;

//Valores de calibracao dos sensores
int valSensorBrancoEsq = 0;
int valSensorBrancoCen = 0;
int valSensorBrancoDir = 0;
int valSensorPretoEsq = 0;
int valSensorPretoCen = 0;
int valSensorPretoDir = 0;
int offsetEsq = 0;
int offsetCen = 0;
int offsetDir = 0;

// Função para salvar calibração
bool salvaCalibracao() {
    StaticJsonDocument<256> doc;

    doc["preto_esq"]   = valSensorPretoEsq;
    doc["branco_esq"]  = valSensorBrancoEsq;
    doc["preto_cen"]   = valSensorPretoCen;
    doc["branco_cen"]  = valSensorBrancoCen;
    doc["preto_dir"]   = valSensorPretoDir;
    doc["branco_dir"]  = valSensorBrancoDir;

    File file = LittleFS.open("/calibracao.json", "w");
    if (!file) {
        Serial.println("Erro ao abrir arquivo para escrita");
        return false;
    }

    serializeJson(doc, file);
    file.close();
    Serial.println("Calibração salva com sucesso!");
    return true;
}

// Função para carregar calibração
bool carregaCalibracao() {
    if (!LittleFS.exists("/calibracao.json")) {
        Serial.println("Arquivo de calibração não encontrado.");
        return false;
    }

    File file = LittleFS.open("/calibracao.json", "r");
    if (!file) {
        Serial.println("Erro ao abrir arquivo para leitura");
        return false;
    }

    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, file);
    if (error) {
        Serial.println("Erro ao ler JSON de calibração");
        file.close();
        return false;
    }

    valSensorPretoEsq  = doc["preto_esq"];
    valSensorBrancoEsq = doc["branco_esq"];
    valSensorPretoCen  = doc["preto_cen"];
    valSensorBrancoCen = doc["branco_cen"];
    valSensorPretoDir  = doc["preto_dir"];
    valSensorBrancoDir = doc["branco_dir"];

    file.close();
    Serial.println("Calibração carregada com sucesso!");
    return true;
}


// === Leitura bruta com cancelamento de luz ambiente ===
int readRawSensor(int sensorPin, int ledPin) {
    const int numLeituras = 5;
    int somaLeituras = 0;

    for (int i = 0; i < numLeituras; i++) {
        digitalWrite(ledPin, HIGH);
        delay(1);
        int leituraIR = analogRead(sensorPin);

        digitalWrite(ledPin, LOW);
        delay(1);
        int leituraAmb = analogRead(sensorPin);

        somaLeituras += (leituraAmb - leituraIR); // Ambiente - IR
        delay(1);
    }

    return somaLeituras / numLeituras;
}


//Faz a calibração dos sensores//
void calibraSensores(){
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
	valSensorPretoEsq = leituraEsq;
	
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
    valSensorPretoCen = leituraCen;
	
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
  valSensorPretoDir = leituraDir;	

}

// === Inicialização dos sensores e calibração ==
void inicializarSensores() {
  pinMode(LED_SENSORES_PIN, OUTPUT);
	pinMode(botao1, INPUT_PULLUP);
  pinMode(botao2, INPUT_PULLUP);
  digitalWrite(LED_SENSORES_PIN, LOW);
  delay(300); // Tempo para estabilizar

  if (!LittleFS.begin(true)) {
        Serial.println("Falha ao montar LittleFS");
        return;
    }
	
  if(digitalRead(botao2) == LOW || !carregaCalibracao()){
    calibraSensores();
    salvaCalibracao();
  }

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
