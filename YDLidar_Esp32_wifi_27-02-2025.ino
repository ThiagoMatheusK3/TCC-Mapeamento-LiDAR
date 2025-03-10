/*
 * YDLIDAR SYSTEM - Integração com ESP32
 * 
 * Adaptação do código original da EAI TEAM (2015 - 2018) para o ESP32
 * Copyright EAI TEAM - http://www.eaibot.com
 * 
 * Adaptação realizada por [THIAGO M. S. DE SOUSA] em [27/02/2025].
 * Descrição: Implementação de suporte para o ESP32, Coleta de dados IMU, Tratamento dos dados WiFI
 * sicronização de dados LiDAR & do sensor MPU6050
 * Este código utiliza a biblioteca YDLIDAR-SDK:
 * https://github.com/YDLIDAR/YDLidar-SDK
 */


#include <YDLidar.h>         // Biblioteca de gerenciamento do LiDAR
#include <QueueList.h>       // Gerenciamento de listas de pontos
#include <HardwareSerial.h>  // Comunicação serial para ESP32
#include "esp32-hal-ledc.h"  // Controle PWM no ESP32
#include <WiFi.h>            // Biblioteca de comunicação WiFi
#include <Wire.h>
#include <MPU6050.h>  // Biblioteca MPU6050 (acelerômetro & giroscópio)

// Configurações do hardware
#define SIZE_BUFFER 2048
#define YDLIDAR_MOTOR_SCTP 2  // Pino para controle PWM do motor
#define YDLIDAR_MOTOR_EN     // Pino para habilitação do motor
#define PWM_FREQ 10000         // Frequência PWM: 10 kHz
#define PWM_RESOLUCAO 8        // Resolução PWM: 8 bits

// Definições do LiDAR
HardwareSerial LidarSerial(2);  // Porta serial ESP32 para o LiDAR
YDLidar lidar;                  // Objeto LiDAR
bool isScanning = false;        // Estado do escaneamento
QueueList<scanPoint> scans;     // Buffer para pontos de escaneamento




//ESp32 -> arduino

HardwareSerial EspSerial(1);
#define ESP_TX 5
#define ESP_RX 4

//TESTE

struct SyncData {
  scanPoint lidarPoint;  // Ponto do LiDAR
  float gyroAngle;       // Ângulo acumulado do giroscópiow

};

QueueList<SyncData> syncScans;  // Lista sincronizada para armazenar dados combinados
#define SIZE_SYNC_BUFFER 2048     // Defina um tamanho adequado

//TESTE


// Configurações WiFi
const char* ssid = "xxxxxx";
const char* password = "xxxxx";
WiFiServer server(80);
WiFiClient client;  // cliente wi-fi;

// Configurações MPU 6050
//Giroscopio
MPU6050 mpu;
float velocidadeAngularZ = 0;
float anguloAcumulado = 0;
float drift = 0;
unsigned long tempoGyro = 0;
float sensibilidade = 65.5;


void setup() {                       // Start
  Serial.begin(115200);              // Comunicação USB serial
  lidar.begin(LidarSerial, 128000);  // (baudrate, SERIAL_8N1, 16, 17) RX/TX // *Editado na biblioteca Ydlidar1.2

  EspSerial.begin(9600, SERIAL_8N1, 4, 5);

  pinMode(YDLIDAR_MOTOR_SCTP, OUTPUT);
  //pinMode(YDLIDAR_MOTOR_EN, OUTPUT);

  ledcAttach(YDLIDAR_MOTOR_SCTP, PWM_FREQ, PWM_RESOLUCAO);  // Configuração do PWM

  setMotorSpeed(167); // 255 ~ 100% duty cycle -> Velocidade minima // 167 =~ 8hz

  //Conexão WiFi
  WiFi.begin(ssid, password);
  delay(5000);
  server.begin();
  ConexaoWiFI();


  //MPU
  Wire.begin();
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);  // Configura para ±500°/s
  drift = calibracaoMPU();
  tempoGyro = millis();
  Serial.println(drift);
}

void loop() { //  AJUSTAR
  verificarCliente();
  LidarData();
  coletaLidar();
}

void LidarData() {
  if (lidar.waitScanDot() == RESULT_OK) {
    coletaMPU();
    isScanning = true;
    scanPoint _point = lidar.getCurrentScanPoint();

    if (_point.distance <= 1000) {
      SyncData syncData;
      syncData.lidarPoint = _point;          // Salva o ponto do LiDAR
      syncData.gyroAngle = anguloAcumulado;  // Salva o ângulo atual do giroscópio

      if (syncScans.count() < SIZE_SYNC_BUFFER) {
        syncScans.push(syncData);  // Armazena os dados sincronizados
      } else {
        syncScans.pop();           // Remove o mais antigo
        syncScans.push(syncData);  // Insere o novo
      }
    }
  }
}

void coletaLidar() {
  if (isScanning) {
    if (syncScans.count() > 0) {
      SyncData data = syncScans.pop();

      String msg =  String(data.lidarPoint.angle, 5) + "," + 
                    String(data.lidarPoint.distance, 5) + "," +
                    String(data.gyroAngle, 5)  + "\n";

      if (client) {
        client.print(msg);
      }
    }
  }
}

float calibracaoMPU() {
  float soma = 0;
  const int amostras = 5000;

  for (int i = 0; i < amostras; i++) {
    soma += mpu.getRotationZ() / sensibilidade;  // Coleta uma amostra
    delay(1);
  }
  return soma / amostras;  // Retorna o valor médio (drift)

}

void coletaMPU() {
  unsigned long tempoAtual = millis();
  float deltaTempo = (tempoAtual - tempoGyro) / 1000.0;  // Intervalo em segundos


  velocidadeAngularZ = mpu.getRotationZ() / sensibilidade - drift;
  anguloAcumulado += velocidadeAngularZ * deltaTempo;

  if (anguloAcumulado >= 360) anguloAcumulado -= 360;
  if (anguloAcumulado <= 0) anguloAcumulado += 360;

  tempoGyro = tempoAtual;
}

// Configura a velocidade do motor via PWM
void setMotorSpeed(uint8_t PWM) {
  ledcWrite(YDLIDAR_MOTOR_SCTP, PWM);
}

void ConexaoWiFI() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(WiFi.localIP());
  } else {
  }
}

void verificarCliente() {
  //conexão do cliente
  if (!client || !client.connected()) {
    client = server.accept();  // Aceita novos clientes
    if (client) {
      Serial.println("Cliente conectado.");
    }
  } else {
    //verifica se há dados recebidos
    if (client.available()) {
      String comando = client.readStringUntil('\n');  // Lê a mensagem do Python
      comando.trim();  // Remove espaços extras

      EspSerial.println(comando);
      //Serial.println(comando);
    }
  }
}