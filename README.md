# hidrologia
código para o monitoramento do nivel de agua 
#include <ModbusMaster.h>  // Biblioteca Modbus para comunicação RS485
#include <HardwareSerial.h>

#define LEVEL_SENSOR_RS485_RX_PIN 16  // Pino RX do RS485 (ESP32)
#define LEVEL_SENSOR_RS485_TX_PIN 17  // Pino TX do RS485 (ESP32)
#define ALERT_THRESHOLD 2.0          // Limite de altura de água para gerar alerta (em metros)

#define SENSOR_MIN_CURRENT 4.0       // Corrente mínima (4mA)
#define SENSOR_MAX_CURRENT 20.0      // Corrente máxima (20mA)
#define SENSOR_MIN_HEIGHT 0.0        // Altura correspondente à corrente mínima (0 metros)
#define SENSOR_MAX_HEIGHT 10.0       // Altura correspondente à corrente máxima (10 metros)

ModbusMaster node;

// Configuração da porta Serial2 para RS485 (para o ESP32)
HardwareSerial RS485Serial(2);  // Usando Serial2 para comunicação RS485

void setup() {
  Serial.begin(115200);  // Inicializa a comunicação serial para monitoramento (pino USB)
  pinMode(LEVEL_SENSOR_RS485_RX_PIN, INPUT);  // Configura pino RX do RS485 como entrada
  pinMode(LEVEL_SENSOR_RS485_TX_PIN, OUTPUT);  // Configura pino TX do RS485 como saída

  // Configura a comunicação RS485 para o sensor de nível
  RS485Serial.begin(9600, SERIAL_8N1, LEVEL_SENSOR_RS485_RX_PIN, LEVEL_SENSOR_RS485_TX_PIN);

  node.begin(1, RS485Serial);  // Endereço Modbus do sensor (1 é um exemplo)

  delay(1000); // Aguarda um momento para garantir que o sensor se inicialize
  Serial.println("Sistema de leitura de nível de água iniciado.");
}

void loop() {
  uint16_t sensorValue = readSensor();  // Lê o valor do sensor de nível
  if (sensorValue != 0xFFFF) {  // Verifica se a leitura foi bem-sucedida
    float current_mA = mapToCurrent(sensorValue); // Mapeia o valor lido para corrente (4-20mA)
    float height = mapToHeight(current_mA);  // Converte a corrente para a altura da coluna de água

    // Exibe os valores no monitor serial
    Serial.print("Corrente (mA): ");
    Serial.print(current_mA);
    Serial.print(" mA  -> Altura da coluna de água: ");
    Serial.print(height);
    Serial.println(" metros");

    // Verifica se o nível de água ultrapassou o limite de 2 metros
    if (height > ALERT_THRESHOLD) {
      Serial.println("ALERTA: A altura da água ultrapassou 2 metros!");
    }
  }

  delay(2000);  // Aguarda 2 segundos antes de ler novamente
}

// Função para ler o valor do sensor via Modbus
uint16_t readSensor() {
  uint8_t result;
  
  result = node.readInputRegisters(0x0000, 2); // Lê dois registradores Modbus do sensor
  if (result == node.ku8MBSuccess) {
    uint16_t rawValue = node.getResponseBuffer(0);  // Obtém o valor bruto do sensor
    return rawValue;
  } else {
    Serial.println("Erro na leitura do sensor de nível!");
    return 0xFFFF;  // Retorna erro se falhar
  }
}

// Função para mapear o valor lido para corrente (4-20mA)
float mapToCurrent(uint16_t rawValue) {
  // Mapeia o valor do sensor (0-65535) para a faixa de corrente 4-20mA
  float current_mA = map(rawValue, 0, 65535, SENSOR_MIN_CURRENT * 1000, SENSOR_MAX_CURRENT * 1000) / 1000.0;
  return current_mA;
}

// Função para mapear a corrente para a altura da coluna de água
float mapToHeight(float current_mA) {
  // Mapeia a corrente (4-20mA) para a altura (0-10 metros)
  float height = (current_mA - SENSOR_MIN_CURRENT) * (SENSOR_MAX_HEIGHT - SENSOR_MIN_HEIGHT) / (SENSOR_MAX_CURRENT - SENSOR_MIN_CURRENT);
  return height;
}
