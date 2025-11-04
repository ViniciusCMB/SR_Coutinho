// Inclusão de bibliotecas
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include "LittleFS.h"
#include <LoRa.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

/* MÓDULO DE AVIÔNICA - START */

// Definições de pinos e constantes
#define INTERVAL 1000

#define LORA_FREQ 868E6  // Frequência de operação
#define SS_LORA 7
#define RST_LORA 1
#define DIO0_LORA 2
#define SYNC_WORD 0xF3  // Código de sincronização

#define BUZZER_PIN 0  // Pino do buzzer

#define RX_GPS 20  // RX do GPS
#define TX_GPS 21  // TX do GPS

// Instanciação de objetos
Adafruit_BMP280 BMP;   // Objeto do BMP280
Adafruit_MPU6050 MPU;  // Objeto do MPU6050
Adafruit_AHTX0 AHT;    // Objeto do AHT20
TinyGPSPlus GPS;       // Objeto do GPS

// Variáveis globais
int packet_count = 0;  // Contador de pacotes
unsigned long previous_millis = 0;
float base_pressure = 0;                // Pressão na base
String file_name = "Dados.csv";         // Nome do arquivo para salvar os dados
String file_dir = "";                   // Diretório do arquivo
const String TEAM_ID = "#261";          // ID da equipe
sensors_event_t acc, gyr, temp;         // Variáveis para armazenar os dados do MPU6050
sensors_event_t humidity, temperature;  // Variáveis para armazenar os dados do AHT20

// Setup da memória LittleFS
bool setupLittleFS() {
  if (!LittleFS.begin(true)) {
    Serial.println("Erro ao montar LittleFS.");
    return false;
  }
  return true;  // Retorna true se tudo ocorreu bem
}

// Setup do módulo BMP280
bool setupBMP() {
  if (!BMP.begin()) {
    Serial.println("Falha no BMP280.");
    return false;
  }
  base_pressure = BMP.readPressure() / 100;
  return true;  // Retorna true se tudo ocorreu bem
}

bool setupAHT() {
  if (!AHT.begin()) {
    Serial.println("Falha no AHT20.");
    return false;
  }
  return true;  // Retorna true se tudo ocorreu bem
}

// Setup do módulo LoRa
bool setupLoRa()
{
  LoRa.setPins(SS_LORA, RST_LORA, DIO0_LORA);
  if (!LoRa.begin(LORA_FREQ))
  {
    Serial.println("Falha ao inicializar o LoRa.");
    return false;
  }
  LoRa.setSyncWord(SYNC_WORD);
  return true; // Retorna true se tudo ocorreu bem
}

// Setup do MPU6050
bool setupMPU() {
  if (!MPU.begin()) {
    Serial.println("Falha no MPU6050.");
    return false;
  }
  return true;  // Retorna true se tudo ocorreu bem
}

// Sinalização com o buzzer
void buzzSignal(String signal) {
  int frequency = 1000;    // Frequência do tom
  if (signal == "Alerta")  // Alerta de erro em alguma configuração
  {
    for (int i = 0; i < 5; i++) {
      tone(BUZZER_PIN, frequency, 200);
      delay(200 + 150);
    }
  } else if (signal == "Sucesso")  // Sinal de sucesso na configuração
  {
    for (int i = 0; i < 3; i++) {
      tone(BUZZER_PIN, frequency, 100);
      delay(100 + 100);
    }
  } else if (signal == "Ativado") {
    tone(BUZZER_PIN, frequency, 500);
  } else if (signal == "Beep")  // Beep de funcionamento padrão
  {
    tone(BUZZER_PIN, frequency, 50);
    delay(100);
  } else {
    Serial.println("Sinal inválido!");
  }
}

// Registra e imprime os dados do momento
// Formato: TEAM_ID,millis,count,altp,temp,umi,p,gp,gr,gy,ap,ar,ay,hora,data,alt,lat,lon,sat,pqd
void logData(unsigned long current_millis) {
  String readings = getDataString();                                                                                   // Obtém os dados do GPS, BMP280 e MPU6050
  String data_string = TEAM_ID + "," + String(current_millis) + "," + String(packet_count) + "," + readings + ",nan";  // String com os dados atuais
  printBoth(data_string);
  appendFile(file_dir, data_string);
  packet_count++;
}

// Escreve os dados no arquivo - escrita
bool writeFile(const String &path, const String &data_string) {
  File file = LittleFS.open(path, FILE_WRITE);
  if (!file)  // Se houver falha ao abrir o arquivo
  {
    Serial.println("Falha ao abrir arquivo para gravação.");
    return false;
  }
  if (file.println(data_string))  // Se a escrita no arquivo for bem-sucedida
  {
    Serial.println("Arquivo escrito.");
  } else  // Se houver falha na escrita
  {
    Serial.println("Falha na gravação do arquivo.");
    file.close();
    return false;
  }
  file.close();
  return true;  // Retorna true se tudo ocorreu bem
}

// Escreve os dados no arquivo - anexação
void appendFile(const String &path, const String &message) {
  File file = LittleFS.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Falha ao abrir arquivo para anexar.");
    return;
  }

  if (!file.println(message)) {
    Serial.println("Falha ao anexar mensagem.");
  } else {
    Serial.println("Mensagem anexada.");
  }
  file.close();
}

// Imprime a mensagem no Serial e no LoRa
void printBoth(const String &message) {
  Serial.println(message);
  sendLoRa(message);
}

// Processa o envio de mensagens LoRa
void sendLoRa(const String &message)
{
  LoRa.beginPacket();
  LoRa.print(message);
  if (LoRa.endPacket())
  {
    Serial.println("Mensagem LoRa enviada.");
    buzzSignal("Beep");
  }
  else
  {
    Serial.println("ERRO ao enviar mensagem LoRa!");
  }
}

// Retorna os dados de hora, data, altitude, latitude, longitude, satélites
String GPSData() {
  while (Serial1.available() > 0) {
    GPS.encode(Serial1.read());
  }

  // ----- Hora -----
  String time_data = "nan";
  if (GPS.time.isValid()) {
    int h = GPS.time.hour();
    int m = GPS.time.minute();
    int s = GPS.time.second();
    char buf[16];
    sprintf(buf, "%02d:%02d:%02d", h, m, s);
    time_data = String(buf);
  }

  // ----- Data -----
  String date_data = "nan";
  if (GPS.date.isValid()) {
    int y = GPS.date.year();
    int mo = GPS.date.month();
    int d = GPS.date.day();
    char buf[16];
    sprintf(buf, "%04d/%02d/%02d", y, mo, d);
    date_data = String(buf);
  }

  // ----- Localização -----
  String alt = "nan";
  String lat = "nan";
  String lon = "nan";
  String sats = "0";

  if (GPS.location.isValid()) {
    alt = String(GPS.altitude.meters(), 2);
    lat = String(GPS.location.lat(), 8);
    lon = String(GPS.location.lng(), 8);
    sats = String(GPS.satellites.value());
  }

  String result = time_data + "," + date_data + "," + alt + "," + lat + "," + lon + "," + sats;
  return result;
}

// Retorna os dados de altitude, temperatura, nan, pressão
String BMP_AHTData() {
  AHT.getEvent(&humidity, &temperature);
  return String(BMP.readAltitude(base_pressure)) + "," + String(BMP.readTemperature()) + "," + String(humidity.relative_humidity) + "," + String(BMP.readPressure() / 100.0F);
}

// Retorna os dados de giroscópio, acelerômetro
String MPUData() {
  MPU.getEvent(&acc, &gyr, &temp);  // Lê os dados do MPU6050

  float ap = acc.acceleration.x;  // Aceleração em x - pitch
  float ar = acc.acceleration.y;  // Aceleração em y - roll
  float ay = acc.acceleration.z;  // Aceleração em z - yaw

  float gp = gyr.gyro.x;  // Giroscópio em x - pitch
  float gr = gyr.gyro.y;  // Giroscópio em y - roll
  float gy = gyr.gyro.z;  // Giroscópio em z - yaw

  return String(gp) + "," + String(gr) + "," + String(gy) + "," + String(ap) + "," + String(ar) + "," + String(ay);
}

// Retorna a string com os dados do BMP280, MPU6050 e GPS
String getDataString() {
  return BMP_AHTData() + "," + MPUData() + "," + GPSData();
}
/* MÓDULO DE AVIÔNICA - END */

/* MÓDULO DE SERVIDOR - START */

// Credenciais de acesso à rede.
String ssid_str = "Servidor " + TEAM_ID;
const char *ssid = ssid_str.c_str();
const char *password = "Iamarobot";

// Instancia um servidor http que escutará na porta 80.
AsyncWebServer server(80);

void setServerRoutes() {
  // Rotas básicas de acesso ao site para o usuário
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/server/index.html", "text/html");
  });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/server/style.css", "text/css");
  });

  server.on("/index.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/server/index.js", "application/javascript");
  });

  // Rotas API que permitem que o front-end tenha acesso aos dados do back-end.
  // Não é exatamente RESTful por limitações de capacidades da lib, de recursos
  // do ESP32 e de minha própria habilidade lol.

  // Essa rota captura todos os arquivos dispostos na raiz do Sistema de
  // Arquivos e retorna alguns de seus dados. É usado na homepage para popular a
  // tabela.
  server.on("/api/files", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument fsFiles;            // Cria um objeto JSON.
    File root = LittleFS.open("/");  // Abre o Sistema de Arquivos na Raiz.

    // Para todos os arquivos nesse diretório:
    File file = root.openNextFile();
    while (file) {
      if (!file.isDirectory()) {
        // Adicione seu nome e tamanho no objeto JSON.
        JsonObject fsFile = fsFiles.add<JsonObject>();
        fsFile["name"] = String(file.name());
        fsFile["size"] = file.size();
      }
      file = root.openNextFile();
    }
    // Por fim, transforme o objeto JSON em sua representação texto puro e o
    // envie para o cliente.
    char jsonString[8000] = { 0 };
    serializeJson(fsFiles, jsonString);
    request->send(200, "application/json; charset=utf-8", jsonString);
  });

  // Essa rota captura um arquivo específico na raiz do Sistema de Arquivos e o
  // retorna para o cliente. Consumida pelo front-end quando os links de abrir e
  // baixar arquivos são selecionados.
  server.on("/api/file", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Só retormamos com sucesso se o cliente requisitou um arquivo.
    if (!(request->hasParam("filename"))) {
      request->send(400, "text/plain; charset=utf-8", "Parâmetro de URL \
          <filename> faltando.");
      return;
    }

    // Só retormamos com sucesso se arquivo requisitado existe no Sistema de
    // Arquivos.
    const AsyncWebParameter *param = request->getParam("filename");
    String filename = param->value();
    if (!LittleFS.exists("/" + filename)) {
      request->send(404, "text/plain; charset=utf-8", "Arquivo <" + filename + "> não encontrado no Sistema de Arquivos");
      return;
    }

    // Retornamos o arquivo para download ou visualização simples dependendo se
    // o cliente enviou também o parâmetro de URL <download>. (Deve ter uma
    // solução mais elegante pra isso).
    if (request->hasParam("download")) {
      request->send(LittleFS, "/" + filename, String(), true);
    } else {
      request->send(LittleFS, "/" + filename, "text/plain; charset=utf-8");
    }
  });

  // Muito semelhante a rota anterior, só que agora que verbo HTTP mudou de GET
  // para DELETE nós deletamos o arquivo no Sistema de Arquivos ao invés de
  // retorná-lo pro cliente. Consumida pelo front-end quando o cliente seleciona
  // o botão de deletar arquivos.
  server.on("/api/file", HTTP_DELETE, [](AsyncWebServerRequest *request) {
    Serial.println(request->method());
    if (!(request->hasParam("filename"))) {
      request->send(400, "text/plain; charset=utf-8", "Parâmetro de URL \
          <filename> faltando.");
      return;
    }

    const AsyncWebParameter *param = request->getParam("filename");
    String filename = param->value();

    if (!LittleFS.exists("/" + filename)) {
      request->send(404, "text/plain; charset=utf-8", "Erro ao tentar deletar \
          arquivo <" + filename + "> inexistente.");
      return;
    }

    if (LittleFS.remove("/" + filename)) {
      request->send(200, "text/plain; charset=utf-8", "Arquivo deletado com \
          sucesso.");
    } else {
      // A essa altura nada deve dar de errado, mas por via das dúvidas...
      request->send(400, "text/plain; charset=utf-8", "Erro desconhecido ao \
          tentar deletar arquivo <" + filename + ">");
    }
  });
}

/* MÓDULO DE SERVIDOR - END */

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(BUZZER_PIN, OUTPUT);
  for (int i = 0; i < 5; i++) {
    Serial.println("Inicializando...");
    delay(1000);
  }

  Serial1.begin(9600, SERIAL_8N1, RX_GPS, TX_GPS);  // Inicia o GPS (precisa ser o Serial1)
  unsigned long start = millis();
  while (millis() - start < 3000)  // Espera 3 segundos para o GPS inicializar
  {
    while (Serial1.available() > 0) {
      GPS.encode(Serial1.read());
    }
  }

  String time_data = "";  // String do horário
  if (GPS.time.isValid()) {
    time_data = String(GPS.time.hour()) + "_" + String(GPS.time.minute()) + "_" + String(GPS.time.second());
  } else {
    // Caso o GPS ainda não tenha hora válida, usa millis() como fallback
    time_data = "init_" + String(millis());
  }

  file_dir = "/" + time_data + "-" + file_name;  // Diretório do arquivo de dados
  Serial.print("Salvando dados em: ");
  Serial.println(file_dir);

  String data_header = "TEAM_ID,millis,count,altp,temp,umi,p,gp,gr,gy,ap,ar,ay,hora,data,alt,lat,lon,sat,pqd";  // Cabeçalho do arquivo
  if (!(setupLittleFS() && writeFile(file_dir, data_header)))                                                   // Inicia a memória interna
  {
    Serial.println("Erro no sistema de arquivos!");
    buzzSignal("Alerta");
    delay(3000);
    ESP.restart();
  }

  /* Server Block - START */

  // Cria ponto de acesso wireless.
  WiFi.softAP(ssid, password);
  Serial.println("Criando ponto de acesso WiFi...");
  Serial.println(WiFi.softAPIP());

  // Configura as rotas do servidor e o inicializa.
  setServerRoutes();
  server.begin();

  /* Server Block - END */

  if (!(setupAHT() && setupBMP() && setupMPU() && setupLoRa()))  // Inicia os módulos AHT, BMP, MPU6050 e LoRa
  {
    printBoth("Erro na configuração dos módulos!");
    buzzSignal("Alerta");
    delay(3000);
  } else {
    printBoth("Todos os módulos iniciados com sucesso!");
    buzzSignal("Sucesso");
  }
}

void loop() {
  // while (Serial1.available() > 0) {
  //   GPS.encode(Serial1.read());
  //   delay(10);
  // }

  unsigned long current_millis = millis();
  if (current_millis - previous_millis >= INTERVAL)  // A cada 200ms
  {
    float altitude = BMP.readAltitude(base_pressure);
    logData(current_millis);
    previous_millis = current_millis;
  }
  delay(50);
}
