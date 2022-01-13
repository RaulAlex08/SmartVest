#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Adafruit_GPS.h>
#include <ArduinoOTA.h>
#include <Thread.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

#include "config.h" //modificar para ssid y passwd
#include "ESP32_Utils.hpp"
#include "ESP32_Utils_OTA.hpp"

WiFiClient thingsBoard;
PubSubClient client(thingsBoard);

const char* mqtt_server = "demo.thingsboard.io";
const char* TOPIC_T = "v1/devices/me/telemetry";
const char* TOPIC_A = "v1/devices/me/attributes";
const char* token_sensor = "xxxxxxxxxxxxxxxxxxxxxx";
const char* token_pulsador = "xxxxxxxxxxxxxxxxxxxx";
const char* token_tracker = "xxxxxxxxxxxxxxxxxxxxxx";
const char* token_id_user = "xxxxxxx";
#define TOKEN_BOT "xxxxxxx:xxxxxxxxxxxxxxxxxxxxxxxxxxx"

// Sensor 1
const byte trig1 = 12; //Pin GPIO12 para el Trigger (Blanco)
const byte echo1 = 14; //Pin GPIO14 para el Echo (Azul)
// Sensor 2
const byte trig2 = 27; //Pin GPIO27 para el Trigger (Blanco)
const byte echo2 = 26; //Pin GPIO26 para el Echo (Azul)
// Zumbador
const byte zumbador = 25; //Pin GPIO25 para Zumbador
// Pulsador
const byte pulsador = 33; //Pin GPIO33 para Interrup del Pulsador
// GPS
#define GPSSerial Serial2 //UART 2 GPIO16 Rx, GPIO17 Tx
Adafruit_GPS GPS(&GPSSerial); // Connect to the GPS on the hardware port
// Bot Telegram
WiFiClientSecure espClient;
UniversalTelegramBot bot(TOKEN_BOT,espClient);
// Threads
Thread gpsThread = Thread(); //Thread para el GPS
Thread botThread = Thread(); // Thread para el Bot de Telegram

long dist1; //distancia en cm
long dist2; //distancia en cm
float latitud; // coordenadas latitud
float longitud; // coordenadas longitud
bool ubiOK = false; //Bool para verificar la correcta lectra del GPS
int alturaUser = 175; // Valor por default de la altura, con este valor, detectará el suelo a una distancia horizontal de 2.8m desde el usuario
bool commandAltura = false; // Bool para saber si ha introducido el comando /altura en el chatbot
volatile bool sos = false; //auxiliar para la interrupcion del botón
uint32_t timer = millis(); //timer para mostrar la ubicación del GPS

/****************************************************************************************
 *                                  SETUP                                               *
 ****************************************************************************************/
void setup() {
  Serial.begin(115200);
  
  espClient.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  ConnectWiFi_STA();
  InitOTA();

  client.setServer(mqtt_server,1883);
  
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  digitalWrite(trig1,LOW);

  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  digitalWrite(trig2,LOW); 

  pinMode(zumbador,OUTPUT);
  
  pinMode(pulsador, INPUT_PULLUP);
  attachInterrupt(pulsador, interrupt_SOS, FALLING);

  GPS.begin(9600);
  // Turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  gpsThread.onRun(lecturaUbi);
  gpsThread.setInterval(1000); //cada 1s se llama a calcular la ubicacion GPS

  bot_setup();
  
  botThread.onRun(checkMessages);
  botThread.setInterval(5000); // cada 5s comprueba los mensajes del chatbot
}


/****************************************************************************************
 *                                  LOOP                                                *
 ****************************************************************************************/
void loop() {

  ArduinoOTA.handle();
  
  client.loop();

  if(botThread.shouldRun())
    botThread.run();
  
  dist1 = lecturaDistancia(trig1,echo1); // TS
  dist2 = lecturaDistancia(trig2,echo2); // BS
  
  riesgoDistancias(dist1, dist2);
  
  mandarDistancia(1,dist1);
  mandarDistancia(2,dist2);

  if(gpsThread.shouldRun())
    gpsThread.run();
    
  if(ubiOK){
    mandarUbi();
    ubiOK = false;
  }
  
  mandarPulsadorSOS();
}


/****************************************************************************************
 *                                  FUNCIONES                                           *
 ****************************************************************************************/

// ******************************************************************** LECTURA DISTANCIA
long lecturaDistancia(byte trig, byte echo){
  long t;
  
  digitalWrite(trig,HIGH);
  delayMicroseconds(10); // Eviamos un pulso de 10us
  digitalWrite(trig,LOW);
  t = pulseIn(echo,HIGH); //Obtenemos el ancho del pulso
  return t/59; //Escalamos el tiempo a una distancia en cm
}

// ******************************************************************** RIESGO DISTANCIAS
void riesgoDistancias(long distTS, long distBS){
  float alpha = 68.8998; // Ángulo entre el usuario y la direccción del sensor
  float alturaOmbligo = alturaUser / 1.618; // El valor 1.618 es el número áureo phi=1.61803398... https://www.comocubriruncuerpo.org/medidas-verticales-a-partir-de-la-estatura/
  long distSegura = alturaOmbligo / cos(alpha);

  if(distTS < 200){
    Serial.println("Riesgo Distancia TS");
    avisoZumb();
  }
  if(distBS < (distSegura - 7)){
    Serial.println("Riesgo Distancia BS objeto en el suelo");
    avisoZumb();
  }
  else if(distBS > (distSegura + 5)){
    Serial.println("Riesgo Distancia BS desnivel o rampa");
    avisoZumb();
  }
}

// ******************************************************************** ZUMBADOR
void avisoZumb(){
  Serial.println("Zumbador ON");
  for(int i=0;i<2;i++){
    digitalWrite(zumbador,HIGH);
    delay(100);
    digitalWrite(zumbador,LOW);
    delay(100);
  }
  digitalWrite(zumbador,LOW);
}

// ******************************************************************** LECTURA UBI
void lecturaUbi(){
  while(true){
    // read data from the GPS in the 'main loop'
    GPS.read();
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
        return; // we can fail to parse a sentence in which case we should just wait for another
    }
  
    // approximately every 20 seconds or so, print out the current stats
    if (millis() - timer > 20000) {
      timer = millis(); // reset the timer
      Serial.print("Fix: "); Serial.print((int)GPS.fix);
      Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
      if (GPS.fix) {
        Serial.print("Location: ");
        int aux_lat_grados = GPS.latitude/100;
        float aux_lat_minutos = (float)GPS.latitude - (aux_lat_grados * 100);
        latitud = aux_lat_grados + (float)(aux_lat_minutos/60);
        Serial.print("Latitud: ");
        Serial.print(latitud);
        Serial.print(", "); 
        int aux_lon_grados = GPS.longitude/100;
        float aux_lon_minutos = (float)GPS.longitude - (aux_lon_grados * 100);
        longitud = (-1) * (aux_lon_grados + (float)(aux_lon_minutos/60));
        Serial.print("Longitud: ");
        Serial.println(longitud);
        ubiOK = true;
      }
    }
  }
}

// ******************************************************************** MANDAR DISTANCIA
void mandarDistancia(int sensor, int distancia){
  char buf2[50];
  const char* fin = "}";
  String buf;

  if (client.connect("Sensor Distancia",token_sensor,"")) {
      Serial.println("Connected Sensor Distancia");
  }

  if(sensor == 1){ // 1 - Sensor de arriba TS
    buf = "{\"distance_TS\":";

    buf.concat(distancia);
    buf.concat(fin);
    Serial.print("Enviado: ");
    Serial.println(buf);  
    buf.toCharArray(buf2,50);
    if(client.publish(TOPIC_T,buf2)){
      Serial.println("OK");
    }
  }
  else if (sensor == 2){ // 2 - Sensor de abajo BS
    buf = "{\"distance_BS\":";
    buf.concat(distancia);
    buf.concat(fin);
    Serial.print("Enviado: ");
    Serial.println(buf);
    buf.toCharArray(buf2,50);
    if(client.publish(TOPIC_T,buf2)){
      Serial.println("OK");
    }
  }
  client.disconnect();
} 

// ******************************************************************** MANDAR PULSADOR
void mandarPulsadorSOS(){
  String buf = "{\"SOS\":";
  const char* fin ="}"; 
  char buf2[50];
  String puls = "false";
  Serial.println("Mandar Pulsador:");
  if (client.connect("SOS",token_pulsador,"")) {
    Serial.println("Connected Pulsador");
  }
  if(sos){
    puls = "true";
  }
  buf.concat(puls);
  buf.concat(fin);
  Serial.print("Enviado: ");
  Serial.println(buf);
  buf.toCharArray(buf2,50);
  if(client.publish(TOPIC_A,buf2)){
    Serial.println("OK");
  }
  client.disconnect();
  sos = false;
}

// ******************************************************************** MANDAR UBI
void mandarUbi(){  
  String buf;
  char buf2[100];
  buf = "{\"latitude\":";
  String buf_aux =",\"longitude\":"; 
  const char* fin = "}";
  Serial.println("Mandar Ubi:");
  if (client.connect("Tracker",token_tracker,"")) {
    Serial.println("Connected Tracker");
  }
  buf.concat(latitud);
  buf_aux.concat(longitud);
  buf.concat(buf_aux);
  buf.concat(fin);
  Serial.print("Enviado: ");
  Serial.println(buf);
  buf.toCharArray(buf2,100);
  if(client.publish(TOPIC_A,buf2)){
    Serial.println("OK");
  }
  client.disconnect();
}

// ******************************************************************** BOT SETUP
void bot_setup()
{
  const String commands = F("["
                            "{\"command\":\"help\",  \"description\":\"Get bot usage help\"},"
                            "{\"command\":\"start\", \"description\":\"Message sent when you open a chat with a bot\"},"
                            "{\"command\":\"interrupt\", \"description\":\"Interrupcion Manual para simulación de boton SOS\"},"
                            "{\"command\":\"setAltura\",\"description\":\"Para introducir la altura del Usuario\"}" // no comma on last command
                            "]");
  bot.setMyCommands(commands);
}
// ******************************************************************** HANDLE NEW MESSAGES
void handleNewMessages(int numNewMessages){
  Serial.println("handleNewMessages");
  Serial.println(numNewMessages);
  String answer;
  for(int i = 0; i < numNewMessages; i++){
    telegramMessage &msg = bot.messages[i];
    String chat_id = String(bot.messages[i].chat_id);

    Serial.println(msg.text);
    //String from_name = bot.messages[i].from_name;
    if(msg.text == "/start"){
      answer = "Bienvenido a SmartVest.\n"; //+ from_name +
      answer += "Usa el comando /help para ver los comandos.\n";
    }
    else if(msg.text == "/help"){
      answer = "/start : Comando inicial para mensaje de bienvenida.\n";
      answer = "/help : Comando para mostrar todos los comandos.\n";
      answer += "/setAltura : Comando para introducir la altura del usuario, en cm. Ej: 175.\n";
      answer += "/interrupt : Comando para simular el boton SOS (solo en prototipo).";
    }
    else if(msg.text == "/setAltura"){
      commandAltura = true;
      answer =  "Introduce la altura.";
    }
    else if(msg.text == "/interrupt"){
      sos = true;
      answer =  "Interrupción Manual.";
    }
    else if(commandAltura){
      commandAltura = false;
      alturaUser = msg.text.toInt();
      answer = "Altura introducida correctamente con valor: " + String(alturaUser);
    }
    else
      answer = "No te he entendido.";
      
    bot.sendMessage(msg.chat_id, answer, "Markdown");
  }
}

// ******************************************************************** COMPROBAR MSG
void checkMessages(){
  Serial.println("Check");
  int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
  while(numNewMessages){
    Serial.print("got response");
    handleNewMessages(numNewMessages);
    numNewMessages = bot.getUpdates(bot.last_message_received + 1);
  }
}

/********************************************************************* 
 *                           INTERRUPT                               *
 *********************************************************************/
void interrupt_SOS(){
  Serial.println("Interrupt");
  sos = true;
}
