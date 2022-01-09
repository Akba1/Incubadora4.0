#include <SPI.h>    // Bus SPI
#include <Adafruit_GFX.h> // Manejo de graficos
#include <Adafruit_ILI9341.h> //Controlador ILI9341
#include <Adafruit_AHT10.h>
#include <RTClib.h> 
#include <AsyncTCP.h>//
#include <ESPAsyncWebServer.h>//
#include <AsyncElegantOTA.h>//
#include <EEPROM.h> 
#include <Servo.h>
#include "bitmaps.h"

#include <PID_v1.h> 


const char* ssid = "Vissoni Toj";
const char* password = "082020Xareni";

#define SERVOMOTOR      13
//#define INDICADOR     12
#define S_R             14
#define HUMIFICADOR     27
#define RESISTENCIA     26
#define VENTILADOR1     25
#define VENTILADOR2     33
#define PANTALLA        32
#define U_D             35
#define TFT_DC          4
#define TFT_CS          15

/// BOTONES
byte seleccion = 1;
byte selecc = 0;
byte selec_config = 1;
byte aceptar;
///CONTROL DE FASES
byte actual=0;
byte dia_de_incub=0;
bool sumado = false;
bool fase_1 = true;
bool fase_2 = true;
byte rst = 0;
int lectura;
byte dias_ff1 = 18;
byte dias_ff2 = 21;
byte dias_config;  
////////PARAMEPTROS PID///  
//const double Kp=1.8,Ki=0.8,Kd=0.8 --Muy brusco
//const double Kp=2.5,Ki=1.2,Kd=3.1; --Oscila mucho
//const double Kp=3.0,Ki=1.2,Kd=0.0; --Oscila 2 grados
int8_t decena = 0, unidad = 0, decimal=1, centesimal=1;
double nuevo_parametro;
double Kp=0.89,Ki=0.35,Kd=0.16;
double Input,Output=110, SP_T, SP_H;
double sp_h1=65.50,sp_h2=70.50,sp_t1=37.77,sp_t2=36.00; 
///VARIABLES PRINCIPALES
float temperature, humidity;
////Angulos del Servo
bool movimiento = false;
byte contador_servo = 0;
///Salidas PWM
const int freq = 1000;
const int CanalRESI = 2;
const int CanalVENT1 = 6;
const int CanalVENT2 = 10;
const int resolution = 8;
///Control de tiempos
int segundo,minuto,hora,dia,mes,anio;
///Variable auxiliar
String cuerda;
//HAY NIEBLA?
bool hay_niebla = false;
//PANTALLA
bool esta_encendida = false;
byte contador_pantalla=0;

// Tiempo actual time
unsigned long currentTime = millis();
// Tiempo previo time
unsigned long previousTime = 0; 
// Definir tiempo maximo en milisegundos (ejemplo: 2000 ms = 2s)
const long timeoutTime = 1000;

AsyncWebServer server(80);
RTC_DS3231 rtc; 
Adafruit_AHT10 aht;
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC); 
Servo MG995; //Greando un objeto Servo
PID myPID(&Input, &Output, &SP_T, Kp, Ki, Kd, DIRECT);//Creanto un objeto PID

void inicializar(){
  pinMode(PANTALLA,OUTPUT);
  digitalWrite(PANTALLA,LOW);
  tft.setRotation(1);//PosicionHorizontal 
  if (! aht.begin())
      while (1) delay(5);
   if (! rtc.begin())
      while (1)  delay(5);    
  if (rtc.lostPower()){ 
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      }
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_GREEN,ILI9341_BLACK);
  tft.setTextSize(5);
  tft.setCursor(20,90);
  tft.print("I2C");
  MG995.attach(SERVOMOTOR);
  MG995.write(90);
  MG995.detach();
  tft.begin();
  pinMode(HUMIFICADOR,OUTPUT);
  digitalWrite(HUMIFICADOR,LOW);
  pinMode(PANTALLA,OUTPUT);
  digitalWrite(PANTALLA,HIGH);
  pinMode(S_R, INPUT);
  pinMode(U_D, INPUT);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_GREEN,ILI9341_BLACK);
  tft.setTextSize(5);
  tft.setCursor(20,90);
  tft.print("I/O"); 
  tft.fillScreen(ILI9341_BLACK);     
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(100);
  EEPROM.begin(20);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_GREEN,ILI9341_BLACK);
  tft.setTextSize(5);
  tft.setCursor(20,90);
  tft.print("WIFI"); 
  ledcSetup(CanalRESI,freq, resolution);
  ledcAttachPin(RESISTENCIA, CanalRESI);
  ledcSetup(CanalVENT1,freq, resolution);
  ledcAttachPin(VENTILADOR1, CanalVENT1);
  ledcSetup(CanalVENT2,freq, resolution);
  ledcAttachPin(VENTILADOR2, CanalVENT2);
  ledcWrite(CanalRESI,0);
  ledcWrite(CanalVENT1,255);
  ledcWrite(CanalVENT2,255);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_GREEN,ILI9341_BLACK);
  tft.setTextSize(5);
  tft.setCursor(20,90);
  tft.print("PWM PORTS");
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_GREEN,ILI9341_BLACK);
  tft.setTextSize(5);
  tft.setCursor(20,90);
  tft.print("LISTO!");
  delay(100);
  tft.fillScreen(ILI9341_BLACK);
  digitalWrite(PANTALLA,HIGH); 
}
void leer_sensor(){
  sensors_event_t humi, temp;
  aht.getEvent(&humi, &temp);
  temperature = temp.temperature;
  humidity = humi.relative_humidity;
}
void resetEe(){
  for (int i = 0; i < 20; i++) {
      EEPROM.write(i, 0);
      delay(10);
    }
  EEPROM.commit();
}
void iniciar(byte lapso, byte a, byte m, byte d, byte h, byte mi, byte s){
  DateTime now = rtc.now();      // D  /H/M/S/
  DateTime future (now + TimeSpan(lapso,0,0,0));
  anio = future.year();
  mes = future.month();
  dia = future.day();
  hora = future.hour();
  minuto = future.minute();
  segundo = future.second();
  
  EEPROM.write(h, hora);
  EEPROM.write(m, minuto);
  EEPROM.write(s, segundo);
  EEPROM.write(a, anio-2000);
  EEPROM.write(m, mes);
  EEPROM.write(d, dia);
  if (EEPROM.commit()) {
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    tft.setCursor(20,65);
    if(lapso==dias_ff1){ 
      cuerda = "FIN DE FASE 1";
    }else{
      cuerda = "FIN DE FASE 2";
    }
    tft.print(cuerda);
    tft.setCursor(20,95);
    cuerda = String(anio)+"-"+String(mes)+"-"+String(dia);
    tft.print(cuerda);
    tft.setCursor(20,125);
    cuerda = String(hora)+":"+String(minuto)+":"+String(segundo);
    tft.print(cuerda);
    delay(900);
  } else {
    tft.setCursor(20,65);  
    cuerda = "ERROR";
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);  // texto en color amarillo  
    tft.print(cuerda); 
    delay(500);
  }
  tft.fillScreen(ILI9341_BLACK);
}
void LeerFecha(byte aa, byte mm, byte dd, byte hh, byte mi, byte se){
    
    hora = int(EEPROM.read(hh));
    minuto = int(EEPROM.read(mi));
    segundo = int(EEPROM.read(se));
    dia = int(EEPROM.read(dd));
    mes = int(EEPROM.read(mm));
    anio = int(EEPROM.read(aa));
    cuerda = String(dia) + "-" + String(mes) + "-20" + String(anio);
    
}
void step_servo(){
    if(not movimiento){
      MG995.attach(SERVOMOTOR);
      movimiento = true;
      MG995.writeMicroseconds(1550);
    }
    contador_servo = contador_servo + 1;
    delay(10);
    if(contador_servo >= 4){
        contador_servo = 0;
        movimiento = false;
        MG995.writeMicroseconds(1500);
        MG995.detach();
    }
    
}
void boton_Humedad(){
    digitalWrite(HUMIFICADOR,HIGH);
    delay(800);
    digitalWrite(HUMIFICADOR,LOW);
}
void ControlHUM(){
  if((humidity < SP_H-0.5)and(not hay_niebla)){
        boton_Humedad();
        hay_niebla = true;
  }else if((humidity > SP_H+0.5)and(hay_niebla)){
        boton_Humedad();
        hay_niebla = false;
  }
}
void leerRTC()
{
   DateTime now = rtc.now();
   LeerFecha(1,2,3,4,5,6);
   /*
   if((now.hour()==hora)and(now.minute()==minuto)and(now.second()==segundo)and(selecc==1)){
     if(sumado == false){
        dia_de_incub = dia_de_incub + 1;
        EEPROM.write(14,dia_de_incub);
        EEPROM.commit();
        if(dia_de_incub>21)dia_de_incub = 0;
        sumado = true;
      }
     }else{
        sumado = false; 
     }
     */
   if((now.year()==anio+2000)and(now.month()==mes)and(now.day()==dia)and(selecc==1)){
     if((now.hour()==hora)and(now.minute()==minuto)and(now.second()==segundo)){
           EEPROM.write(0,2);      
           EEPROM.commit(); 
     }         
   }
   LeerFecha(7,8,9,10,11,12);
    if((now.year()==anio+2000)and(now.month()==mes)and(now.day()==dia)and(selecc==1)){
        if((now.hour()==hora)and(now.minute()==minuto)and(now.second()==segundo)){
          resetEe();
          if(hay_niebla){
            boton_Humedad();
            hay_niebla = false;
          }
          ledcWrite(CanalRESI,0);
          ledcWrite(CanalVENT1,255);
          ledcWrite(CanalVENT1,255);
        } 
        
    }
    
    anio = now.year();
    mes = now.month();
    dia = now.day();
    hora = now.hour();
    minuto = now.minute();
    segundo = now.second();
   
}
void setPoints(double set_temp, double set_humi){
  Input = temperature;
  SP_T = set_temp;
  SP_H = set_humi;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(105, 180);
}
void reiniciar(){
    rst=0;
    dia_de_incub=0;
    selecc = 0;
    resetEe();
    if(hay_niebla){
        boton_Humedad();
        hay_niebla = false;
    }
    fase_1 = true;
    fase_2 = true;
    MG995.attach(SERVOMOTOR);
    MG995.write(90);
    MG995.detach();
    Output = 110;
    ledcWrite(CanalRESI,Output);
    myPID.SetMode(MANUAL);
    ledcWrite(CanalVENT1,255);
    ledcWrite(CanalVENT2,255);
    tft.fillScreen(ILI9341_BLACK);  
    tft.setTextSize(2);  
    tft.setCursor(0,205);        
    tft.print("REINICIADO");
    delay(500);
}

void Control(){
  leer_sensor();
  Input = temperature;
  myPID.Compute();
  ledcWrite(CanalRESI, Output);
  EEPROM.write(13,Output);      
  EEPROM.commit();
  ControlHUM();
  if(temperature>=SP_T+0.03){
     ledcWrite(CanalVENT1,0);
     Output = Output - 15;
     if(Output <= 105)Output=105;  
  }else{
     ledcWrite(CanalVENT1,255);  
  }
  if((actual==1 and segundo%20==0) or movimiento == true){
      step_servo(); 
  }
}
void resetear(){
  if(lectura == 1){
    rst = rst + 1;
    if(rst > 5){
      reiniciar();
      tft.fillScreen(ILI9341_BLACK);  
    }
    delay(30);
  }else{
    rst = 0;
  }  
}
void menu(){
    int up_down = digitalRead(U_D);
    if(up_down == 1)
      seleccion = seleccion + 1;
    if(seleccion > 4)
      seleccion = 1;
      
    tft.drawBitmap(240,180,Pollo1_icon, 50, 50, ILI9341_YELLOW);  
    tft.setTextSize(3);
    tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
    tft.setCursor(30,40);
    tft.print("Ciclo Completo");
    tft.setCursor(30,70);
    tft.print("Fase 1");
    tft.setCursor(30,100);
    tft.print("Fase 2");
    tft.setCursor(30,130);
    tft.print("Configuracion");
    /// CONEXION A INTERNET
    tft.setTextSize(2);
    tft.setCursor(40,190);
    tft.drawBitmap(7,180,wifi_icon, 30, 30, ILI9341_NAVY); 
    IPAddress broadCast = WiFi.localIP();
    broadCast[4] = 255;        
    tft.print(broadCast);
    tft.setTextSize(3);
    delay(200);
    switch(seleccion){
      case 1:
        tft.setTextColor(ILI9341_BLACK,ILI9341_WHITE);
        tft.setCursor(30,40);
        tft.print("Ciclo Completo");
        break;
      case 2:
        tft.setTextColor(ILI9341_BLACK,ILI9341_WHITE);
        tft.setCursor(30,70);
        tft.print("Fase 1");
        break;
      case 3:
        tft.setTextColor(ILI9341_BLACK,ILI9341_WHITE);
        tft.setCursor(30,100);
        tft.print("Fase 2");
        break;
      case 4:
        tft.setTextColor(ILI9341_BLACK,ILI9341_WHITE);
        tft.setCursor(30,130);
        tft.print("Configuracion");
        break;
      default:
        selecc = 4;
        break;
    }
    delay(200);
    if(lectura == 1){
      
      lectura = 0;
      tft.fillScreen(ILI9341_BLACK);   
      selecc = seleccion;
      if(selecc != 4) ledcWrite(CanalVENT2,0);
      if(selecc == 1){
        dia_de_incub = dia_de_incub + 1;
        EEPROM.write(0, 1);
        iniciar(dias_ff1,1,2,3,4,5,6);
        iniciar(dias_ff2,7,8,9,10,11,12);
      }
      delay(300);
    }
    
}
void disp_hr_con(){
    tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
    tft.setTextSize(2);
    
    /// HORA
    tft.drawBitmap(10,1,Reloj_icon, 30, 30, ILI9341_WHITE); 
    tft.setCursor(40,10);
    tft.print(hora);      
    tft.print(":");
    if(minuto<10){
      cuerda = "0"+String(minuto);
      tft.print(cuerda); 
    }else{
      tft.print(minuto); 
    }  
    tft.print(":");
    if(segundo<10){
      cuerda = "0"+String(segundo);
      tft.print(cuerda); 
    }else{
      tft.println(segundo); 
    }
    /// REINICIO
    tft.setTextSize(3);
    tft.setCursor(245,10);
    if(rst!=0 and actual!=0){
      tft.print(rst);
    }else{
      tft.print(" ");
    } 
}
void disp_Ctrl_Dt(){
  bool show;
  tft.setTextSize(3);
  if(dia_de_incub!=0){
    tft.setCursor(240,150);
    cuerda = "  "+String(dia_de_incub);
    tft.print(cuerda);
  }
  
  switch(actual){
      case 0:
        show = 0;
        tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);  // texto en color negro
        break;
      case 1:
        tft.drawBitmap(240,180,Pollo2_icon, 43, 48, ILI9341_YELLOW);
        show = 1;
        tft.drawBitmap(10,35,Calor_icon, 35, 46, ILI9341_RED);
        tft.drawBitmap(10,95,Humedad_icon, 38, 38, ILI9341_BLUE);
        break;
      case 2:   
        tft.drawBitmap(240,150,Pollo3_icon, 70, 70, ILI9341_YELLOW);
        show = 1;
        tft.drawBitmap(10,35,Calor_icon, 35, 46, ILI9341_RED);
        tft.drawBitmap(10,95,Humedad_icon, 38, 38, ILI9341_BLUE);
        break;
      default:
        show = 0;
        tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);
        break;
  }
  tft.setTextSize(4);
  tft.setCursor(50,45);
  if(show == 1){
    tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
  }else{
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);
  }
  cuerda = String(temperature)+"C";
  tft.print(cuerda);
  tft.setCursor(50,95);
  if(show == 1){
    tft.setTextColor(ILI9341_BLUE, ILI9341_BLACK);
  }else{
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);
  }
  cuerda = String(humidity)+"% " + String(hay_niebla);  
  tft.print(cuerda); 
  tft.setCursor(30,165);
  if(show == 1){
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  }else{
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);
  }
  cuerda = String(((Output-105)/75)*100)+"%  ";
  tft.print(cuerda); 
  if(movimiento == 1){
    tft.drawBitmap(200,130,Mov_icon, 19, 19, ILI9341_WHITE);
  }else{
    tft.setCursor(200,130);
    tft.print("  ");
  }  
}

void settings(){
  tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
  tft.setCursor(30,30);
  tft.setTextSize(4);
  tft.print("CONFIG");
  resetear();
  int up_down = digitalRead(U_D);
  if(up_down == 1)
    seleccion = seleccion + 1;
  if(seleccion > 4)
    seleccion = 1;
    
  tft.setTextSize(2);
  tft.setCursor(30,80);
  tft.setTextColor(ILI9341_GREEN,ILI9341_BLACK);
  tft.print("Gallus Gallus Domesticus");
  tft.setCursor(30,110);
  tft.setTextColor(ILI9341_YELLOW,ILI9341_BLACK);
  tft.print("Cotornix Cotornix");
  tft.setCursor(30,140);
  tft.setTextColor(ILI9341_RED,ILI9341_BLACK);
  tft.print("Anas Domesticus");
  tft.setCursor(30,170);
  tft.setTextColor(ILI9341_BLUE,ILI9341_BLACK);
  tft.print("Manual");
  delay(200);
  switch(seleccion){
      case 1:
        tft.setTextColor(ILI9341_BLACK,ILI9341_GREEN);
        tft.setCursor(30,80);
        tft.print("Gallus Gallus Domesticus");
        break;
      case 2:
        tft.setTextColor(ILI9341_BLACK,ILI9341_YELLOW);
        tft.setCursor(30,110);
        tft.print("Cotornix Cotornix");
        break;
      case 3:
        tft.setTextColor(ILI9341_BLACK,ILI9341_RED);
        tft.setCursor(30,140);
        tft.print("Anas Domesticus");
        break;
      case 4:
        tft.setTextColor(ILI9341_BLACK,ILI9341_BLUE);
        tft.setCursor(30,170);
        tft.print("Manual");
        break;
      default:
        selecc = 0;
        break;
    }
    delay(200);
    //lectura = digitalRead(S_R);
    if(lectura == 1){
      lectura = 0;
      tft.fillScreen(ILI9341_BLACK);   
      selec_config = seleccion;
      selecc = 5;
      delay(200);
    }
    
}
/*
void actualizar_decimal(){
    
  }
void actualizar_entero(){
  
  }*/
void parametro(){
    tft.setTextSize(3);
    tft.setCursor(10,90);
    tft.print(String(decena));
    tft.setCursor(30,90);
    tft.print(String(unidad));
    tft.setCursor(50,90);
    tft.print(".");
    tft.setCursor(60,90);
    tft.print(String(decimal));
    tft.setCursor(80,90);
    tft.print(String(centesimal));
    delay(200);
    nuevo_parametro =(10*decena) + unidad + (decimal / 10) + (centesimal/100);
    int pasar = digitalRead(S_R);
    int sumar = digitalRead(U_D);    
    tft.setTextColor(ILI9341_BLACK,ILI9341_BLACK); 
    switch(seleccion){
      case 1:
        tft.setCursor(10,90);
        tft.print(String(decena));
        if(sumar == 1)unidad = unidad + 1;
        if(unidad > 9)unidad = 0;
        break;
      case 2:
        tft.setCursor(30,90);
        tft.print(String(unidad));
        if(sumar == 1)unidad = unidad + 1;
        if(unidad > 9)unidad = 0;
        break;
      case 3:
        tft.setCursor(60,90);
        tft.print(String(decimal));
        if(sumar == 1)decimal = decimal + 1;
        if(decimal > 9)decimal = 0;
        break;
      case 4:
        tft.setCursor(80,90);
        tft.print(String(centesimal));
        if(sumar == 1) centesimal = centesimal + 1;
        if(centesimal > 9) centesimal = 0;
        break;
      default:
        seleccion = 1;
        break;
    }
    delay(200);
    if(pasar == 1 and aceptar < 1)
      seleccion = seleccion + 1;
    if(seleccion > 4) seleccion = 1;
    if(pasar == 1){
      aceptar = aceptar + 1;
      if(aceptar > 9){
        aceptar = 0;
        selecc = 0;
        tft.fillScreen(ILI9341_BLACK);
        tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
        tft.setTextSize(4);
        tft.setCursor(20,90);
        tft.print("Guardado");
        tft.setCursor(20,120);
        tft.print(cuerda);
        delay(900);
        tft.fillScreen(ILI9341_BLACK);
      }
    }else{
      aceptar = 0;
    }
}
void cambiar_parametro(){
    tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
    tft.setCursor(30,30);
    tft.setTextSize(2);
    int pasar = digitalRead(U_D);
    switch(selec_config){
      case 1:
        tft.print("Gallus Gallus Domesticus");
        sp_t1 = 37.77;
        sp_h1 = 65.50;
        sp_t2 = 36.00;
        sp_h2 = 70.00;
        dias_ff1 = 18;
        dias_ff2 = 21;
        selecc = 0;
        break;
      case 2:
        tft.print("Coturnix Coturnix");
        sp_t1 = 38.2;
        sp_h1 = 65.50;
        sp_t2 = 38.8;
        sp_h2 = 70.00;
        dias_ff1 = 14;
        dias_ff2 = 18;
        selecc = 0;
        break;
      case 3: 
        tft.print("Anas Platyrhynchos Domesticus");
        sp_t1 = 37.77;
        sp_h1 = 65.50;
        sp_t2 = 36.00;
        sp_h2 = 70.00;
        dias_ff1 = 15;
        dias_ff2 = 18;
        selecc = 0;
        break;
      case 4: 
        tft.print("Manual");
        parametro();
        break;
      default:
        break; 
    } 
  }

void setup(){
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Servidor Inicializado");
  });
  inicializar();
  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
}


void loop(){ 
  AsyncElegantOTA.loop();
  leerRTC();
  disp_hr_con();
  byte scr_on = digitalRead(U_D);
  if(scr_on == 1 or esta_encendida == true){
    if(lectura == 1) contador_pantalla = 0;
    esta_encendida = true;
    digitalWrite(PANTALLA,LOW);
    contador_pantalla = contador_pantalla + 1;
    if(contador_pantalla >= 100){
      esta_encendida = false;
      digitalWrite(PANTALLA,HIGH) ;
      contador_pantalla = 0;
      }
      
    }
  lectura = digitalRead(S_R);
  switch(selecc){
    case 0:
      actual = 0;
      menu();
      myPID.SetMode(MANUAL);
      break;
    case 1:   
      actual = int(EEPROM.read(0));       
      disp_Ctrl_Dt();
      break;
    case 2:
      actual = 1;
      disp_Ctrl_Dt();
      break;
    case 3:
      actual = 2;
      disp_Ctrl_Dt();
      break;
    case 4:
      settings();
      break;
    case 5:
      cambiar_parametro();
      break;
    default:
      break;  
  }
      
  switch(actual){
    case 0:
      break;
   case 1:
         if(fase_1 = true){
              setPoints(sp_t1,sp_h1);
              fase_1 = false;
         }
         Control();    //ESTA LINEA CONTROLA SERVO, FOCO, /, VENTILADORES
         resetear();
         break;
   case 2:
        if(fase_2 = true){
              setPoints(sp_t2,sp_h2);
              fase_2 = false;
       }
        Control();    //ESTA LINEA CONTROLA SERVO, FOCO, HUMIFICADOR, VENTILADORES
        resetear();
        break;
   default:
       EEPROM.write(0,0);
       EEPROM.commit();
       break;
  }
}
