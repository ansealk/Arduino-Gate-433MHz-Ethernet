////////////////////////////////////////////////////////////////////////////////////////
// Шлюз между радиосетью на частоте 433MHz и сетью Ethernet 
// 
//--------------------------------------------------------------
// Версия:          0.10  
// Автор:           ansealk
//---------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////
//  История версий
//  0.01  -  07.09.2015  -  16 602 ( 6%) / 1 674 (20%) Первый релиз. Пока берем тестовые данные
//  0.02  -  08.09.2015  -                             Добавлен флаг на тестовые данные. Код раскидан по функциям
//  0.03  -  08.09.2015  -  19 864 ( 7%) / 2 031 (24%) Добавлен вывод на дисплей. Добавен флаг вывода на дисплей
//  0.10  -  15.09.2015  -  22 060 ( 8%) / 2 628 (32%) Добавлена работа с приемником 433MHz. Чтение и вывод данных
//                                                     с датчика на кухне. Исправлен вывод на дисплей.
//  0.11  -  15.09.2015  -  22 120 ( 8%) / 3 248 (39%) Произведена подготовка для подключения RTC. 
//                                                     Размер XML-буфера увеличен до 1000 символов.
//  0.12  -  16.09.2015  -  23 142 ( 9%) / 3 354 (40%) Добавлено чтение и вывод датчика в комнате.



////////////////////////////////////////////////////////////////////////////////////////
// Принцип работы
// Принимает данные с радиомодулей на частоте 433MHz и выводит данные на веб-страницу

////////////////////////////////////////////////////////////////////////////////////////
// Используемые компоненты:
//     1. Arduino Mega 2560 Rev3
//     2. Ethernet-шилд W5100
//     3. Приемник радиосигнала XY-MK-5V на частоте 433MHz

////////////////////////////////////////////////////////////////////////////////////////
// Описание структуры для работы с радиоканалом
//Она должна быть определена одинаково на приёмнике и передатчике
//Размер структуры не должен превышать 26 байт (ограничение VirtualWire)
// device_id       - Идентификатор устройства, отправившего пакет. 
//                   Тип данных: byte (диапазон значений от 0 до 255)
// destination_id  - Идентификатор устройства, кому предназначен пакет. 
//                   Тип данных тот же, что и у device_id. 
//                   Пакеты всё равно будут получаться всеми приёмниками, 
//                   но уже программой на самом приёмнике можно "отсекать" пакеты, 
//                   которые устройству не предназначены. Так же можно принять то, 
//                   что значение "0" в данном поле означает широковещательный пакет. 
// packet_id       - Идентификатор пакета. Тип: unsigned int (2 байта, 0-65535). 
//                   По замыслу, при отправке пакет "помечается" случайным числом, 
//                   что может быть использовано для повторной отправки одного и того
//                   же пакета несколько раз с каким-то интервалом - ввиду ненадёжности 
//                   протокола это имеет смысл, но принимающее устройство должно
//                   фильтровать повторные команды дабы не выполнять одно и то же
//                   действие в качестве реакции на пакет данных. 
// command         - Тип команды. Тип данных: byte (1 байт, 0-255). 
//                   Это так называемый "класс команды", а по сути информация о том, 
//                   что за данные мы посылаем. Например, мы можем составить собственную
//                   таблицу команд, отведя для команды управления открытием/закрытием
//                   номер 10, а для команды передачи данных о температуре номер 15. 
//                   Главное, чтобы эта таблица у нас была постоянной. 
//                   А можно поступить ещё хитрее - подсмотреть возможные команды в том
//                   же протоколе ZWave и использовать у себя их таблицу, чтобы было 
//                   всё "как у взрослых", и не нужно было заботиться о сохранности 
//                   этих ценных сведений. 
// data            - Cобственно данные. Тип данных: int (2 байта, -32,768 - 32,767).
//                   В этом поле мы передаём непосредственно данные в виде одного числа. 
//                   Температуру можно передать (например, умножив на 100),
//                   статус датчика движения, команду для приёмника с реле. 
//                   Текстовые данные на внешний дисплей, конечно, не отправишь, 
//                   но такой цели и не ставилось. 
// Итого у нас получилась структура в 7 байт
//struct SEND_DATA_STRUCTURE
//{
//   byte device_id;         
//   byte destination_id;    
//   unsigned int packet_id;         
//   byte command;
//   int data;
// };

////////////////////////////////////////////////////////////////////////////////////////
// Подключаемые компоненты
//
//  W5100 Ethernet-shield
//  10   ->  10  SS for Ethernet
//  04   ->  04  SS for SDCard
//  ISP  ->  ISP Используется полностью для работы с сетью
//
//  XY-MK-5V приемник 433MHz
//  VCC  ->  5V
//  GND  ->  GND
//  ADAT ->  D5
//
//  LCD I2C 20x4
//  VCC  ->  5V
//  GMD  ->  GND
//  SCL  ->  21(SCL)
//  SDA  ->  20(SDA)

#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Ethernet.h>
#include <VirtualWire.h>
#include <Rtc_Pcf8563.h>
#include <LiquidCrystal_I2C.h>
#include <EasyTransferVirtualWire.h>

////////////////////////////////////////////////////////////////////////////////////////
// Флаги вывода и отладки
#define DEBUG    1  // 2 - Отладка по COM-порту с ожиданием открытия консоли
                    // 1 - Отладка по COM-порту
                    // 0 - без отладки
#define XML      1  // 1 - Вывод данных в формате XML через веб
                    // 0 - без вывода
#define TESTDATA 0  // 0 - данные будут поступать с реальных датчиков
                    // 1 - использовать тестовые данные
#define STATIC   1  // 0 - использовать DHCP
                    // 1 - статическая адресация, использовать данные, указаные ниже
#define DISPLAY  1  // 0 - не использовать вывод на дисплей
                    // 1 - Выводить данные на дисплей
                  
////////////////////////////////////////////////////////////////////////////////////////
// Дисплей
#define I2C_ADDR    0x27    // Адрес дисплея
// Переназначение пинов для I2C модуля
#define BACKLIGHT_PIN 3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7
// Инициализация дисплея
LiquidCrystal_I2C lcd(I2C_ADDR,
                      En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin,
                      BACKLIGHT_PIN, POSITIVE);
int count;


////////////////////////////////////////////////////////////////////////////////////////
// Определения и константы

// Пины
#define RECEIVE_PIN       44
#define SDA_PIN           20
#define SCL_PIN           21

#define  BROADCAST        0

// Команды
#define CMD_KITCHEN_TEMP  1
#define CMD_KITCHEN_HUM   2
#define CMD_CUPBOARD_TEMP 3
#define CMD_CUPBOARD_HUM  4
#define CMD_ROOM_TEMP     5
#define CMD_ROOM_HUM      6
#define CMD_EXT_TEMP      7
#define CMD_EXT_HUM       8
#define CMD_PRESSURE      9
#define CMD_ALTITUDE     10
#define CMD_RAIN         11
#define CMD_ROOM_L       12
#define CMD_EXT_L        13
#define CMD_KITCHEN_L    14


// Делители для данных
#define  X1               1
#define  X10              10
#define  X100             100

#define RF_SPEED          2000  //скорость работы приемника, байт/сек
#define COM_SPEED         57600 //скорость работы COM-порта, байт/сек


////////////////////////////////////////////////////////////////////////////////////////
// Структура для передачи данных по радиоканалу
struct SEND_DATA_STRUCTURE
{
  byte device_id;         // ID передатчика устрйства
  byte destination_id;    // ID приемника
  unsigned int packet_id; // ID пакета
  byte command;           // код команды
  int data;               // данные
} mydata;

////////////////////////////////////////////////////////////////////////////////////////
// Структура для хранения данных с метеостанции
// Держим данные с плавающей точкой в char* 
// - printf не умеет работать с числами с точкой
struct METEO_DATA
{
  char tRoom[7];      // -XX.XX - 6 символов + конец строки
  char hRoom[6];      // XX.XX  - 5 символов + конец строки
  char tKitchen[7];
  char hKitchen[6];
  char tServer[7];
  char hServer[6];
  char tExt[7];
  char hExt[6];
  
  char pressure[7];  // XXX.XX  - 6 символов + конец строки
  char altitude[7];  // XXX.XX  - 6 символов + конец строки
  
  char lRoom[7];    // XXXX.XX  - 6 символов + конец строки
  char lKitchen[7];
  char lExt[7];
  
  char rain[6];
  
  String rtcDate;
  String rtcTime;
} meteoData;

////////////////////////////////////////////////////////////////////////////////////////
// Настройки сети
byte mac[] = {0x00,0x15,0xF2,0xD3,0x45,0xED}; // mac-адрес
#if STATIC
  IPAddress ip (172, 16 ,0 ,81);
  IPAddress gw   ( 172,16,0,1    );
  IPAddress dns1 ( 172,16,0,250  );
  IPAddress mask ( 255,255,255,0 );
#endif
EthernetServer server(80); 
char buffer[1000];

////////////////////////////////////////////////////////////////////////////////////////
// 433MHz
EasyTransferVirtualWire ET;
unsigned int unique_device_id = 0; // Уникальный код устройства

////////////////////////////////////////////////////////////////////////////////////////
// Часы
Rtc_Pcf8563 rtc; 

/////////////////////////////////////
// Устанавливаем время на часах (используем только вручную)
void SetClock()
{
  rtc.initClock();
  //day, weekday, month, century(1=1900, 0=2000), year(0-99)
  rtc.setDate(14, 6, 3, 1, 10);
  //hr, min, sec
  rtc.setTime(1, 15, 0);
}

////////////////////////////////////////////////////////////////////////////////////////
// Запись в EEPROM
// p_address  - адрес, по которому будет осуществляться запись
// p_value    - Значение, которое будет записано 
void EEPROMWriteInt(int p_address, unsigned int p_value)
{
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

////////////////////////////////////////////////////////////////////////////////////////
// Чтение из EEPROM
// Возвращает прочитаное значение
// p_address  - адрес, по которому будет осуществляться чтение
unsigned int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

////////////////////////////////////////////////////////////////////////////////////////
// Заполнение структуры METEO_DATA тестовыми данными
void fillTestData()
{
  dtostrf(12.01,  4, 2, meteoData.tRoom);
  dtostrf(22.02  ,  4, 2, meteoData.hRoom);
  dtostrf(29.03  ,  4, 2, meteoData.tKitchen);
  dtostrf(23.04  ,  4, 2, meteoData.hKitchen);
  dtostrf(34.05  ,  4, 2, meteoData.tServer);
  dtostrf(56.06  ,  4, 2, meteoData.hServer);
  dtostrf(-26.07 ,  4, 2, meteoData.tExt);
  dtostrf(72.08  ,  4, 2, meteoData.hExt);
  dtostrf(741.09,  4, 2, meteoData.pressure);
  dtostrf(121.10,  4, 2, meteoData.altitude);
  dtostrf(235.11  ,  4, 2, meteoData.lRoom);
  dtostrf(268.12  ,  4, 2, meteoData.lKitchen);
  dtostrf(945.13  ,  4, 2, meteoData.lExt);
  dtostrf(1155  ,  4, 0, meteoData.rain);
}

////////////////////////////////////////////////////////////////////////////////////////
// Создание XML-страницы
void createXML()
{
  EthernetClient client = server.available();
  if (client)  // Если подключился новый клиент 
  {
    #if DEBUG
      Serial.println("new client");
    #endif
    // HTTP-запрос кончается поустой строкой
    boolean currentLineIsBlank = true;
    while (client.connected()) 
    {
      if (client.available()) 
      {
        char c = client.read(); // посимвольно получаем HTTP-запрос
        #if DEBUG
          Serial.write(c);
        #endif

        // Если мы получили символ перевода строки и следующая строка 
        // пустая, то считаем, что весь HTTP-запрос получен и будем
        // формировать ответ к нему 
        if (c == '\n' && currentLineIsBlank) 
        {
          // Формируем ответ XML
          sprintf
          (
            buffer, 
            "<?xml version=\"1.0\"?>\n"
            "<sensors>\n"
            "<temperatureRoom>%s</temperatureRoom>\n"
            "<humidityRoom>%s</humidityRoom>\n"
            "<temperatureExernal>%s</temperatureExernal>\n"
            "<humidityExernal>%s</humidityExernal>\n"
            "<temperatureKitchen>%s</temperatureKitchen>\n"
            "<humidityKitchen>%s</humidityKitchen>\n"
            "<temperatureServer>%s</temperatureServer>\n"
            "<humidityServer>%s</humidityServer>\n"
            "<pressure>%s</pressure>\n"
            "<rain>%d</rain>\n"
            "<altitude>%s</altitude>\n"
            "<luxRoom>%s</luxRoom>\n"
            "</sensors>"
            ,
            meteoData.tRoom,
            meteoData.hRoom,
            meteoData.tExt,
            meteoData.hExt,
            meteoData.tKitchen,
            meteoData.hKitchen,
            meteoData.tServer,
            meteoData.hServer,
            meteoData.pressure,
            meteoData.rain,
            meteoData.altitude,
            meteoData.lRoom
          );
          client.println(buffer);
          break;
        }
        if (c == '\n') 
        {
          // you're starting a new line
          currentLineIsBlank = true;
        }
        else if (c != '\r') 
        {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    #if STATIC
      Serial.println("client disconnected");
    #endif
  }
}

////////////////////////////////////////////////////////////////////////////////////////
// Вывод на дисплей
void displayOutput()
{
  char str[21];
  // Будем выводить раз в примерно 5 секунд
  if (count == 5000) // Если счетчик дошел до 5к, то выводим
  { 

    lcd.setCursor ( 0, 0 );
    sprintf(str,"R t:%.3s h:%.2s%% %.7s", 
      meteoData.tRoom, meteoData.hRoom, meteoData.lRoom);
    lcd.print(str);
    lcd.setCursor ( 0, 1 );
    sprintf(str,"K t:%.3s h:%.2s%% %.7s", 
      meteoData.tKitchen, meteoData.hKitchen, meteoData.lKitchen);
    lcd.print(str);
    lcd.setCursor ( 0, 2 );
    sprintf(str,"E t:%.3s h:%.2s%% %.7s",
      meteoData.tExt, meteoData.hExt, meteoData.lExt);
    lcd.print(str);
    lcd.setCursor ( 0, 3 );
    sprintf(str,"p:%6s r:%4s %4.4s", 
      meteoData.pressure, meteoData.rain, meteoData.altitude);
    lcd.print(str);
    count = 0 ;      // и обнуляем
  }
  else
  {
    count++;        // иначе просто увеличиваем счетчик на 1
  }
}


////////////////////////////////////////////////////////////////////////////////////////
// Назначаем id устройства
int deviceID()
{
  randomSeed(analogRead(0));
  // Читаем/записываем Device ID
  #if DEBUG
    Serial.print("Getting Device ID... ");
  #endif
  unique_device_id=EEPROMReadInt(0);
  if (unique_device_id<10000 || unique_device_id>60000) 
  {
    #if DEBUG
      Serial.print("N/A, updating... "); 
    #endif
    unique_device_id=random(10000, 60000);
    EEPROMWriteInt(0, unique_device_id);
  }
  #if DEBUG
    Serial.println(unique_device_id);
  #endif
  return unique_device_id;
}


////////////////////////////////////////////////////////////////////////////////////////
// Инициализируем приемник
void initReceiver()
{
  ET.begin(details(mydata));  //Для передачи будем использовать структуру
  vw_set_rx_pin(RECEIVE_PIN); //установка пина data-входа приемника
  vw_setup(RF_SPEED);             // Скорость приёма
  vw_rx_start();              // Запуск режима приёма
}

////////////////////////////////////////////////////////////////////////////////////////
//  Cохраняет полученные данные в структуру и выводит сообщение об этом
//  int data    -  поле структуры mydata, bp rjnjhjuj берутся данные
//  char* field -  симпольное поле meteoData, куда пишем данные
//  int mul     -  использовать ли множитель(делитель) для данных?
//       X1     -  Оставить знначение как есть
//       X10    -  Уменьшить в 10 раз
//       X100   -  Уменьшить в 100 раз
void StoreData(int data, char* field, int mul, char* str)
{
  //dtostrf( float d, int w, int r, char* str) - перевод из числа в сторку
  //  float d  -  число, которое переводим в строку
  //  int   w  -  минимальный размер получаемой строки
  //  int   r  -  количестов знаков после запятой
  //  char* str-  стока, в которую записывается полученое значение
  dtostrf ( (float)data/mul , 3, 2, field );
  
  #if DEBUG
          Serial.print ("  Correct command received: ");
          Serial.print (str);
          Serial.print (": ");
          Serial.println (field);
          Serial.println ("End package)");
  #endif
}

/*
void storeDateAndTime()
{
  meteoData.rtcDate = rtc.formatDate(RTCC_DATE_WORLD);
  meteoData.rtcTime = rtc.formatTime(RTCC_TIME_HMS);
}
*/

////////////////////////////////////////////////////////////////////////////////////////
//  Печатает весь полученный пакет
void showPackage()
{
  #if DEBUG
    Serial.println ("(Begin package");
    Serial.print("  Receiving package. DEVICE_ID:");
    Serial.print(mydata.device_id);
    Serial.print(" DESTINATION_ID: ");
    Serial.print(mydata.destination_id);
    Serial.print(" PACKET_ID: ");
    Serial.print(mydata.packet_id);
    Serial.print(" COMMAND_ID: ");
    Serial.print(mydata.command);
    Serial.print(" DATA: ");
    Serial.println(mydata.data);
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////
//  Получение данных
void Receiveing()
{
  if(ET.receiveData())
  {
    //Если получили данные, то разбираем их
    if( mydata.destination_id == BROADCAST || 
        mydata.destination_id == unique_device_id)
    {
      showPackage();
      switch (mydata.command)
      {
        case CMD_KITCHEN_TEMP:
          StoreData ( mydata.data, meteoData.tKitchen, X100, "KITCHEN_TEMP" );
          break;
        case CMD_KITCHEN_HUM:
          StoreData ( mydata.data, meteoData.hKitchen, X100, "KITCHEN_HUM" );
          break;
        case CMD_CUPBOARD_TEMP:
          StoreData ( mydata.data, meteoData.tServer , X100, "SERVER_TEMP" );
          break;
        case CMD_CUPBOARD_HUM:
          StoreData ( mydata.data, meteoData.hServer , X100, "SERVER_HUM" );
          break;
        case CMD_ROOM_TEMP:
          StoreData ( mydata.data, meteoData.tRoom , X100, "ROOM_TEMP" );
          break;
        case CMD_ROOM_HUM:
          StoreData ( mydata.data, meteoData.hRoom , X100, "ROOM_HUM" );
          break;
        case CMD_EXT_TEMP:
          StoreData ( mydata.data, meteoData.tExt , X100, "EXT_TEMP" );
          break;
        case CMD_EXT_HUM:
          StoreData ( mydata.data, meteoData.hExt , X100, "EXT_HUM" );
          break;
        case CMD_PRESSURE:
          StoreData ( mydata.data, meteoData.pressure, X1,"CMD_PRESSURE" );
          break;
        case CMD_ALTITUDE:
          StoreData ( mydata.data, meteoData.altitude , X1, "EXT_ALTITUDE" );
          break;
        case CMD_ROOM_L:
          StoreData ( mydata.data, meteoData.lRoom , X1, "ROOM_L" );
          break;
        case CMD_EXT_L:
          StoreData ( mydata.data, meteoData.lExt , X1, "EXT_L" );
          break;
        case CMD_KITCHEN_L:
          StoreData ( mydata.data, meteoData.lKitchen , X1, "KITCHEN_L" );
          break;
        case CMD_RAIN:
          StoreData ( mydata.data, meteoData.rain , X1, "RAIN" );
          break;
        default:
          #if DEBUG
            Serial.println ("  Unknown command"); 
            Serial.println ("End package)"); 
          #endif
          break;
      }
    meteoData.rtcDate = rtc.formatDate(RTCC_DATE_WORLD);
    meteoData.rtcTime  = rtc.formatTime(RTCC_TIME_HMS);
    #if DEBUG
      Serial.print (meteoData.rtcDate); 
      Serial.print (" "); 
      Serial.println (meteoData.rtcTime); 
    #endif
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////
// SETUP
void setup() 
{
  #if DEBUG
    Serial.begin(COM_SPEED);
    Serial.println("Debug is ON");
  #endif  
  #if DEBUG == 2
    while(!Serial){}
  #endif  

  deviceID();
  initReceiver();
  #if STATIC
    #if DEBUG
      Serial.print("Apply static IP Address...");
    #endif
    Ethernet.begin(mac, ip, dns1, gw, mask);
  #else
    #if DEBUG
      Serial.print("Apply IP Address over DHCP...");
    #endif
    Ethernet.begin(mac);
  #endif
  #if DEBUG
    Serial.println("DONE");
  #endif
  
  #if DISPLAY
    lcd.begin(20,4);
    lcd.setBacklight(HIGH);
    count = 0;
  #endif
  
  server.begin();
  
  #if DEBUG
    Serial.print("  IP.....");
    Serial.println(Ethernet.localIP());
    Serial.print("  MASK...");
    Serial.println(Ethernet.subnetMask());
    Serial.print("  GATE...");
    Serial.println(Ethernet.gatewayIP());
    Serial.print("  DNS....");
    Serial.println(Ethernet.dnsServerIP());
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////
// LOOP
void loop() 
{
  #if TESTDATA
    fillTestData();
  #else
    Receiveing();
  #endif

  #if XML
    createXML();
  #endif
  
  #if DISPLAY
    displayOutput();
  #endif
}
