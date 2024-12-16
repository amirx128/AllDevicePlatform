#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <painlessMesh.h>
// #include <WiFi.h>
#include "DHT.h"
#include <Adafruit_Sensor.h>
#include <HardwareSerial.h>
#include <set>
// #include <ESPmDNS.h>
#include <Arduino.h>
#include <map>
#include <ArduinoJson.h>

#include <FS.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>

///

std::vector<String> mainNetworkList = {"1744193321", "3939444277", "2147483647", "4444", "5555"};
std::set<String> receivedMessageIds; // مجموعه شناسه‌های پیام‌های دریافتی
std::vector<std::pair<String, int>> RelayOnNode;

AsyncWebServer server(80);
painlessMesh mesh;

HardwareSerial SIM808(1); // استفاده از سریال سخت‌افزاری

const int SIM808_RX = 16; // پایه‌های RX و TX که به SIM808 متصل می‌شوند
const int SIM808_TX = 17;

#pragma region variables

#define ssidNameDefultValue "ESP32_ap_free"
#define ssidPassDefultValue "123456789"
#define nodeNameDefultValue "nodeName"

String ssidName = ssidNameDefultValue;     // نام شبکه Wi-Fi
String ssidPassword = ssidPassDefultValue; // رمز عبور
String nodeName = nodeNameDefultValue;     // نام نود به صورت دستی

struct NodeData
{
    uint32_t id;                  // شناسه منحصر به فرد نود
    String name;                  // نام نود
    bool relayStatus;             // وضعیت رله‌ها (4 رله)
    float Humidity;               // رطوبت
    float Temperature;            // دما (در سلسیوس)
    float TemperatureFahrenheit;  // دما (در فارنهایت)
    float HeatIndex;              // شاخص حرارت (در سلسیوس)
    float HeatIndexFahrenheit;    // شاخص حرارت (در فارنهایت)
    float smokValue;              // مقدار سنسور گاز
    int gasDetected;              // وضعیت شناسایی گاز (0 = عدم شناسایی، 1 = شناسایی شده)
    unsigned long lastLiveMinute; // زمان آخرین فعالیت
};

// تعریف std::map برای ذخیره اطلاعات نودها
std::map<uint32_t, NodeData> nodeInfo;
#define IsRootNode true

TaskHandle_t buttonTaskHandle = NULL;
bool relayStates[4] = {false, false, false, false};        // وضعیت رله‌ها
bool buttonPressedFlags[4] = {false, false, false, false}; // پرچم‌های فشار دکمه‌ها

const char *appConfigFilename = "/appConfigDataFile.txt"; // نام فایل برای ذخیره‌سازی داده‌ها

#define buzzer_pin 14

#define pir_1_pin 18
#define pir_2_pin 22
#define pir_3_pin 23
#define Relay_1_pin 120

#define SENSOR_SW_420_PIN 12

#define Remote_Key_0_Pin 4
#define Remote_Key_1_Pin 27
#define Remote_Key_2_Pin 160
#define Remote_Key_3_Pin 25

volatile int state1 = 0; // وضعیت رله 1
volatile int state2 = 0; // وضعیت رله 2
volatile int state3 = 0; // وضعیت رله 3
volatile int state4 = 0; // وضعیت رله 4

String nodeId;
Scheduler userScheduler; // تعریف userScheduler

volatile bool key1Pressed = false; // برای شناسایی دکمه فشرده شده
volatile bool key2Pressed = false; // برای شناسایی دکمه فشرده شده
volatile bool key3Pressed = false; // برای شناسایی دکمه فشرده شده
volatile bool key4Pressed = false; // برای شناسایی دکمه فشرده شده

DynamicJsonDocument sharedDoc(1024);

#define LOCAL_ALERT_PIN 26       // پین LED محلی
#define REMOTE_ALERT_PIN 19      // پین LED ریموت
#define Device_Is_Arm_Led_Pin 21 // پین LED ریموت

#define LocalControllerLedPin 2  // پین
#define LOCAL_LED_BLINK_COUNT 10 // پین LED محلی
#define REMOTE_LED_BLINK_COUNT 5 // پین LED ریموت

// زمان‌های تأخیر
#define LOCAL_LED_DELAY 250  // مدت زمان روشن و خاموش شدن LED در میلی‌ثانیه
#define REMOTE_LED_DELAY 250 // مدت زمان روشن و خاموش شدن LED ریموت در میلی‌ثانیه
#define PIR_TRIGGER_TIME 150 // مدت زمان تحریک PIR به میلی‌ثانیه
#define RECONNECT_DELAY 3000 // مدت زمان تأخیر برای تلاش مجدد در میلی‌ثانیه

#define temperature_pin 15
bool temperature_IsEnable = true;
#define temperature_IsEnable_DefultValue true

#define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321
DHT dht(temperature_pin, DHTTYPE);

// متغیرهای مربوط به زمان‌بندی
unsigned long lastLocalLedToggle = 0;
unsigned long lastRemoteLedToggle = 0;
unsigned long lastReconnectAttempt = 0; // برای تایمر تلاش مجدد اتصال

TaskHandle_t taskReconnectMesh;
TaskHandle_t taskcheckAllPir;
TaskHandle_t taskCheckSensor_SW_420;
TaskHandle_t taskCheckSensor_Pmu;

bool reconnectMeshTaskRunning = false;
bool checkPIR1TaskRunning = false;
bool checkPIR2TaskRunning = false;
bool checkPIR3TaskRunning = false;

#define pir_1_IsEnable_DefultValue true
#define Buzzer_IsEnable_DefultValue true

#define Relay_1_IsEnables_DefultValue false
#define Relay_2_IsEnables_DefultValue false
#define Relay_3_IsEnables_DefultValue false
#define Relay_4_IsEnables_DefultValue false

bool pir_1_IsEnable = pir_1_IsEnable_DefultValue;
bool Buzzer_IsEnable = Buzzer_IsEnable_DefultValue;

bool relay1_IsEnable = Relay_1_IsEnables_DefultValue;
bool relay2_IsEnable = Relay_2_IsEnables_DefultValue;
bool relay3_IsEnable = Relay_3_IsEnables_DefultValue;
bool relay4_IsEnable = Relay_4_IsEnables_DefultValue;

#define smok_digital_pin 32
#define smok_analog_pin 33
bool smok_sensor_IsEnable = true;
#define smok_sensor_IsEnable_DefultValue true

#define device_status_DefultValue "1"             // رشته "1"
String device_status = device_status_DefultValue; // 0 => disable   ,  1 => enable  ,  -1 => half enable

unsigned long lastBroadcastTime = 0;          // زمان آخرین ارسال
const unsigned long broadcastInterval = 5000; // فاصله زمانی 5 ثانیه

#define MAX_RECONNECT_ATTEMPTS 100 // حداکثر تعداد تلاش‌ها برای اتصال
int reconnectAttempts = 0;         // شمارنده تلاش‌ها

void displayNodeInfo();

void onNewConnection(uint32_t nodeId);
void AllarmError(const int &CountInSec, const int &totalSecends, const int &On_Off_Time_1Meen100);
void VibrationAllarmError(const int &CountInSec, const int &totalSecends, const int &On_Off_Time_1Meen100);
String sendStartupSMS(String function_number, String function_msg);
void CheckSensor_SW_420(void *parameter);
void CheckMPUSensor();

void reconnectMeshTask(void *parameter);
void checkAllPirTask(void *parameter);
void checkAllPIR(); // پیش‌اعلام تابع checkPIR
void sendCommands();
void broadcastLocalDatabase();
String generateNodeIdentifier(const String &nodeName, const String &callType);
void receivedCallback(uint32_t from, String &msg);

void SendRelayCommand(); // تابع ارسال

void triggerLocalLed();
void triggerRemoteLed();
bool reconnectMesh(); // تابع برای تلاش مجدد اتصال Mesh
void localLedTask();
void localBuzzerTask();
void sendAllNodeInfo(uint32_t nodeId);
void broadcastToAll(String &msg);

String extractJsonValue(const String &json, const String &key);

float Humidity = 0, Temperature = 0, TemperatureFahrenheit = 0, HeatIndex = 0, HeatIndexFahrenheit = 0, smokValue = 0;
int gasDetected = 0;

#pragma endregion variables

String hexToUnicode(String hex)
{
    String result = "";
    for (int i = 0; i < hex.length(); i += 4)
    {
        // هر 4 رقم را به یک کاراکتر تبدیل کن
        String unicodeChar = hex.substring(i, i + 4);
        int charCode = strtol(unicodeChar.c_str(), NULL, 16);
        result += (char)charCode;
    }
    return result;
}

void sendRequestNodeInfoToAllNodes()
{
    String messageId = generateNodeIdentifier(nodeName, "requestInfo");

    // ایجاد یک پیام درخواست برای تمام نودها با اضافه کردن messageId
    String requestMessage = "{\"type\": \"REQUEST_NODE_DATABASE_INFO\", \"messageId\": \"" + messageId + "\"}";

    Serial.println("send___REQUEST_NODE_DATABASE_INFO  :       " + messageId);

    // ارسال پیام به تمام نودها (برودکست)
    broadcastToAll(requestMessage);
    Serial.println("Request sent to all nodes.");
}
bool initModuleSim808(String command, String expectedResponse, int timeout)
{
    Serial.print("Sending command: ");
    Serial.println(command);
    SIM808.println(command); // ارسال دستور AT به ماژول

    long int time = millis();
    while ((millis() - time) < timeout)
    {
        if (SIM808.available())
        { // بررسی داده‌های دریافتی
            String response = SIM808.readString();
            Serial.print("Response: ");
            Serial.println(response);

            if (response.indexOf(expectedResponse) != -1)
            {
                Serial.println("Command executed successfully.");
                return true; // اگر پاسخ مورد نظر دریافت شد
            }
        }
    }

    Serial.print("Failed to execute command: ");
    Serial.println(command); // نمایش دستور ناموفق در سریال مانیتور
    return false;            // اگر پاسخ موردنظر در زمان مشخص دریافت نشود
}

void monitorInputSms()
{
    // بررسی اگر SIM808 داده‌ای ارسال کرده است
    if (SIM808.available())
    {
        String response = SIM808.readString();
        Serial.println("Raw Response from SIM808:");
        Serial.println(response);

        // بررسی وجود پاسخ پیامک
        if (response.indexOf("+CMGR:") != -1)
        {
            int senderStartIndex = response.indexOf("\",\"") + 3;
            int senderEndIndex = response.indexOf("\"", senderStartIndex);
            String senderNumber = response.substring(senderStartIndex, senderEndIndex);

            int textStartIndex = response.indexOf("\n", senderEndIndex) + 1;
            String smsText = response.substring(textStartIndex);

            // چاپ شماره فرستنده و متن پیام
            Serial.println("------------ SMS Details ------------");
            Serial.print("Sender Number: ");
            Serial.println(senderNumber);
            Serial.print("Message Text: ");
            Serial.println(smsText);
            Serial.println("-------------------------------------");
        }
        else
        {
            Serial.println("No valid SMS data found.");
        }
    }
}

// ارسال دستورات AT به SIM808
void sendATCommandSSSS(String command)
{
    Serial.println("Sending AT Command: " + command);
    SIM808.println(command); // ارسال دستور AT به SIM808
    delay(1000);             // منتظر پاسخ از SIM808
    while (SIM808.available())
    {
        Serial.write(SIM808.read()); // چاپ پاسخ از SIM808
    }
}

// خواندن و ارسال فایل صوتی به SIM808
void sendAudioFileToSIM808(const char *filePath)
{
    if (!SPIFFS.exists(filePath))
    {
        Serial.println("Audio file not found!");
        return;
    }

    File audioFile = SPIFFS.open(filePath, "r");
    if (!audioFile)
    {
        Serial.println("Failed to open audio file!");
        return;
    }

    Serial.println("Streaming audio file to SIM808...");
    while (audioFile.available())
    {
        char audioBuffer[64];
        size_t bytesRead = audioFile.readBytes(audioBuffer, sizeof(audioBuffer));
        SIM808.write((uint8_t *)audioBuffer, bytesRead);
        delay(10); // تنظیم نرخ ارسال
    }

    audioFile.close();
    Serial.println("Audio streaming complete.");
}

// اجرای تماس و ارسال فایل صوتی
void makeCall(String phoneNumber)
{
    // ارسال دستور تماس
    String atCommand = "ATD" + phoneNumber + ";"; // دستور تماس
    Serial.println("Dialing: " + phoneNumber);
    sendATCommandSSSS(atCommand); // ارسال دستور تماس

    // انتظار برای دریافت پاسخ "CONNECT" از SIM808 برای اطمینان از برقراری تماس
    unsigned long startTime = millis();
    bool callConnected = false;
    while (millis() - startTime < 10000)
    { // انتظار حداکثر 10 ثانیه
        if (SIM808.available())
        {
            String response = SIM808.readString();
            if (response.indexOf("CONNECT") != -1)
            {
                Serial.println("Call connected!");
                callConnected = true;
                break;
            }
        }
    }

    if (callConnected)
    {
        // ارسال فایل صوتی هنگام برقراری تماس
        sendAudioFileToSIM808("/allarmVoice.amr");
    }
    else
    {
        Serial.println("Failed to connect the call.");
    }

    // قطع تماس پس از ارسال فایل
    delay(1000);              // مدت زمان ارسال صوت
    sendATCommandSSSS("ATH"); // دستور قطع تماس
    Serial.println("Call ended.");
}

String sendStartupSMS(String function_number, String function_msg)
{
    Serial.println("Sending startup SMS...");

    // اطمینان از تنظیم مجدد حالت متن
    if (!initModuleSim808("AT+CMGF=1", "OK", 1000))
    {
        Serial.println("init failed");
        return "false";
    }

    // ارسال دستور شروع پیامک
    SIM808.print("AT+CMGS=\"");
    SIM808.print(function_number); // شماره مقصد
    SIM808.println("\"");
    delay(2000); // زمان انتظار برای پاسخ

    // ارسال متن پیامک
    SIM808.print(function_msg);
    delay(100);

    // ارسال Ctrl+Z برای ارسال پیام
    SIM808.write(26);
    delay(3000); // زمان کافی برای ارسال پیامک

    Serial.println("Startup SMS sent.");
    Serial.println("send    " + String(function_msg) + "to  :   " + String(function_number));
    return "true";
}
#define PWR_PIN 26456

void powerOnSIM808()
{
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);  // اتصال به GND
    delay(2000);                 // نگه داشتن به مدت 2 ثانیه
    digitalWrite(PWR_PIN, HIGH); // آزاد کردن پایه
}

void SetupSim()
{
    powerOnSIM808();
    SIM808.begin(9600, SERIAL_8N1, SIM808_RX, SIM808_TX); // شروع سریال برای SIM808
    delay(5000);

    Serial.println("Initializing SIM808...");

    delay(1000);

    // بررسی دستورات AT در حین راه‌اندازی
    if (!initModuleSim808("AT", "OK", 1000))
        return;
    if (!initModuleSim808("ATE1", "OK", 1000))
        return;
    if (!initModuleSim808("AT+CPIN?", "READY", 2000))
        return;
    if (!initModuleSim808("AT+CREG?", "0,1", 3000))
        return; // بررسی ثبت در شبکه
    if (!initModuleSim808("AT+CMGF=1", "OK", 1000))
        return; // تنظیم حالت TEXT
    if (!initModuleSim808("AT+CNMI=2,2,0,0,0", "OK", 1000))
        return; // مدیریت پیامک ورودی

    Serial.println("SIM808 Initialized Successfully.");
}
void CheckSpaceSensorTask()
{
    // خواندن مقدار گاز و دود
    int _smokeValue = smokValue = analogRead(smok_analog_pin);
    int _gasDetected = gasDetected = digitalRead(smok_digital_pin); // وضعیت وجود گاز

    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = Humidity = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = Temperature = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    float f = TemperatureFahrenheit = dht.readTemperature(true);

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) || isnan(f))
    {
        // Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }

    // Compute heat index in Fahrenheit (the default)
    float hif = HeatIndexFahrenheit = dht.computeHeatIndex(f, h);
    // Compute heat index in Celsius (isFahreheit = false)
    float hic = HeatIndex = dht.computeHeatIndex(t, h, false);

    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.print(F("°C "));
    Serial.print(f);
    Serial.print(F("°F  Heat index: "));
    Serial.print(hic);
    Serial.print(F("°C "));
    Serial.print(hif);
    Serial.println(F("°F"));
    Serial.println("smokeValue :  " + String(smokValue) + "    gasDetected  : " + _gasDetected);
    vTaskDelay(100);
    if (smokValue > 800)
    {
        // AllarmError(0, 0, 0);
        Serial.println("gas gas gas gas gas gas gas gas gas gas gas gas gas gas gas");
    }
}
String readData(const String &key)
{
    Serial.print("new key: " + key);
    File file = SPIFFS.open(appConfigFilename, FILE_READ);
    String fileContent = "";

    if (file)
    {
        while (file.available())
        {
            fileContent += char(file.read());
        }
        file.close();
    }

    // پیدا کردن مقدار کلید
    int startPos = fileContent.indexOf(key);
    if (startPos != -1)
    {
        int endPos = fileContent.indexOf("\n", startPos);
        String keyValue = fileContent.substring(startPos, endPos);
        int separatorIndex = keyValue.indexOf("=");
        if (separatorIndex != -1)
        {
            Serial.println(" .... in read data method ..... " + keyValue.substring(separatorIndex + 1));
            return keyValue.substring(separatorIndex + 1);
        }
    }
    Serial.println(" .... in read data method ..... not found key");

    return "notfound"; // اگر کلید پیدا نشد، رشته خالی بر می‌گرداند
}
String ReadKey(const String &key, const String &defaultValue)
{
    Serial.print("read new key.... : : : ");
    String value = readData(key);

    if (value == "notfound") // اگر مقدار خالی است
    {
        Serial.println("key " + key + " value : notFound");
        Serial.println("returned  :::   " + defaultValue);
        return defaultValue; // در صورتی که مقداری پیدا نشد
    }
    else // اگر مقداری پیدا شد
    {
        Serial.println("key " + key + " value : " + value);
        return value; // برگرداندن مقدار خوانده شده
    }
}

bool ReadKeyBool(const String &key, bool defaultValue)
{
    String value = ReadKey(key, String(defaultValue));
    value.toLowerCase();     // تبدیل رشته به حروف کوچک
    if (value == "notfound") // اگر مقدار خالی است
    {
        Serial.println("key " + key + " value ::: notFound");
        Serial.println("returned  ::   " + defaultValue);

        return defaultValue; // در صورتی که مقداری پیدا نشد
    }
    bool result = (value == "true" || value == "on" || value == "1");
    return result;
}

int ReadKeyInt(const String &key, int defaultValue)
{
    String value = ReadKey(key, String(defaultValue));
    if (value == "notfound") // اگر مقدار خالی است
    {
        Serial.println("key " + key + " value : notFound");
        return defaultValue; // در صورتی که مقداری پیدا نشد
    }
    int num = 0;
    try
    {
        num = std::stoi(value.c_str()); // تبدیل به عدد صحیح
    }
    catch (const std::invalid_argument &e)
    {
        // در صورتی که ورودی عددی نباشد
        Serial.println("Invalid input for integer conversion");
        num = -21774; // یا مقدار پیش‌فرض
    }
    return num;
}

void blinkLED(int pin, int times, int delayMs)
{
    for (int i = 0; i < times; i++)
    {
        digitalWrite(pin, HIGH); // روشن کردن LED
        delay(delayMs);
        digitalWrite(pin, LOW); // خاموش کردن LED
        delay(delayMs);
    }
}

// تابعی برای ذخیره داده‌ها به صورت key-value
void saveData(const String &key, const String &value)
{
    Serial.println("saveData key values  :  " + key + "   =>>>>>>>>>>>>>>>>>> " + value);

    File file = SPIFFS.open(appConfigFilename, FILE_READ);
    String fileContent = "";

    // اگر فایل وجود داشت، داده‌های قبلی را بخوانیم
    if (file)
    {
        while (file.available())
        {
            fileContent += char(file.read());
        }
        file.close();
    }

    // حالا، به‌روزرسانی داده‌ها
    int startPos = fileContent.indexOf(key);
    if (startPos != -1)
    {
        // اگر کلید قبلاً وجود داشت، مقدار جدید را جایگزین کنیم
        int endPos = fileContent.indexOf("\n", startPos);
        fileContent.replace(fileContent.substring(startPos, endPos), key + "=" + value);
    }
    else
    {
        // اگر کلید جدید است، به فایل اضافه کنیم
        fileContent += key + "=" + value + "\n";
    }

    // حالا، داده‌ها را دوباره در فایل ذخیره می‌کنیم
    file = SPIFFS.open(appConfigFilename, FILE_WRITE);
    if (file)
    {
        file.print(fileContent);
        file.close();
    }
}

void VibrationAllarmError(const int &CountInSec, const int &totalSecends, const int &On_Off_Time_1Meen100)
{
    for (int i = 0; i < CountInSec; i++)
    {
        for (int j = 0; j < totalSecends; j++)
        {
            digitalWrite(buzzer_pin, HIGH); // روشن کردن بیزر
                                            // if (Buzzer_IsEnable)

            delay(On_Off_Time_1Meen100 * 100); // 100 میلی‌ثانیه روشن

            digitalWrite(buzzer_pin, LOW); // روشن کردن بیزر
            delay(100);                    // 100 میلی‌ثانیه خاموش
        }
        delay(50); // 1 ثانیه سکوت
    }
}

void AllarmError(const int &CountInSec, const int &totalSecends, const int &On_Off_Time_1Meen100)
{

    for (int i = 0; i < CountInSec; i++)
    {
        for (int j = 0; j < totalSecends; j++)
        {
            digitalWrite(LOCAL_ALERT_PIN, HIGH); // روشن کردن بیزر

            delay(On_Off_Time_1Meen100 * 100); // 100 میلی‌ثانیه روشن

            digitalWrite(LOCAL_ALERT_PIN, LOW); // روشن کردن بیزر
            delay(100);                         // 100 میلی‌ثانیه خاموش
        }
        delay(1000); // 1 ثانیه سکوت
    }
}

void loadAndSetVariables()
{
    pir_1_IsEnable = ReadKeyBool("pir_1_IsEnable", pir_1_IsEnable_DefultValue);
    relay1_IsEnable = ReadKeyBool("relay1_IsEnable", Relay_1_IsEnables_DefultValue);
    relay2_IsEnable = ReadKeyBool("relay2_IsEnable", Relay_2_IsEnables_DefultValue);
    relay3_IsEnable = ReadKeyBool("relay3_IsEnable", Relay_3_IsEnables_DefultValue);
    relay4_IsEnable = ReadKeyBool("relay4_IsEnable", Relay_4_IsEnables_DefultValue);
    Buzzer_IsEnable = ReadKeyBool("Buzzer_IsEnable", Buzzer_IsEnable_DefultValue);

    ssidName = ReadKey("ssidName", ssidNameDefultValue);
    ssidPassword = ReadKey("ssidPassword", ssidPassDefultValue);

    nodeId = ReadKey("nodeId", String(mesh.getNodeId()));
    nodeName = ReadKey("nodeName ", nodeNameDefultValue);

    smok_sensor_IsEnable = ReadKeyBool("smok_sensor_IsEnable", smok_sensor_IsEnable_DefultValue);
    temperature_IsEnable = ReadKeyBool("temperature_IsEnable", temperature_IsEnable_DefultValue);

    device_status = ReadKey("device_status", device_status_DefultValue); // 0 => disable   ,  1 => enable  ,  -1 => half enable
    delay(500);
}

// تابع برای گرفتن رشته ترکیبی از زمان و نام نود
String generateNodeIdentifier(const String &nodeName, const String &CallType)
{
    unsigned long currentTime = millis();       // دریافت زمان از شروع دستگاه (بر حسب میلی‌ثانیه)
    unsigned long seconds = currentTime / 1000; // تبدیل میلی‌ثانیه به ثانیه
    unsigned long minutes = seconds / 60;       // تبدیل ثانیه به دقیقه
    unsigned long hours = minutes / 60;         // تبدیل دقیقه به ساعت
    unsigned long days = hours / 24;            // تبدیل ساعت به روز
    unsigned long years = days / 365;           // تقریباً تبدیل روز به سال

    unsigned long milliseconds = currentTime % 1000; // باقی‌مانده میلی‌ثانیه‌ها

    // ترکیب زمان (سال، روز، ساعت، دقیقه، ثانیه، میلی‌ثانیه) با نام نود
    String timeString = "";
    // timeString = "ReceiverId_____Receiver___Node___Id_____";
    timeString = "IAmId---";
    timeString += String(years) + "Y-" + String(days % 365) + "D-" +
                  String(hours % 24) + "H-" + String(minutes % 60) + "M-" +
                  String(seconds % 60) + "S-" + String(milliseconds) + "ms";

    // ترکیب زمان و نام نود

    String nid = String(mesh.getNodeId());
    String result = timeString;
    result += "-" + nid + "-" + String(nodeName) + "-" + CallType;

    return result;
}
String generateRelayCommand(String messageId)
{
    int cou = 1;
    // شروع ساخت پیام
    String message = "{\"type\": \"RELAY_COMMAND\", \"messageId\": \"" + messageId + "\"";

    bool relayStatus = false;
    for (size_t i = 0; i < mainNetworkList.size(); ++i)
    {
        String id = mainNetworkList[i];            // شناسه به صورت استرینگ
        NodeData &nodeData = nodeInfo[id.toInt()]; // دسترسی به نود موجود

        cou++;
        if (nodeData.id > 0)
        {
            relayStatus = nodeData.relayStatus;

            message += ", \"Relay" + String(cou) + "___on___" + id + ":" + String(relayStatus ? "1" : "0");
        }
    }

    // بستن پیام
    message += "}";

    return message;
}

void SendRelayCommand()
{
    // تولید شناسه منحصر به فرد برای پیا

    // چاپ مقادیر RelayOnNode در Serial Monitor
    Serial.println("RelayOnNode Content:");
    for (const auto &[id, counter] : RelayOnNode)
    {
        Serial.print("ID: ");
        Serial.print(id);
        Serial.print(", Counter: ");
        Serial.println(counter);
    }

    String messageId = generateNodeIdentifier(nodeName, "relayCommand");
    String message = generateRelayCommand(messageId);

    // ارسال پیام به همه نودها

    Serial.println("SendRelayCommand  =======>>> " + message);
    broadcastToAll(message);
}

void SaveVariablesInDb()
{

    int cou = 0;

    for (const auto &nodePair : nodeInfo)
    {
        uint32_t nodeId = nodePair.first; // تبدیل String به عدد
        if (cou == 0)
        {
            NodeData &nodeData = nodeInfo[nodeId]; // دسترسی به نود موجود
            nodeData.relayStatus = relay1_IsEnable;
        }
        else if (cou == 1)
        {
            NodeData &nodeData = nodeInfo[nodeId]; // دسترسی به نود موجود
            nodeData.relayStatus = relay2_IsEnable;
        }
        else if (cou == 2)
        {
            NodeData &nodeData = nodeInfo[nodeId]; // دسترسی به نود موجود
            nodeData.relayStatus = relay3_IsEnable;
        }
        else if (cou == 3)
        {
            NodeData &nodeData = nodeInfo[nodeId]; // دسترسی به نود موجود
            nodeData.relayStatus = relay4_IsEnable;
        }
    }
    saveData("pir_1_IsEnable", pir_1_IsEnable ? "true" : "false");
    saveData("relay1_IsEnable", relay1_IsEnable ? "true" : "false");
    saveData("relay2_IsEnable", relay2_IsEnable ? "true" : "false");
    saveData("relay3_IsEnable", relay3_IsEnable ? "true" : "false");
    saveData("relay4_IsEnable", relay4_IsEnable ? "true" : "false");
    saveData("Buzzer_IsEnable", Buzzer_IsEnable ? "true" : "false");
    saveData("smok_sensor_IsEnable", smok_sensor_IsEnable ? "true" : "false");
    saveData("temperature_IsEnable", temperature_IsEnable ? "true" : "false");

    saveData("device_status", device_status);

    saveData("ssidName", ssidName);
    saveData("ssidPassword", ssidPassword);

    saveData("nodeId", String(nodeId));
    saveData("nodeName ", nodeName);

    saveData("smok_sensor_IsEnable", smok_sensor_IsEnable ? "true" : "false");
    saveData("temperature_IsEnable", temperature_IsEnable ? "true" : "false");

    uint32_t currentNodeId = mesh.getNodeId();
    nodeInfo[currentNodeId].name = String(nodeName);

    nodeInfo[currentNodeId].relayStatus = relay1_IsEnable;
    broadcastLocalDatabase();
}

void resetFactoryConfig()
{

    Serial.println("Formatting SPIFFS...");

    saveData("pir_1_IsEnable", pir_1_IsEnable_DefultValue ? "true" : "false");
    saveData("relay1_IsEnable", Relay_1_IsEnables_DefultValue ? "true" : "false");
    saveData("relay2_IsEnable", Relay_2_IsEnables_DefultValue ? "true" : "false");
    saveData("relay3_IsEnable", Relay_3_IsEnables_DefultValue ? "true" : "false");
    saveData("relay4_IsEnable", Relay_4_IsEnables_DefultValue ? "true" : "false");

    saveData("Buzzer_IsEnable", Buzzer_IsEnable_DefultValue ? "true" : "false");

    saveData("smok_sensor_IsEnable", smok_sensor_IsEnable_DefultValue ? "true" : "false");
    saveData("temperature_IsEnable", temperature_IsEnable_DefultValue ? "true" : "false");

    saveData("device_status", String(device_status_DefultValue));

    saveData("ssidName", ssidNameDefultValue);
    saveData("ssidPassword", ssidPassDefultValue);

    saveData("nodeId", String(mesh.getNodeId()));
    saveData("nodeName ", nodeNameDefultValue);

    ESP.restart(); // ریست کردن ESP32
}

void setAllPinModes()
{

    pinMode(LocalControllerLedPin, OUTPUT);
    // pinMode(Device_Is_Arm_Led_Pin, OUTPUT);
    pinMode(buzzer_pin, OUTPUT); // تنظیم پین بیزر به عنوان خروجی
    pinMode(LOCAL_ALERT_PIN, OUTPUT);
    pinMode(REMOTE_ALERT_PIN, OUTPUT);

    pinMode(SENSOR_SW_420_PIN12, INPUT_PULLUP);
    pinMode(pir_1_pin, INPUT);

    pinMode(Relay_1_pin, OUTPUT);

    pinMode(Remote_Key_0_Pin, INPUT_PULLDOWN);
    pinMode(Remote_Key_1_Pin, INPUT_PULLDOWN);
    pinMode(Remote_Key_2_Pin, INPUT_PULLDOWN);
    pinMode(Remote_Key_3_Pin, INPUT_PULLDOWN);
}
// تابع برای تبدیل کاراکترهای خاص به URL-encoded
String urlencode(String str)
{
    String encoded = "";
    char c;
    for (int i = 0; i < str.length(); i++)
    {
        c = str.charAt(i);
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~')
        {
            encoded += c; // اضافه کردن کاراکترهایی که نیاز به encoding ندارند
        }
        else
        {
            encoded += "%" + String(c, HEX); // تبدیل به فرمت hexadecimal
        }
    }
    return encoded;
}

// String ReturnCheckBoxesHtmlCode(const std::vector<int> &mainList)
// {
//     String html = "";

//     for (size_t i = 0; i < mainList.size(); i++)
//     {
//         int id = mainList[i];
//         auto it = nodeInfo.find(id); // بررسی وجود آیتم در nodeInfo

//         bool isNodeAvailable = (it != nodeInfo.end());
//         String nodeNameTemp = isNodeAvailable ? it->second.name : "Unknown Node";
//         bool relayIsEnable = isNodeAvailable ? it->second.relayStatus : false;

//         String index = String(i + 1); // شماره اندیس برای نمایش

//         // تنظیم HTML
//         html += "<div style='background-color: #e0f7e0; padding: 10px; margin-bottom: 10px; border: 1px solid #ccc;'>";
//         html += "<input type='hidden' name='relay" + index + "_IsEnable' value='" + String(relayIsEnable) + "'>";

//         String relayStateStringTemp = relayIsEnable ? "checked" : "";
//         String disabledString = isNodeAvailable ? "" : "disabled style='background-color: gray;'";

//         html += "switch " + index + " is on (in *" + nodeNameTemp + "*) : ";
//         html += "<input type='checkbox' name='relay" + index + "_IsEnable_chk' ";
//         html += relayStateStringTemp + " ";
//         html += disabledString + "><br>";

//         html += "</div>";
//     }

//     return html;
// }

String generateCheckboxHtml()
{
    // لیست ایندکس‌های رله
    String html;
    int counter = 1;

    // پاک کردن محتوای قبلی RelayOnNode
    // RelayOnNode.clear();

    for (size_t i = 0; i < mainNetworkList.size(); ++i)
    {
        String id = mainNetworkList[i]; // شناسه به صورت استرینگ
        bool isEnabledTemp = false;
        String nodeName = "Unknown Node";
        String checkedString = "";
        String disabledString = "disabled";
        String backgroundColor = "#f4f4f4"; // رنگ پیش‌فرض

        // جستجوی ID در nodeInfo
        for (const auto &[nodeId, nodeData] : nodeInfo)
        {
            if (String(nodeId) == id)
            {
                isEnabledTemp = true;
                nodeName = nodeData.name;
                checkedString = nodeData.relayStatus ? "checked" : "";
                disabledString = "";
                backgroundColor = nodeData.relayStatus ? "#e0f7e0" : "#f9d6d5"; // سبز برای فعال، قرمز روشن برای غیرفعال
                break;
            }
        }

        // افزودن عضو جدید به RelayOnNode
        RelayOnNode.push_back({id, counter});

        // تولید HTML برای هر رله
        html += "<div style='background-color: " + backgroundColor + "; padding: 10px; margin-bottom: 10px; border: 1px solid #ccc;'>";
        html += "<input type='hidden' name='relay_" + id + "_IsEnable' value='" + String(isEnabledTemp ? "true" : "false") + "'>";
        html += "switch" + String(counter) + "( in  * " + nodeName + " * )  Is On <input type='checkbox' name='relay_" + id + "_IsEnable_chk' " + checkedString + " " + disabledString + "><br>";
        html += "</div>";

        // افزایش شمارنده
        counter++;
    }

    return html;
}

String getLocalNodeInfoAsJson()
{
    StaticJsonDocument<2048> doc; // اندازه مناسب برای ذخیره اطلاعات
    JsonArray nodesArray = doc.to<JsonArray>();

    for (const auto &[id, node] : nodeInfo)
    {
        JsonObject nodeObj = nodesArray.createNestedObject(); // اضافه کردن نود جدید به آرایه
        nodeObj["id"] = id;
        nodeObj["name"] = node.name;
        nodeObj["relayStatus"] = node.relayStatus ? "true" : "false"; // مقدار relayStatus به صورت true یا false
        nodeObj["Humidity"] = node.Humidity;
        nodeObj["Temperature"] = node.Temperature;
        nodeObj["TemperatureFahrenheit"] = node.TemperatureFahrenheit;
        nodeObj["HeatIndex"] = node.HeatIndex;
        nodeObj["HeatIndexFahrenheit"] = node.HeatIndexFahrenheit;
        nodeObj["smokValue"] = node.smokValue;
        nodeObj["gasDetected"] = node.gasDetected;
    }

    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString;
}
void SaveLog(const String &logType, long dateTime, const String &log)
{

    // todo
    //  SaveLog("INFO", 1672457693, "System started");
    //  SaveLog("ERROR", 1672457745, "Failed to connect to WiFi");
    //  SaveLog("DEBUG", 1672457800, "Memory usage: 50%");
    Serial.println("SaveLog: " + logType + " - " + String(dateTime) + " - " + log);
    String logDataFilename = "\\todo";
    // خواندن محتوای قبلی فایل
    String fileContent = "";
    File file = SPIFFS.open(logDataFilename, FILE_READ);

    if (file)
    {
        while (file.available())
        {
            fileContent += char(file.read());
        }
        file.close();
    }

    // اضافه کردن لاگ جدید
    fileContent += logType + "|" + String(dateTime) + "|" + log + "\n";

    // نوشتن محتوای جدید در فایل
    file = SPIFFS.open(logDataFilename, FILE_WRITE);
    if (file)
    {
        file.print(fileContent);
        file.close();
    }
}

void renderingPageHtml()
{
    server.on("/check_SPIFFS_Logs", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        String response = "=== SPIFFS Logs ===\n";
String logDataFilename ="\\todo";
        // بررسی وضعیت SPIFFS
        if (!SPIFFS.begin(true)) {
            response += "Failed to initialize SPIFFS.\n";
        } else {
            File file = SPIFFS.open(logDataFilename, FILE_READ);
            if (!file) {
                response += "Error: Unable to open " + String(logDataFilename) + "\n";
            } else {
                response += "=== Content of " + String(logDataFilename) + " ===\n";
                while (file.available()) {
                    response += (char)file.read(); // خواندن محتوای فایل
                }
                file.close();
            }
        }

        // ارسال پاسخ به مرورگر
        request->send(200, "text/plain", response); });

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        Serial.println("on load start....");

displayNodeInfo();
String html = "<html><head><style>";
html += "body {font-family: Arial, sans-serif;display: flex; height: 100vh;background-color: #f0f0f0;margin: 0;}";
html +=  ".left { width: 100%;   background-color: #6ec1e4;      padding-top: 20px;         margin: 0 auto;}";
html += ".right { width: 100%;   background-color: #d86ee4;                                 margin: 0 auto;"+
html += "    display: flex;";                /* فعال کردن Flexbox */
html += "    justify-content: center;";    /* تراز کردن افقی */
html += "    padding-top: 200px;";          /* فاصله از بالا */
html += "    height: 100%;";               /* ارتفاع کامل */
html += "    background-color: #d86ee4;";  /* رنگ پس‌زمینه */
html += "}";
html += ".switch-container {";
html += "    position: relative;";         /* برای امکان انتقال */
html += "    width: 100px;";
html += "    height: 200px;";
html += "    background-color: #ddd;";
html += "    border-radius: 20px;";
html += "    box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);";
html += "    cursor: pointer;";
html += "}";
html += ".switch {";
html += "    position: absolute;";
html += "    top: 100px;";
html += "    left: 20px;";
html += "    width: 60px;";
html += "    height: 160px;";
html += "    background-color: #fff;";
html += "    border-radius: 10px;";
html += "    box-shadow: inset 0 0 10px rgba(0, 0, 0, 0.1);";
html += "    transition: transform 0.3s ease, background-color 0.3s ease;";
html += "}";
html += ".switch.on {";
html += "    transform: translateY(0);";
html += "    background-color: green;";
html += "}";
html += ".switch.off {";
html += "    transform: translateY(120px);";
html += "    background-color: red;";
html += "}";

html += ".button { padding: 10px; margin: 10px; border: none; cursor: pointer; font-size: 16px; }";
html += ".green { background-color: green; color: white; }";
html += ".yellow { background-color: yellow; color: black; }";
html += ".green { background-color: green; color: black; }";
html += ".red { background-color: red; color: white; }";
html += ".black { background-color: black; color: white; }";
html += "button:disabled {background-color: grey;color: white;cursor: not-allowed;}";
html += ".switch-container {position: relative;width: 100px;height: 200px;background-color: #ddd;border-radius: 20px;box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);cursor: pointer;}";
html += ".switch {position: absolute;top: 20px;left: 20px;width: 60px;height: 160px;background-color: #fff;border-radius: 10px;box-shadow: inset 0 0 10px rgba(0, 0, 0, 0.1);transition: transform 0.3s ease, background-color 0.3s ease;}";
html += ".switch.on {transform: translateY(0);background-color: green;}";
html += ".switch.off {transform: translateY(120px);background-color: red;}"; 
html += "</style></head><body>";
String nodesString  = getLocalNodeInfoAsJson();


Serial.println(nodesString);

String part1 = R"( 
<!-- دراپ‌داون برای انتخاب نود -->
<select id='nodeSelector' onchange='loadNodeData()'>
    <option value=''>Select Node...</option>
    <!-- لیست نودها به صورت داینامیک بارگذاری می‌شود -->
</select>

<script>

    // داده‌های نودها مستقیماً اینجا قرار داده می‌شوند
    var nodeInfo = )";

    String part2=R"(;

    // این تابع برای پر کردن دراپ‌داون با استفاده از داده‌های استاتیک نودها استفاده می‌شود
    function loadNodeSelector() {
        var myNodeSelector = document.getElementById('nodeSelector');
                myNodeSelector.innerHTML = '<option value="">Select Node...</option>';


    console.log("loadNodeSelector start");

        // استفاده از داده‌های استاتیک برای پر کردن دراپ‌داون
        nodeInfo.forEach(function(node) {
    console.log("loadNodeSelector forEach : "+node.name);

            var option = document.createElement('option');
            option.value = node.id; // مقدار را ID نود قرار می‌دهیم
            debugger;

            option.textContent = node.name; // نام نود را به عنوان متن انتخاب قرار می‌دهیم
            myNodeSelector.appendChild(option); // اضافه کردن گزینه به دراپ‌داون

    console.log("end of    loadNodeSelector forEach : "+node.name);

        });
    }

    // این تابع برای بارگذاری داده‌های مربوط به نود انتخابی فراخوانی می‌شود
    function loadNodeData() {
        var nodeId = document.getElementById('nodeSelector').value;
        if (nodeId) {
            var selectedNode = nodeInfo.find(node => node.id == nodeId);
            if (selectedNode) {
                updateForm(selectedNode); // به‌روزرسانی فرم با داده‌های نود
            }
        }
    }
    // این تابع فرم را با داده‌های نود به‌روز می‌کند
function updateForm(data) {


        document.querySelector('[name="pir_1_IsEnable_chk"]').checked = nodeInfo[0].relayStatus;
        debugger;
        document.querySelector('[name="Buzzer_IsEnable_chk"]').checked = data.Buzzer_IsEnable;

        // بررسی و تنظیم چک‌باکس‌ها
        for (let i = 0; i < 4; i++) {
            const checkbox = document.querySelector(`[name="relay${i + 1}_IsEnable_chk"]`);

            if (nodeInfo[i] && nodeInfo[i].relayStatus !== undefined) {
                // اگر داده موجود است
                checkbox.checked = nodeInfo[i].relayStatus;
                checkbox.disabled = false; // فعال کردن چک‌باکس
                checkbox.style.backgroundColor = ""; // بازگرداندن رنگ به پیش‌فرض
            } else {
                // اگر داده موجود نیست یا خانه خالی است
                checkbox.checked = false; // مقدار پیش‌فرض
                checkbox.disabled = true; // غیرفعال کردن چک‌باکس
                checkbox.style.backgroundColor = "gray"; // تغییر رنگ به خاکستری
            }
        }


        document.querySelector('[name="smok_sensor_IsEnable_chk"]').checked = data.smok_sensor_IsEnable;
        document.querySelector('[name="temperature_IsEnable_chk"]').checked = data.temperature_IsEnable;

        document.querySelector('[name="smokValue"]').value = data.smokValue;
        document.querySelector('[name="gasDetected"]').value = data.gasDetected;
        document.querySelector('[name="humidity"]').value = data.Humidity;
        document.querySelector('[name="temperature"]').value = data.Temperature;
        document.querySelector('[name="temperatureFahrenheit"]').value = data.TemperatureFahrenheit;
        document.querySelector('[name="heatIndex"]').value = data.HeatIndex;
        document.querySelector('[name="heatIndexFahrenheit"]').value = data.HeatIndexFahrenheit;
    }


window.onload = function() {
    // کدهایی که می‌خواهید هنگام بارگذاری صفحه اجرا شوند
    console.log("window.onload  start!");
    loadNodeSelector();
    console.log("window.onload end !!!");
    console.log(nodeInfo[0].name);
    
        document.querySelector('[name="pir_1_IsEnable"]').value =nodeInfo[0].name;

    

};


</script>

)";


String multiLineString1 =  part1+ nodesString +String(part2.c_str());
html += "<div class='left'>";

html+=String(multiLineString1.c_str());


html+="  <p>nodes .... </p>";
for (const auto &node : nodeInfo) {
        uint32_t nodeId = node.first;
        String _nodeName = node.second.name;

        bool relayStatus = node.second.relayStatus;

        html += "<p>NodeId: " + String(nodeId) + ", NodeName: " + _nodeName + ", LED Status: " + (relayStatus ? "ON" : "OFF") + "</p>\n";
    }

html += "<h1>Device Settings</h1>";
html += "<form action='/save' method='POST'  onsubmit='updateCheckboxValues()'>";

html+= generateCheckboxHtml();


html += "<div style='background-color: #e9a71e; padding: 10px; margin-bottom: 10px; border: 1px solid #ccc;'>";
html += "<input type='hidden' name='pir_1_IsEnable' value='" + String(pir_1_IsEnable ? "true" : "false") + "'>";
html += "PIR Is Enable: <input type='checkbox' name='pir_1_IsEnable_chk' " + String(pir_1_IsEnable ? "checked" : "") + "><br>";
html += "</div>";

html += "<div style='background-color: #f1e6f1; padding: 10px; margin-bottom: 10px; border: 1px solid #ccc;'>";
html += "<input type='hidden' name='Buzzer_IsEnable' value='" + String(Buzzer_IsEnable ? "true" : "false") + "'>";
html += "Buzzer  Is Enable: <input type='checkbox' name='Buzzer_IsEnable_chk' " + String(Buzzer_IsEnable ? "checked" : "") + "><br>";
html += "</div>";


html += "<div style='background-color: #e0f7e0; padding: 10px; margin-bottom: 10px; border: 1px solid #ccc;'>";
html += "<input type='hidden' name='smok_sensor_IsEnable' value='" + String(smok_sensor_IsEnable ? "true" : "false") + "'>";
html += "Smoke Sensor Is Enable: <input type='checkbox' name='smok_sensor_IsEnable_chk' " + String(smok_sensor_IsEnable ? "checked" : "") + "><br>";
html += "</div>";

html += "<div style='background-color: #e0f7e0; padding: 10px; margin-bottom: 10px; border: 1px solid #ccc;'>";
html += "<input type='hidden' name='temperature_IsEnable' value='" + String(temperature_IsEnable ? "true" : "false") + "'>";
html += "Temperature Is Sensor Enable: <input type='checkbox' name='temperature_IsEnable_chk' " + String(temperature_IsEnable ? "checked" : "") + "><br>";
html += "</div>";

html += "<div style='background-color: #859965; padding: 10px; margin-bottom: 10px; border: 1px solid #ccc;'>";
html += "<input type='hidden' name='device_status' id='device_status' value='" + device_status + "'><br>";

html += "<div style='display: flex; justify-content: space-between; width: 100%;'>";
html += "<label for='ssid_Name' style='width: 150px;'>ssid name:</label>";
html += "<input type='text' name='ssid_Name' value='" + String(ssidName) + "'><br>";
html += "</div>";
html += "</br>";
html += "</br>";

html += "<div style='display: flex; justify-content: space-between; width: 100%;'>";
html += "<label for='ssid_Password' style='width: 150px;'>ssid  password:</label>";
html += "<input type='text' name='ssid_Password' value='" + String(ssidPassword) + "'><br>";
html += "</div>";

html += "<div style='display: flex; justify-content: space-between; width: 100%;'>";
html += "<label for='node_Name' style='width: 150px;'> node  Name:</label>";
html += "<input type='text' name='node_Name' value='" + String(nodeName) + "'><br>";
html += "</div>";

html += "<div style='display: flex; justify-content: space-between; width: 100%;'>";
html += "<label for='node_Id' style='width: 150px;'> node  Id:</label>";
html += "<input type='text' name='node_Id' value='" + String(nodeId) + "'><br>";
html += "</div>";

html += "</div>";


// ایجاد کادر سبز کم رنگ با عرض کشیده‌شده
html += "<div style='border: 2px solid #008000; background-color: #e0f7e0; padding: 10px; width: 100%; box-sizing: border-box;'>";  // div بیرونی

html += "<h3 style='text-align: center;'>Space Sensors</h3>";  // عنوان بالای کادر

// استفاده از Flexbox برای چیدمان فیلدها
html += "<div style='display: flex; flex-direction: column; gap: 10px; width: 100%;'>";  // div داخلی که از عرض 100% استفاده می‌کند

// فیلدها با تکس باکس‌های هم‌اندازه
html += "<div style='display: flex; justify-content: space-between; width: 100%;'>";
html += "<label for='smokValue' style='width: 150px;'>smokValue:</label>";
html += "<input type='text' name='smokValue' value='" + String(smokValue) + "' style='width: 150px;'>";
html += "</div>";

html += "<div style='display: flex; justify-content: space-between; width: 100%;'>";
html += "<label for='gasDetected' style='width: 150px;'>gas detected:</label>";
html += "<input type='text' name='gasDetected' value='" + String(gasDetected) + "' style='width: 150px;'>";
html += "</div>";

html += "<div style='display: flex; justify-content: space-between; width: 100%;'>";
html += "<label for='humidity' style='width: 150px;'>Humidity:</label>";
html += "<input type='text' name='humidity' value='" + String(Humidity) + "' style='width: 150px;'>";
html += "</div>";

html += "<div style='display: flex; justify-content: space-between; width: 100%;'>";
html += "<label for='temperature' style='width: 150px;'>Temperature:</label>";
html += "<input type='text' name='temperature' value='" + String(Temperature) + "' style='width: 150px;'>";
html += "</div>";

html += "<div style='display: flex; justify-content: space-between; width: 100%;'>";
html += "<label for='temperatureFahrenheit' style='width: 150px;'>TemperatureFahrenheit:</label>";
html += "<input type='text' name='temperatureFahrenheit' value='" + String(TemperatureFahrenheit) + "' style='width: 150px;'>";
html += "</div>";

html += "<div style='display: flex; justify-content: space-between; width: 100%;'>";
html += "<label for='heatIndex' style='width: 150px;'>HeatIndex:</label>";
html += "<input type='text' name='heatIndex' value='" + String(HeatIndex) + "' style='width: 150px;'>";
html += "</div>";

html += "<div style='display: flex; justify-content: space-between; width: 100%;'>";
html += "<label for='heatIndexFahrenheit' style='width: 150px;'>HeatIndexFahrenheit:</label>";
html += "<input type='text' name='heatIndexFahrenheit' value='" + String(HeatIndexFahrenheit) + "' style='width: 150px;'>";
html += "</div>";

html += "</div>";  // بستن Flexbox (div داخلی)
html += "</div>";  // بستن کادر (div بیرونی)



html += "<input type='submit'  class='button green' value='Save Settings'>";
html += "</form>"; 

html += "<form action='/restartDevice' method='POST'>";
html += "<input type='submit' class='button yellow'  value=' restart '>";
html += "</form>";

html += "<form action='/resetFactory' method='POST'>";
html += "<input type='submit' class='button red'  value=' reset Factory Settings'>";
html += "</form>";


html += "<form action='/check_SPIFFS_Mem' method='POST'>";
html += "<input type='submit' class='button green'  value=' check_SPIFFS_Mem'>";
html += "</form>";

html += "<form action='/send_message' method='GET'>";
html += "<input type='submit' class='button green'  value=' send message'>";
html += "</form>";

// مقدار پیش‌فرض وضعیت دستگاه
html += "<div class='right'>";
html += "<h2 style='display: block; margin: 10px 0;'>Device Status</h2>";  // نمایش در یک خط جداگانه و اضافه کردن فاصله
html += "<div class='switch-container' onclick='toggleSwitch()'>";
if (device_status == "1") {
    html += "<div id='switch' class='switch on'></div>";  // وضعیت روشن
} else {
    html += "<div id='switch' class='switch off'></div>";  // وضعیت خاموش
}
html += "</div>";
html += "</div>";


html += "<script>";

html += "function toggleSwitch() {";
html += "  const switchElement = document.getElementById('switch');";
html += "  if (switchElement.classList.contains('on')) {";
html += "    switchElement.classList.remove('on');";
html += "    switchElement.classList.add('off');";
html += "    alert('Device Off');";
html += "    document.getElementById('device_status').value='0';";
html += "  } else {";
html += "    switchElement.classList.remove('off');";
html += "    switchElement.classList.add('on');";
html += "    alert('Device On');";
html += "    document.getElementById('device_status').value='1';";
html += "  }";
html += "}";

html += "function updateCheckboxValues() {";
html += "  var value_pir_1 = document.querySelector('input[name=\"pir_1_IsEnable_chk\"]').checked ? 'true' : 'false';";
html += "  document.querySelector('input[name=\"pir_1_IsEnable\"]').value = value_pir_1;";

html += "  var value_relay1_IsEnable = document.querySelector('input[name=\"relay1_IsEnable_chk\"]').checked ? 'true' : 'false';";
html += "  document.querySelector('input[name=\"relay1_IsEnable\"]').value = value_relay1_IsEnable;";

html += "  var value_relay2_IsEnable = document.querySelector('input[name=\"relay2_IsEnable_chk\"]').checked ? 'true' : 'false';";
html += "  document.querySelector('input[name=\"relay2_IsEnable\"]').value = value_relay2_IsEnable;";

html += "  var value_relay3_IsEnable = document.querySelector('input[name=\"relay3_IsEnable_chk\"]').checked ? 'true' : 'false';";
html += "  document.querySelector('input[name=\"relay3_IsEnable\"]').value = value_relay3_IsEnable;";

html += "  var value_relay4_IsEnable = document.querySelector('input[name=\"relay4_IsEnable_chk\"]').checked ? 'true' : 'false';";
html += "  document.querySelector('input[name=\"relay4_IsEnable\"]').value = value_relay4_IsEnable;";

html += "  var value_smok_sensor = document.querySelector('input[name=\"smok_sensor_IsEnable_chk\"]').checked ? 'true' : 'false';";
html += "  document.querySelector('input[name=\"smok_sensor_IsEnable\"]').value = value_smok_sensor;";

html += "  var value_temperature = document.querySelector('input[name=\"temperature_IsEnable_chk\"]').checked ? 'true' : 'false';";
html += "  document.querySelector('input[name=\"temperature_IsEnable\"]').value = value_temperature;";

html += "  var value_Buzzer_IsEnable = document.querySelector('input[name=\"Buzzer_IsEnable_chk\"]').checked ? 'true' : 'false';";
html += "  document.querySelector('input[name=\"Buzzer_IsEnable\"]').value = value_Buzzer_IsEnable;";

html += "}";




html += "function toggleSwitch() {";
html += "  const switchElement = document.getElementById('switch');";
html += "  if (switchElement.classList.contains('on')) {";
html += "    switchElement.classList.remove('on');";
html += "    switchElement.classList.add('off');";
html += "    alert('Device Off');";
html += "   document.getElementById('device_status').value='0';";

html += "  } else {";
html += "    switchElement.classList.remove('off');";
html += "    switchElement.classList.add('on');";
html += "   document.getElementById('device_status').value='1';";

html += "    alert('Device On');";
html += "  }";
html += "}";

html += "</script>";

html += "</body></html>";


        request->send(200, "text/html", html); });

    // مسیر برای آپدیت وضعیت دستگاه
    server.on("/updateDeviceStatus", HTTP_POST, [](AsyncWebServerRequest *request)
              {
        String status = request->arg("device_status");
        if (status == "1") {
            device_status = "1";
        } else if (status == "0") {
            device_status = "0";
        }
        SaveVariablesInDb();
        request->send(200, "text/plain", "Device status updated"); });

    // مسیر برای ریست کردن تنظیمات به کارخانه
    server.on("/resetFactory", HTTP_POST, [](AsyncWebServerRequest *request)
              {
                  Serial.println("Resetting to factory settings...");
                  request->send(200, "text/plain", "Device reset to factory settings.");
                  resetFactoryConfig(); // فراخوانی تابع ریست به تنظیمات کارخانه
              });
    // مسیر برای ریست کردن تنظیمات به کارخانه
    server.on("/check_SPIFFS_Mem", HTTP_POST, [](AsyncWebServerRequest *request)
              {
    String response = "=== SPIFFS Status ===\n";

    // بررسی وضعیت SPIFFS
    if (!SPIFFS.begin(true)) {
        response += "Failed to initialize SPIFFS.\n";
    } else {
        size_t totalBytes = SPIFFS.totalBytes();
        size_t usedBytes = SPIFFS.usedBytes();
        size_t freeBytes = totalBytes - usedBytes;

        response += "Total Space: " + String(totalBytes) + " bytes\n";
        response += "Used Space: " + String(usedBytes) + " bytes\n";
        response += "Free Space: " + String(freeBytes) + " bytes\n\n";

        // خواندن محتوای فایل
        File file = SPIFFS.open("/appConfigDataFile.txt", FILE_READ);
        if (!file) {
            response += "Error: Unable to open /appConfigDataFile.txt\n";
        } else {
            response += "=== Content of /appConfigDataFile.txt ===\n";
            while (file.available()) {
                response += (char)file.read(); // خواندن محتوای فایل
            }
            file.close();
        }
    }

    // ارسال پاسخ به مرورگر
    request->send(200, "text/plain", response); });

    // صفحه‌ای برای نمایش داده‌های SPIFFS
    server.on("/show_SPIFFS_data", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    String data = request->getParam("data")->value();

    String html = "<!DOCTYPE html><html lang=\"fa\"><head><meta charset=\"UTF-8\"><title>SPIFFS Data</title></head><body>";
    html += "<h1>SPIFFS Data</h1>";
    html += "<pre>" + data + "</pre>";
    html += "</body></html>";

    // ارسال HTML برای نمایش داده‌ها
    request->send(200, "text/html", html); });

    server.on("/send_message", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    // صفحه ورودی برای ارسال پیام
    String html = "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\">";
    html += "<title>Send Message</title></head><body>";
    html += "<h1>Send Message</h1>";
    html += "<form action='/send__message' method='POST'>";
    html += "<label for='number'>Phone Number:</label><br>";
    html += "<input type='text' id='number' name='number' value='+989335017186'><br><br>";
    html += "<label for='message'>Message:</label><br>";
    html += "<input type='text' id='message' name='message' value='message from esp32 web'><br><br>";
    html += "<input type='submit' class='button green' value='Send Message'>";
    html += "</form>";
    html += "</body></html>";
    request->send(200, "text/html", html); });

    server.on("/send__message", HTTP_POST, [](AsyncWebServerRequest *request)
              {
    String number = "";
    String message = "";

    // خواندن داده‌ها از فرم
    if (request->hasParam("number", true)) {
        number = request->getParam("number", true)->value();
    }
    if (request->hasParam("message", true)) {
        message = request->getParam("message", true)->value();
    }

    // ارسال پیام
    String response = sendStartupSMS(number, message);

    // ریدایرکت به صفحه نمایش نتیجه
    request->redirect("/message_result?result=" + urlencode(response)); });

    // صفحه نمایش نتیجه
    server.on("/message_result", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    String result = request->getParam("result")->value();
    String html = "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\">";
    html += "<title>Message Result</title></head><body>";
    html += "<h1>Message Result</h1>";
    html += "<pre>" + result + "</pre>";
    html += "<a href='/send_message'>Back</a>";
    html += "</body></html>";
    request->send(200, "text/html", html); });

    // مسیر برای ریست کردن دستگاه
    server.on("/restartDevice", HTTP_POST, [](AsyncWebServerRequest *request)
              {
        Serial.println("restartDevice...");
        ESP.restart(); // ریست کردن ESP32
        request->send(200, "text/plain", "restartDevice... please wait..."); });

    server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request)
              {
    // دریافت و ذخیره اطلاعات فرم
    String pir_1 = request->arg("pir_1_IsEnable");
    String pir_2 = request->arg("pir_2_IsEnable");
    String pir_3 = request->arg("pir_3_IsEnable");
    String buz = request->arg("Buzzer_IsEnable");
    String relay1 = request->arg("relay1_IsEnable");
    String relay2 = request->arg("relay2_IsEnable");
    String relay3 = request->arg("relay3_IsEnable");
    String relay4 = request->arg("relay4_IsEnable");

    String zone_1 = request->arg("zone_1_IsEnable");
    String zone_2 = request->arg("zone_2_IsEnable");

    String smok_sensor = request->arg("smok_sensor_IsEnable");
    String temperature = request->arg("temperature_IsEnable");

    String device_status_value = request->arg("device_status");

    // مدیریت مقادیر چک‌باکس‌ها
    pir_1_IsEnable = pir_1 !="false"; 
    Buzzer_IsEnable = buz !="false";
    relay1_IsEnable = relay1 !="false"; 
    relay2_IsEnable = relay2 !="false"; 
    relay3_IsEnable = relay3 !="false"; 
    relay4_IsEnable = relay4 !="false"; 

    smok_sensor_IsEnable = smok_sensor !="false";
    temperature_IsEnable = temperature !="false";

    // مدیریت وضعیت دستگاه
    if (device_status_value == "") {
        device_status = 1;  // مقدار پیش‌فرض
    } else {
        device_status = device_status_value;
    }

    // مدیریت مقادیر SSID
    ssidName = request->arg("ssid_Name");
    ssidPassword = request->arg("ssid_Password");
    nodeName = request->arg("node_Name");

    // ذخیره در پایگاه داده
    SaveVariablesInDb();
    SendRelayCommand();
    // انتقال کاربر به صفحه اصلی
    // request->redirect("/");

        // request->send(200, "text/html", "<h1>Settings Saved!</h1>");
// ارسال پاسخ HTML به همراه اسکریپت جاوااسکریپت برای ریدایرکت
    String response = "<html><body>";
    response += "<h1>Settings Saved!</h1>";
    response += "<script>";
    response += "setTimeout(function() { window.location.href = '/'; }, 1000);"; // ریدایرکت پس از ۱ ثانیه
    response += "</script>";
    response += "</body></html>";

    request->send(200, "text/html", response); });
}
#pragma mesh

void localLedTask(int count)
{
    // for (int i = 0; i < count * 2; i++)
    // { // استفاده از LOCAL_LED_BLINK_COUNT

    //     digitalWrite(LOCAL_ALERT_PIN, HIGH);
    //     if (Buzzer_IsEnable)
    //         digitalWrite(buzzer_pin, HIGH);
    //     // Serial.println("Local buzzer ON (buzzer_pin)");
    //     vTaskDelay(150); // تأخیر بین هر تلاش
    //     digitalWrite(LOCAL_ALERT_PIN, LOW);
    //     if (Buzzer_IsEnable)
    //         digitalWrite(buzzer_pin, LOW);

    //     // Serial.println("Local buzzer OFF (buzzer_pin)");
    //     vTaskDelay(150); // تأخیر بین هر تلاش
    // }
}

// تابع ارسال پیام هشدار به نودهای دیگر
void sendAlertMessage()
{
    String _nodeId = nodeId = mesh.getNodeId(); // شماره نود
    String message = "{\"nodename\":\"" + nodeName + "\", \"nodeid\":" + String(nodeId) + ", \"message\":\"alert\"}";
    // Serial.println("Sending alert message: " + message);
    broadcastToAll(message); // ارسال پیام هشدار به همه نودها
}

void triggerRemoteLed()
{
    for (int i = 0; i < REMOTE_LED_BLINK_COUNT; i++)
    {
        // روشن و خاموش کردن LED ریموت
        digitalWrite(REMOTE_ALERT_PIN, !digitalRead(REMOTE_ALERT_PIN));
        Serial.println(digitalRead(REMOTE_ALERT_PIN) ? "Remote LED ON (Pin 4)" : "Remote LED OFF (Pin 4)");
        lastRemoteLedToggle = millis(); // زمان آخرین تغییر وضعیت LED

        vTaskDelay(REMOTE_LED_DELAY); // تأخیر بین هر تغییر (برای جلوگیری از خاموش شدن سریع LED)
    }
}

int pirCount = 0;
unsigned long lastPirUpdate = 0; // زمان آخرین تغییر در pirCount
unsigned long lastResetTime = 0; // زمان آخرین ریست شدن pirCount

const int adcResolution = 12; // دقت ADC
float voltage;

void checkAllPIR()
{
    pir_1_IsEnable = true; /// این خاط رو پاک کن پاک
    int separatorIndex = device_status.indexOf("1");
    if (separatorIndex == -1 && false)
    {
        // Serial.println("device is disarm");
        digitalWrite(LOCAL_ALERT_PIN, HIGH);
        vTaskDelay(250);
        digitalWrite(LOCAL_ALERT_PIN, LOW);
        vTaskDelay(250);
        digitalWrite(LOCAL_ALERT_PIN, HIGH);
        vTaskDelay(250);
        digitalWrite(LOCAL_ALERT_PIN, LOW);
        vTaskDelay(250);
        digitalWrite(LOCAL_ALERT_PIN, HIGH);
        vTaskDelay(250);
        digitalWrite(LOCAL_ALERT_PIN, LOW);
        vTaskDelay(5000);
        return;
    }
    else
    {
        // Serial.println("device is arm");
    }
    if (pir_1_IsEnable)
    {
        unsigned long currentMillis1 = millis(); // زمان جاری

        int pirStatus1 = digitalRead(pir_1_pin); // خواندن وضعیت سنسور PIR

        if (pirStatus1 == HIGH)
        {

            vTaskDelay(50);
            pirCount++;
            lastPirUpdate = currentMillis1;
            Serial.println("pir 1 Count  : pin 18 :    " + String(pirCount));
            if (pirCount >= 3)
            {
                AllarmError(2, 2, 3);
                pirCount = 0;
                lastResetTime = currentMillis1;
            }
        }

        if (currentMillis1 - lastResetTime >= 3000 && currentMillis1 - lastPirUpdate >= 1000)
        {
            pirCount = 0;
            lastResetTime = currentMillis1;
            // Serial.println("pirCount1 reset due to inactivity.: pin 13 ");
        }
    }
}

unsigned long lastSuccessTime = 0; // زمان آخرین موفقیت
unsigned long retryDelay = 3;
unsigned long lastAttemptTime = 0; // زمان آخرین تلاش
int reconnectAttempts2 = 0;        // شمارنده تلاش‌ها
int maxAttempts = 10;              // حداکثر تعداد تلاش‌ها

bool reconnectMesh()
{
    mesh.update();

    Serial.println("Attempting to reconnect to the mesh network...");

    if (millis() - lastSuccessTime < retryDelay * 1000)
    {
        Serial.println("Mesh is already connected, no need to reconnect.");
        return true;
    }

    int attemptCount = 0;
    while (attemptCount < maxAttempts)
    {
        if (mesh.getNodeId() > 0)
        {
            int meshSizeTemp = mesh.getNodeList().size();
            if (meshSizeTemp >= 1)
            {
                uint32_t nodeId1 = mesh.getNodeId(); // ID اولین نود

                for (uint32_t nodeId2 : mesh.getNodeList())
                {

                    if (nodeId1 != nodeId2) // مطمئن شوید که نود خودش نیست
                    {
                        if (!mesh.isConnected(nodeId2)) // چک کنید که آیا نود قبلاً به نود دیگر وصل نشده است
                        {
                            Serial.println("try to Connected to node: " + String(nodeId2));

                            // mesh.connect(nodeId2); // اتصال خودکار به نود دیگر
                        }
                        else
                        {
                            Serial.println("is Connected to node: " + String(nodeId2));
                        }
                    }
                    Serial.println("in reconnectMesh while   mesh size: " + String(meshSizeTemp) + " .... Node id: " + String(nodeId));
                }
            }
            else
            {
                Serial.println("in reconnectMesh while .... just me One Node id:   " + String(mesh.getNodeId()));

                vTaskDelay(1000);
                mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
                mesh.init(ssidName, ssidPassword, &userScheduler, 5555);
                vTaskDelay(1000);

                Serial.println("in reconnectMesh while .... just me On receive just me On receive   " + String(mesh.getNodeId()));

                mesh.onReceive(&receivedCallback); // ثبت Callback یکبار
                mesh.onNewConnection(&onNewConnection);
                mesh.onDroppedConnection([](uint32_t nodeId)
                                         { Serial.println("Connection dropped with node: " + String(nodeId)); });
            }

            lastSuccessTime = millis();
            reconnectAttempts2 = 0; // ریست شمارنده تلاش‌ها
            return true;
        }
        else
        {
            Serial.println("Reconnecting...         :::::::>>>>>>>>>>          " + String(attemptCount));

            mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
            mesh.init(ssidName, ssidPassword, &userScheduler, 5555);
            mesh.onReceive(&receivedCallback); // ثبت Callback یکبار
            mesh.onNewConnection(&onNewConnection);
            mesh.onDroppedConnection([](uint32_t nodeId)
                                     { Serial.println("Connection dropped with node: " + String(nodeId)); });

            server.begin();
            delay(1000); // تأخیر برای پایداری بیشتر
        }
        attemptCount++;
    }

    Serial.println("Failed to connect after maximum attempts.");
    reconnectAttempts2++;
    if (reconnectAttempts2 >= 50)
    {
        Serial.println("Restarting ESP...");
        ESP.restart();
    }

    return false;
}

// تسک‌ها

void reconnectMeshTask(void *parameter)
{
    Serial.println("start task reconnectMeshTask ,before return");
    if (reconnectMeshTaskRunning)
        return; // Prevent re-running task
    Serial.println("start task reconnectMeshTask ,after return");

    reconnectMeshTaskRunning = true;

    while (true)
    {
        Serial.println("task reconnectMeshTask while");

        reconnectMesh();

        Serial.println("broadcastLocalDatabase            in        reconnectMeshTask while");
        broadcastLocalDatabase();
        vTaskDelay(1000);

        Serial.println("sendRequestNodeInfoToAllNodes            in        reconnectMeshTask while");
        sendRequestNodeInfoToAllNodes();
        vTaskDelay(1000);

        vTaskDelay(RECONNECT_DELAY); // Delay between retries
    }
    reconnectMeshTaskRunning = false; // Task complete

    Serial.println("task reconnectMeshTask complete");
}

void checkAllPirTask(void *parameter)
{
    Serial.println("start task checkPIRTask, before return");
    if (checkPIR1TaskRunning)
    {
        // Serial.println("checkPIRTask is already running, returning");
        return; // Prevent re-running task
    }
    Serial.println("start task checkPIRTask, after return");

    checkPIR1TaskRunning = true;

    while (true)
    {

        checkAllPIR();

        vTaskDelay(PIR_TRIGGER_TIME); // Delay to avoid too frequent checking
    }

    checkPIR1TaskRunning = false; // Task complete
    Serial.println("task checkPIRTask complete");
}

#pragma end mesh

#pragma region remoteButtonTask

// void remoteButtonTask(void *parameter)
// {
//     vTaskDelay(5000); // تأخیر کوتاه برای جلوگیری از بار پردازشی

//     for (;;)
//     {
//         // خواندن وضعیت فعلی دکمه
//         bool currentButtonState0 = digitalRead(Remote_Key_0_Pin);

//         // اگر دکمه فشار داده شده است (HIGH)
//         if (currentButtonState0 == HIGH && !buttonPressedFlags[0])
//         {
//             buttonPressedFlags[0] = true;
//             relayStates[0] = !relayStates[0];

//             if (device_status == "1")
//                 device_status = "0";
//             else if (device_status == "0")
//                 device_status = "1";
//             saveData("device_status", String(device_status));

//             Serial.println("device status is :::::::::::::::::::::::::::::   " + device_status);
//         }

//         // ریست پرچم اگر دکمه دیگر فشار داده نشده باشد
//         if (currentButtonState0 == LOW)
//         {
//             buttonPressedFlags[0] = false;
//         }

//         vTaskDelay(10 / portTICK_PERIOD_MS); // تأخیر کوتاه برای جلوگیری از بار پردازشی

//         bool currentButtonState1 = digitalRead(Remote_Key_1_Pin);

//         // اگر دکمه فشار داده شده است (HIGH)
//         if (currentButtonState1 == HIGH && !buttonPressedFlags[1])
//         {
//             buttonPressedFlags[1] = true;                        // پرچم تنظیم شود
//             relay1_IsEnable = relayStates[1] = !relay1_IsEnable; // وضعیت رله تغییر کند
//             digitalWrite(Relay_1_pin, relay1_IsEnable);          // تنظیم وضعیت رله
//             Serial.print("Relay 1 toggled to: ");
//             Serial.println(relayStates[0] ? "ON" : "OFF");
//         }

//         // ریست پرچم اگر دکمه دیگر فشار داده نشده باشد
//         if (currentButtonState1 == LOW)
//         {
//             buttonPressedFlags[1] = false;
//         }

//         bool currentButtonState2 = digitalRead(Remote_Key_2_Pin);

//         // اگر دکمه فشار داده شده است (HIGH)
//         if (currentButtonState2 == HIGH && !buttonPressedFlags[2])
//         {
//             buttonPressedFlags[2] = true;     // پرچم تنظیم شود
//             relayStates[2] = !relayStates[2]; // وضعیت رله تغییر کند
//             // digitalWrite(2, relayStates[2]); // تنظیم وضعیت رله

//             Buzzer_IsEnable = !Buzzer_IsEnable;

//             Serial.print("currentButtonState  2222222222222222 toggled to: ");
//             Serial.println((relayStates[2] ? "ON___" : "OFF___") + Buzzer_IsEnable);
//         }

//         // ریست پرچم اگر دکمه دیگر فشار داده نشده باشد
//         if (currentButtonState2 == LOW)
//         {
//             buttonPressedFlags[2] = false;
//         }

//         bool currentButtonState3 = digitalRead(Remote_Key_3_Pin);

//         // اگر دکمه فشار داده شده است (HIGH)
//         if (currentButtonState3 == HIGH && !buttonPressedFlags[3])
//         {
//             buttonPressedFlags[3] = true;     // پرچم تنظیم شود
//             relayStates[3] = !relayStates[3]; // وضعیت رله تغییر کند
//             // digitalWrite(1111, relayStates[3]); // تنظیم وضعیت رله
//             Serial.print("currentButtonState    33333333333 toggled to: ");
//             Serial.println(relayStates[3] ? "ON" : "OFF");
//         }

//         // ریست پرچم اگر دکمه دیگر فشار داده نشده باشد
//         if (currentButtonState3 == LOW)
//         {
//             buttonPressedFlags[3] = false;
//         }
//     }
// }

#pragma endregion

void displayNodeInfo()
{
    for (const auto &node : nodeInfo)
    {
        uint32_t nodeId = node.first;
        String _nodeName = node.second.name;

        bool relayStatus = node.second.relayStatus;

        Serial.println("<p>NodeId: " + String(nodeId) + ", NodeName:::: " + _nodeName + ", LED Status: " + (relayStatus ? "ON" : "OFF") + "</p>\n");
    }
}

String extractJsonValue(const String &msg, const String &key)
{
    // ایجاد یک سند JSON با اندازه مناسب
    DynamicJsonDocument sharedDoc(1024);

    // دی‌سریالایز کردن JSON
    DeserializationError error = deserializeJson(sharedDoc, msg);
    if (error)
    {
        Serial.println("JSON parsing failed: " + String(error.c_str()));
        return "";
    }

    // تابع بازگشتی برای پیدا کردن مقدار کلید
    std::function<String(JsonVariant, const String &)> findKeyValue = [&](JsonVariant node, const String &searchKey) -> String
    {
        if (node.is<JsonObject>())
        {
            for (JsonPair keyValue : node.as<JsonObject>())
            {
                // مقایسه درست با تبدیل به String
                if (String(keyValue.key().c_str()) == searchKey)
                {
                    return keyValue.value().as<String>();
                }
                // بررسی تو در تو
                String nestedResult = findKeyValue(keyValue.value(), searchKey);
                if (!nestedResult.isEmpty())
                {
                    return nestedResult;
                }
            }
        }
        else if (node.is<JsonArray>())
        {
            for (JsonVariant arrayElement : node.as<JsonArray>())
            {
                String nestedResult = findKeyValue(arrayElement, searchKey);
                if (!nestedResult.isEmpty())
                {
                    return nestedResult;
                }
            }
        }
        return "";
    };

    // جستجوی کلید
    String result = findKeyValue(sharedDoc.as<JsonVariant>(), key);
    if (result.isEmpty())
    {
        Serial.println("Key not found: " + key);
    }
    else
    {
        Serial.println("Value found for key: " + key + " : " + result);
    }
    return result;
}

std::map<String, String> deserializeCustomFormat(const String &msg)
{
    std::map<String, String> result;

    // حذف شروع و پایان آبجکت
    String content = msg.substring(5, msg.length() - 5);

    // جدا کردن فیلدها
    int startIndex = 0;
    while (true)
    {
        int fieldEnd = content.indexOf(",,,,,", startIndex);
        if (fieldEnd == -1)
        {
            fieldEnd = content.length();
        }

        String field = content.substring(startIndex, fieldEnd);
        int separatorIndex = field.indexOf("=====");
        if (separatorIndex != -1)
        {
            String key = field.substring(0, separatorIndex);
            String value = field.substring(separatorIndex + 5);
            result[key] = value;
        }

        if (fieldEnd == content.length())
        {
            break;
        }
        startIndex = fieldEnd + 5;
    }

    return result;
}

void UpdateNodeData(uint32_t nodeId)
{
    // بررسی اینکه آیا نود در لیست موجود است یا نه
    if (nodeInfo.find(nodeId) != nodeInfo.end())
    {
        NodeData &nodeData = nodeInfo[nodeId]; // دسترسی به نود موجود

        nodeData.name = nodeName;
        // آپدیت داده‌های نود
        nodeData.relayStatus = relay1_IsEnable;                 // تغییر وضعیت رله‌ها
        nodeData.Humidity = Humidity;                           // تغییر رطوبت
        nodeData.Temperature = Temperature;                     // تغییر دما
        nodeData.TemperatureFahrenheit = TemperatureFahrenheit; // تغییر دما (فارنهایت)
        nodeData.HeatIndex = HeatIndex;                         // تغییر شاخص حرارت
        nodeData.HeatIndexFahrenheit = HeatIndexFahrenheit;     // تغییر شاخص حرارت (فارنهایت)
        nodeData.smokValue = smokValue;                         // تغییر مقدار گاز
        nodeData.gasDetected = gasDetected;                     // تغییر وضعیت شناسایی گاز

        Serial.printf("NodeId %u updated in nodeInfo\n", nodeId);
    }
    else
    {
        Serial.printf("NodeId %u not found in nodeInfo\n", nodeId);
    }
}

String getValueForKey(const String &msg, const String &key)
{
    // حذف شروع و پایان آبجکت
    String content = msg.substring(5, msg.length() - 5);

    // جستجوی کلید
    String searchKey = key + "=====";
    int keyIndex = content.indexOf(searchKey);
    if (keyIndex == -1)
    {
        return ""; // کلید پیدا نشد
    }

    // موقعیت مقدار بعد از "====="
    int valueStart = keyIndex + searchKey.length();
    int valueEnd = content.indexOf(",,,,,", valueStart);
    if (valueEnd == -1)
    {
        valueEnd = content.length(); // آخرین مقدار در رشته
    }

    // استخراج مقدار کلید
    return content.substring(valueStart, valueEnd);
}

void removeInactiveNodes()
{
    // unsigned long currentMillis = millis();
    // unsigned long currentMinute = currentMillis / 60000;

    // for (auto it = nodeInfo.begin(); it != nodeInfo.end();)
    // {
    //     if (currentMinute - it->second.lastLiveMinute > 3)
    //     {
    //         Serial.println("Removing inactive node: " + it->second.name);
    //         it = nodeInfo.erase(it); // حذف نود غیرفعال
    //     }
    //     else
    //     {
    //         ++it;
    //     }
    // }
}
void receiveNodeInfo(const String &jsonString)
{

    // دیسریالایز کردن داده
    DeserializationError error = deserializeJson(sharedDoc, jsonString);

    if (error)
    {
        Serial.println("Failed to parse JSON: " + String(error.c_str()));
        return;
    }

    // دسترسی به آرایه "nodes"
    JsonArray nodesArray = sharedDoc["nodes"].as<JsonArray>();

    if (nodesArray.isNull())
    {
        Serial.println("No 'nodes' array found in JSON.");
        return;
    }
    unsigned long currentMillis = millis();

    // پیمایش آرایه "nodes"
    for (JsonObject nodeObj : nodesArray)
    {
        uint32_t __idTemp = nodeObj["id"];
        String __nameTemp = nodeObj["name"];
        Serial.println("Processing node: " + __nameTemp);

        String relayStateTemp = nodeObj["relayStatus"];

        bool __relayStatusTemp = (relayStateTemp == "true" ? true : false);
        float __humidityTemp = nodeObj["Humidity"] | 0.0; // مقدار پیش‌فرض
        float __temperatureTemp = nodeObj["Temperature"] | 0.0;
        float __temperatureFahrenheitTemp = nodeObj["TemperatureFahrenheit"] | 0.0;
        float __heatIndexTemp = nodeObj["HeatIndex"] | 0.0;
        float __heatIndexFahrenheitTemp = nodeObj["HeatIndexFahrenheit"] | 0.0;
        float __smokValueTemp = nodeObj["smokValue"] | 0.0;
        int __gasDetectedTemp = nodeObj["gasDetected"] | 0;

        // بررسی اینکه آیا نود موجود است یا باید اضافه شود
        if (nodeInfo.find(__idTemp) != nodeInfo.end())
        {
            Serial.println("Updating existing node.");
            // نود موجود است، آپدیت می‌شود
            NodeData &existingNode = nodeInfo[__idTemp];
            existingNode.name = __nameTemp;
            existingNode.relayStatus = __relayStatusTemp;
            existingNode.Humidity = __humidityTemp;
            existingNode.Temperature = __temperatureTemp;
            existingNode.TemperatureFahrenheit = __temperatureFahrenheitTemp;
            existingNode.HeatIndex = __heatIndexTemp;
            existingNode.HeatIndexFahrenheit = __heatIndexFahrenheitTemp;
            existingNode.smokValue = __smokValueTemp;
            existingNode.gasDetected = __gasDetectedTemp;
            existingNode.lastLiveMinute = currentMillis / 60000; // به‌روزرسانی زمان فعالیت
        }
        else
        {
            Serial.println("Adding new node.");
            // نود جدید است، اضافه می‌شود
            NodeData newNode = {__idTemp, __nameTemp, __relayStatusTemp, __humidityTemp,
                                __temperatureTemp, __temperatureFahrenheitTemp, __heatIndexTemp,
                                __heatIndexFahrenheitTemp, __smokValueTemp, __gasDetectedTemp, currentMillis};

            nodeInfo[__idTemp] = newNode;
        }
    }

    // چاپ نودهای به‌روز شده
    Serial.println("Updated node information:");
    for (const auto &nodePair : nodeInfo)
    {
        const NodeData &node = nodePair.second;
        Serial.println("Node ID: " + String(node.id) + ", Name: " + node.name + ", Temperature: " + String(node.Temperature));
    }

    // سریالایز کردن داده‌ها
    DynamicJsonDocument outputDoc(2048);
    JsonArray updatedNodes = outputDoc.createNestedArray("nodes");

    for (const auto &nodePair : nodeInfo)
    {
        JsonObject nodeObj = updatedNodes.createNestedObject();
        const NodeData &node = nodePair.second;

        nodeObj["id"] = node.id;
        nodeObj["name"] = node.name;
        nodeObj["relayStatus"] = node.relayStatus ? "true" : "false";
        nodeObj["Humidity"] = node.Humidity;
        nodeObj["Temperature"] = node.Temperature;
        nodeObj["TemperatureFahrenheit"] = node.TemperatureFahrenheit;
        nodeObj["HeatIndex"] = node.HeatIndex;
        nodeObj["HeatIndexFahrenheit"] = node.HeatIndexFahrenheit;
        nodeObj["smokValue"] = node.smokValue;
        nodeObj["gasDetected"] = node.gasDetected;
    }

    // سریالایز کردن داده‌ها و چاپ آن
    String serializedString;
    serializeJson(outputDoc, serializedString);
    Serial.println("Serialized Node Info: " + serializedString);

    // removeInactiveNodes();
    // ارسال داده‌ها به همه
    // broadcastToAll(serializedString);
}

void LogCommandError(int nodeIdMesh, String Command)
{

    Serial.println("LogCommandError" + nodeIdMesh + String("    Command:    ") + Command);
}
int broadcastCommandCount = 0;

void ExecuteCommand(String &msg)
{
    StaticJsonDocument<512> doct;
    DeserializationError error = deserializeJson(doct, msg);

    if (error)
    {
        Serial.println("Error parsing RELAY_COMMAND message!");
        return;
    }

    bool commandFound = false;
    int counterLocal = 1;

    // جستجوی کامند در RelayOnNode
    for (const auto &[relayNodeIdtemp, counterInLoop] : RelayOnNode)
    {
        if (counterLocal == counterInLoop)
        {
            counterLocal++;
            commandFound = true;

            // بررسی اینکه آیا این گره هدف هست
            if (String(mesh.getNodeId()) == relayNodeIdtemp)
            {
                // کنترل وضعیت پین‌ها بر اساس دستورات

                // دریافت وضعیت رله‌ها از پیام دریافتی
                bool relay1Status = doct["relay1"] == 1;
                bool relay2Status = doct["relay2"] == 1;
                bool relay3Status = doct["relay3"] == 1;
                bool relay4Status = doct["relay4"] == 1;

                Serial.println("relay1Status: " + relay1Status + String("  relay2Status: " + relay2Status) + String("  relay3Status: " + relay3Status) + String("  relay4Status: " + relay4Status));

                // تغییر وضعیت پین‌ها بر اساس وضعیت رله‌ها
                if (relay1Status)
                {
                    relay1_IsEnable = true;

                    digitalWrite(Relay_1_pin, HIGH); // روشن کردن رله 1
                }
                else
                {
                    relay1_IsEnable = false;

                    digitalWrite(Relay_1_pin, LOW); // خاموش کردن رله 1
                }

                if (relay2Status)
                {
                    relay2_IsEnable = true;

                    digitalWrite(Relay_1_pin, HIGH); // روشن کردن رله 2
                }
                else
                {
                    relay2_IsEnable = false;

                    digitalWrite(Relay_1_pin, LOW); // خاموش کردن رله 2
                }

                if (relay3Status)
                {
                    relay3_IsEnable = true;

                    digitalWrite(Relay_1_pin, HIGH); // روشن کردن رله 3
                }
                else
                {
                    relay3_IsEnable = false;

                    digitalWrite(Relay_1_pin, LOW); // خاموش کردن رله 3
                }

                if (relay4Status)
                {
                    relay4_IsEnable = true;
                    digitalWrite(Relay_1_pin, HIGH); // روشن کردن رله 4
                }
                else
                {
                    relay4_IsEnable = false;

                    digitalWrite(Relay_1_pin, LOW); // خاموش کردن رله 4
                }
            }
            break;
        }
        else
        {
            Serial.println("counterLocal != counter: counterLocal: " + String(counterLocal) + " counterInLoop: " + String(counterInLoop));
        }
    } // end of for loop

    SaveVariablesInDb();
    broadcastCommandCount++;
    if (broadcastCommandCount < 3)
    {
        broadcastToAll(msg);
        broadcastCommandCount = 0;
    }

    // اگر کامند اشتباه بود، لاگ خطا ثبت شود
    if (!commandFound)
    {
        // فراخوانی تابع LogCommandError در صورت وجود خطا در کامند
        LogCommandError(mesh.getNodeId(), "Invalid command for node");
    }
}
int receivedCallbackCounter = 0;
void receivedCallback(uint32_t from, String &msg)
{
    receivedCallbackCounter++;

    Serial.println("-----------------------------------------------------new receive call bask start     " + String(receivedCallbackCounter));
    String messageId = extractJsonValue(msg, "messageId");
    if (receivedMessageIds.find(messageId) != receivedMessageIds.end())
    {
        // پیام تکراری است، پس آن را نادیده می‌گیریم
        Serial.println("Duplicate message, ignoring...");
        return;
    }
    Serial.printf("Received message from NodeId = %u: %s\n", from, msg.c_str());

    displayNodeInfo();

    String _type = extractJsonValue(msg, "type");
    Serial.println("callback type :           " + _type);

    // چک کردن اینکه آیا این پیام قبلاً دریافت شده یا نه

    receivedMessageIds.insert(messageId);

    if (mesh.getNodeId() == from)
    {
        // پیام تکراری است، پس آن را نادیده می‌گیریم
        Serial.println("get message from me  me  me  me  me  me  me  me  me  me  me brodcast...");
        return;
    }

    if (_type == "REQUEST_NODE_DATABASE_INFO")
    {
        // اگر پیام درخواست برای دریافت اطلاعات نود بود، تابع broadcastLocalDatabase را صدا می‌زنیم
        Serial.println("callback___REQUEST_NODE_DATABASE_INFO  received msg Id:       " + messageId);

        broadcastLocalDatabase();
    }
    if (_type == "NODE_INFO")
    {

        receiveNodeInfo(msg);
    }
    if (_type == "RELAY_COMMAND")
    {
        ExecuteCommand(msg); // اجرای دستور
                             // sendCommands();
    }
}

void broadcastLocalDatabase()
{
    // ایجاد یک JsonDocument برای ارسال اطلاعات به صورت JSON

    String jsonString;
    JsonDocument doc;

    // مشخصات اضافی
    doc["type"] = "NODE_INFO";
    doc["messageId"] = generateNodeIdentifier(nodeName, "broadcastLocalDatabase"); // فرض بر اینکه یک تابع برای تولید messageId دارید

    // ایجاد آرایه برای ذخیره اطلاعات تمام نودها
    JsonArray nodesArray = doc.createNestedArray("nodes"); // آرایه ایجاد کنید

    int index = 0; // ایجاد یک کانتر برای ایندکس
    Serial.println("strart broadcastLocalDatabase create json  json  json  json  json  json  json  json ");
    unsigned long currentMillis = millis(); // زمان فعلی

    for (const auto &nodePair : nodeInfo)
    {
        const NodeData &node = nodePair.second;

        // اگر ایندکس برای یک نود جدید هنوز وجود ندارد، ایجاد کنید
        JsonObject nodeObj = nodesArray.createNestedObject();

        Serial.println("new row" + node.name);
        // اطلاعات نود را به آرایه اضافه کنید

        nodeObj["id"] = node.id;
        nodeObj["name"] = node.name;
        nodeObj["relayStatus"] = node.relayStatus ? "true" : "false";
        nodeObj["Humidity"] = node.Humidity;
        nodeObj["Temperature"] = node.Temperature;
        nodeObj["TemperatureFahrenheit"] = node.TemperatureFahrenheit;
        nodeObj["HeatIndex"] = node.HeatIndex;
        nodeObj["HeatIndexFahrenheit"] = node.HeatIndexFahrenheit;
        nodeObj["smokValue"] = node.smokValue;
        nodeObj["lastLiveMinute"] = currentMillis / 60000;

        index++; // ایندکس را به‌روز کنید
    }

    // سریالایز کردن JSON و ارسال آن
    serializeJson(doc, jsonString);
    Serial.println("Broadcasting node info: " + jsonString);

    broadcastToAll(jsonString);
}

long broadcastToAllCounter = 0;
unsigned long lastTaskTime = 0;

void onNewConnection(uint32_t nodeId)
{
    Serial.printf("New connection established: NodeId = %u\n", nodeId);

    String messageId = generateNodeIdentifier(nodeName, "newConnection");

    // ایجاد یک پیام درخواست برای تمام نودها با اضافه کردن messageId
    String requestMessage = "{\"type\": \"REQUEST_NODE_DATABASE_INFO\", \"messageId\": \"" + messageId + "\"}";

    // ارسال درخواست اطلاعات به نود جدید
    mesh.sendSingle(nodeId, requestMessage);

    // انتشار اطلاعات خود به نود جدید
    broadcastLocalDatabase();
}

void sendToSpecificNodes(String &msg)
{
    for (const auto &nodeIdStr : mainNetworkList)
    {
        uint32_t nodeId = nodeIdStr.toInt(); // تبدیل String به عدد

        String searchStr = "___Receiver___Node___Id___";
        int index = msg.indexOf(searchStr);
        if (index != -1)
        { // اگر پیدا شد
            msg = msg.substring(0, index) + nodeIdStr + msg.substring(index + searchStr.length());
        }

        mesh.sendSingle(nodeId, msg); // ارسال پیام به نود موردنظر

        // Serial.println("Message sent to Node ID: " + nodeIdStr);
        vTaskDelay(50);
        // mesh.onReceive(&receivedCallback); // ثبت callback
    }
}
void broadcastToAll(String &msg)
{

    unsigned long currentTime = millis();

    Serial.println("broadcastToAllCounter  :::::::::::::::::::::::::::::::::::>>>>>>>>>>>   " + String(broadcastToAllCounter));
    lastTaskTime = currentTime;
    sendToSpecificNodes(msg);
    vTaskDelay(50);
    broadcastToAllCounter++;
}

void addmeToNodeInfo()
{
    uint32_t id = mesh.getNodeId();
    // بررسی اینکه آیا نود جاری قبلاً به لیست اضافه شده است یا نه
    if (nodeInfo.find(id) == nodeInfo.end())
    {

        NodeData newNodeData; //(id, std::string(nodeName.c_str())); // ایجاد یک نود جدید
        newNodeData.id = id;
        newNodeData.name = nodeName;
        newNodeData.relayStatus = relay1_IsEnable;
        newNodeData.Humidity = Humidity;
        newNodeData.Temperature = Temperature;
        newNodeData.TemperatureFahrenheit = TemperatureFahrenheit;
        newNodeData.HeatIndex = HeatIndex;
        newNodeData.HeatIndexFahrenheit = HeatIndexFahrenheit;
        newNodeData.smokValue = smokValue;
        newNodeData.gasDetected = gasDetected;

        nodeInfo[id] = newNodeData; // اضافه کردن نود به لیست
        Serial.printf("Added current node to nodeInfo: NodeId = %u\n", id);
    }
}

void HandleSerialMonitorCommands()
{
    String command = Serial.readStringUntil('\n');
    command.trim(); // حذف فضاهای اضافی

    // بررسی نوع دستور وارد شده
    if (command.startsWith("save"))
    {
        // دستور save <key> <value>
        int spaceIndex = command.indexOf(' ', 5); // پیدا کردن اولین فاصله بعد از "save"
        if (spaceIndex != -1)
        {
            String key = command.substring(5, spaceIndex);
            String value = command.substring(spaceIndex + 1);
            saveData(key, value);
            Serial.println("Data saved.");
        }
        else
        {
            Serial.println("Error: Invalid save command.");
        }
    }
    else if (command.startsWith("read"))
    {
        // دستور read <key>
        String key = command.substring(5);
        String value = ReadKey(key, "");
        Serial.println(value);
    }
    else if (command.startsWith("update"))
    {
        // دستور update <key> <new_value>
        int spaceIndex = command.indexOf(' ', 7); // پیدا کردن اولین فاصله بعد از "update"
        if (spaceIndex != -1)
        {
            String key = command.substring(7, spaceIndex);
            String newValue = command.substring(spaceIndex + 1);
            saveData(key, newValue);
            Serial.println("Data updated.");
        }
        else
        {
            Serial.println("Error: Invalid update command.");
        }
    }
    else
    {
        Serial.println("Error: Invalid command.");
    }
}

void setup_Old()
{

    //     reconnectAttempts = 0;
    //     Serial.begin(9600);
    //     blinkLED(LocalControllerLedPin, 10, 500); // 10 بار چشمک با فاصله 500 میلی‌ثانیه

    //     delay(1000);
    //     if (!SPIFFS.begin(true))
    //     {

    //         Serial.println("SPIFFS initialization failed!");
    //         return;
    //     }
    //     Serial.println("start.................... start......");
    //     loadAndSetVariables();
    //     delay(1000);

    //     setAllPinModes();

    //     mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION); // انتخاب نوع پیام‌های لاگ
    //     mesh.init(ssidName, ssidPassword, &userScheduler, 3256);
    //     mesh.onReceive(&receivedCallback); // ثبت Callback یکبار

    //     displayNodeInfo();
    //     server.begin();

    //     mesh.onNewConnection(&onNewConnection);

    //     mesh.onDroppedConnection([](uint32_t nodeId)
    //                              { Serial.println("Connection dropped with node: " + String(nodeId)); });

    //     Serial.print("Node ID: ");

    //     dht.begin();
    //     // ایجاد تسک‌ها در setup()
    //     xTaskCreate(reconnectMeshTask, "ReconnectMeshTask", 4096, NULL, 1, &taskReconnectMesh);
    //     xTaskCreate(checkAllPirTask, "taskCheckPIRs", 4096, NULL, 1, &taskcheckAllPir);
    //     // xTaskCreate(remoteButtonTask, "ButtonTask", 2048, NULL, 1, &buttonTaskHandle);

    //     renderingPageHtml();

    //     AllarmError(1, 3, 2);
    //     addmeToNodeInfo();

    //     if (!MDNS.begin("esp32"))
    //     { // "esp32" نام نود در شبکه
    //         Serial.println("Error starting mDNS");
    //         return;
    //     }

    //     SetupSim();

    //     reconnectMesh();

    //     broadcastLocalDatabase();
    //     sendRequestNodeInfoToAllNodes();

    //     CheckSpaceSensorTask();
}

void HandleMeshStroryyyyyyyy()
{
    unsigned long currentTime = millis();
    // mesh.onReceive(&receivedCallback); // ثبت callback

    // Initialize mesh network
    mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
    mesh.init(ssidName, ssidPassword, &userScheduler, 5555);
    mesh.onReceive(&receivedCallback); // ثبت Callback یکبار
    mesh.onNewConnection(&onNewConnection);
    mesh.onDroppedConnection([](uint32_t nodeId)
                             { Serial.println("Connection dropped with node: " + String(nodeId)); });

    mesh.update(); // بروزرسانی وضعیت شبکه Mesh و پردازش پیام‌ها

    std::list<uint32_t> nodessList = mesh.getNodeList();

    // چاپ شناسه نودها
    for (auto &nodeId : nodessList)
    {
        Serial.print("Node ID: ");
        Serial.println(nodeId);
    }

    Serial.println("reconnectMesh - reconnectMesh - reconnectMesh ");
    broadcastLocalDatabase();
    sendRequestNodeInfoToAllNodes();

    lastBroadcastTime = currentTime; // به‌روزرسانی زمان ارسال
}
Adafruit_MPU6050 mpu;

void setupMpuSensor()
{

    Serial.begin(115200);
    while (!Serial)
        delay(10); // صبر برای سریال

    if (!mpu.begin())
    {
        Serial.println("خطا در پیدا کردن MPU6050!");
        while (1)
            ;
    }

    Serial.println("MPU6050 متصل شد.");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    //  mpu.setSampleRate(50); // تنظیم نرخ نمونه‌برداری روی 50 هرتز
    // mpu.setDLPFMode(MPU6050_DLPF_5HZ); // تنظیم LPF روی 5 هرتز

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void setup()
{
    setupMpuSensor();

    reconnectAttempts = 0;
    Serial.begin(9600);
    blinkLED(LocalControllerLedPin, 10, 500); // 10 بار چشمک با فاصله 500 میلی‌ثانیه

    delay(1000);
    if (!SPIFFS.begin(true))
    {
        Serial.println("SPIFFS initialization failed!");
    }
    else
    {
        Serial.println("SPIFFS initialized successfully.");
    }
    loadAndSetVariables();
    delay(1000);

    setAllPinModes();

    HandleMeshStroryyyyyyyy();
    Serial.println("setup after HandleMeshStroryyyyyyyy");

    displayNodeInfo();
    server.begin();

    // Initialize DHT sensor
    dht.begin();

    // // Register tasks
    // if (xTaskCreate(reconnectMeshTask, "ReconnectMeshTask", 4096, NULL, 1, &taskReconnectMesh) != pdPASS)
    // {
    //     Serial.println("Failed to create ReconnectMeshTask.");
    // }
    if (xTaskCreate(checkAllPirTask, "taskCheckPIRs", 4096, NULL, 1, &taskcheckAllPir) != pdPASS)
    {
        Serial.println("Failed to create PIR Task.");
    }
    if (xTaskCreate(CheckSensor_SW_420, "__taskCheckSensor___SW_420", 4096, NULL, 1, &taskCheckSensor_SW_420) != pdPASS)
    {
        Serial.println("Failed to create __taskCheckSensor___SW_420 Task.");
    }

    //  if (xTaskCreate(CheckMPUSensor, "__task_CheckSensor_Pmu__", 4096, NULL, 1, &taskCheckSensor_Pmu) != pdPASS)
    // {
    //     Serial.println("Failed to create __taskCheckSensor___SW_420 Task.");
    // }
    // if (xTaskCreate(remoteButtonTask, "ButtonTask", 2048, NULL, 1, &buttonTaskHandle) != pdPASS)
    // {
    //     Serial.println("Failed to create Button Task.");
    // }

    renderingPageHtml();

    AllarmError(1, 3, 2);
    addmeToNodeInfo();

    // if (!MDNS.begin("esp32"))
    // { // "esp32" نام نود در شبکه
    //     Serial.println("Error starting mDNS.");
    // }
    // else
    // {
    //     Serial.println("mDNS started successfully.");
    // }
    Serial.println("Setup sim......................::::::::::::::::::::::::::::::::::::::::::");
    Serial.println("Setup sim......................::::::::::::::::::::::::::::::::::::::::::");
    Serial.println("Setup sim......................::::::::::::::::::::::::::::::::::::::::::");

    SetupSim();

    Serial.println("Setup sim......................::::::::::::::::::::::::::::::::::::::::::");
    Serial.println("Setup sim......................::::::::::::::::::::::::::::::::::::::::::");
    Serial.println("Setup sim......................::::::::::::::::::::::::::::::::::::::::::");

    // Call remaining setup functions
    broadcastLocalDatabase();
    sendRequestNodeInfoToAllNodes();
    CheckSpaceSensorTask();

    // Log setup complete
    Serial.println("Setup complete......................::::::::::::::::::::::::::::::::::::::::::");
    Serial.println("Setup complete......................::::::::::::::::::::::::::::::::::::::::::");
    Serial.println("Setup complete......................::::::::::::::::::::::::::::::::::::::::::");
    Serial.println("Setup complete......................::::::::::::::::::::::::::::::::::::::::::");
    Serial.println("Setup complete......................::::::::::::::::::::::::::::::::::::::::::");

    // ارسال پیام
    String response = sendStartupSMS("+989335017186", "device setup done");
    Serial.println("response........response........response: ::::::::::::" + response);

    Serial.println("Setup complete......................::::::::::::::::::::::::::::::::::::::::::");
    Serial.println("Setup complete......................::::::::::::::::::::::::::::::::::::::::::");

    makeCall("+989335017186");
    Serial.println("Setup complete......................::::::::::::::::::::::::::::::::::::::::::");
    Serial.println("Setup complete......................::::::::::::::::::::::::::::::::::::::::::");
}

void CheckSensor_SW_420(void *parameter)
{

    while (true)
    {
        // Serial.println("CheckSensor____SW_420            CheckSensor____SW_420!        CheckSensor_SW____420 ");

        int sensorValue = digitalRead(SENSOR_SW_420_PIN12); // خواندن وضعیت سنسور
        if (sensorValue == HIGH)
        { // اگر لرزشی تشخیص داده شود

            VibrationAllarmError(3, 2, 1);
            Serial.println("Vibratiooooooooooooon Detected!        " + String(sensorValue));
        }
        else
        {
            // Serial.println("No Viiiiiiiiibration.        " + String(sensorValue));
        }
        vTaskDelay(10); // تاخیر برای جلوگیری از نویز
    }
}

void CheckMPUSensor()
{
    vTaskDelay(50);
    /* خواندن داده‌های سنسور */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // متغیرهای جدید برای مقادیر رند شده
    float accelX = round(a.acceleration.x * 10) / 10.0;
    float accelY = round(a.acceleration.y * 10) / 10.0;
    float accelZ = round(a.acceleration.z * 10) / 10.0;

    float gyroX = round(g.gyro.x * 10) / 10.0;
    float gyroY = round(g.gyro.y * 10) / 10.0;
    float gyroZ = round(g.gyro.z * 10) / 10.0;

    if (gyroX == 0 && gyroY == 0 && gyroZ == 0)
        // Serial.println("all gyroX gyroX  gyroX is 0000000000000000000000");

        // بررسی مقادیر رند شده برای صفر بودن
        if (accelX == 0 && accelY == 0 && accelZ == 0)
        {
            // Serial.println("all accelX accelX accelX  is 0000000000000000000000");
            if (gyroX == 0 && gyroY == 0 && gyroZ == 0)
            {
                Serial.println("all is 0000000000000000000000");
                return;
            }
        }
    /* نمایش داده‌ها در سریال مانیتور */
    Serial.print("shetab: ");
    Serial.print("the      X  : ");
    Serial.print(a.acceleration.x, 1);
    Serial.print(", the      Y   ");
    Serial.print(a.acceleration.y, 1);
    Serial.print(", the      Z  : ");
    Serial.print(a.acceleration.z, 1);
    Serial.println(" m/s^2");

    Serial.print("Zhiroskop ");
    Serial.print("X: ");
    Serial.print(g.gyro.x, 1);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y, 1);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z, 1);
    Serial.println(" rad/s");

    Serial.print("damaa");
    Serial.print(temp.temperature);
    Serial.println(" °C");

    delay(50);
}

void loop()
{

    CheckMPUSensor();
    // CheckSensor_SW_420();
    monitorInputSms();
    // vTaskDelay(10);
    CheckSpaceSensorTask();

    // HandleMeshStroryyyyyyyy();
    // بررسی اینکه آیا داده‌ای از سریال وارد شده است
    if (Serial.available() > 0)
    {
        HandleSerialMonitorCommands();
    }
}
