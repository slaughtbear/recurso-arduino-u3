// Librerías
#include <Wire.h> // Comunicación con el sensor
#include <Adafruit_BMP280.h> // Manejo del sensor
#include "BluetoothSerial.h" // Manejo de Bluetooth
#include <SPI.h> // Comunicación default a los pines del sensor

// Se verifica si el Bluetooth está habilitado en la configuración del ESP32
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Objeto que maneja la comunicación Bluetooth
BluetoothSerial SerialBT;

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

// Objeto para interactuar con el sensor
Adafruit_BMP280 bmp;

// Variables que almacenan los datos sensados
float temperature = 0;
float pressure    = 0;
float altitude    = 0;

// Variables que gestionan el tiempo
unsigned long elapsedMillis = 0; 
unsigned long update_interval = 10000;  // Actualización de datos cada 5 segundos

// Variables que manejan mensajes por Bluetooth
String message = "";
char incomingChar;
String temperatureString = "";

// Se inicializa el sensor
void Bmp280_Init(){ // Una vez inicializado
delay(100); // Se espera 100 milisegundos para asegurar que todo esté listo
  Serial.println("BMP280");
  if (!bmp.begin()){ // De otra forma...
    Serial.println("No se encontro un sensor valido"); // Muestra un error si no se ecuentra el sensor
  }

  // Configuración del sensor
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL, // Modo de operación normal
    Adafruit_BMP280::SAMPLING_X2,  // Factor de muestreo para temperatura
    Adafruit_BMP280::SAMPLING_X16, // Factor de muestreo para presión
    Adafruit_BMP280::FILTER_X16, // Filtro de media móvil
    Adafruit_BMP280::STANDBY_MS_500 // Tiempo de espera de 500 ms
  );
}

// Actualización de las lecturas del sensor
void updateSensorReadings() { 
  // Lectura de datos del sensor
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure() / 100.0F;
  altitude = bmp.readAltitude();

  // Verificación por si alguna lectura fallo
  if (isnan(temperature) || isnan(pressure) || isnan(altitude) ) {
    Serial.println(F("Failed to read from bmp280 sensor!")); // Se muestra un mensaje de error
    return;
  }

  // Impresión de las lecturas dentro del monitor serial
  Serial.printf("Temperature reading: %.2f \n", temperature);
  Serial.printf("Pressure reading: %.2f \n", pressure);
  Serial.printf("altitude reading: %.2f \n", altitude);
 
}

// Funciones que obtienen la lectura de datos como strings
String readTemperature() {
     return String(temperature);
}

String readPressure() {
      return String(pressure);
}
String readAltitude() {
      return String(altitude);
}

void bluetooth_Init(){
  delay(100);
  // Bluetooth device name
  if (!SerialBT.begin("ESP32")){
    Serial.println("No se encontro bluetooth");
  }else {
    Serial.println("bluetooth listo");
  }
  
}

// Envío de datos a través de Bluetooth
void sendDataToBluetooth(){
    Serial.println("Sending Data to Bluetooth "); // Se imprime un mensaje indicando que sí se están mandando datos
     // Envía temperatura, presión y altitud a través de Bluetooth, separados por "//"
    SerialBT.println(readTemperature() + "//" + readPressure() + "//" + readAltitude());
    Serial.println("End transmission"); // Mensaje que indica el fin de la transmisión
}

// Se leen los mensajes recibidos por Bluetooth y dependiendo de...
void readDataFromBluetooth(){
  if (SerialBT.available()){ // Si hay datos para leer
    char incomingChar = SerialBT.read(); // Lee el carácter recibido
    if (incomingChar != '\n'){ // Y si el carácter no es un salto de línea...
      message += String(incomingChar);  // Se agrega el carácter al mensaje
    }
    else{ // De otra forma
      message = ""; // Se reinicia el mensaje al encontrar un salto de línea
    }
    Serial.write(incomingChar); // Finalmente se imrpime el carácter recibido en el monitor serial 
  }
    // Si el mensaje es "obtener"
    if (message =="obtener"){
      updateSensorReadings(); // Se actualizan las lecturas del sensor
      sendDataToBluetooth(); // Se envían los datos a través de Bluetooth
    }
    delay(3); // Delay de 3 segundos para evitar una sobrecarga
}

// Se envían los datos del sensor en intervalos regulares
void uploadSensorData() {
  if (millis() - elapsedMillis > update_interval){ // Si ha pasado el intervalo de actualización...
    elapsedMillis = millis();  // Actualiza el tiempo transcurrido
    updateSensorReadings(); // Actualiza las lecturas del sensor
    sendDataToBluetooth(); // Envía los datos a través de Bluetooth
  }
}


void setup() {
  Serial.begin(115200);  // Inicia la comunicación serial a una velocidad de 115200 baudios.
  Bmp280_Init();  // Llama a una función para inicializar el sensor BMP280 (presión y temperatura).
  bluetooth_Init();  // Llama a una función para inicializar el módulo Bluetooth.
}

void loop() {
  uploadSensorData();  // Llama a una función para cargar los datos del sensor (como la temperatura y la presión) a algún lugar, posiblemente a través de Bluetooth o a una base de datos.
  readDataFromBluetooth();  // Llama a una función para leer datos entrantes desde el módulo Bluetooth.
}
