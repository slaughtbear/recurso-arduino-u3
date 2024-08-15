// Librerías
#include <Wire.h> // Comunicación con el sensor
#include <Adafruit_BMP280.h> // Manejo del sensor
#include <SPI.h> // Comunicación default a los pines del sensor

// Interacción con dispositivos Bluetooth BLE
#include <BLEDevice.h> // ---> Configuración e inicialización
#include <BLEServer.h> // ---> Creación y manejo de servicios - Procesamiento solicitudes de lectura y escritura
#include <BLEUtils.h> // ---> Trabajo con UUIDs (servicios y características de datos, y descriptores de características)
#include <BLE2902.h> // ---> Descriptor BLE para notificaciones

// Pines del sensor
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // Objeto para interactuar con el sensor

// Variables que almacenan los datos sensados
float temperature = 0;
float pressure    = 0;
float altitude    = 0;

// CONFIGURACIÓN DEL SERVIDOR BLE
BLEServer *pServer = NULL; // pServer se encarga de gestionar conexiones entre el ESP32 y el dispositivo móvil
BLECharacteristic *pCharacteristic = NULL; // pCharacteristic permite leer y escribir valores asociados a características

#define SERVICE_UUID "0000181a-0000-1000-8000-00805f9b34fb" // Servicio que proporciona información sobre el medio ambiente (temperatura, presión, etc)
#define CHARACTERISTIC_UUID "00002a6e-0000-1000-8000-00805f9b34fb" // Accede a la característica de Device Name en un dispositivo BLE y lee o escribe el nombre del dispositivo

// Se inicializa el sensor
void Bmp280_Init(){ // Una vez inicializado
  delay(100); // Se espera 100 milisegundos para asegurar que todo esté listo
  Serial.println("BMP280");
  if (!bmp.begin()){ // De otra forma...
    Serial.println("No se encontro un sensor valido"); // Muestra un error si no se encuentra el sensor
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

// INICIALIZACIÓN Y CONFIGURACIÓN DEL BLUETOOTH BLE
void setupBLE() {
  BLEDevice::init("ESP32_IVN"); // Inicialización del nombre
  pServer = BLEDevice::createServer(); // Servidor BLE (dispositivo) que ofrece servicios y responde solicitudes de dispositivos clientes
  BLEService *pService = pServer->createService(SERVICE_UUID); // Se accede al UUID del servicio BLE previamente establecido
  
  // CREACIÓN DE CARACTERÍSTICA BLE
  pCharacteristic = pService->createCharacteristic( // Contenedor de datos que pueden ser leídos o notificados a dispositivos clientes
                       CHARACTERISTIC_UUID,
                       BLECharacteristic::PROPERTY_READ | // Permite que un cliente lea el valor de la característica
                       BLECharacteristic::PROPERTY_NOTIFY // Permite que el servidor notifique automáticamente a los clientes cuando el valor de la característica cambia
                     );
  
  pCharacteristic->addDescriptor(new BLE2902()); // Descriptor para notificaciones
  
  pService->start(); // Inicia el servicio BLE, haciendo disponible el ESP32 a otros dispositivos
  
  // CONFIGURACIÓN Y PUBLICIDAD BLE
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising(); // Obtención del objeto de publicidad para configurar cómo se presenta el dispositivo a clientes
  pAdvertising->addServiceUUID(SERVICE_UUID); // Se añade el UUID del servicio 
  pAdvertising->setScanResponse(true); // Configuración de la respuesta de escaneo, enviando información adicional durante el proceso de escaneo en otros dispositivos
  pAdvertising->setMinPreferred(0x06); 
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising(); // Inicia la publicidad del ESP32, haciéndolo visible a los dispositivos que estén escaneando
  Serial.println("El dispositivo BLE ya se encuentra disponible...");
}

// Actualización de las lecturas del sensor
void updateSensorReadings() { 
  // Lectura de datos del sensor
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure() / 100.0F;
  altitude = bmp.readAltitude();

  // Verificación por si alguna lectura falló
  if (isnan(temperature) || isnan(pressure) || isnan(altitude)) {
    Serial.println(F("Error al leer datos del sensor bmp280!")); // Se muestra un mensaje de error
    return;
  }

  // Impresión de las lecturas dentro del monitor serial
  Serial.printf("Temperature reading: %.2f \n", temperature);
  Serial.printf("Pressure reading: %.2f \n", pressure);
  Serial.printf("Altitude reading: %.2f \n", altitude);
}

// Funciones que obtienen la lectura de datos como strings
String readTemperature() {
     return String(temperature);
}

String readPressure() {
      return String(pressure);
}
float readAltitude() {
      return bmp.readAltitude(1013.25); // Calcula la altitud con la presión estándar al nivel del mar
}

// FUNCIÓN PARA MANDAR DATOS
void sendDataToBLE() {
  String data = readTemperature() + "//" + readPressure() + "//" + String(readAltitude(), 2); // Cadena que devuelve todas las lecturas juntas
  pCharacteristic->setValue(data.c_str()); // Se convierten los datos leídos a características
  pCharacteristic->notify(); // Se notifica que el valor de la característica cambió y se debe leer el nuevo valor 
  Serial.println("Datos enviados vía BLE: " + data);
}

void uploadSensorData() {
  static unsigned long elapsedMillis = 0; // Temporalizador que almacena el tiempo en milisegundos de cuando se ejecutó por última vez la función
  unsigned long update_interval = 10000; // Actualización de datos cada 10 segundos
  
  // Se calcula cuánto tiempo pasó desde la última actualización de datos
  if (millis() - elapsedMillis > update_interval) { // Si ha pasado más de 10 segundos...
    elapsedMillis = millis(); // El temporizador se reinicia
    updateSensorReadings(); // Se actualizan los valores actuales leídos por el sensor
    sendDataToBLE(); // Se envían los datos actuales 
  }
}

void setup() {
  Serial.begin(115200);  // Inicia la comunicación serial a una velocidad de 115200 baudios.
  Bmp280_Init();  // Llama a una función para inicializar el sensor BMP280 (presión y temperatura).
  setupBLE();  // Llama a una función para inicializar el módulo Bluetooth BLE.
}

void loop() {
  uploadSensorData();  // Llama a una función para cargar los datos del sensor (como la temperatura y la presión) a algún lugar, posiblemente a través de Bluetooth o a una base de datos.
}
