#include <Arduino.h>
#include <NimBLEDevice.h>

/*
  KRAKEN - Prototipo electrónico
  ESP32-C3 Mini + TB6612FNG + 2 motores N20 de 6 V
  Control mediante Bluefruit Connect por Bluetooth BLE.
*/

// Pines del motor izquierdo (canal A)
constexpr uint8_t AIN1_PIN = 0;
constexpr uint8_t AIN2_PIN = 1;
constexpr uint8_t PWMA_PIN = 3;

// Pines del motor derecho (canal B)
constexpr uint8_t BIN1_PIN = 4;
constexpr uint8_t BIN2_PIN = 5;
constexpr uint8_t PWMB_PIN = 6;

/*
  Los motores están montados como espejo, por lo que
  uno necesita invertir su sentido de giro.
*/
constexpr bool LEFT_REVERSED  = false;
constexpr bool RIGHT_REVERSED = true;

// Velocidad inicial: 180 de 255 ≈ 71 %
uint8_t driveSpeed = 180;

// UUID del servicio Nordic UART Service para Bluefruit
constexpr char NUS_SERVICE_UUID[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char NUS_RX_UUID[]      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char NUS_TX_UUID[]      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

/*
  Controla un motor.
  command:
    positivo → un sentido
    negativo → sentido contrario
    0        → detenido
*/
void setOneMotor(uint8_t in1, uint8_t in2, uint8_t pwmPin,
                 int16_t command, bool reversed) {

  // Limitamos el comando al rango del PWM
  command = constrain(command, -255, 255);

  // Invertimos el sentido si el motor está montado al revés
  if (reversed) command = -command;

  if (command > 0) {
    // Motor hacia adelante
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, command);

  } else if (command < 0) {
    // Motor hacia atrás
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -command);

  } else {
    // Motor detenido
    analogWrite(pwmPin, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

/*
  Controlamos los dos motores al mismo tiempo.
  Los valores pueden ser positivos o negativos
  dependiendo de la dirección.
*/
void drive(int16_t left, int16_t right) {
  setOneMotor(AIN1_PIN, AIN2_PIN, PWMA_PIN, left, LEFT_REVERSED);
  setOneMotor(BIN1_PIN, BIN2_PIN, PWMB_PIN, right, RIGHT_REVERSED);
}

/*
  Frenado corto usando la configuración del TB6612:
  IN1 = HIGH, IN2 = HIGH y PWM = HIGH.
*/
void stopMotors() {
  analogWrite(PWMA_PIN, 255);
  analogWrite(PWMB_PIN, 255);

  digitalWrite(AIN1_PIN, HIGH);
  digitalWrite(AIN2_PIN, HIGH);
  digitalWrite(BIN1_PIN, HIGH);
  digitalWrite(BIN2_PIN, HIGH);
}

// Movimientos básicos del robot
void forward()  { drive( driveSpeed,  driveSpeed); }
void backward() { drive(-driveSpeed, -driveSpeed); }
void turnLeft() { drive(-driveSpeed,  driveSpeed); }
void turnRight(){ drive( driveSpeed, -driveSpeed); }

/*
  Verifica que los datos recibidos tengan el formato
  esperado por el Control Pad de Bluefruit.
*/
bool validBluefruitPacket(const uint8_t* data, size_t len) {
  if (len != 5 || data[0] != '!' || data[1] != 'B')
    return false;

  // Calculamos el checksum para verificar el paquete
  uint8_t checksum = 255;

  for (size_t i = 0; i < len - 1; ++i)
    checksum -= data[i];

  return checksum == data[len - 1];
}

/*
  Procesa los botones enviados desde Bluefruit.

  1 → velocidad 47 %
  2 → velocidad 71 %
  3 → velocidad 100 %
  4 → paro
  5 → izquierda
  6 → derecha
  7 → atrás
  8 → adelante
*/
void processBluefruitButton(uint8_t button, bool pressed) {
  Serial.printf("Boton %u: %s\n",
                button,
                pressed ? "presionado" : "soltado");

  // Botones para seleccionar la velocidad o detener
  if (pressed) {
    if (button == 1) {
      driveSpeed = 120;
      Serial.println("Velocidad: 47 %");
      return;
    }

    if (button == 2) {
      driveSpeed = 180;
      Serial.println("Velocidad: 71 %");
      return;
    }

    if (button == 3) {
      driveSpeed = 255;
      Serial.println("Velocidad: 100 %");
      return;
    }

    if (button == 4) {
      stopMotors();
      Serial.println("PARO");
      return;
    }
  }

  /*
    Cuando se suelta un botón de movimiento,
    el robot se detiene.
  */
  if (!pressed) {
    if (button >= 5 && button <= 8)
      stopMotors();

    return;
  }

  // Ejecutamos el movimiento correspondiente
  switch (button) {
    case 5: turnLeft();  break;
    case 6: turnRight(); break;
    case 7: backward();  break;
    case 8: forward();   break;
    default: break;
  }
}

/*
  También podemos probar el robot desde la pestaña UART
  escribiendo:
    F = adelante
    B = atrás
    L = izquierda
    R = derecha
    S = detener
*/
void processTextCommand(const uint8_t* data, size_t len) {
  if (len == 0) return;

  // Convertimos el comando a mayúscula
  char command = toupper(static_cast<char>(data[0]));

  switch (command) {
    case 'F': forward();   break;
    case 'B': backward();  break;
    case 'L': turnLeft();  break;
    case 'R': turnRight(); break;
    case 'S': stopMotors(); break;
    default: return;
  }

  Serial.printf("Comando UART: %c\n", command);
}

/*
  Callback que se ejecuta automáticamente cuando
  llegan datos desde el celular por Bluetooth.
*/
class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic,
               NimBLEConnInfo& connInfo) override {

    NimBLEAttValue value = characteristic->getValue();
    const uint8_t* data = value.data();
    const size_t len = value.size();

    // Si es un paquete de Bluefruit, procesamos el botón
    if (validBluefruitPacket(data, len)) {
      const uint8_t button = data[2] - '0';
      const bool pressed = data[3] == '1';

      processBluefruitButton(button, pressed);
    } else {
      // Si no, lo tratamos como comando de texto
      processTextCommand(data, len);
    }
  }
};

/*
  Controla los eventos de conexión y desconexión.
*/
class ServerCallbacks : public NimBLEServerCallbacks {

  // Cuando el celular se conecta
  void onConnect(NimBLEServer* server,
                 NimBLEConnInfo& connInfo) override {
    Serial.println("Telefono conectado por BLE");
  }

  // Si se pierde la conexión, detenemos el robot
  void onDisconnect(NimBLEServer* server,
                    NimBLEConnInfo& connInfo,
                    int reason) override {
    stopMotors();
    Serial.println("Telefono desconectado: motores detenidos");

    // Volvemos a permitir conexiones
    NimBLEDevice::startAdvertising();
  }
};

// Objetos para manejar los eventos de BLE
RxCallbacks rxCallbacks;
ServerCallbacks serverCallbacks;

void setup() {
  // Iniciamos comunicación con el monitor serial
  Serial.begin(115200);

  // Configuramos los pines del TB6612 como salidas
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMA_PIN, OUTPUT);

  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);

  // PWM de 8 bits: valores de 0 a 255
  analogWriteResolution(PWMA_PIN, 8);
  analogWriteResolution(PWMB_PIN, 8);

  // Frecuencia PWM de 20 kHz
  analogWriteFrequency(PWMA_PIN, 20000);
  analogWriteFrequency(PWMB_PIN, 20000);

  // Iniciamos con los motores detenidos
  stopMotors();

  // Iniciamos Bluetooth con el nombre KRAKEN
  NimBLEDevice::init("KRAKEN");

  // Creamos el servidor BLE
  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(&serverCallbacks, false);

  // Creamos el servicio UART
  NimBLEService* uartService =
      server->createService(NUS_SERVICE_UUID);

  /*
    TX se crea para que Bluefruit reconozca
    correctamente el perfil UART.
  */
  uartService->createCharacteristic(
      NUS_TX_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  // RX recibe los comandos del celular
  NimBLECharacteristic* rxCharacteristic =
      uartService->createCharacteristic(
          NUS_RX_UUID,
          NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);

  // Asociamos RX con la función que procesa los comandos
  rxCharacteristic->setCallbacks(&rxCallbacks);

  // Iniciamos el servicio UART
  uartService->start();

  // Configuramos el advertising de Bluetooth
  NimBLEAdvertising* advertising =
      NimBLEDevice::getAdvertising();

  advertising->addServiceUUID(NUS_SERVICE_UUID);
  advertising->enableScanResponse(true);

  // KRAKEN queda visible para conectarse desde el celular
  NimBLEDevice::startAdvertising();

  Serial.println("KRAKEN listo. Busca KRAKEN en Bluefruit Connect.");
}

void loop() {
  /*
    No necesitamos revisar Bluetooth aquí porque
    los comandos se procesan mediante callbacks.
  */
  delay(20);
}