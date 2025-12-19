# Robot Siguelíneas con Comunicación IoT

![Estado del Proyecto](https://img.shields.io/badge/estado-completado-success)
![Plataforma](https://img.shields.io/badge/plataforma-Arduino%20%2B%20ESP32-blue)
![Comunicación](https://img.shields.io/badge/protocolo-MQTT-orange)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-red)

Sistema de robot autónomo siguelíneas con arquitectura dual (Arduino UNO + ESP32-CAM), comunicación IoT mediante MQTT, control PID, detección de obstáculos y recuperación automática de línea perdida.

**Equipo:** ER404 | **ID:** 1

---

## 📋 Tabla de Contenidos

- [Introducción y Objetivos](#-introducción-y-objetivos)
- [Sensores](#-sensores)
- [ESP32-CAM](#-esp32-cam)
  - [Funcionalidades Implementadas](#funcionalidades-implementadas)
  - [Protocolo MQTT](#protocolo-mqtt)
- [Arduino UNO](#-arduino-uno)
  - [Sistema de Tiempo Real con FreeRTOS](#sistema-de-tiempo-real-con-freertos)
  - [Máquina de Estados Finita (FSM)](#máquina-de-estados-finita-fsm)
  - [Control PID Simplificado (PD)](#control-pid-simplificado-pd)
  - [Comportamiento de Recuperación de Línea](#comportamiento-de-recuperación-de-línea)
- [Protocolo de Comunicación Serie](#-protocolo-de-comunicación-serie)
- [Operación del Robot](#-operación-del-robot)
  - [Estado: SEGUIR_LINEA](#estado-seguir_linea--verde)
  - [Estado: LINEA_PERDIDA](#estado-linea_perdida--rojo)
  - [Estado: OBSTACULO_DETECTADO](#estado-obstaculo_detectado--blanco)

---

## 📖 Introducción y Objetivos

Este proyecto implementa un **robot siguelíneas autónomo** desarrollado como parte de la asignatura de Sistemas Empotrados y de Tiempo Real (SETR) de la Universidad Rey Juan Carlos.

### Objetivos del Proyecto

Se encuentran todas las especificaciones y objetivos del proyecto en la wiki de la página:

📚 **Documentación completa:** [Wiki del Proyecto - P4FollowLine](https://gitlab.eif.urjc.es/roberto.calvo/setr/-/wikis/P4FollowLine)

---

## 📡 Sensores

### Sensores Infrarrojos ITR20001 (×3)

Los sensores de línea son el componente principal para el seguimiento del circuito.

**Especificaciones:**
- **Tipo:** Sensor infrarrojo reflectivo analógico
- **Cantidad:** 3 unidades (izquierda, centro, derecha)
- **Umbral configurado:** 700 (valores >700 = línea negra detectada)

**Conexiones:**
```
Sensor Izquierdo  → Pin A2
Sensor Central    → Pin A1
Sensor Derecho    → Pin A0
```

### Sensor Ultrasónico HC-SR04

Utilizado para la detección del obstáculo final.

**Especificaciones:**
- **Rango de medición:** 2 cm - 400 cm
- **Precisión:** ±3 mm
- **Frecuencia:** 40 KHz

**Conexiones:**
```
TRIG → Pin 13 (genera pulso)
ECHO → Pin 12 (recibe eco)
VCC  → 5V
GND  → GND
```

### LED RGB NeoPixel

Indicador visual del estado del robot mediante un LED RGB direccionable.

**Especificaciones:**
- **Tipo:** WS2812B (NeoPixel)
- **Control:** Pin 4 (Arduino)
- **Librería:** FastLED

**Código de colores del sistema:**

| Color | Estado | Significado |
|-------|--------|-------------|
| 🔵 **Azul** | Inicialización | Sistema arrancando |
| 🟡 **Amarillo** | Handshake | Esperando confirmación ESP32 |
| 🟢 **Verde** | Siguiendo línea | Línea detectada, operación normal |
| 🔴 **Rojo** | Línea perdida | Búsqueda activa de línea |
| ⚪ **Blanco** | Obstáculo | Fin de recorrido, obstáculo detectado |

La implementación de este LED se ha realizado mediante la librería **FastLED.h**.

---

## 🌐 ESP32-CAM

El ESP32-CAM gestiona la conectividad WiFi y la publicación de mensajes MQTT al servidor remoto.

### Funcionalidades Implementadas

#### 1. Gestión de Conectividad WiFi

El ESP32 se conecta automáticamente a la red WiFi. En este caso, para no depender de Eduroam, se ha decidido conectar el robot a la WiFi privada de uno de los dispositivos móviles.

**Máquina de estados de conexión:**
```cpp
enum class SystemState {
  INIT_WIFI,          // Inicialización WiFi
  CONNECTING_WIFI,    // Proceso de conexión
  INIT_MQTT,          // Inicialización MQTT
  CONNECTING_MQTT,    // Conexión al broker
  WAIT_HANDSHAKE,     // Esperando START_LAP del Arduino
  RUNNING             // Operación normal
};
```

#### 2. Comunicación Serie con Arduino

El ESP32 utiliza **Serial2** (hardware UART) para comunicarse con el Arduino:

```cpp
#define RXD2 33  // GPIO33 - RX
#define TXD2 4   // GPIO4  - TX

Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
```

#### 3. Handshake de Sincronización

El ESP32 **no permite que el robot inicie** hasta confirmar conectividad completa. El sistema implementa un mecanismo de handshake donde el Arduino envía mensajes START_LAP que son ignorados hasta que el ESP32 confirma:
1. Conexión WiFi establecida
2. Conexión MQTT al broker activa

Una vez confirmadas ambas conexiones, el ESP32 responde al Arduino con un ACK, permitiendo iniciar el recorrido.

#### 4. Gestión de PINGs Periódicos

El ESP32 envía automáticamente mensajes PING cada **4 segundos**.

**Nota importante:** Al principio del proyecto se había implementado la tarea de PING en el Arduino, pero finalmente se vio que era mejor su control exclusivo en el ESP32, para aumentar la capacidad de respuesta del robot.

```cpp
void handlePing() {
  if (!lap_active || lap_finished) return;
  
  unsigned long current_time = millis();
  
  if (current_time - last_ping_time >= 4000) {
    unsigned long elapsed = current_time - lap_start_time;
    publishMQTT(MsgType::PING, elapsed);
    last_ping_time = current_time;
  }
}
```

### Protocolo MQTT

MQTT (Message Queuing Telemetry Transport) es un protocolo de mensajería ligero diseñado para dispositivos IoT con recursos limitados.

#### Configuración del Broker

```cpp
#define MQTT_SERVER "193.147.79.118"  // teachinghub.eif.urjc.es
#define MQTT_PORT   21883
#define MQTT_TOPIC  "/SETR/2025/1/"

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_PORT);
Adafruit_MQTT_Publish publisher = Adafruit_MQTT_Publish(&mqtt, MQTT_TOPIC);
```

#### Formato de Mensajes

Todos los mensajes se publican en **formato JSON**:

```json
{
  "team_name": "ER404",
  "id": "1",
  "action": "PING",
  "time": 8000
}
```

#### Tipos de Mensajes MQTT

| Mensaje | Cuándo se envía | Campos adicionales |
|---------|-----------------|-------------------|
| `START_LAP` | Al iniciar el recorrido | - |
| `PING` | Cada 4 segundos | `time` (ms transcurridos) |
| `LINE_LOST` | Al perder la línea | - |
| `INIT_LINE_SEARCH` | Inicio de búsqueda | - |
| `LINE_FOUND` | Línea recuperada | - |
| `STOP_LINE_SEARCH` | Fin de búsqueda | - |
| `OBSTACLE_DETECTED` | Obstáculo detectado | `distance` (cm) |
| `END_LAP` | Fin del recorrido | `time` (ms totales) |
| `VISIBLE_LINE` | Estadísticas finales | `value` (% de línea vista) |

---

## 🤖 Arduino UNO

El Arduino UNO es el **cerebro del robot**, encargado del control en tiempo real de todos los sensores y actuadores.

### Sistema de Tiempo Real con FreeRTOS

El Arduino implementa **multitarea cooperativa** utilizando FreeRTOS, permitiendo ejecutar múltiples tareas concurrentes sin bloqueos.

#### Tareas Implementadas

```cpp
void setup() {
  // Tarea 1: Lectura continua del ultrasonido
  xTaskCreate(ultrasonic_task, "Ultrasonic", 128, NULL, 3, NULL);
  
  // Tarea 2: Control de movimiento y FSM
  xTaskCreate(move_task, "Move", 256, NULL, 2, NULL);
}
```

| Tarea | Periodo | Stack | Prioridad | Función |
|-------|---------|-------|-----------|---------|
| `ultrasonic_task` | 30 ms | 128 bytes | 3 (Alta) | Medir distancia constantemente |
| `move_task` | 10 ms | 256 bytes | 2 (Media) | Control PID y FSM |

**Ventajas del enfoque RTOS:**
- ✅ Lectura continua del ultrasonido sin interferir con el control de motores
- ✅ Detección inmediata de obstáculos (prioridad alta)
- ✅ Código modular y fácil de mantener
- ✅ Uso eficiente de recursos limitados

### Máquina de Estados Finita (FSM)

El robot opera mediante una FSM con 3 estados principales:

```cpp
enum class RobotState : uint8_t {
  SEGUIR_LINEA,          // Estado 0: Operación normal
  LINEA_PERDIDA,         // Estado 1: Búsqueda activa
  OBSTACULO_DETECTADO    // Estado 2: Fin de recorrido
};
```

#### Diagrama de Transiciones

```
                    ┌─────────────────┐
         ┌─────────►│ SEGUIR_LINEA    │◄──────────┐
         │          │                 │           │
         │          │ • Control PID   │           │
         │          │ • LED Verde     │           │
         │          └────────┬────────┘           │
         │                   │                    │
         │        línea NO detectada    línea detectada
         │                   │                    │
         │                   ▼                    │
         │          ┌─────────────────┐           │
         │          │ LINEA_PERDIDA   │───────────┘
         │          │                 │
         │          │ • Giro búsqueda │
         │          │ • LED Rojo      │
         │          └────────┬────────┘
         │                   │
         │          obstáculo detectado
         │                   │
         │                   ▼
         │          ┌─────────────────┐
         └──────────│ OBSTACULO       │
                    │ DETECTADO       │
                    │                 │
                    │ • Detención     │
                    │ • LED Blanco    │
                    └─────────────────┘
```

### Control PID Simplificado (PD)

El algoritmo de control implementado es un **controlador PD (Proporcional-Derivativo)** optimizado para seguimiento de línea.

#### Algoritmo PD

```cpp
// Parámetros del controlador (ajustados experimentalmente)
const float Kp = 960.0;  // Ganancia proporcional
const float Kd = 96.0;   // Ganancia derivativa

// Cálculo de la salida PD
float pd_output = Kp * error + Kd * (error - last_error);

// Velocidad adaptativa según error
if (abs(error) >= 2) {
  base_speed = 90;   // Reducir velocidad en curvas cerradas
} else {
  base_speed = 120;  // Velocidad normal en rectas
}

// Ajuste diferencial de motores
motor_left  = constrain(base_speed - pd_output, 0, max_speed);
motor_right = constrain(base_speed + pd_output, 0, max_speed);

// Aplicar a motores
analogWrite(PIN_Motor_PWMA, motor_right);
digitalWrite(PIN_Motor_AIN_1, HIGH);
analogWrite(PIN_Motor_PWMB, motor_left);
digitalWrite(PIN_Motor_BIN_1, HIGH);

// Guardar error para siguiente iteración
last_error = error;
```

**Explicación de parámetros:** Los valores de Kp y Kd se obtuvieron mediante prueba y error. Inicialmente se usó Kp=500, pero el robot no corregía suficientemente rápido en curvas. Al incrementar a 960, las correcciones fueron más agresivas y el seguimiento mejoró significativamente.

**¿Por qué PD y no PID completo?**
- El término integral (I) no es necesario porque el sistema no presenta error en estado estacionario significativo
- Simplifica el código y reduce carga computacional
- El término derivativo (D) proporciona suficiente estabilidad en curvas

### Comportamiento de Recuperación de Línea

Cuando se detecta pérdida de línea (`!l && !m && !r`), el robot entra en modo búsqueda:

```cpp
case RobotState::LINEA_PERDIDA:
{
  // Girar hacia el lado donde se vio la línea por última vez
  if (last_valid_error < 0) {
    // Última detección a la izquierda → girar izquierda
    motor_left = 0;
    motor_right = search_speed;  // 240 PWM
  } else {
    // Última detección a la derecha → girar derecha
    motor_left = search_speed;
    motor_right = 0;
  }
  
  leds[0] = CRGB::Red;
  FastLED.show();
  break;
}
```

**Estrategia de búsqueda:**
1. Memoriza el `last_valid_error` cuando detecta línea
2. Al perder la línea, gira en dirección al último error
3. Gira sobre su propio eje (un motor parado, otro a velocidad máxima)
4. Envía mensajes MQTT: `LINE_LOST`, `INIT_LINE_SEARCH`
5. Al re-detectar línea: `LINE_FOUND`, `STOP_LINE_SEARCH`

**Ventajas:**
- ⚡ Recuperación más rápida que búsqueda aleatoria
- 🎯 Aprovecha la inercia del movimiento
- ⏱️ Tiempo límite: 5 segundos máximo

---

## 🔌 Protocolo de Comunicación Serie

El sistema implementa un protocolo binario personalizado para la comunicación entre Arduino y ESP32.

### Estructura del Mensaje

Ambos microcontroladores usan un protocolo de **7 bytes** con marcadores de inicio/fin:

```cpp
#define START_BYTE 0xAA
#define END_BYTE   0x55

struct __attribute__((packed)) Message {
  uint8_t start;      // 0xAA (marcador de inicio)
  MsgType type;       // Tipo de mensaje (1 byte)
  uint32_t data;      // Datos asociados (4 bytes)
  uint8_t end;        // 0x55 (marcador de fin)
};
// Tamaño total: 7 bytes
```

### Tipos de Mensajes

```cpp
enum class MsgType : uint8_t {
  START_LAP           = 0,  // Arduino → ESP32
  OBSTACLE_DETECTED   = 1,  // Arduino → ESP32
  END_LAP             = 2,  // Arduino → ESP32
  LINE_LOST           = 3,  // Arduino → ESP32
  PING                = 4,  // (generado por ESP32)
  INIT_LINE_SEARCH    = 5,  // Arduino → ESP32 (opcional)
  STOP_LINE_SEARCH    = 6,  // Arduino → ESP32 (opcional)
  LINE_FOUND          = 7,  // Arduino → ESP32 (opcional)
  VISIBLE_LINE        = 8   // Arduino → ESP32 (opcional)
};
```

**Características del protocolo:**
- 🔒 Marcadores de inicio/fin para sincronización
- ✅ Validación de integridad
- 🔄 Recuperación ante pérdida de sincronización
- ⚡ Transmisión a 115200 baudios
- 📦 Tamaño fijo de 7 bytes por mensaje

---

## 🎮 Operación del Robot

### Estado: SEGUIR_LINEA (🟢 Verde)

**Descripción:**
- Robot sigue la línea con control PID
- Ajusta velocidad según curvatura
- Envía estadísticas a ESP32

**Comportamiento esperado:**
- Movimiento suave en rectas
- Correcciones rápidas en curvas
- Velocidad adaptativa (90-120 PWM)

---

### Estado: LINEA_PERDIDA (🔴 Rojo)

Se activa cuando los 3 sensores IR no detectan línea.

**Secuencia de acciones:**
```
1. Envía mensaje LINE_LOST → MQTT
2. Envía INIT_LINE_SEARCH → MQTT
3. Gira hacia el último lado detectado
4. Tiempo límite: 5 segundos
```

**Si recupera la línea:**
```
1. Envía STOP_LINE_SEARCH → MQTT
2. Envía LINE_FOUND → MQTT
3. Vuelve a estado SEGUIR_LINEA
4. Bonificación: -2% en tiempo final
```

**Si NO recupera en 5s:**
```
1. Test se da por finalizado
2. Vuelta declarada nula
```

---

### Estado: OBSTACULO_DETECTADO (⚪ Blanco)

Se activa cuando el ultrasonido detecta objeto a ≤10 cm.

**Secuencia de acciones:**
```
1. Detención inmediata de motores
2. Envía OBSTACLE_DETECTED con distancia → MQTT
3. Envía END_LAP con tiempo total → MQTT
4. Envía VISIBLE_LINE con estadísticas → MQTT
5. Sistema se mantiene detenido
```

---

<div align="center">

**🤖 Robot Siguelíneas ER404 - SETR 2024/2025 🏁**

*Desarrollado con ❤️ y muchas horas de debugging*

📚 [Wiki del Proyecto](https://gitlab.eif.urjc.es/roberto.calvo/setr/-/wikis/P4FollowLine)

[⬆️ Volver arriba](#robot-siguelíneas-con-comunicación-iot)

</div>
