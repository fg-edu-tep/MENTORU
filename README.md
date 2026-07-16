# MENTORU

Sistema de monitoreo biométrico distribuido basado en ESP32. Varios dispositivos vestibles (reloj, gafas, pecho, sensor GSR) miden señales fisiológicas y de movimiento, y las transmiten de forma inalámbrica por **ESP-NOW** a un receptor central que las expone como JSON vía HTTP.

Proyecto de la asignatura *Fundamentos de Electrónica* (Universidad de los Andes).

## Arquitectura

Cada dispositivo sensor transmite tramas de tamaño fijo (88 bytes) usando el protocolo propio **ESB-P** (*ESP-NOW Sensor Broadcast Protocol*, ver [`ESB-P_ESP-NOW SENSOR BROADCAST PROTOCOL DRAFT 1.pdf`](<./ESB-P_  ESP-NOW SENSOR BROADCAST PROTOCOL           DRAFT 1.pdf>)). Cada trama tiene:

```c
typedef struct __attribute__((packed)) {
  char    deviceId[16];      // identifica el dispositivo
  char    broadcastType[16]; // tipo de dato: FORM, HR, VMEDIDO, ACCEL, FLEX...
  uint8_t payload[56];       // datos específicos de cada tipo
} espMessage;
```

Un único receptor ESP32 escucha todas las tramas, agrupa por `deviceId` + `broadcastType`, y sirve el último valor de cada uno en `GET /devices` como JSON.

## Dispositivos

| Carpeta | Dispositivo | Descripción |
|---|---|---|
| [`MasterReciever/`](MasterReciever) | Receptor | Proyecto PlatformIO (ESP32-S3). SoftAP + ESP-NOW, sirve `/devices`. Incluye además un sensor "Vest" propio a bordo (MPU-6050 + flex) que se muestrea localmente. |
| [`Wk/`](Wk) | Reloj | Pantalla LCD redonda + touch (CST816S) + sensor de ritmo cardíaco MAX30102, interfaz LVGL con asistente de formulario. Transmite `FORM` y `HR`. |
| [`Arduino/GSR/`](Arduino/GSR) | Sensor GSR | Lee la respuesta galvánica de la piel por ADC y transmite `VMEDIDO`. |
| [`GSR sensor/`](GSR%20sensor) | Sensor GSR (PCB) | Diseño KiCad del PCB del sensor GSR, con archivos de fabricación (gerbers). |
| [`PCB_Gafas/`](PCB_Gafas) | Gafas | Diseño KiCad del PCB del dispositivo de gafas (acelerómetro, `ACCEL`). |
| [`PCB_Pecho/`](PCB_Pecho) | Pecho | Diseño KiCad del PCB del dispositivo de pecho (acelerómetro + flex, `ACCEL`/`FLEX`). |
| [`Fuente Fundamentos/`](<Fuente Fundamentos>) · [`PCB_Fuente/`](PCB_Fuente) | Fuente de alimentación | Diseño de la fuente/boost converter (Altium y KiCad respectivamente). |
| [`Arduino/ESP32RECEPTORA/`](Arduino/ESP32RECEPTORA) | Receptor (versión anterior) | Primera versión del receptor, solo maneja Reloj (`FORM`/`HR`) y GSR (`VMEDIDO`). Reemplazada por `MasterReciever/`. |

## Hardware / PCBs (KiCad)

`KiCad_FootPrints/` contiene las librerías de footprints y símbolos usadas por los diseños anteriores:

- `usini_kicad_sensors-main/` — librería de sensores I2C de terceros (ADXL345, BMP180, MPL3115A2, PAJ7620, SI7021, TSL2561, INA219, BH1750, BME280/680, MAX30102, MPU6050, VL53L0X, MLX90614, conector de audio TRRS).
- `XIAO_ESP32C3-main/` — símbolo/footprint del módulo Seeed XIAO ESP32C3.
- `ESP32-S3/`, `BoostConverter/`, `CostumFuse/`, `Transformer/` — footprints propios adicionales.

## Compilar y flashear

- **`MasterReciever/`** — proyecto PlatformIO, board `esp32-s3-devkitc-1`, framework Arduino. Compilar con `pio run` o desde la extensión de PlatformIO.
- **`Wk/`** (`Wk.ino`) y **`Arduino/*/`** — sketches de Arduino IDE, requieren el core de ESP32 y las librerías `lvgl`, `TFT_eSPI`, `MAX30105`/`heartRate`, `esp_now`.

## Documentación

- [`ESB-P_ESP-NOW SENSOR BROADCAST PROTOCOL DRAFT 1.pdf`](<./ESB-P_  ESP-NOW SENSOR BROADCAST PROTOCOL           DRAFT 1.pdf>) — especificación del protocolo de comunicación entre dispositivos y receptor.
- [`Informe_Final.pdf`](Informe_Final.pdf) — informe final del proyecto.
