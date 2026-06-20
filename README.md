# Prueba de Comunicación ESP32

Este directorio contiene los archivos necesarios para realizar una prueba básica de comunicación serial (vía USB) con un ESP32, sin necesidad de conectarle ningún sensor o actuador adicional.

## Archivos

- `test_esp32.ino`: Código en C++ para cargar en el ESP32 utilizando Arduino IDE o PlatformIO.
- `test_serial.py`: Script en Python para ejecutarse en tu computadora y verificar que se puedan enviar y recibir mensajes al ESP32.

## Pasos para la prueba

### 1. Cargar el código en el ESP32
1. Abre el archivo `test_esp32.ino` en Arduino IDE.
2. Selecciona la placa (ej. "DOIT ESP32 DEVKIT V1") y el puerto correcto (`/dev/ttyUSB0` en Linux generalmente).
3. Compila y sube el código al ESP32.

### 2. Ejecutar la prueba desde la computadora
Asegúrate de tener instalada la librería `pyserial`. Si no la tienes, instálala con:
```bash
pip install pyserial
```

Luego, ejecuta el script de Python:
```bash
python test_serial.py
```

Si tu ESP32 está en un puerto diferente (por ejemplo, `/dev/ttyUSB1`), puedes pasarlo como argumento:
```bash
python test_serial.py /dev/ttyUSB1
```

### Resultados esperados
El script enviará la palabra `PING` por comunicación serial. El ESP32 deberá recibirla e inmediatamente contestar `PONG - La comunicacion bidireccional funciona correctamente!`. Todo esto se imprimirá en la consola de tu computadora, confirmando que la transmisión y recepción de datos es exitosa.
