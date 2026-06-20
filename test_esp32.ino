void setup() {
  // Inicializamos la comunicación serial a 115200 baudios (muy común para el ESP32)
  Serial.begin(115200);
  
  // Pequeña pausa para estabilizar
  delay(1000);
  
  // Enviamos un mensaje inicial para confirmar que el ESP32 encendió
  Serial.println("ESP32 iniciado y listo para recibir comandos.");
}

void loop() {
  // Verificamos si hay datos disponibles en el puerto serial
  if (Serial.available() > 0) {
    // Leemos hasta encontrar un salto de línea
    String comando = Serial.readStringUntil('\n');
    
    // Limpiamos los espacios y caracteres invisibles (como \r)
    comando.trim();
    
    // Si recibimos algún texto
    if (comando.length() > 0) {
      Serial.print("ESP32 recibio: ");
      Serial.println(comando);
      
      // Verificamos si es el comando de prueba "PING"
      if (comando == "PING") {
        Serial.println("PONG - La comunicacion bidireccional funciona correctamente!");
      }
    }
  }
}
