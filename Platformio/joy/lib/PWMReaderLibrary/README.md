# PWMReader Library

La **PWM Reader Library** es una biblioteca de Arduino diseñada para facilitar la lectura de señales PWM utilizando interrupciones. 

## Características

- Lee el ciclo de trabajo, el período y el tiempo del último flanco de subida de una señal PWM.
- Habilita y deshabilita las interrupciones para evitar condiciones de carrera.

## Instalación

1. Descarga la biblioteca como un archivo ZIP desde [este enlace](https://github.com/racarla96/PWMReaderLibrary/archive/main.zip).

2. Abre el entorno de desarrollo de Arduino.

3. Ve a `Sketch > Include Library > Add .ZIP Library`.

4. Selecciona el archivo ZIP que descargaste en el paso 1.

## Uso

```cpp
#include "PWMReader.hpp"

// Define an instance of the PWMReader class
int PWMreader_pin = 25;
PWMReader pwmReader;

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);

  // Initialize the PWMReader for pin defined on PWMreader_pin
  if (pwmReader.begin(PWMreader_pin)) {
    Serial.println("PWMReader initialized successfully!");
  } else {
    Serial.println("Failed to initialize PWMReader. Check the pin number.");
    while(1);
  }
}

void loop() {
  // Read and print PWM signal information
  unsigned long dutyCycle = pwmReader.getDutyCicle_us();
  unsigned long period = pwmReader.getPeriod_us();
  unsigned long lastRisingEdge = pwmReader.getTimeLastRisingEdge_us();

  Serial.print("Duty Cycle (us): ");
  Serial.println(dutyCycle);

  Serial.print("Period (us): ");
  Serial.println(period);

  Serial.print("Time of Last Rising Edge (us): ");
  Serial.println(lastRisingEdge);

  delay(1000); // Wait for 1 second before reading again
}

```

## Contribuciones
¡Las contribuciones son bienvenidas! Si encuentras errores o tienes sugerencias para mejorar esta biblioteca, crea un problema en el repositorio de GitHub.

## Licencia
Esta biblioteca está bajo la Licencia MIT. Consulta el archivo LICENSE para obtener más información.
