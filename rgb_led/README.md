Controlador de LED RGB con Botón para ESP32

1. Descripción General

Este proyecto te permite controlar un LED RGB conectado a un ESP32. Al encender el dispositivo, el LED se ilumina en rojo. Cada vez que presionas el botón "BOOT" de la placa, el color del LED cambia siguiendo una secuencia predefinida de siete colores.
El código está dividido en dos partes principales:
Una librería reutilizable (rgb_led.h y rgb_led.c) para manejar cualquier LED RGB.
Un programa principal (app_main.c) que utiliza la librería y añade la lógica para cambiar de color con un botón.

2. Requisitos de Hardware

Una placa de desarrollo ESP32.
Un LED RGB.
3 resistencias (valor usado: 220Ω).
Protoboard y jumpers.

3. Esquema de Conexión

Debes conectar los componentes de la siguiente manera. Los pines GPIO están definidos en el código, pero puedes cambiarlos si lo necesitas.

Pin Rojo del LED → Resistencia → GPIO 12 del ESP32.
Pin Verde del LED → Resistencia → GPIO 14 del ESP32.
Pin Azul del LED → Resistencia → GPIO 13 del ESP32.
Pin Común (Cátodo, el más largo) → GND del ESP32.

El botón BOOT ya está integrado en la placa y conectado internamente al GPIO 0, por lo que no necesitas conectar ningún botón externo.

4. Funcionamiento

Al iniciar, el LED se encenderá en color rojo.
Presiona el botón BOOT de la placa.
El color del LED cambiará al siguiente en la secuencia. La secuencia completa es:
Rojo
Verde
Azul
Amarillo
Magenta
Cian
Blanco
(Vuelve a empezar en Rojo)
El monitor serie abierto, verás mensajes de depuración que indican cuándo se inicializa el sistema y a qué color se cambia tras cada pulsación.

5. Descripción del Código

El proyecto se organiza en las siguientes funciones clave:
Librería rgb_led:

void rgb_led_init(int red_gpio, int green_gpio, int blue_gpio);
Propósito: Configura el controlador PWM (LEDC) del ESP32.
Uso: Debes llamarla una sola vez al inicio de tu programa para decirle a la librería qué pines GPIO vas a usar para cada color.
void rgb_led_set_color(uint8_t red, uint8_t green, uint8_t blue);
Propósito: Cambia el color del LED.
Uso: Le pasas tres valores entre 0 (apagado) y 255 (máximo brillo) para definir la intensidad de cada color primario. Por ejemplo, rgb_led_set_color(255, 0, 255); producirá un color magenta.

Programa Principal app_main:
void button_init(void);
Propósito: Configura el GPIO 0 (botón BOOT) como una entrada y activa una interrupción. La interrupción permite que el ESP32 reaccione instantáneamente a la pulsación del botón sin tener que comprobar su estado constantemente.

static void IRAM_ATTR button_isr_handler(void* arg);
Propósito: Es la función que se ejecuta cuando se presiona el botón. Incluye un mecanismo antirrebote (debounce) para evitar que una sola pulsación se detecte como varias. Su única tarea es "avisar" al bucle principal de que el botón ha sido presionado.

void app_main(void);
Propósito: Es el corazón del programa.
Llama a rgb_led_init() y button_init() para configurar el hardware.
Establece el color inicial en rojo.
Entra en un bucle infinito donde espera la señal de la interrupción del botón.
Cuando recibe la señal, actualiza el color del LED al siguiente de la secuencia.

6. Personalización
Cambiar los pines del LED: Modifica los valores en las siguientes líneas del archivo app_main:

    #define RGB_LED_RED_GPIO    12
    #define RGB_LED_GREEN_GPIO  14
    #define RGB_LED_BLUE_GPIO   13

Cambiar la secuencia de colores: Modifica el bloque switch (color_index) dentro del bucle while(1) en app_main. Puedes añadir, eliminar o cambiar los colores a tu gusto. Si cambias el número total de colores, no olvides actualizar el número en esta línea:


    // Si tienes 8 colores en total, cámbialo a % 8
    color_index = (color_index + 1) % 7;