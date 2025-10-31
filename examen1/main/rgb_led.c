#include "rgb_led.h"
/*──────────────────────────────────────────────────────────────────────────────
  Dependencia del header del módulo
  ------------------------------------------------------------------------------
  - Expone las firmas públicas (rgb_led_init, rgb_led_set_color) y los tipos
    rgb_led_pins_t / rgb_color_t.
  - rgb_led.h ya incluye <stdint.h> (para uint8_t) y "driver/ledc.h" (API LEDC).
  - No se incluyen headers extra aquí para no duplicar dependencias.
──────────────────────────────────────────────────────────────────────────────*/


// Definiciones de canales y resolución (sin cambios)
/*──────────────────────────────────────────────────────────────────────────────
  Mapeo lógico de canales LEDC
  ------------------------------------------------------------------------------
  - El periférico LEDC del ESP32 ofrece varios canales numerados (0..7 en HS/LS).
  - Aquí fijamos un mapeo estable: R→CH0, G→CH1, B→CH2. Mantenerlo constante
    simplifica el razonamiento y el debug.
  - NOTA: Si en otro lugar del proyecto usas LEDC con los mismos canales, debes
    coordinar para no reconfigurarlos accidentalmente (o usar otro timer/ch).
──────────────────────────────────────────────────────────────────────────────*/
#define LEDC_CHANNEL_R LEDC_CHANNEL_0   // Canal PWM asignado al ROJO
#define LEDC_CHANNEL_G LEDC_CHANNEL_1   // Canal PWM asignado al VERDE
#define LEDC_CHANNEL_B LEDC_CHANNEL_2   // Canal PWM asignado al AZUL

/*──────────────────────────────────────────────────────────────────────────────
  Resolución del duty (LEDC_TIMER_8_BIT)
  ------------------------------------------------------------------------------
  - 8 bits ⇒ duty válido en [0..255]. Coincide 1:1 con rgb_color_t.{r,g,b}.
  - Si quisieras más resolución (10–13 bits), cambia esta constante y recuerda
    ESCALAR en rgb_led_set_color (p.ej. duty = color.r * (2^res - 1) / 255).
──────────────────────────────────────────────────────────────────────────────*/
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT


// --- FUNCIÓN DE INICIALIZACIÓN MODIFICADA ---
void rgb_led_init(const rgb_led_pins_t *pins) {
/*──────────────────────────────────────────────────────────────────────────────
  Propósito
  ------------------------------------------------------------------------------
  - Configurar el temporizador LEDC (frecuencia + resolución de duty).
  - Configurar y asociar TRES canales (R,G,B) a sus GPIO correspondientes.
  - Dejar el LED apagado inicialmente (duty = 0 en cada canal).
  Consideraciones:
  - Esta función DEBE llamarse antes de rgb_led_set_color().
  - No valida 'pins' ni disponibilidad de GPIO/LEDC (misma funcionalidad pedida).
    En producción añadirías:
      if (!pins) return/ESP_ERROR_CHECK(...);
──────────────────────────────────────────────────────────────────────────────*/

    // 1. Configuración del temporizador LEDC (sin cambios)
    /*------------------------------------------------------------------------
      ledc_timer_config_t
      - speed_mode: HIGH o LOW speed mode. LOW es suficiente para ~kHz
        y libera los canales HS si los necesitas para otras tareas.
      - timer_num: cuál de los timers LEDC (0..3). Usamos TIMER_0 para RGB.
      - duty_resolution: #bits del duty; aquí LEDC_DUTY_RES = 8 bits.
      - freq_hz: frecuencia de PWM. 5 kHz es un buen compromiso: por encima
        de la banda audible y con buena linealidad en LED comunes.
      - clk_cfg: fuente de reloj (AUTO selecciona óptima según el SoC).
    ------------------------------------------------------------------------*/
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,  // Modo bajo (suficiente para LED y más flexible en pines)
        .timer_num        = LEDC_TIMER_0,         // Usamos el Timer 0 para los tres canales
        .duty_resolution  = LEDC_DUTY_RES,        // Resolución definida arriba (8 bits → duty 0..255)
        .freq_hz          = 5000,                 // 5 kHz: evita parpadeo/audible; apto para la mayoría de LED
        .clk_cfg          = LEDC_AUTO_CLK         // Selección automática del reloj por el driver
    };
    ledc_timer_config(&ledc_timer);
    /*------------------------------------------------------------------------
      - ledc_timer_config() programa registros del timer (prescalers, etc.).
      - No devuelve esp_err_t en esta forma (en headers modernos sí lo hace).
      - Si quisieras auditar errores, usa ESP_ERROR_CHECK(ledc_timer_config(&...))
        con la firma que devuelva esp_err_t.
    ------------------------------------------------------------------------*/

    // 2. Configuración del canal para el color ROJO
    //    Ahora usamos pins->red_pin para acceder al GPIO
    /*------------------------------------------------------------------------
      ledc_channel_config_t
      - channel: qué canal LEDC usaremos (ver #defines arriba).
      - duty: duty inicial. 0 → LED apagado.
      - gpio_num: el pin físico que emitirá la señal PWM de este canal.
      - speed_mode: debe coincidir con el del timer (LOW_SPEED aquí).
      - hpoint: desplazamiento de fase (en ticks). 0 = sin desfase (común).
      - timer_sel: a qué timer está atado este canal. Usamos TIMER_0 para todos
                   los canales RGB, de modo que compartan freq/resolución.
    ------------------------------------------------------------------------*/
    ledc_channel_config_t ledc_channel_r = {
        .channel    = LEDC_CHANNEL_R,         // Canal lógico reservado para ROJO
        .duty       = 0,                      // Apagado al iniciar
        .gpio_num   = pins->red_pin,          // GPIO físico para ROJO (p.ej., 25)
        .speed_mode = LEDC_LOW_SPEED_MODE,    // Debe coincidir con el timer configurado
        .hpoint     = 0,                      // Sin desplazamiento de fase
        .timer_sel  = LEDC_TIMER_0            // Asocia este canal al Timer 0
    };
    ledc_channel_config(&ledc_channel_r);
    /*------------------------------------------------------------------------
      - El driver configurará el mux del pin al periférico y lo dejará listo.
      - Si el pin elegido no soporta esa función en ese modo, el driver fallará.
    ------------------------------------------------------------------------*/

    // 3. Configuración del canal para el color VERDE
    //    Ahora usamos pins->green_pin
    ledc_channel_config_t ledc_channel_g = {
        .channel    = LEDC_CHANNEL_G,         // Canal lógico para VERDE
        .duty       = 0,                      // Apagado al iniciar
        .gpio_num   = pins->green_pin,        // GPIO físico para VERDE (p.ej., 26)
        .speed_mode = LEDC_LOW_SPEED_MODE,    // Consistente con el timer
        .hpoint     = 0,                      // Sin desplazamiento de fase
        .timer_sel  = LEDC_TIMER_0            // Misma base temporal que ROJO/AZUL
    };
    ledc_channel_config(&ledc_channel_g);

    // 4. Configuración del canal para el color AZUL
    //    Ahora usamos pins->blue_pin
    ledc_channel_config_t ledc_channel_b = {
        .channel    = LEDC_CHANNEL_B,         // Canal lógico para AZUL
        .duty       = 0,                      // Apagado al iniciar
        .gpio_num   = pins->blue_pin,         // GPIO físico para AZUL (p.ej., 27)
        .speed_mode = LEDC_LOW_SPEED_MODE,    // Consistente con el timer
        .hpoint     = 0,                      // Sin desplazamiento de fase
        .timer_sel  = LEDC_TIMER_0            // Misma base temporal que R/G
    };
    ledc_channel_config(&ledc_channel_b);
    /*------------------------------------------------------------------------
      Resultado esperado:
      - Tres salidas PWM inicializadas y desactivadas, compartiendo el mismo
        temporizador (misma frecuencia y resolución).
      - Lista la capa de hardware para recibir cambios de duty en set_color().
    ------------------------------------------------------------------------*/
}


// --- FUNCIÓN PARA ESTABLECER COLOR MODIFICADA ---
void rgb_led_set_color(rgb_color_t color) {
/*──────────────────────────────────────────────────────────────────────────────
  Propósito
  ------------------------------------------------------------------------------
  - Ajustar el ciclo de trabajo (duty) de cada canal (R,G,B) según el color
    solicitado (bytes 0..255), y aplicar la actualización a los registros
    de hardware con ledc_update_duty().
  - Mantiene exactamente la funcionalidad pedida: sin escalado ni inversión.
  Notas:
  - Con resolución de 8 bits, el valor del duty coincide con color.{r,g,b}.
  - Si en el futuro cambias la resolución del timer, aquí deberías ESCALAR.
──────────────────────────────────────────────────────────────────────────────*/

    // Actualizar el ciclo de trabajo para cada canal usando color.r, color.g, color.b
    /*------------------------------------------------------------------------
      - ledc_set_duty() solo escribe el valor en la estructura de control /
        registro shadow del canal; NO toma efecto hasta que llames a
        ledc_update_duty(), que sincroniza con el hardware.
      - Separar set_duty y update_duty por canal te permite construir el
        nuevo estado y luego aplicarlo. Aquí actualizamos cada canal de forma
        inmediata tras el set, lo cual es correcto y simple.
    ------------------------------------------------------------------------*/
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_R, color.r);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_R);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_G, color.g);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_G);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_B, color.b);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_B);

    /*------------------------------------------------------------------------
      Consideraciones prácticas:
      - Ánodo común: si tu LED es de ánodo común, aumentar duty puede APAGAR
        el canal (lógica invertida). En ese caso, podrías invertir:
          duty_hw = MAX_DUTY - color.x
        Pero eso sería un CAMBIO de funcionalidad; por eso aquí lo dejamos
        tal cual y solo lo documentamos.
      - Gamma: la respuesta visual del LED no es lineal. Para un control más
        “perceptivamente” lineal, aplica una LUT gamma a color.{r,g,b} antes
        del set_duty. Nuevamente, sería un cambio funcional, no se aplica aquí.
      - Atomicidad: si quisieras que los tres canales se actualicen EXACTAMENTE
        al mismo tiempo, podrías setear los tres duties primero y luego llamar
        update_duty para cada canal; la diferencia temporal aquí es despreciable
        a 5 kHz para la gran mayoría de aplicaciones de iluminación.
    ------------------------------------------------------------------------*/
}