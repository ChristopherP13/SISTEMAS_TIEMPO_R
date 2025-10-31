#ifndef RGB_LED_H
#define RGB_LED_H
/*──────────────────────────────────────────────────────────────────────────────
  INCLUDE GUARDS (protección contra inclusiones múltiples)
  ------------------------------------------------------------------------------
  - Este par #ifndef / #define / #endif evita que el preprocesador procese el
    mismo header más de una vez por unidad de traducción.
  - ¿Por qué es importante? Si otro .h (o el propio .c) incluye este archivo
    directa o indirectamente varias veces, sin guards aparecerían errores por
    redefinición de tipos/funciones.
  - Convención: el macro suele derivarse del nombre del archivo, en MAYÚSCULAS
    y con guiones bajos. Aquí: RGB_LED_H. Perfecto.
──────────────────────────────────────────────────────────────────────────────*/

#include <stdint.h>
/*──────────────────────────────────────────────────────────────────────────────
  <stdint.h> (estándar C99)
  ------------------------------------------------------------------------------
  - Provee definiciones de tipos de ancho fijo como uint8_t, int32_t, etc.
  - Este header ES NECESARIO porque abajo usamos uint8_t en rgb_color_t.
  - No dependas de que "otro" header lo incluya indirectamente: inclúyelo tú.
──────────────────────────────────────────────────────────────────────────────*/

#include "driver/ledc.h"
/*──────────────────────────────────────────────────────────────────────────────
  driver/ledc.h (ESP-IDF)
  ------------------------------------------------------------------------------
  - API del periférico LEDC (LED Controller) del ESP32: PWM por hardware con
    temporizador + canales. Permite ajustar frecuencia, resolución y duty cycle.
  - ¿Por qué aquí en el .h y no solo en el .c?
      * Porque la interfaz pública está conceptualmente ligada a LEDC (PWM),
        y los usuarios del módulo pueden querer conocer esa dependencia.
      * Además, algunas apps podrían necesitar tipos LEDC en firmas futuras.
  - Si quisieras desacoplar más el header de ESP-IDF, podrías mover este include
    al .c y ocultar los detalles, pero en esta versión es aceptable.
──────────────────────────────────────────────────────────────────────────────*/


// ============================================================================
// --- NUEVAS ESTRUCTURAS PÚBLICAS (parte del contrato/ABI de la librería) ---
// ============================================================================

/**
 * @brief Describe el mapeo físico de pines GPIO para un LED RGB.
 *
 *  - Cada campo es un número de GPIO (por ejemplo: 25, 26, 27).
 *  - Los pines deben ser compatibles con LEDC si piensas usar PWM (no todos
 *    los GPIO soportan todas las funciones; revisar documentación del ESP32).
 *  - Esta estructura es INTENCIONALMENTE simple: solo asignación de pines.
 *    * Ventaja: el API queda estable aunque decidas cambiar frecuencia o
 *      resolución PWM internamente sin romper a los usuarios.
 *    * Extensiones futuras: si luego necesitas exponer freq/resolución por
 *      configuración, podrías crear otro struct rgb_led_cfg_t sin romper éste.
 */
typedef struct {
    int red_pin;    ///< GPIO asignado al canal ROJO   (ej: 25)
    int green_pin;  ///< GPIO asignado al canal VERDE  (ej: 26)
    int blue_pin;   ///< GPIO asignado al canal AZUL   (ej: 27)
} rgb_led_pins_t;
/*──────────────────────────────────────────────────────────────────────────────
  Notas de diseño:
  ------------------------------------------------------------------------------
  - Tipo elegido: int. En ESP-IDF también existe gpio_num_t (enum). Usar int
    hace el header un poco más "agnóstico" pero pierde chequeo de tipo fuerte.
  - Recomendación en proyectos grandes: documentar explícitamente si el LED es
    CÁTODO COMÚN o ÁNODO COMÚN. Si es ánodo común, quizá haya que invertir
    el duty internamente (duty_invertido = max_duty - duty_normal).
──────────────────────────────────────────────────────────────────────────────*/


/**
 * @brief Representa un color en espacio RGB de 8 bits por canal (0–255).
 *
 *  - La elección de uint8_t mapea naturalmente a resoluciones PWM de 8 bits.
 *  - Si internamente LEDC usa más bits (p. ej., 10–13), se puede escalar:
 *      duty = (valor * ((1 << resolution_bits) - 1)) / 255
 *  - Este tipo favorece APIs atómicas (pasas el color completo coherente en un
 *    solo argumento) evitando cambios parciales e inconsistentes.
 */
typedef struct {
    uint8_t r;  ///< Intensidad ROJO   [0..255]
    uint8_t g;  ///< Intensidad VERDE  [0..255]
    uint8_t b;  ///< Intensidad AZUL   [0..255]
} rgb_color_t;
/*──────────────────────────────────────────────────────────────────────────────
  Consideraciones de percepción y hardware:
  ------------------------------------------------------------------------------
  - Gamma: el ojo humano no percibe linealmente. Una rampa 0..255 no “se ve”
    lineal. Si te preocupa precisión visual, aplica corrección gamma en la
    implementación antes de fijar el duty.
  - Encapsulación: es mejor tratar el color como entidad única para soportar
    futuras operaciones (fade, cross-fade, HSV→RGB) sin cambiar la firma.
──────────────────────────────────────────────────────────────────────────────*/


// ============================================================================
// --- PROTOTIPOS DE FUNCIONES (INTERFAZ PÚBLICA DEL MÓDULO) -------------------
// ============================================================================

/**
 * @brief Inicializa el subsistema de LED RGB:
 *        - Configura el temporizador LEDC (frecuencia + resolución).
 *        - Configura TRES canales LEDC (uno por cada componente R/G/B).
 *        - Asocia cada canal al GPIO correspondiente y deja duty=0 (apagado).
 *
 * @param pins Puntero a la estructura con los GPIO para R, G y B.
 *
 * Reglas y expectativas:
 *  - DEBE llamarse UNA VEZ antes de rgb_led_set_color().
 *  - No retiene (no modifica) *pins; puedes pasarle un literal compuesto
 *    (ej. (rgb_led_pins_t){25,26,27}).
 *  - Errores típicos a validar en la IMPLEMENTACIÓN (.c):
 *      * Comprobar que pins != NULL (defensivo).
 *      * Asegurar que los pines son válidos y están libres (si procede).
 *      * Configurar frecuencia/resolución PWM adecuadas (p. ej., 1 kHz, 8 bits).
 *  - Consideración de thread-safety:
 *      * Inicializar LEDC normalmente se hace en un único hilo (app_main) antes
 *        de arrancar tareas que llamen set_color. Si piensas permitir reinicialización,
 *        protege internamente con un mutex o “once flag”.
 */
void rgb_led_init(const rgb_led_pins_t *pins);


/**
 * @brief Fija el color del LED RGB en una única operación lógica.
 *
 * @param color Estructura con intensidades R, G, B en [0..255] cada una.
 *
 * Contrato y comportamiento esperado:
 *  - Llamable desde cualquier contexto de tarea (NO desde ISR si la
 *    implementación usa llamadas LEDC bloqueantes; si lo necesitas, crea
 *    una variante *_from_isr).
 *  - Debe convertir cada byte a duty PWM según la resolución LEDC vigente.
 *  - Debe llamar a ledc_set_duty(...) y ledc_update_duty(...) por canal.
 *  - Debe manejar internamente inversiones si el hardware es ánodo común,
 *    o exponer una opción de configuración en init si prefieres.
 *
 * Rendimiento:
 *  - Tres llamadas set_duty + tres update_duty por invocación; en 1 kHz de
 *    cambios no suele ser problema. Para fades rápidos, considera colas y
 *    una tarea exclusiva de animación que “suavice” transiciones.
 */
void rgb_led_set_color(rgb_color_t color);


// (Opcional a futuro)
// void rgb_led_deinit(void);
// void rgb_led_set_gamma(float gamma);
// void rgb_led_set_inverted(bool inverted);
// etc.

#endif // RGB_LED_H
/*──────────────────────────────────────────────────────────────────────────────
  FIN DEL INCLUDE GUARD
  ------------------------------------------------------------------------------
  - El comentario al final ayuda a visualizar qué #endif cierra qué #if(n)def
    cuando los archivos tienen varias secciones y directivas anidadas.
──────────────────────────────────────────────────────────────────────────────*/