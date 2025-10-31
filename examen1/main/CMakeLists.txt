# Declara los archivos de código fuente que componen tu componente "main"
set(SOURCES "uart_events_example_main.c" "rgb_led.c")

# Registra el componente en el sistema de construcción de ESP-IDF
idf_component_register(SRCS ${SOURCES}
                       # Indica que los archivos .h están en el directorio actual (main/)
                       INCLUDE_DIRS "."
                       # Declara las dependencias con otros componentes de ESP-IDF
                       PRIV_REQUIRES driver esp_adc)