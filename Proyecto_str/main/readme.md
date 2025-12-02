PROYECTO: Ventilador Inteligente para Cuna con ESP32 — Requerimientos
1. Descripción general
Diseñar un sistema de ventilación inteligente usando un ESP32, un sensor PIR, un sensor de
temperatura,
un ventilador DC con control PWM y una interfaz web. El sistema debe tener tres modos de operación:
Manual, Automático y Programado por Registros. En modos automáticos o programados el ventilador
solo
puede funcionar si se detecta presencia mediante el PIR.
El proyecto debe permitir actualización del firmware mediante OTA (Over-The-Air).
2. Modos de operación
2.1. Modo Manual
- El ventilador no depende del PIR ni de la temperatura.
- El usuario debe seleccionar un valor de PWM (0–100%) desde la página web.
- Este valor debe quedar almacenado en la flash del programa.
- El ventilador debe usar exactamente ese valor de PWM.

2.2. Modo Automático (presencia + PWM proporcional)
Si el PIR no detecta presencia → 0% PWM.
Si hay presencia:
- T_min → 0% PWM
- T_max → 100% PWM
- Entre T_min y T_max → PWM proporcional según la temperatura.
Los valores T_min y T_max deben guardarse en la flash del programa.
2.3. Modo Programado por Registros (mínimo 3 registros)
Cada registro debe incluir:
- Hora de inicio

- Hora de fin
- Temperatura T_0% (para 0% PWM)
- Temperatura T_100% (para 100% PWM)
- Activar / Desactivar
Todos los registros deben guardarse en la flash del programa y poder consultarse en la web.
Comportamiento general:
- El ventilador solo funciona si hay presencia.
- Solo funciona durante los intervalos definidos por un registro activo.
- Si la hora coincide con un registro activo: PWM proporcional entre T_0% y T_100%.
- Si no coincide con ningún registro activo: 0%.
3. Requerimientos del servidor web
3.1 Estado actual
Debe mostrar:
- Temperatura actual
- Estado del PIR
- Modo actual
- PWM actual

3.2 Selección de modo
El usuario debe poder elegir entre: Manual, Automático, Programado.
3.3 Modo Manual
- Control de PWM (0–100%)
- Guardar en la flash del programa
3.4 Modo Automático

- Campos: T_min, T_max
- Guardar valores en la flash

3.5 Modo Programado por Registros
Para cada uno de los 3 registros:
- Activo / Inactivo
- Hora inicio
- Hora fin
- T_0%
- T_100%
La página debe mostrar también los valores guardados actualmente.
4. Requerimientos adicionales
- La hora debe obtenerse mediante NTP o equivalente.
- Todas las configuraciones deben guardarse en la flash del programa.
- El proyecto debe permitir actualización OTA del firmware.
- El sistema debe restaurar los valores guardados tras reiniciar.