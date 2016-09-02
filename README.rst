***********************************
**Control de bobinadora** - Arduino
***********************************

Sencillo programa para Arduino que implementa un menú de selección y edición
de parámetros para el control de una bobinadora de hilo para impresión 3D, mediante un
LCD de 2x16, 2 botones simples, y un rotary encoder.

- Selección entre 10 programas, cada uno con una configuración de:
   * numero_movimientos,
   * distancia_movimiento_num_pasos,
   * velocidad_movimiento_rpm,
   * num_periodos_freno,
   * distancia_offset_num_pasos;
- Control de rangos de variables;
- Persistencia en EEPROM de cambios de configuración;
- Interfaz vía rotary encoder para selección de programa, selección de variable,
  y ajuste del valor de la variable. Control mediante click, y salida de configuración con doble click,
  con guardado en EEPROM si hay cambios en la configuración del programa;
- Control de bobinadora con stepper para dirigir el enrolle y motor devanador:
   * Sensor de final de carrera para ajuste inicial del cabezal,
   * 2x Sensores de entrada en PARO por fallo,
   * Botón de MARCHA / REANUDACIÓN:
       - Inicio de programa de bobinado con ajuste inicial.
         (ajusta el cabezal y espera confirmación para inicio de programa)
       - Reanudación directa del programa tras PARO.
   * Botón de STOP / RESET:
       - Entrada en PARO manual.
       - 2º vez para RESET de programa.

-------------------------------

- Dependencias:

.. code:: bash

    ClickEncoder, elapsedMillis, LCD, LiquidCrystal_I2C (F Malpartida), TimerOne.
    EEPROM, Stepper, Wire.
..