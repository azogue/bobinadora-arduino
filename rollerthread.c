// ---------------------------------------------------------------------------
// Created by Eugenio Panadero on 25/08/16.
// Copyright 2016 - Under creative commons license 3.0:
//        Attribution-ShareAlike CC BY-SA
//
// This software is furnished "as is", without technical support, and with no
// warranty, express or implied, as to its usefulness for any purpose.
//
//
// * Name:     Arduino - Control de bobinadora
// * Autor:    Eugenio Panadero Maciá
// * Email:    eugenio.panadero@gmail.com
// * Date:     2016/08/25
//
// Control de bobinadora:
// - Selección entre 10 programas, cada uno con una configuración de:
//    uint16_t numero_movimientos;
//    uint16_t distancia_movimiento_num_pasos;
//    uint16_t velocidad_movimiento_rpm;
//    uint16_t decimas_segundo_freno;
//    uint16_t distancia_offset_num_pasos;
// - Control de rangos de variables;
// - Persistencia en EEPROM de cambios de configuración;
// - Interfaz vía rotary encoder para selección de programa, selección de
// variable, y ajuste del valor de la variable. Control mediante click, y
// salida de configuración con doble click, con guardado en EEPROM si hay
// cambios en la configuración del programa;
// - Bobinadora con stepper para dirigir el enrolle y motor devanador.
//    * Sensor de final de carrera para ajuste inicial del cabezal,
//    * 2x Sensores de entrada en PARO por fallo,
//    * Botón de MARCHA / REANUDACIÓN:
//        - Inicio de programa de bobinado con ajuste inicial
//        (ajusta el cabezal y espera confirmación para inicio de programa)
//        - Reanudación directa del programa tras PARO
//    * Botón de STOP / RESET:
//        - Entrada en PARO manual
//        - 2º vez para RESET de programa
//
// ---------------------------------------------------------------------------
/*
    Cambios 04/2017:

    - [X] Cambio de PINs para trabajar con PLC Leonardo
    - [X] Edicion de parámetros por cifras
    - [X] Fusión de sensores de fallo.
    - [X] Separación de frenos.
    - [X] Switch para usar LCD / monitor serie
    - [X] Optimización memoria para PLC Leonardo
    - [X] Modificación de frenado para comenzar a frenar ∆T antes de finalización de programa
*/

//**********************************
//** FLAGS *************************
//**********************************
// Switch para mostrar info en Serial
#define VERBOSE

// Switch para usar LCD 2x16 (vs Serial)
#define USE_LCD

// Switch para usar pines en PLC Mega
#define USE_PLC_MEGA

// Negación del pin enable driver del stepper
#define NEGATE_LOGIC_DEVANADOR

// Cambio sentido de devanador
#define MOVIMIENTO_EN_IDA_INICIAL       false

//**********************************
//** PINOUT ************************
//**********************************

#ifdef USE_PLC_MEGA
  #define PIN_BOTON_PARO_RESET      18    // 3           // Botón de PARO / RESET  (Interrupt)
  #define PIN_FALLO_1_2             19  // 19          // Señal de Fallo 1 o 2 (input único) (Interrupt)

  #define PIN_BOTON_START           14   // 4           // Botón de START / REANUDACIÓN
  #define PIN_FIN_CARRERA_HOME      15   // 7           // Sensor de final de carrera Home

  #define PIN_ROTARY_ENC_CLK        A2
  #define PIN_ROTARY_ENC_DT         A1
  #define PIN_ROTARY_ENC_SW         A0

  #define PIN_1_MOTOR_STEP          2   // 8           // Stepper PIN1 (step)
  #define PIN_2_MOTOR_STEP_DIR      3   // 9           // Stepper PIN2 (dirección)
  #define PIN_MOTOR_DEVANADOR       50  // 10          // Motor devanador (enable driver)
  #define PIN_MOTOR_VARIADOR        25  // 11          // Variador
  #define PIN_MOTOR_FRENO_1         22  // 12          // Freno 1
  #define PIN_MOTOR_FRENO_2         23                 // Freno 2
#else
  #define PIN_BOTON_PARO_RESET      0    // 3           // Botón de PARO / RESET  (Interrupt)
  #define PIN_FALLO_1_2             1  // 19          // Señal de Fallo 1 o 2 (input único) (Interrupt)

  #define PIN_BOTON_START           A3   // 4           // Botón de START / REANUDACIÓN
  #define PIN_FIN_CARRERA_HOME      A4   // 7           // Sensor de final de carrera Home

  #define PIN_ROTARY_ENC_CLK        A2  // A2
  #define PIN_ROTARY_ENC_DT         A1  // A1
  #define PIN_ROTARY_ENC_SW         A0  // A0

  #define PIN_1_MOTOR_STEP          3   // 8           // Stepper PIN1
  #define PIN_2_MOTOR_STEP_DIR      2   // 9           // Stepper PIN2

  #define PIN_MOTOR_DEVANADOR       9  // 10          // Motor devanador (enable driver)
  #define PIN_MOTOR_VARIADOR        8  // 11          // Variador
  #define PIN_MOTOR_FRENO_1         4  // 12          // Freno 1
  #define PIN_MOTOR_FRENO_2         7                 // Freno 2
#endif

//**********************************
//** Librerías *********************
//**********************************
#include <ClickEncoder.h>
#include <EEPROM.h>
#include <elapsedMillis.h>
#include <Stepper.h>
#include <TimerOne.h>
#include <FlexiTimer2.h>
#include <Wire.h>
#ifdef USE_LCD
  #include <LCD.h>
  #include <LiquidCrystal_I2C.h>  // F Malpartida's NewLiquidCrystal library
#endif

//**********************************
//** Configuración *****************
//**********************************
#define FORZAR_REGRABADO_DEFAULTS false       // Activando, se fuerza la re-escritura de los parámetros por defecto de los 10 programas

#define NUM_PROGRAMAS             10
#define NUM_OPCIONES              5
#define STEPS_MOTOR               200         // Stepper config.

#define DELAY_MS_CMD_MENU         500         // Delay en navegación en estados de edicion, para mostrar mensajes breves en el LCD, en ms
#define DELAY_ENTRE_PARO_RESET_MS 5000        // Tiempo mínimo entre estado de PARO y RESET a modo SELECC, en ms
#define DELAY_TIMEOUT_EDIT_VAR_MS 10000       // Tiempo de standby antes de la aceptación automática del valor editado.
#define TIEMPO_FRENADO_MS         5000        // Tiempo de activación del freno, en ms. Coincidente con el rango máximo del parámetro 'Freno segs'.

// Resto
#define LCD_CHARS                 16
#define LCD_LINES                 2

#define STATE_SELECC              0
#define STATE_MARCHA              1
#define STATE_PARO                2
#define STATE_CONFIG              3
#define STATE_RESET               4

#define TIPO_PARADA_FINALIZACION  0
#define TIPO_PARADA_MANUAL        1
#define TIPO_PARADA_EMERGENCIA    2

#define CASEBREAK(label) case label: break;

//**********************************
//** Variables *********************
//**********************************
// Textos LCD
const char* OPCIONES_PROGRAMA[] = {"- Num movs",
                                   "- Distanc. pasos",
                                   "- Velocidad rpm",
                                   "- Freno d_segs",
                                   "- Offset pasos"};
const char* OPCIONES_PROGRAMA_CORTAS[5] = {"N Movs", "Dist. NP", "Vel. rpm", "Freno d_seg", "Offset NP"};

// Configuraciones por defecto de los programas de bobinado
double PERIODO_FRENO_SEG = .1;
uint16_t OPCIONES_RANGO_MIN[5] = {0, 100, 10, 0, 0};
uint16_t OPCIONES_RANGO_MAX[5] = {1500, 3000, 400, (uint16_t)(TIEMPO_FRENADO_MS / 100), 1000};

uint16_t VALORES_DEFECTO_P1[5] = {43, 900, 127, 0, 70};
uint16_t VALORES_DEFECTO_P2[5] = {43, 900, 127, 0, 70};
uint16_t VALORES_DEFECTO_P3[5] = {43, 900, 127, 0, 70};
uint16_t VALORES_DEFECTO_P4[5] = {43, 900, 127, 0, 70};
uint16_t VALORES_DEFECTO_P5[5] = {43, 900, 127, 0, 70};
uint16_t VALORES_DEFECTO_P6[5] = {43, 900, 127, 0, 70};
uint16_t VALORES_DEFECTO_P7[5] = {43, 900, 127, 0, 70};
uint16_t VALORES_DEFECTO_P8[5] = {43, 900, 127, 0, 70};
uint16_t VALORES_DEFECTO_P9[5] = {43, 900, 127, 0, 70};
uint16_t VALORES_DEFECTO_P10[5] = {43, 900, 127, 0, 70};

// Struct de configuración de programa de bobinado
struct config_prog_t
{
  uint16_t numero_movimientos;
  uint16_t distancia_movimiento_num_pasos;
  uint16_t velocidad_movimiento_rpm;
  uint16_t decimas_segundo_freno;
  uint16_t distancia_offset_num_pasos;
};
const int eeAddressDelta = sizeof(config_prog_t);
config_prog_t programas[NUM_PROGRAMAS];

// Contadores de tiempo
elapsedMillis sinceStart;
elapsedMillis sinceStatus;
elapsedMillis sinceBrake;

// Variables 'volátiles' (se escriben en interrupciones)
volatile uint8_t estado_general;
volatile uint8_t tipo_parada;
volatile int8_t pin_fallo = -1;
volatile bool freno_activo;
volatile unsigned long activacion_freno_elapsedMillis;

// Otras variables de estado
uint8_t programa_seleccionado;
uint8_t variable_seleccionada;
bool interruptor;

uint8_t sensor_final_carrera = LOW;
uint8_t estadoBoton_start = 0;
uint8_t estadoBotonLast_start = 0;

// LCD
#ifdef USE_LCD
  LiquidCrystal_I2C lcd( 0x3F, 2,   1,  0,  4,  5,  6,  7, 3, POSITIVE);
#endif

// Rotary encoder
ClickEncoder *rotary_encoder;
bool hay_doble_click_rotary_enc = false;
bool hay_click_rotary_enc = false;
void timerIsr() {
  rotary_encoder->service();
}

// Stepper
Stepper myStepper(STEPS_MOTOR, PIN_1_MOTOR_STEP, PIN_2_MOTOR_STEP_DIR);  //(pasos por vuelta, step, dir)
uint16_t contador_vueltas_motor;
bool movimiento_en_ida;

//**********************************
//** SETUP INICIAL *****************
//**********************************

void setup()
{
  Serial.begin(9600);
  setup_rotary_encoder();
#ifdef USE_LCD
  setup_lcd();
#endif
  setup_motor_bobinadora();
  setup_configuraciones_programa();
  setup_inputs();
  setup_interrupts();

  programa_seleccionado = 0;
  variable_seleccionada = 0;
  interruptor = false;
  set_estado_seleccion();
}


//**********************************
//** Métodos para el setup *********
//**********************************

void setup_inputs()
{
  pinMode(PIN_BOTON_START, INPUT);
  pinMode(PIN_FIN_CARRERA_HOME, INPUT); // Final de carrera Home
}

void setup_interrupts()
{
  pinMode(PIN_BOTON_PARO_RESET, INPUT);
  pinMode(PIN_FALLO_1_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_BOTON_PARO_RESET), isr_set_estado_paro_o_reset_manual, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_FALLO_1_2), isr_set_estado_paro_por_fallo, RISING);
}

void setup_motor_bobinadora()
{
  pinMode(PIN_MOTOR_DEVANADOR, OUTPUT);   // Enable Motor Devanador
  pinMode(PIN_MOTOR_VARIADOR, OUTPUT);    // Relé Variador
  pinMode(PIN_MOTOR_FRENO_1, OUTPUT);     // Relé Freno
  pinMode(PIN_MOTOR_FRENO_2, OUTPUT);     // Relé Freno

  // Apagando motores al inicio
  desactiva_freno();              // Desactivamos freno
  i_opera_devanador(LOW); // Desactivamos motor devanador
  digitalWrite(PIN_MOTOR_VARIADOR, LOW);    // Desactivamos variador

  contador_vueltas_motor = 0;
  i_reset_direccion_devanador();
}

#ifdef USE_LCD
void setup_lcd()
{
  lcd.begin(LCD_CHARS, LCD_LINES);
  lcd.setBacklight(HIGH); // Activamos la retroiluminacion
  lcd.clear();
}
#endif

void setup_rotary_encoder()
{
  rotary_encoder = new ClickEncoder(PIN_ROTARY_ENC_CLK, PIN_ROTARY_ENC_DT, PIN_ROTARY_ENC_SW, 1);
  rotary_encoder->setAccelerationEnabled(false);

  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);
}

void setup_configuraciones_programa()
{
  bool es_primer_inicio;
  int eeAddress = 0;

  EEPROM.get(eeAddress, es_primer_inicio);

#ifdef VERBOSE
  Serial.print("es_primer_inicio: ");
  Serial.println(es_primer_inicio);
#endif
  if (es_primer_inicio | FORZAR_REGRABADO_DEFAULTS)
  {
#ifdef VERBOSE
    Serial.println("Se procede a grabar la EEPROM con los programas por defecto");
#endif
    eeAddress += sizeof(bool);
    eeAddress = eeprom_write_config_defecto(eeAddress, 0, VALORES_DEFECTO_P1);
    eeAddress = eeprom_write_config_defecto(eeAddress, 1, VALORES_DEFECTO_P2);
    eeAddress = eeprom_write_config_defecto(eeAddress, 2, VALORES_DEFECTO_P3);
    eeAddress = eeprom_write_config_defecto(eeAddress, 3, VALORES_DEFECTO_P4);
    eeAddress = eeprom_write_config_defecto(eeAddress, 4, VALORES_DEFECTO_P5);
    eeAddress = eeprom_write_config_defecto(eeAddress, 5, VALORES_DEFECTO_P6);
    eeAddress = eeprom_write_config_defecto(eeAddress, 6, VALORES_DEFECTO_P7);
    eeAddress = eeprom_write_config_defecto(eeAddress, 7, VALORES_DEFECTO_P8);
    eeAddress = eeprom_write_config_defecto(eeAddress, 8, VALORES_DEFECTO_P9);
    eeAddress = eeprom_write_config_defecto(eeAddress, 9, VALORES_DEFECTO_P10);
    EEPROM.put(0, false);  //Marca de configuración existente, para no sobreescribir en cada reinicio
  }

  //Lectura de configuraciones:
  eeprom_read_configuraciones();
}

void eeprom_read_configuraciones()
{
  int eeAddress = sizeof(bool);
  for (int i=0; i < NUM_PROGRAMAS; i++)
  {
    config_prog_t conf_i;

    EEPROM.get(eeAddress, conf_i);
    programas[i] = conf_i;
    eeAddress += eeAddressDelta;
#ifdef VERBOSE
    Serial.print("Leido PROGRAMA ");
    Serial.print(i + 1);
    Serial.println(" desde EEPROM:");
    print_config_programa(i);
#endif
  }
}

int eeprom_write_config_defecto(int eeAddress, int indice_programa, const uint16_t* valores_programa)
{
  config_prog_t conf_i;

  conf_i.numero_movimientos = valores_programa[0];
  conf_i.distancia_movimiento_num_pasos = valores_programa[1];
  conf_i.velocidad_movimiento_rpm = valores_programa[2];
  conf_i.decimas_segundo_freno = valores_programa[3];
  conf_i.distancia_offset_num_pasos = valores_programa[4];

  EEPROM.put(eeAddress, conf_i);
  eeAddress += eeAddressDelta;

#ifdef VERBOSE
  Serial.print("Se graba PROGRAMA ");
  Serial.print(indice_programa + 1);
  Serial.println(" en EEPROM: ");
  i_print_config_programa(indice_programa, conf_i);
  Serial.println("");
#endif
  return eeAddress;
}

void i_reset_direccion_devanador()
{
  movimiento_en_ida = MOVIMIENTO_EN_IDA_INICIAL;
}

//**********************************
//** Bucle general *****************
//**********************************

void loop()
{
  uint8_t estado_general_cycle;

  noInterrupts();
  estado_general_cycle = estado_general;
  interrupts();

  comprueba_tiempo_de_frenado();
  read_start_button();
  // Modos de funcionamiento: CONFIG / SELEC / MARCHA / RESET / PARO

  // Inicio de programa seleccionado
  if ((estado_general_cycle == STATE_SELECC) & (i_sube_flanco(estadoBotonLast_start, estadoBoton_start)))
  {
    set_texto_lcd(String("* RUNNING P_") + (programa_seleccionado + 1), String("Starting..."));
#ifdef VERBOSE
    Serial.print("Comienza el PROGRAMA ");
    Serial.println(programa_seleccionado + 1);
#endif
    set_estado_marcha(false, programas[programa_seleccionado].velocidad_movimiento_rpm);
  }
  // Aguarda cambio de programa o entrada a configuración de programa mediante rotary encoder
  else if (estado_general_cycle == STATE_SELECC)
  {
    seleccion_programa();
  }
  // Modo de edicion de variables del programa seleccionado
  else if (estado_general_cycle == STATE_CONFIG)
  {
    configuracion_parametros_programa_actual();
  }
  // En modo marcha...
  else if (estado_general_cycle == STATE_MARCHA)
  {
    cuenta_vueltas_motor(programas[programa_seleccionado].distancia_movimiento_num_pasos,
                         programas[programa_seleccionado].numero_movimientos,
                         programas[programa_seleccionado].decimas_segundo_freno);
  }
  else if (estado_general_cycle == STATE_RESET)
  {
    // Reset del programa seleccionado
    set_texto_fila_lcd(String("** RESET P_") + (programa_seleccionado + 1), 0);
#ifdef VERBOSE
    Serial.print("Se resetea el PROGRAMA_");
    Serial.println(programa_seleccionado + 1);
#endif
    delay(DELAY_MS_CMD_MENU);
    set_estado_seleccion();
  }
  // En modo paro, aguardando reset o reanudación
  else // if (estado_general_cycle == STATE_PARO)
  {
    if (tipo_parada != TIPO_PARADA_FINALIZACION)
    {
      i_opera_devanador(LOW);  // Desactivamos motor devanador
      sinceStart = 0;
      sinceStatus = 0;
      if (tipo_parada == TIPO_PARADA_EMERGENCIA)
      {
        set_texto_fila_lcd(String("** ERROR P_") + (programa_seleccionado + 1), 0);
#ifdef VERBOSE
        Serial.print("** PARADA DE EMERGENCIA del PROGRAMA_");
        Serial.println(programa_seleccionado + 1);
#endif
      }
      else
      {
        set_texto_fila_lcd(String("** STOP P_") + (programa_seleccionado + 1), 0);
#ifdef VERBOSE
        Serial.print("** PARADA MANUAL del PROGRAMA_");
        Serial.println(programa_seleccionado + 1);
#endif
      }
      tipo_parada = TIPO_PARADA_FINALIZACION;
    }

#ifdef VERBOSE
    // Para depurar sensores de fallo:
    if (pin_fallo != -1)
    {
      Serial.print("** Entrada en PARO por activacion de sensor en PIN: ");
      Serial.println(pin_fallo);
      pin_fallo = -1;
    }
#endif

    // Reanudación del programa seleccionado
    if (i_sube_flanco(estadoBotonLast_start, estadoBoton_start))
    {
      set_texto_lcd(String("* RE-RUN P_") + (programa_seleccionado + 1), String("En paro: ") + i_time_string(sinceStart));
#ifdef VERBOSE
      Serial.print("Se reanuda el PROGRAMA ");
      Serial.println(programa_seleccionado + 1);
#endif
      delay(DELAY_MS_CMD_MENU);
      set_estado_marcha(true, programas[programa_seleccionado].velocidad_movimiento_rpm);
    }
    else  // Actualiza LCD status
    {
      if (sinceStatus > 1000)
      {
        set_texto_fila_lcd(String("*Waiting... ") + i_time_string(sinceStart), 1);
        sinceStatus = 0;
      }
    }
  }
  update_last_state_start_button();
}


//**********************************
//** Métodos ***********************
//**********************************

void vuelca_nueva_config_programa(uint8_t indice_programa, const config_prog_t nueva_config)
{
  programas[indice_programa].numero_movimientos = nueva_config.numero_movimientos;
  programas[indice_programa].distancia_movimiento_num_pasos = nueva_config.distancia_movimiento_num_pasos;
  programas[indice_programa].velocidad_movimiento_rpm = nueva_config.velocidad_movimiento_rpm;
  programas[indice_programa].decimas_segundo_freno = nueva_config.decimas_segundo_freno;
  programas[indice_programa].distancia_offset_num_pasos = nueva_config.distancia_offset_num_pasos;
}

#ifdef VERBOSE
void print_config_programa(uint8_t indice_programa)
{
  config_prog_t conf_print;
  conf_print = programas[indice_programa];
  i_print_config_programa(indice_programa, conf_print);
}
#endif

void read_start_button()
{
  estadoBoton_start = digitalRead(PIN_BOTON_START);
}

void update_last_state_start_button()
{
  estadoBotonLast_start = estadoBoton_start;
}

void ajuste_inicial_cabezal_bobinadora(uint16_t distanciaoffset)
{
  int signo;

  if (MOVIMIENTO_EN_IDA_INICIAL)
    signo = 1;
  else
    signo = -1;

#ifdef VERBOSE
  Serial.print("s_finalcarrera: ");
  Serial.println(sensor_final_carrera);
#endif
  sensor_final_carrera = digitalRead(PIN_FIN_CARRERA_HOME);
  //Giramos el motor hasta que encuentre el final de carrera "home"
  myStepper.setSpeed(100);
  i_opera_devanador(HIGH); // Activamos motor devanador

  while(sensor_final_carrera == LOW)
  {
    myStepper.step(-signo);
    sensor_final_carrera = digitalRead(PIN_FIN_CARRERA_HOME);
  }
  // Una vez tenemos el devanador en el home lo llevamos a su distancia offset
#ifdef VERBOSE
  Serial.print("Avanza Offset: ");
  Serial.println(distanciaoffset * signo);
#endif
  myStepper.step(distanciaoffset * signo);
}

void i_opera_devanador(bool state)
{
#ifdef NEGATE_LOGIC_DEVANADOR
  state = !state;
#endif
  digitalWrite(PIN_MOTOR_DEVANADOR, state);   // Desactivamos motor devanador
}

void activa_freno()
{
  if ((estado_general == STATE_MARCHA) & !freno_activo) // Para activarlo una única vez
  {
#ifdef VERBOSE
    Serial.println("* FRENO ON");
#endif
    digitalWrite(PIN_MOTOR_FRENO_1, HIGH);
    digitalWrite(PIN_MOTOR_FRENO_2, HIGH);
    freno_activo = true;
    activacion_freno_elapsedMillis = sinceBrake;
  }
}

void desactiva_freno()
{
#ifdef VERBOSE
  Serial.println("* FRENO OFF");
#endif
  digitalWrite(PIN_MOTOR_FRENO_1, LOW);
  digitalWrite(PIN_MOTOR_FRENO_2, LOW);
  freno_activo = false;
  activacion_freno_elapsedMillis = 0;
  sinceBrake = 0;
}

void comprueba_tiempo_de_frenado()
{
  if (freno_activo)
  {
    unsigned long tiempo_freno_activo;

    tiempo_freno_activo = sinceBrake - activacion_freno_elapsedMillis;
    if (tiempo_freno_activo > TIEMPO_FRENADO_MS)
      desactiva_freno();
  }
}

long int calcula_tiempo_hasta_final_de_programa(int delta_movimiento_ms, uint16_t nmovimientos)
{
  long int delta_to_final_ms;

  delta_to_final_ms = (long int)delta_movimiento_ms * (long int)(nmovimientos - contador_vueltas_motor);
#ifdef VERBOSE
    Serial.print("DEBUG DELTA TO FINISH: ");
    Serial.print(delta_to_final_ms);
    Serial.print(" ms; DELTA MOV: ");
    Serial.print(delta_movimiento_ms);
    Serial.print(" ms; NMOVS: ");
    Serial.print(nmovimientos);
    Serial.print("; VUELTAS: ");
    Serial.println(contador_vueltas_motor);
#endif
  return delta_to_final_ms;
}

void cuenta_vueltas_motor(uint16_t distanciamov, uint16_t nmovimientos, uint16_t decimas_segundo_freno)
{
  int signo;

  if (movimiento_en_ida)
    signo = 1;
  else
    signo = -1;
  movimiento_en_ida = !movimiento_en_ida;

  if (contador_vueltas_motor < nmovimientos)
  {
    long int delta_movimiento_ms, delta_to_final_ms;

#ifdef VERBOSE
    Serial.print("Distancia mov step: ");
    Serial.print((int)distanciamov * signo);
    Serial.print("; VUELTA n=");
    Serial.print(contador_vueltas_motor);
    Serial.print("/");
    Serial.println(nmovimientos);
#endif
    set_texto_fila_lcd(String("*MOVS: ") + (contador_vueltas_motor + 1) + String("/") + nmovimientos, 1);
    sinceStart = 0;
    myStepper.step((int)distanciamov * signo); //Distancia en número de pasos
    delta_movimiento_ms = sinceStart;
    contador_vueltas_motor += 1;
    delta_to_final_ms = calcula_tiempo_hasta_final_de_programa(delta_movimiento_ms, nmovimientos);
    // si el frenado debe producirse antes de que acabe el siguiente mov:
    if (!freno_activo & (delta_to_final_ms - (decimas_segundo_freno * PERIODO_FRENO_SEG * 1000) < delta_movimiento_ms))
    {
      long int delta_to_frenado_ms = delta_to_final_ms - (decimas_segundo_freno * PERIODO_FRENO_SEG * 1000);

      if (delta_to_frenado_ms < 100)
      {
#ifdef VERBOSE
        Serial.print("DEBUG: ACTIVACION INSTANTANEA FRENO. DELTA_FRENADO_MS=");
        Serial.println(delta_to_frenado_ms);
#endif
        activa_freno();
      }
      else
      {
#ifdef VERBOSE
        Serial.print("DEBUG: SET TIMER TO FRENADO EN DELTA_FRENADO_MS=");
        Serial.println(delta_to_frenado_ms);
#endif
        FlexiTimer2::set(delta_to_frenado_ms, activa_freno);
        FlexiTimer2::start();
      }
    }
  }
  else
  {
#ifdef VERBOSE
    Serial.print("END OK PROGRAMA_");
    Serial.println(programa_seleccionado + 1);
#endif
    set_estado_paro_por_finalizacion();
#ifdef VERBOSE
    Serial.print("Entrada en SELECC tras terminar PROGRAMA_");
    Serial.println(programa_seleccionado + 1);
#endif
    set_estado_seleccion();
  }
}

void set_estado_marcha(bool desde_paro, uint16_t velocidadmov)
{
  if (!desde_paro)
  {
    // Ajuste de bobina inicial
    ajuste_inicial_cabezal_bobinadora(programas[programa_seleccionado].distancia_offset_num_pasos);
    set_texto_fila_lcd(String("Waiting to start"), 1);
#ifdef VERBOSE
    Serial.print("Esperando a comenzar P_");
    Serial.println(programa_seleccionado + 1);
#endif
    // Esperar a botón start
    while (true)
    {
      read_start_button();
      if (i_sube_flanco(estadoBotonLast_start, estadoBoton_start))
        break;
      update_last_state_start_button();
    }

    i_reset_direccion_devanador();
    contador_vueltas_motor = 0;
  }

  estado_general = STATE_MARCHA;
  i_reset_rotary_encoder();
  sinceStart = 0;
  sinceStatus = 0;

  myStepper.setSpeed(velocidadmov);     // Velocidad del devanador
  desactiva_freno();            // Desactivamos freno
  i_opera_devanador(HIGH);      // Activamos motor devanador
  digitalWrite(PIN_MOTOR_VARIADOR, HIGH);   // Activamos variador
}

void set_estado_paro_general(int tipo_entrada_en_paro, int pin_responsable_paro)
{
  // PARO del programa si está en marcha
  if (estado_general == STATE_MARCHA)
  {
    digitalWrite(PIN_MOTOR_VARIADOR, LOW);      // Desactivamos variador

    // TODO revisar timer stop
    FlexiTimer2::stop();
    activa_freno();                 // Frenado instantáneo

    if (tipo_entrada_en_paro == TIPO_PARADA_FINALIZACION)
    {
      i_opera_devanador(LOW);  // Desactivamos motor devanador
      sinceStart = 0;
      sinceStatus = 0;
    }

    // Set estado PARO
    estado_general = STATE_PARO;
    tipo_parada = tipo_entrada_en_paro;
    pin_fallo = pin_responsable_paro;
  }
}

void isr_set_estado_paro_por_fallo()
{
  // PARO del programa con sensor FALLO_1 ó FALLO_2
  set_estado_paro_general(TIPO_PARADA_EMERGENCIA, PIN_FALLO_1_2);
}

void isr_set_estado_paro_o_reset_manual()
{
  // Entrada a RESET manual tras PARO (con botón PARO/RESET)
  if ((estado_general == STATE_PARO) && (sinceStart > DELAY_ENTRE_PARO_RESET_MS))
  {
    i_opera_devanador(LOW); // Desactivamos motor devanador
    estado_general = STATE_RESET;
    tipo_parada = TIPO_PARADA_MANUAL;
    pin_fallo = -1;
  }
  // PARO manual del programa en marcha (con botón PARO/RESET)
  else
  {
    set_estado_paro_general(TIPO_PARADA_MANUAL, PIN_BOTON_PARO_RESET);
  }
}

void set_estado_paro_por_finalizacion()
{
  set_estado_paro_general(TIPO_PARADA_FINALIZACION, -1);
}

void set_estado_seleccion()
{
  set_texto_lcd(String("* Selecciona P:"), String("- PROGRAMA ") + (programa_seleccionado + 1));
  estado_general = STATE_SELECC;
  tipo_parada = TIPO_PARADA_FINALIZACION;
  pin_fallo = -1;
  i_reset_rotary_encoder();
  // TODO revisar timer stop
  FlexiTimer2::stop();
}

void set_estado_config()
{
  set_texto_lcd(String("** Edicion P_") + (programa_seleccionado + 1), String(OPCIONES_PROGRAMA[variable_seleccionada]));
  estado_general = STATE_CONFIG;
}

void eeprom_save_config_programa(int indice_programa)
{
  int eeAddress;
  config_prog_t conf_i;

  conf_i = programas[indice_programa];
  eeAddress = sizeof(bool) + sizeof(config_prog_t) * indice_programa;
  EEPROM.put(eeAddress, conf_i);

#ifdef VERBOSE
  Serial.print("Se sobreescribe PROGRAMA ");
  Serial.print(indice_programa + 1);
  Serial.println(" en EEPROM: ");
  print_config_programa(indice_programa);
  Serial.println("");
#endif
}

void seleccion_programa()
{
  // Aguarda cambio de programa o entrada a configuración de programa --> rotary encoder
  ClickEncoder::Button b = rotary_encoder->getButton();
  bool hay_cambio;

  hay_cambio = i_hay_cambio_unitario_rotary_encoder(&programa_seleccionado, NUM_PROGRAMAS - 1);
  if (hay_cambio)
    set_texto_fila_lcd(String("- PROGRAMA ") + (programa_seleccionado + 1), 1);

  // Entrada a config con click o doble click del rotary encoder
  if (b != ClickEncoder::Open)
  {
    switch (b)
    {
      CASEBREAK(ClickEncoder::Pressed);
      CASEBREAK(ClickEncoder::Held);
      CASEBREAK(ClickEncoder::Released);
      case ClickEncoder::Clicked:
      case ClickEncoder::DoubleClicked:
        set_estado_config();
        break;
    }
  }
  // Para frenar el rotary en la selección de programa
  delay(300);

#ifdef USE_LCD
  // Refresco LCD
  i_refresca_inf_izq_lcd();
#endif
}

void configuracion_parametros_programa_actual()
{
  // Modo de edicion de variables del programa seleccionado
  bool programa_modificado = false;

  // Loop hasta que se entra en edicion de parámetro o se sale de configuración:
  i_reset_rotary_encoder();

#ifdef VERBOSE
  Serial.print("--> modo config de P_");
  Serial.println(programa_seleccionado + 1);
  print_config_programa(programa_seleccionado);
#endif

  while (!hay_doble_click_rotary_enc)
  {
    bool hay_cambio;
    ClickEncoder::Button b = rotary_encoder->getButton();

    hay_cambio = i_hay_cambio_unitario_rotary_encoder(&variable_seleccionada, NUM_OPCIONES - 1);
    if (hay_cambio)
      set_texto_fila_lcd(String(OPCIONES_PROGRAMA[variable_seleccionada]), 1);

    if (b != ClickEncoder::Open)
    {
      switch (b)
      {
        CASEBREAK(ClickEncoder::Pressed);
        CASEBREAK(ClickEncoder::Held);
        CASEBREAK(ClickEncoder::Released);
        case ClickEncoder::Clicked: // edicion de parámetro
        {
          config_prog_t programa_edit = programas[programa_seleccionado];
          bool hay_cambio_valor;

          switch (variable_seleccionada)
          {
            case 0:
              hay_cambio_valor = edicion_variable_por_cifras(OPCIONES_PROGRAMA_CORTAS[variable_seleccionada],
                                                             &programa_edit.numero_movimientos,
                                                             OPCIONES_RANGO_MIN[variable_seleccionada], OPCIONES_RANGO_MAX[variable_seleccionada], 1.);
              break;
            case 1:
              hay_cambio_valor = edicion_variable_por_cifras(OPCIONES_PROGRAMA_CORTAS[variable_seleccionada],
                                                             &programa_edit.distancia_movimiento_num_pasos,
                                                             OPCIONES_RANGO_MIN[variable_seleccionada], OPCIONES_RANGO_MAX[variable_seleccionada], 1.);
              break;
            case 2:
              hay_cambio_valor = edicion_variable_por_cifras(OPCIONES_PROGRAMA_CORTAS[variable_seleccionada],
                                                             &programa_edit.velocidad_movimiento_rpm,
                                                             OPCIONES_RANGO_MIN[variable_seleccionada], OPCIONES_RANGO_MAX[variable_seleccionada], 1.);
              break;
            case 3:
              hay_cambio_valor = edicion_variable_por_cifras(OPCIONES_PROGRAMA_CORTAS[variable_seleccionada],
                                                             &programa_edit.decimas_segundo_freno,
                                                             OPCIONES_RANGO_MIN[variable_seleccionada], OPCIONES_RANGO_MAX[variable_seleccionada], PERIODO_FRENO_SEG);
              break;
            case 4:
              hay_cambio_valor = edicion_variable_por_cifras(OPCIONES_PROGRAMA_CORTAS[variable_seleccionada],
                                                             &programa_edit.distancia_offset_num_pasos,
                                                             OPCIONES_RANGO_MIN[variable_seleccionada], OPCIONES_RANGO_MAX[variable_seleccionada], 1.);
              break;
            }
          if (hay_cambio_valor)
          {
            programa_modificado = true;
            // volcado de valores!
            vuelca_nueva_config_programa(programa_seleccionado, programa_edit);
          }
#ifdef VERBOSE
          Serial.print("EXIT EDIT VAR. Changes? ");
          Serial.println(hay_cambio_valor);
          Serial.print("; prog modified? ");
          Serial.println(programa_modificado);
          print_config_programa(programa_seleccionado);
#endif
          set_estado_config();
          break;
        }
        case ClickEncoder::DoubleClicked: // Salida a selección de programa
        {
#ifdef VERBOSE
          Serial.print("Exiting config mode of P_");
          Serial.println(programa_seleccionado + 1);
          print_config_programa(programa_seleccionado);
#endif

          // Grabación de programa seleccionado en EEPROM
          if (programa_modificado)
          {
            eeprom_save_config_programa(programa_seleccionado);
            set_texto_lcd(String("** Guardado"), String("--> PROGRAMA ") + (programa_seleccionado + 1));
            delay(DELAY_MS_CMD_MENU);
          }
          set_estado_seleccion();
          hay_doble_click_rotary_enc = true;
          break;
        }
      }
    }
    // Para frenar el rotary en la selección de variable a editar
    delay(300);
  }
}

const char *i_text_variation_order_mag(uint8_t order_mag)
{
  switch (order_mag)
  {
    case 0:
      return "   - 1 +: ";
    case 1:
      return "  - 10 +: ";
    case 2:
      return " - 100 +: ";
    case 3:
      return "- 1000 +: ";
    case 4:
      return "-10000 +: ";
  }
}

void i_trim_number(uint16_t valor, float factor_conv_variable,
                   uint8_t pos_cifra, uint8_t order_max,
                   uint8_t *cifra,
                   String *tnum_before, String *tnum_after,
                   String *opc_before, String *opc_after)
{
  String text_value = String((valor * factor_conv_variable)/ pow(10., order_max), order_max);

  *tnum_before = String(text_value.substring(2, 2 + order_max - pos_cifra - 1));
  *tnum_after = String(text_value.substring(2 + order_max - pos_cifra));
  *cifra = text_value.substring(1 + order_max - pos_cifra, 2 + order_max - pos_cifra).toInt();
  if (opc_before != NULL)
    //*opc_before = String("+-(10^" + String(pos_cifra) + "): " + *tnum_before + "_");
    *opc_before = String(i_text_variation_order_mag(pos_cifra) + *tnum_before + "_");
  if (opc_after != NULL)
    *opc_after = String("_" + *tnum_after);

#ifdef VERBOSE
  Serial.print("** DEBUG TRIM. VALOR:");
  Serial.print(valor);
  Serial.print(", pos:");
  Serial.print(pos_cifra);
  Serial.print(", CIFRA: ");
  Serial.print(*cifra);
  Serial.print(", tnum_before: ");
  Serial.print(*tnum_before);
  Serial.print(", tnum_after: ");
  Serial.println(*tnum_after);
#endif
}

bool edicion_variable_por_cifras(const char *ref_variable,
                                 uint16_t *valor_edit,
                                 uint16_t valor_min, uint16_t valor_max, float factor_conv_variable)
{
  // Separación de la variable a editar por cifras separadas (# dependiente del rango). Se comienza a editar la cifra de unidades (drcha),
  // y, mediante *clicks del encoder*, se va cambiando de cifra. Para salir con el nuevo valor, *doble click* del encoder*
  bool hay_cambio, hay_doble_click_en_edit_var;
  String before, after, text_value, tnum_before, tnum_after;
  uint8_t pos_cifra, order_max, cifra;
  int valor_ant = (int)(*valor_edit * factor_conv_variable);
  int valor = valor_ant;

  sinceStart = 0;
  pos_cifra = 0;
  order_max = (uint8_t)ceil(log10(valor_max));
  i_trim_number(valor, factor_conv_variable, pos_cifra, order_max,
                &cifra, &tnum_before, &tnum_after, &before, &after);

  i_reset_rotary_encoder();
  hay_cambio = false;
  hay_doble_click_en_edit_var = false;

  //set_texto_lcd(String("* Configurando:"), String(ref_variable + " (" + valor_ant + ")"));
  set_texto_lcd(String(ref_variable) + " (" + valor_ant + ")", String("- <--> + :  ") + valor);
  delay(DELAY_MS_CMD_MENU / 2);

  set_texto_fila_lcd(String(before + String(cifra) + after), 1);
  while (!hay_doble_click_en_edit_var)  // Salida de edicion por cifras mediante doble click
  {
    hay_cambio = i_hay_cambio_unitario_rotary_encoder(&cifra, 9);
    if (hay_cambio)
    {
      valor = String(tnum_before + String(cifra) + tnum_after).toInt();
      //Serial.println(String("NEW --> ") + String(valor) + String(", -> ") + tnum_before + String(" + ") + String(cifra) + String(" + ") + tnum_after);
      set_texto_fila_lcd(String(before + String(cifra) + after), 1);
      sinceStart = 0;
    }
    else
    {
      ClickEncoder::Button b = rotary_encoder->getButton();
      if (b != ClickEncoder::Open)
      {
        switch (b)
        {
          CASEBREAK(ClickEncoder::Pressed);
          CASEBREAK(ClickEncoder::Held);
          CASEBREAK(ClickEncoder::Released);
          case ClickEncoder::Clicked:
            // Rotación de cifra
            if (pos_cifra + 1 < order_max)
              pos_cifra += 1;
            else
              pos_cifra = 0;
            i_trim_number(valor, factor_conv_variable, pos_cifra, order_max,
                          &cifra, &tnum_before, &tnum_after, &before, &after);
#ifdef VERBOSE
            Serial.println(String("ROTACION CIFRA_CLICK --> ") + String(valor) + String(", -> ") + tnum_before + String(" + ") + String(cifra) + String(" + ") + tnum_after);
#endif
            set_texto_fila_lcd(String(before + String(cifra) + after), 1);
            sinceStart = 0;
            break;
          case ClickEncoder::DoubleClicked:
#ifdef VERBOSE
            Serial.println(String("SALIDA DBLCLICK --> ") + String(valor) + String(", -> ") + tnum_before + String(" + ") + String(cifra) + String(" + ") + tnum_after);
#endif
            hay_doble_click_en_edit_var = true;
            break;
        }
      }
    }

    // Salida por tiempo máximo de edicion sin cambios
    if (sinceStart > DELAY_TIMEOUT_EDIT_VAR_MS)
    {
#ifdef VERBOSE
      Serial.println("Saliendo de edicion por timeout");
#endif
      hay_doble_click_en_edit_var = true;
    }

    // Para frenar el rotary en la edicion por cifras
    delay(300);
  }
  if (valor < valor_min)
    valor = valor_min;
  else if (valor > valor_max)
    valor = valor_max;

  set_texto_fila_lcd(String("Se acepta: ") + (int)(valor / factor_conv_variable), 1);
  delay(DELAY_MS_CMD_MENU);
//  hay_doble_click_rotary_enc = false;
  if (valor != valor_ant)
  {
#ifdef VERBOSE
    Serial.print("Saliendo de edicion por cifras con cambios: ");
    Serial.print(valor_ant);
    Serial.print(" --> ");
    Serial.println(valor);
#endif
    *valor_edit = (uint16_t)(valor / factor_conv_variable);
    return true;
  }
  return false;
}

#ifdef USE_LCD
void i_refresca_inf_izq_lcd()
{
  // Refresco LCD
  if (sinceStatus > 500)
  {
    interruptor = !interruptor;
    lcd.setCursor(0, 1);
    if (interruptor)
      lcd.print("+");
    else
      lcd.print("-");
    sinceStatus = 0;
  }
}
#endif

String i_time_string(float millis)
{
  return String(millis / 1000., 1);
}

static inline int8_t i_signo(int val)
{
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

void i_reset_rotary_encoder()
{
  hay_doble_click_rotary_enc = false;
  hay_click_rotary_enc = false;
  (void)rotary_encoder->getValue();
}

bool i_hay_cambio_unitario_rotary_encoder(uint8_t *variable_edit, uint16_t valor_max)
{
  int old_value = (int)(*variable_edit);
  int signo_cambio = (int)i_signo(rotary_encoder->getValue());

  if (((signo_cambio > 0) & (old_value < valor_max)) | ((signo_cambio < 0) & (old_value > 0)))
  {
//#ifdef VERBOSE
//    Serial.print("DEBUG CAMBIO UNITARIO ENCODER: OLD=");
//    Serial.print(old_value);
//    Serial.print(", SIGNO=");
//    Serial.print(signo_cambio);
//    Serial.print(", NEW=");
//    Serial.print(old_value + signo_cambio);
//    Serial.print(", uint8=");
//    Serial.println((uint8_t)(old_value + signo_cambio));
//#endif

    old_value += signo_cambio;
    *variable_edit = (uint8_t)old_value;
    return true;
  }
  return false;
}

bool i_sube_flanco(int estado_ant, int new_estado)
{
  if ((new_estado != estado_ant) & (new_estado == HIGH))
    return true;
  return false;
}

#ifdef VERBOSE
void i_print_config_programa(uint8_t indice_programa, const config_prog_t conf_print)
{
  Serial.print("PROG_");
  Serial.print(indice_programa + 1);
  Serial.print(": MOVS=");
  Serial.print(conf_print.numero_movimientos);
  Serial.print("; DIST (num. pasos)=");
  Serial.print(conf_print.distancia_movimiento_num_pasos);
  Serial.print("; VEL (rpm)=");
  Serial.print(conf_print.velocidad_movimiento_rpm);
  Serial.print("; FRENO (s)=");
  Serial.print(conf_print.decimas_segundo_freno * PERIODO_FRENO_SEG);
  Serial.print("; OFFSET (num. pasos)=");
  Serial.println(conf_print.distancia_offset_num_pasos);
}
#endif

String i_completa_fila_led(String orig)
{
  String lcdprint = orig;
  if (lcdprint.length() < LCD_CHARS)  // Rellena cadena con espacios (para borrar la línea)
    for (int i=lcdprint.length(); i < LCD_CHARS; i++)
      lcdprint += " ";
  else if (lcdprint.length() > LCD_CHARS)  // Trunca cadena (controlar desde emisor!)
    lcdprint = lcdprint.substring(0, LCD_CHARS);
  return lcdprint;
}

void set_texto_fila_lcd(String fila, unsigned char n_fila)
{
#ifdef USE_LCD
  lcd.setCursor(0, n_fila);
  lcd.print(i_completa_fila_led(fila));
#else
  Serial.print("LCD");
  Serial.print(n_fila + 1);
  Serial.print("_:   >>> ");
  Serial.print(i_completa_fila_led(fila));
  Serial.println(" <<<");
#endif
}

void set_texto_lcd(String fila_1, String fila_2)
{
#ifdef USE_LCD
  //lcd.clear();
  set_texto_fila_lcd(fila_1, 0);
  set_texto_fila_lcd(fila_2, 1);
#else
  //          ________________________
  // LCD1_:   >>> TESTING ...      <<<
  // LCD2_:   >>> TESTING ...      <<<
  //          ------------------------
  Serial.println("         ________________________");
  set_texto_fila_lcd(fila_1, 0);
  set_texto_fila_lcd(fila_2, 1);
  Serial.println("         ------------------------");
#endif
}
