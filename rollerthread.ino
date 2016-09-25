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
//    uint16_t num_periodos_freno;
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

//**********************************
//** Librerías *********************
//**********************************

#include <ClickEncoder.h>
#include <EEPROM.h>
#include <elapsedMillis.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>  // F Malpartida's NewLiquidCrystal library
#include <Stepper.h>
#include <TimerOne.h>
#include <Wire.h>


//**********************************
//** Definiciones ******************
//**********************************

// PINOUT
#define PIN_BOTON_START           4           // Botón de START / REANUDACIÓN

#define PIN_BOTON_PARO_RESET      3           // Botón de PARO / RESET  (Interrupt)
#define PIN_FALLO_1               18          // Señal de Fallo 1 (Interrupt)
#define PIN_FALLO_2               19          // Señal de Fallo 2 (Interrupt)

#define PIN_ROTARY_ENC_CLK        A2
#define PIN_ROTARY_ENC_DT         A1
#define PIN_ROTARY_ENC_SW         A0

#define PIN_FIN_CARRERA_HOME      7           // Sensor de final de carrera Home

#define PIN_1_MOTOR_STEP          8           // Stepper PIN1
#define PIN_2_MOTOR_STEP          9           // Stepper PIN2
#define STEPS_MOTOR               200         // Stepper config.

#define PIN_MOTOR_DEVANADOR       10          // Motor devanador
#define PIN_MOTOR_VARIADOR        11          // Variador
#define PIN_MOTOR_FRENO           12          // Freno

#define LED_PLACA                 13          // Simulando el encendido del motor, y/o como led de actividad


// Resto

#define VERBOSE                   true
#define FORZAR_REGRABADO_DEFAULTS false       // Activando, se fuerza la re-escritura de los parámetros por defecto de los 10 programas

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

#define NUM_PROGRAMAS             10
#define NUM_OPCIONES              5
#define DELAY_MS_CMD_MENU         500

#define CASEBREAK(label) case label: break;


//**********************************
//** Variables *********************
//**********************************

const char* OPCIONES_PROGRAMA[] = {"- Num movs",
                                   "- Distanc. pasos",
                                   "- Velocidad rpm",
                                   "- Freno segs",
                                   "- Offset pasos"};
const char* OPCIONES_PROGRAMA_CORTAS[5] = {"N Movs", "Dist. NP", "Vel. rpm", "Freno seg", "Offset NP"};
double PERIODO_FRENO_SEG = .5;
uint16_t OPCIONES_RANGO_MIN[5] = {0, 100, 10, 0, 0};
uint16_t OPCIONES_RANGO_MAX[5] = {1500, 3000, 400, (uint16_t)(5 / PERIODO_FRENO_SEG), 1000};

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

struct config_prog_t
{
  uint16_t numero_movimientos;
  uint16_t distancia_movimiento_num_pasos;
  uint16_t velocidad_movimiento_rpm;
  uint16_t num_periodos_freno;
  uint16_t distancia_offset_num_pasos;
};
const int eeAddressDelta = sizeof(config_prog_t);
config_prog_t programas[NUM_PROGRAMAS];

volatile uint8_t estado_general;
volatile uint8_t tipo_parada;
uint8_t programa_seleccionado;
uint8_t variable_seleccionada;
bool interruptor;

uint8_t sensor_final_carrera = LOW;
uint8_t estadoBoton_start = 0;
uint8_t estadoBotonLast_start = 0;

LiquidCrystal_I2C lcd( 0x3F, 2,   1,  0,  4,  5,  6,  7, 3, POSITIVE);

ClickEncoder *rotary_encoder;
bool hay_doble_click_rotary_enc = false;
bool hay_click_rotary_enc = false;
void timerIsr() {
  rotary_encoder->service();
}

elapsedMillis sinceStart;
elapsedMillis sinceStatus;

Stepper myStepper(STEPS_MOTOR, PIN_1_MOTOR_STEP, PIN_2_MOTOR_STEP);  //(pasos por vuelta, step, dir)
int contador_vueltas_motor;
bool movimiento_en_ida;


//**********************************
//** SETUP INICIAL *****************
//**********************************

void setup()
{
  Serial.begin(9600);
  setup_rotary_encoder();
  setup_lcd();
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
  pinMode(PIN_FALLO_1, INPUT);
  pinMode(PIN_FALLO_2, INPUT);
  //attachInterrupt(digitalPinToInterrupt(pin), ISR, mode);
  attachInterrupt(digitalPinToInterrupt(PIN_BOTON_PARO_RESET), isr_set_estado_paro_o_reset_manual, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_FALLO_1), isr_set_estado_paro_por_fallo, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_FALLO_2), isr_set_estado_paro_por_fallo, RISING);
}

void setup_motor_bobinadora()
{
  pinMode(LED_PLACA, OUTPUT);  //Led Arduino Mega

  pinMode(PIN_MOTOR_DEVANADOR, OUTPUT); // Enable Motor Devanador
  pinMode(PIN_MOTOR_VARIADOR, OUTPUT);  // Relé Variador
  pinMode(PIN_MOTOR_FRENO, OUTPUT);  // Relé Freno

  // Apagando motores al inicio
  digitalWrite(PIN_MOTOR_FRENO, LOW);    //Desactivamos freno
  digitalWrite(PIN_MOTOR_DEVANADOR, LOW);  // Desactivamos motor devanador
  digitalWrite(PIN_MOTOR_VARIADOR, LOW);    //Desactivamos variador

  contador_vueltas_motor = 0;
  i_reset_direccion_devanador();
}

void setup_lcd()
{
  lcd.begin(LCD_CHARS, LCD_LINES);
  lcd.setBacklight(HIGH); // Activamos la retroiluminacion
  lcd.clear();
}

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
  if (VERBOSE)
  {
    Serial.print("es_primer_inicio: ");
    Serial.println(es_primer_inicio);
  }
  if (es_primer_inicio | FORZAR_REGRABADO_DEFAULTS)
  {
    if (VERBOSE)
      Serial.println("Se procede a grabar la EEPROM con los programas por defecto");
    eeAddress += sizeof(bool);
    eeAddress = eeprom_write_config_defecto(eeAddress, 0, VALORES_DEFECTO_P1, VERBOSE);
    eeAddress = eeprom_write_config_defecto(eeAddress, 1, VALORES_DEFECTO_P2, VERBOSE);
    eeAddress = eeprom_write_config_defecto(eeAddress, 2, VALORES_DEFECTO_P3, VERBOSE);
    eeAddress = eeprom_write_config_defecto(eeAddress, 3, VALORES_DEFECTO_P4, VERBOSE);
    eeAddress = eeprom_write_config_defecto(eeAddress, 4, VALORES_DEFECTO_P5, VERBOSE);
    eeAddress = eeprom_write_config_defecto(eeAddress, 5, VALORES_DEFECTO_P6, VERBOSE);
    eeAddress = eeprom_write_config_defecto(eeAddress, 6, VALORES_DEFECTO_P7, VERBOSE);
    eeAddress = eeprom_write_config_defecto(eeAddress, 7, VALORES_DEFECTO_P8, VERBOSE);
    eeAddress = eeprom_write_config_defecto(eeAddress, 8, VALORES_DEFECTO_P9, VERBOSE);
    eeAddress = eeprom_write_config_defecto(eeAddress, 9, VALORES_DEFECTO_P10, VERBOSE);
    EEPROM.put(0, false);  //Marca de configuración existente, para no sobreescribir en cada reinicio
  }

  //Lectura de configuraciones:
  eeprom_read_configuraciones(VERBOSE);
}

void eeprom_read_configuraciones(bool verbose_mode)
{
  int eeAddress = sizeof(bool);
  for (int i=0; i < NUM_PROGRAMAS; i++)
  {
    config_prog_t conf_i;

    EEPROM.get(eeAddress, conf_i);
    programas[i] = conf_i;
    eeAddress += eeAddressDelta;
    if (verbose_mode)
    {
      Serial.print("Leido PROGRAMA ");
      Serial.print(i + 1);
      Serial.println(" desde EEPROM:");
      print_config_programa(i);
    }
  }
}

int eeprom_write_config_defecto(int eeAddress, int indice_programa, const uint16_t* valores_programa, bool verbose_mode)
{
  config_prog_t conf_i;

  conf_i.numero_movimientos = valores_programa[0];
  conf_i.distancia_movimiento_num_pasos = valores_programa[1];
  conf_i.velocidad_movimiento_rpm = valores_programa[2];
  conf_i.num_periodos_freno = valores_programa[3];
  conf_i.distancia_offset_num_pasos = valores_programa[4];

  EEPROM.put(eeAddress, conf_i);
  eeAddress += eeAddressDelta;

  if (verbose_mode)
  {
    Serial.print("Se graba PROGRAMA ");
    Serial.print(indice_programa + 1);
    Serial.println(" en EEPROM: ");
    i_print_config_programa(indice_programa, conf_i);
    Serial.println("");
  }
  return eeAddress;
}

void i_reset_direccion_devanador()
{
  movimiento_en_ida = false;
}

//**********************************
//** Bucle general *****************
//**********************************

void loop()
{
  read_start_button();

  // Modos de funcionamiento: CONFIG / SELEC / MARCHA / RESET / PARO

  // Inicio de programa seleccionado
  if ((estado_general == STATE_SELECC) & (i_sube_flanco(estadoBotonLast_start, estadoBoton_start)))
  {
    set_texto_lcd(String("* RUNNING P_") + (programa_seleccionado + 1), String("Starting..."));
    if (VERBOSE)
    {
      Serial.print("Comienza el PROGRAMA ");
      Serial.println(programa_seleccionado + 1);
    }
    set_estado_marcha(false, programas[programa_seleccionado].velocidad_movimiento_rpm);
  }
  // Aguarda cambio de programa o entrada a configuración de programa mediante rotary encoder
  else if (estado_general == STATE_SELECC)
  {
    seleccion_programa();
  }
  // Modo de edición de variables del programa seleccionado
  else if (estado_general == STATE_CONFIG)
  {
    configuracion_parametros_programa_actual();
  }
  // En modo marcha...
  else if (estado_general == STATE_MARCHA)
  {
    cuenta_vueltas_motor(programas[programa_seleccionado].distancia_movimiento_num_pasos,
                         programas[programa_seleccionado].velocidad_movimiento_rpm,
                         programas[programa_seleccionado].numero_movimientos,
                         programas[programa_seleccionado].num_periodos_freno);
  }
  else if (estado_general == STATE_RESET)
  {
    // Reset del programa seleccionado
    set_texto_fila_lcd(String("** RESET P_") + (programa_seleccionado + 1), 0);
    if (VERBOSE)
    {
      Serial.print("Se resetea el PROGRAMA ");
      Serial.println(programa_seleccionado + 1);
    }
    delay(DELAY_MS_CMD_MENU);
    set_estado_seleccion();
  }
  // En modo paro, aguardando reset o reanudación
  else // if (estado_general == STATE_PARO)
  {
    if (tipo_parada != TIPO_PARADA_FINALIZACION)
    {
      digitalWrite(PIN_MOTOR_DEVANADOR, LOW);  // Desactivamos motor devanador
      sinceStart = 0;
      sinceStatus = 0;
      if (tipo_parada == TIPO_PARADA_EMERGENCIA)
      {
        set_texto_fila_lcd(String("** ERROR P_") + (programa_seleccionado + 1), 0);
      }
      else
      {
        set_texto_fila_lcd(String("** STOP P_") + (programa_seleccionado + 1), 0);
      }
      if (VERBOSE)
      {
        Serial.print("** PARADA DE EMERGENCIA del PROGRAMA ");
        Serial.println(programa_seleccionado + 1);
      }
      tipo_parada = TIPO_PARADA_FINALIZACION;
    }

    // Reanudación del programa seleccionado
    if (i_sube_flanco(estadoBotonLast_start, estadoBoton_start))
    {
      set_texto_lcd(String("* RE-RUN P_") + (programa_seleccionado + 1), String("En paro: ") + i_time_string(sinceStart));
      if (VERBOSE)
      {
        Serial.print("Se reanuda el PROGRAMA ");
        Serial.println(programa_seleccionado + 1);
      }
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
  programas[indice_programa].num_periodos_freno = nueva_config.num_periodos_freno;
  programas[indice_programa].distancia_offset_num_pasos = nueva_config.distancia_offset_num_pasos;
}

void print_config_programa(uint8_t indice_programa)
{
  config_prog_t conf_print;
  conf_print = programas[indice_programa];
  i_print_config_programa(indice_programa, conf_print);
}

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
  if (VERBOSE)
  {
    Serial.print("sensor_final_carrera: ");
    Serial.println(sensor_final_carrera);
  }
  sensor_final_carrera = digitalRead(PIN_FIN_CARRERA_HOME);
  //Giramos el motor hasta que encuentre el final de carrera "home"
  myStepper.setSpeed(100);
  digitalWrite(PIN_MOTOR_DEVANADOR, HIGH);  // Activamos motor devanador

  while(sensor_final_carrera == LOW)
  {
    myStepper.step(1);
    sensor_final_carrera = digitalRead(PIN_FIN_CARRERA_HOME);
  }
  // Una vez tenemos el devanador en el home lo llevamos a su distancia offset
  if (VERBOSE)
  {
    Serial.print("Avanzando Offset: ");
    Serial.println(-distanciaoffset);
  }
  myStepper.step(-distanciaoffset);
}

void cuenta_vueltas_motor(uint16_t distanciamov, uint16_t velocidadmov, uint16_t nmovimientos, uint16_t num_periodos_freno)
{
  int signo;

  if (movimiento_en_ida)
    signo = 1;
  else
    signo = -1;
  movimiento_en_ida = !movimiento_en_ida;

  if (contador_vueltas_motor < nmovimientos)
  {
    if (VERBOSE)
    {
      Serial.print("Distancia movimiento step: ");
      Serial.print((int)distanciamov * signo);
      Serial.print("; VUELTA n=");
      Serial.print(contador_vueltas_motor);
      Serial.print("/");
      Serial.println(programas[programa_seleccionado].numero_movimientos);
    }
    set_texto_fila_lcd(String("*MOVS: ") + (contador_vueltas_motor + 1) + String("/") + programas[programa_seleccionado].numero_movimientos, 1);
    myStepper.step((int)distanciamov * signo); //Distancia en número de pasos
    contador_vueltas_motor += 1;
  }
  else
  {
    set_estado_paro_por_finalizacion(num_periodos_freno);
    set_estado_seleccion();
  }
}

void proceso_de_freno_motor(uint16_t num_periodos_freno)
{
  float tiempo_espera_para_frenado_ms = num_periodos_freno * PERIODO_FRENO_SEG * 1000;
  uint16_t periodos_freno = 0;
  elapsedMillis sinceFreno = 0;

  set_texto_fila_lcd(String("* Wait for brake"), 1);
  if (VERBOSE)
  {
    Serial.print("** Comienza el temporizado de ");
    Serial.print(tiempo_espera_para_frenado_ms / 1000.);
    Serial.println(" seg. para frenar el motor.");
  }

  // Esperamos hasta frenar:
  while(sinceFreno < tiempo_espera_para_frenado_ms)
  {
    if ((sinceFreno / 1000.) / PERIODO_FRENO_SEG > periodos_freno)
    {
      periodos_freno += 1;
      set_texto_fila_lcd(String("* Braking in ") + ((num_periodos_freno - periodos_freno) * PERIODO_FRENO_SEG), 1);
      if (VERBOSE)
      {
        Serial.print("* Braking in ");
        Serial.println((num_periodos_freno - periodos_freno) * PERIODO_FRENO_SEG);
      }
    }
  }
  //Activamos freno
  if (VERBOSE)
    Serial.println("* ACTIVANDO FRENO");
  digitalWrite(PIN_MOTOR_FRENO, HIGH);

  // Informamos y salimos del proceso de frenado
  set_texto_fila_lcd(String("** Motor Brake"), 1);
  delay(DELAY_MS_CMD_MENU);
}

void set_estado_marcha(bool desde_paro, uint16_t velocidadmov)
{
  if (!desde_paro)
  {
    // Ajuste de bobina inicial
    ajuste_inicial_cabezal_bobinadora(programas[programa_seleccionado].distancia_offset_num_pasos);
    set_texto_fila_lcd(String("Waiting to start"), 1);
    if (VERBOSE)
    {
      Serial.print("Esperando a comenzar P_");
      Serial.println(programa_seleccionado + 1);
    }
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
  hay_doble_click_rotary_enc = false;
  hay_click_rotary_enc = false;
  sinceStart = 0;
  sinceStatus = 0;

  myStepper.setSpeed(velocidadmov);  // Velocidad del devanador
  digitalWrite(PIN_MOTOR_FRENO, LOW);   //Desactivamos freno

  digitalWrite(PIN_MOTOR_DEVANADOR, HIGH);  // Activamos motor devanador
  digitalWrite(PIN_MOTOR_VARIADOR, HIGH);  //Activamos variador
}

void isr_set_estado_paro_por_fallo()
{
  // PARO del programa en marcha (con sensores FALLO_1, o FALLO_2)
  if (estado_general == STATE_MARCHA)
  {
    digitalWrite(LED_PLACA, LOW);             // Máquina en paro
    //digitalWrite(PIN_MOTOR_DEVANADOR, LOW); // Desactivamos motor devanador
    digitalWrite(PIN_MOTOR_VARIADOR, LOW);    //Desactivamos variador
    digitalWrite(PIN_MOTOR_FRENO, HIGH);      // Frenado instantáneo

    // Set estado PARO
    estado_general = STATE_PARO;
    tipo_parada = TIPO_PARADA_EMERGENCIA;
  }
}

void isr_set_estado_paro_o_reset_manual()
{
  // PARO manual del programa en marcha (con botón PARO/RESET)
  if (estado_general == STATE_MARCHA)
  {
    digitalWrite(LED_PLACA, LOW);             // Máquina en paro
    //digitalWrite(PIN_MOTOR_DEVANADOR, LOW); // Desactivamos motor devanador
    digitalWrite(PIN_MOTOR_VARIADOR, LOW);    //Desactivamos variador
    digitalWrite(PIN_MOTOR_FRENO, HIGH);      // Frenado instantáneo

    // Set estado PARO
    estado_general = STATE_PARO;
    tipo_parada = TIPO_PARADA_MANUAL;
  }
  // Entrada a RESET manual tras PARO (con botón PARO/RESET)
  // TODO Comprobar que la isr no se active 2 veces (paro + reset) con el pulsador. ¿Fijar t_paro para comparar?
  // else if (estado_general == STATE_PARO && sinceStart > 1000)
  else if (estado_general == STATE_PARO)
  {
    digitalWrite(PIN_MOTOR_DEVANADOR, LOW); // Desactivamos motor devanador
    estado_general = STATE_RESET;
    tipo_parada = TIPO_PARADA_FINALIZACION;
  }
}

void set_estado_paro_por_finalizacion(uint16_t num_periodos_freno)
{
  digitalWrite(LED_PLACA, LOW);             // Máquina en paro
  digitalWrite(PIN_MOTOR_DEVANADOR, LOW);   // Desactivamos motor devanador
  digitalWrite(PIN_MOTOR_VARIADOR, LOW);    // Desactivamos variador

  // Frenado antes de selección:
  proceso_de_freno_motor(num_periodos_freno);

  estado_general = STATE_PARO;
  sinceStart = 0;
  sinceStatus = 0;
  tipo_parada = TIPO_PARADA_FINALIZACION;
}

void set_estado_seleccion()
{
  set_texto_lcd(String("* Selecciona P:"), String("- PROGRAMA ") + (programa_seleccionado + 1));
  estado_general = STATE_SELECC;
  tipo_parada = TIPO_PARADA_FINALIZACION;
  hay_doble_click_rotary_enc = false;
  hay_click_rotary_enc = false;
  (void)rotary_encoder->getValue();
}

void set_estado_config()
{
  set_texto_lcd(String("** Edicion P_") + (programa_seleccionado + 1), String(OPCIONES_PROGRAMA[variable_seleccionada]));
  estado_general = STATE_CONFIG;
  hay_doble_click_rotary_enc = false;
  hay_click_rotary_enc = false;
}

void eeprom_save_config_programa(int indice_programa, bool verbose_mode)
{
  int eeAddress;
  config_prog_t conf_i;

  conf_i = programas[indice_programa];
  eeAddress = sizeof(bool) + sizeof(config_prog_t) * indice_programa;
  EEPROM.put(eeAddress, conf_i);

  if (verbose_mode)
  {
    Serial.print("Se sobreescribe PROGRAMA ");
    Serial.print(indice_programa + 1);
    Serial.println(" en EEPROM: ");
    print_config_programa(indice_programa);
    Serial.println("");
  }
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

  // Refresco LCD
  i_refresca_inf_izq_lcd();
}

void configuracion_parametros_programa_actual()
{
  // Modo de edición de variables del programa seleccionado
  bool programa_modificado = false;

  // Loop hasta que se entra en edición de parámetro o se sale de configuración:
  hay_doble_click_rotary_enc = false;
  hay_click_rotary_enc = false;

  if (VERBOSE)
  {
    Serial.print('Entrando en modo configuracion del P_');
    Serial.println(programa_seleccionado + 1);
    print_config_programa(programa_seleccionado);
  }

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
        case ClickEncoder::Clicked: // Edición de parámetro
        {
          config_prog_t programa_edit = programas[programa_seleccionado];
          bool hay_cambio_valor;

          switch (variable_seleccionada)
          {
            case 0:
              hay_cambio_valor = edicion_variable_con_rotary_encoder(String(OPCIONES_PROGRAMA_CORTAS[variable_seleccionada]),
                                                                     &programa_edit.numero_movimientos,
                                                                     OPCIONES_RANGO_MIN[variable_seleccionada], OPCIONES_RANGO_MAX[variable_seleccionada], 1.);
              break;
            case 1:
              hay_cambio_valor = edicion_variable_con_rotary_encoder(String(OPCIONES_PROGRAMA_CORTAS[variable_seleccionada]),
                                                                     &programa_edit.distancia_movimiento_num_pasos,
                                                                     OPCIONES_RANGO_MIN[variable_seleccionada], OPCIONES_RANGO_MAX[variable_seleccionada], 1.);
              break;
            case 2:
              hay_cambio_valor = edicion_variable_con_rotary_encoder(String(OPCIONES_PROGRAMA_CORTAS[variable_seleccionada]),
                                                                     &programa_edit.velocidad_movimiento_rpm,
                                                                     OPCIONES_RANGO_MIN[variable_seleccionada], OPCIONES_RANGO_MAX[variable_seleccionada], 1.);
              break;
            case 3:
              hay_cambio_valor = edicion_variable_con_rotary_encoder(String(OPCIONES_PROGRAMA_CORTAS[variable_seleccionada]),
                                                                     &programa_edit.num_periodos_freno,
                                                                     OPCIONES_RANGO_MIN[variable_seleccionada], OPCIONES_RANGO_MAX[variable_seleccionada], PERIODO_FRENO_SEG);
              break;
            case 4:
              hay_cambio_valor = edicion_variable_con_rotary_encoder(String(OPCIONES_PROGRAMA_CORTAS[variable_seleccionada]),
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
          if (VERBOSE)
          {
            Serial.print("Saliendo de edición de valor. Hay cambios? ");
            Serial.println(hay_cambio_valor);
            Serial.print("; programa_modificado? ");
            Serial.println(programa_modificado);
            print_config_programa(programa_seleccionado);
          }
          set_estado_config();
          break;
        }
        case ClickEncoder::DoubleClicked: // Salida a selección de programa
        {
          if (VERBOSE)
          {
            Serial.print('Saliendo del modo configuracion del P_');
            Serial.println(programa_seleccionado + 1);
            print_config_programa(programa_seleccionado);
          }

          // Grabación de programa seleccionado en EEPROM
          if (programa_modificado)
          {
            eeprom_save_config_programa(programa_seleccionado, VERBOSE);
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

bool edicion_variable_con_rotary_encoder(String ref_variable, uint16_t *valor_edit, uint16_t valor_min, uint16_t valor_max, float factor_conv_variable)
{
  // Loop hasta que se selecciona valor con click o doble click:
  //String nombre_var = ref_variable.substring(2);
  String nombre_var = ref_variable;
  int valor_encoder;

  int valor_ant = *valor_edit;
  int valor = valor_ant;
  int last_valor = valor_ant;

  //Serial.println("Entrando en config var=" + nombre_var);
  (void)rotary_encoder->getValue();
  hay_doble_click_rotary_enc = false;
  set_texto_lcd(String("* Configurando:"), String(nombre_var + " (" + (valor_ant * factor_conv_variable) + ")"));
  delay(DELAY_MS_CMD_MENU / 2);
  set_texto_lcd(String(nombre_var + " (" + (valor_ant * factor_conv_variable) + ")"), String("- <--> + :  ") + (valor * factor_conv_variable));
  while (!hay_doble_click_rotary_enc & !hay_click_rotary_enc)
  {
    valor_encoder = rotary_encoder->getValue();
    //if (abs(valor_encoder) > 0)
    //  Serial.println(valor_encoder * factor_conv_variable);
    valor += valor_encoder;
    if (valor != last_valor)
    {
      if (valor < valor_min)
        valor = valor_min;
      else if (valor > valor_max)
        valor = valor_max;
      set_texto_fila_lcd(String("- <--> + :  ") + (valor * factor_conv_variable), 1);
    }
    last_valor = valor;

    ClickEncoder::Button b = rotary_encoder->getButton();
    if (b != ClickEncoder::Open)
    {
      switch (b)
      {
        CASEBREAK(ClickEncoder::Pressed);
        CASEBREAK(ClickEncoder::Held);
        CASEBREAK(ClickEncoder::Released);
        case ClickEncoder::Clicked:
          hay_click_rotary_enc = true;
          break;
        case ClickEncoder::DoubleClicked:
          hay_doble_click_rotary_enc = true;
          break;
      }
    }
    // Para no avanzar demasiado por paso! --> TODO Mejorar con el sistema de aceleración
    delay(200);
  }
  //set_texto_lcd(String("** Guardado"), String("--> PROGRAMA ") + (programa_seleccionado + 1));
  set_texto_fila_lcd(String("Se acepta: ") + (valor * factor_conv_variable), 1);

  delay(DELAY_MS_CMD_MENU / 2);
  hay_doble_click_rotary_enc = false;
  hay_click_rotary_enc = false;
  if (valor != valor_ant)
  {
    *valor_edit = valor;
    return true;
  }
  return false;
}

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

String i_time_string(float millis)
{
  return String(millis / 1000., 1);
}

bool i_hay_cambio_unitario_rotary_encoder(uint8_t *variable_edit, uint16_t valor_max)
{
  int valor_encoder = rotary_encoder->getValue();
  if (abs(valor_encoder) > 1)
    valor_encoder /= abs(valor_encoder);
  if ((abs(valor_encoder) > 0) & (((*variable_edit > 0) & (valor_encoder == -1)) | ((*variable_edit < valor_max) & (valor_encoder == 1))))
  {
    *variable_edit += valor_encoder;
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

void i_print_config_programa(uint8_t indice_programa, const config_prog_t conf_print)
{
  Serial.print("PROGRAMA_");
  Serial.print(indice_programa + 1);
  Serial.print(": MOVS=");
  Serial.print(conf_print.numero_movimientos);
  Serial.print("; DIST (num. pasos)=");
  Serial.print(conf_print.distancia_movimiento_num_pasos);
  Serial.print("; VEL (rpm)=");
  Serial.print(conf_print.velocidad_movimiento_rpm);
  Serial.print("; FRENO (s)=");
  Serial.print(conf_print.num_periodos_freno * PERIODO_FRENO_SEG);
  Serial.print("; OFFSET (num. pasos)=");
  Serial.println(conf_print.distancia_offset_num_pasos);
}

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
  lcd.setCursor(0, n_fila);
  lcd.print(i_completa_fila_led(fila));
}

void set_texto_lcd(String fila_1, String fila_2)
{
  //lcd.clear();
  set_texto_fila_lcd(fila_1, 0);
  set_texto_fila_lcd(fila_2, 1);
}
