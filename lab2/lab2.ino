/*
* Laboratorio 2 - Sistemas de Control 2
* Jacqueline Guarcax y Gabriela Iriarte
* 10/02/2020
* Este programa controla un sistema de tercer orden con un PID
* y puede hacer el muestreo con 4 métodos distintos de muestreo.
* En la salida del control se coloca un DAC serial 4921.
* 
* 
*          |͞͞͞͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞ |    |͞͞͞͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞ |    |͞͞͞͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞͞͞͞ ͞ |    
* step --->| ADC TIVA  |--->| DAC serial|--->|  Planta   |---> Y
*          |___________|    |___________|    |___________| |
*                ˄                                         |
*                |_________________________________________| 
*                
*/

#include <SPI.h>
#include "wiring_private.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"

// MUESTREO
#define COTA_SUP  3.3
#define COTA_INF  0
#define timer_val 80000.0
#define dT 0.001

// Pin de Chip Select que usamos para el control del DAC serial
const int CSpin = PB_0;

// Pin de chip select por default de la tiva
// No lo usamos pero es necesario recordar que NO DEBE
// de ser usado.
const int CSpin_tiva = PB_5;

// Declaración del pin de entrada (analógico)
const int inputPin = A0; // PE_3 
const int yPin = A1; // no se que pin es xdxd

// Contador para pruebas
int Contador = 0;

// Variable donde almacenamos la entrada del sistema
int entrada;

// VARIABLES PARA PID
float r = 0;
float y = 0;
float e_k = 0;
float e_k_1 = 0;
float e_k_2 = 0;
float u_k = 0;
float u_k_1 = 0;
float u_k_2 = 0;

float E_k = 0;
float eD = 0;

// Variable uk pero tipo entero
// Para enviar al DAC
int u_k_int = 0;

// Variables para cambio de método
// Por default se tendrá el método de Tustin
String metodo = "Tustin";
float b0 = 29.8;
float b1 = -55.3;
float b2 = 25.7;
float a1 = 0.162;
float a2 = 0.838;

void setup() {

  if (metodo == "zpm") {
   b0 = 136.0;
   b1 = -270.0;
   b2 = 134.0;
   a1 = 1.1;
   a2 = -0.104;
  }
  else if (metodo == "zoh") {
   b0 = 342.0;
   b1 = -683.0;
   b2 = 341.0;
   a1 = 1.8;
   a2 = -0.797;
  }
  else if (metodo == "euler") {
   b0 = 106.0;
   b1 = -211.0;
   b2 = 105.0;
   a1 = 1.31;
   a2 = -0.306;
  }

  Serial.begin(9600); // Para debugging (imprimir en pantalla)

  // Configuración de los pines a utilizar
  pinMode(CSpin_tiva, OUTPUT); // No lo utilizamos porque da problema
  pinMode(CSpin, OUTPUT); // Nuestro ChipSelect

  pinMode(inputPin, INPUT); // Entrada del control
  pinMode(yPin, INPUT); // Salida
  
  // start the SPI library:
  SPI.begin();
  configureTimer1A();

}

void loop() {
  // nada 
  
}


/*
 * sendSPI manda el entero data al DAC serial y utiliza el pin cspin
 * como chip select del dac en vez de usar el de la tiva porque el de
 * la tiva solo deja enviar un byte a la vez y este dac serial requiere
 * que se le envien dos bytes a la vez.
 */
 
void sendSPI(int data, byte cspin){
  int conf = 0b01110000;
  int dummy = (data & 0b0000111100000000)>>8;
  byte byte1 = conf | dummy;
  byte byte2 = (byte)((data & 0b0000000011111111));
  digitalWrite(cspin, LOW); 
  SPI.transfer(byte1);
  SPI.transfer(byte2);
  digitalWrite(cspin, HIGH);
  }

// Función que configura el timer (1A en este ejemplo)
void configureTimer1A(){
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // Enable Timer 1 Clock
  ROM_IntMasterEnable(); // Enable Interrupts
  ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // Configure Timer Operation as Periodic
  
  // Configure Timer Frequency
  // El tercer argumento ("CustomValue") de la siguiente función debe ser un entero, no un float.
  // Ese valor determina la frecuencia (y por lo tanto el período) del timer.
  // La frecuecia está dada por: MasterClock / CustomValue
  // En el Tiva C, el MasterClock es de 80 MHz.
  // Ejemplos:
  
  // Si se quiere una frecuencia de 1 Hz, el CustomValue debe ser 80000000. 80MHz/80M = 1 Hz
  // Si se quiere una frecuencia de 1 kHz, el CustomValue debe ser 80000. 80MHz/80k = 1 kHz
  ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, timer_val); // El último argumento es el CustomValue

  // 160000000 - 0.5 Hz
  // 40000000  - 2   Hz
  // deltaT = 1 ms debería de funcionar bien
  // Al parecer, no hay función ROM_TimerIntRegister definida. Usar la de memoria FLASH
  // El prototipo de la función es:
  //    extern void TimerIntRegister(uint32_t ui32Base, uint32_t ui32Timer, void (*pfnHandler)(void));
  // Con el tercer argumento se especifica el handler de la interrupción (puntero a la función).
  // Usar esta función evita tener que hacer los cambios a los archivos internos de Energia,
  // sugeridos en la página de donde se tomó el código original.
  TimerIntRegister(TIMER1_BASE, TIMER_A, &Timer1AHandler);
  
  ROM_IntEnable(INT_TIMER1A);  // Enable Timer 1A Interrupt
  ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // Timer 1A Interrupt when Timeout
  ROM_TimerEnable(TIMER1_BASE, TIMER_A); // Start Timer 1A
  
}

// Handler (ISR) de la interrupción del Timer
void Timer1AHandler(void){
  //Required to launch next interrupt
  ROM_TimerIntClear(TIMER1_BASE, TIMER_A);

  /*
   // Diente de sierra para pruebas
  Contador += 1;
  if (Contador == 4095){
    Contador = 0;
  }
  */

  //Serial.print("\t output = "); 
  //Serial.println(Contador); 
  
  

  // Convertimos los datos a voltaje para evitar overflow en el manejo de
  // números muy grandes y para fácil visualización de los datos para
  // debugging. El ADC de la TivaC tiene una resolución de 12 bits.
  
  r = 3.3*analogRead(inputPin)/4095.0;
  y = 3.3*analogRead(yPin)/4095.0;

  // Cálculo del error
  e_k = r - y;

  // PID
  u_k = b0*e_k + b1*e_k_1 + b2*e_k_2 + a1*e_k + a2*e_k;
  
  e_k_2 = e_k_1;
  u_k_2 = u_k_1;
  
  u_k_1 = u_k;
  e_k_1 = e_k;
  
  // Se trunca el valor de u_k 
  if (u_k > COTA_SUP){
    u_k = COTA_SUP;
    }
  else if (u_k < COTA_INF){
    u_k = COTA_INF;
    }

  // Se regresa el mapeo a bits. El DAC serial tiene una resolución de 12 bits.
  u_k_int = (int)(u_k*4095.0/3.3);

  // Descomentar las siguientes 2 líneas para replicar la señal de entrada:
  //entrada = analogRead(inputPin);
  //sendSPI(entrada, CSpin);

  // Envío de los datos al DAC serial
  sendSPI(u_k_int, CSpin);
}
  
