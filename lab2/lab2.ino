#include <SPI.h>
#include "wiring_private.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"

// MUESTREO
#define timer_val 8.0
#define dT 
const int CSpin = PB_0;
const int CSpin_tiva = PB_5;
const int inputPin = PB_6;
int Contador = 0;

void setup() {

  Serial.begin(9600); 
  pinMode(CSpin_tiva, OUTPUT); // No lo utilizamos porque da problema
  pinMode(CSpin, OUTPUT); // Nuestro ChipSelect
  
  // start the SPI library:
  SPI.begin();
  configureTimer1A();

}

void loop() {
  // put your main code here, to run repeatedly: 
  
}

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
   //Diente de sierra
  Contador += 1;
  if (Contador == 4095){
    Contador = 0;
  }
  */

  //Serial.print("\t output = "); 
  //Serial.println(Contador); 
 
  sendSPI(inputPin, CSpin);
  
}
  
