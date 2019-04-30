
#include <avr/pgmspace.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

/* ****************************************************************** */
/* *** Definições diversas ****************************************** */

// Descomente para inverter a rotação (somente) do Motor A
// => Necessário com os motores montados simetricamente

#define SSPEED        115200   // Velocidade da interface serial

#define FULL_BAT        8000   // Valor em mV para bat. completamente carregada
#define DEAD_BAT        6000   // Valor em mV para bat. esgotada ( recarregar )

/* ****************************************************************** */
/* Limitações de PWM e velocidade para uso no controlador PID ******* */

#define PWM_MIN         0x0F   // PWM mín. p/ garantir movimento das duas rodas
#define PWM_MAX         0x9F   // PWM máx. para que os motores tenham aprox. 5V
#define SPD_MIN           50   // Vel. mín. em mm/s ( Condição: PWM > PWM_MIN )
#define SPD_MAX          500   // Vel. máx. em mm/s ( Condição: PWM < PWM_MAX )

/* ****************************************************************** */
/* Estas são as conexões de hardware mapeadas aos pinos do Arduino ** */

#define LED         4      // Led conectado a uma saída digital

#define RADIO_CE    7      // Pino CE do módulo de rádio
#define RADIO_CS    8      // Pino CS do módulo do rádio
#define RADIO_A0   A4      // Bit 0 do end. do rádio (LOW = ligado)
#define RADIO_A1   A5      // Bit 1 do end. do rádio (LOW = ligado)

#define IRQ_ENC_A   2      // Pino de interrupção do Encoder A
#define IRQ_ENC_B   3      // Pino de interrupção do Encoder B
#define IRQ_RADIO   5      // Pino de interrupção do Rádio

#define HBRID_EN    6      // Habilita a ponte H (High)
#define MTR_AIN1   A2      // Bit 0 - Controle da ponte H do Motor A
#define MTR_AIN2   A3      // Bit 1 - Controle da ponte H do Motor A
#define MTR_BIN1   A1      // Bit 0 - Controle da ponte H do Motor B
#define MTR_BIN2   A0      // Bit 1 - Controle da ponte H do Motor B
#define MTR_PWMA    9      // Sinal de PWM para controle  do Motor A
#define MTR_PWMB   10      // Sinal de PWM para controle  do Motor B

#define VOLT_BAT   A7      // Tensão da bateria -> Vcc/10

/* ******************************************************************* */
/* Definições de estruturas de dados ( funcionais, status e controle ) */

typedef union {
  struct {
    uint8_t  pwm_motor_B;        // (bits 0-7)   8 bits: Valor do PWM do Motor B
    uint8_t  pwm_motor_A;        // (bits 8-15)  8 bits: Valor do PWM do Motor A
    uint8_t  dir_motor_B : 2,    // (bits 16-17) 2 bits: BIN1 e BIN2  da ponte H
             dir_motor_A : 2,    // (bits 18-19) 2 bits: AIN1 e AIN2  da ponte H
             ign1_4b     : 4;    // (bits 20-23) 4 bits não utilizados (padding)
    uint8_t  ign2_8b;            // (bits 24-31) 8 bits não utilizados (padding)
  } config;
  uint32_t status = 0;             // Leitura/Escrita simuntânea do conjunto de variáveis.
} TMotCtrl;

typedef struct {
  uint32_t last_v = 0; // Tempo variavel
  uint32_t last_10ms   = 0;   // Controle das tarefas executadas a cada 10ms
  uint32_t last_100ms  = 0;   // Controle das tarefas executadas a cada 100ms
  uint32_t last_1000ms = 0;   // Controle das tarefas executadas a cada 1000ms
} TasksTCtr;

/* ******************************************************************* */
/* *** Variáveis globais e instanciações ***************************** */


TasksTCtr tasks;		// Contagem de tempo para execução de tarefas

uint32_t  blinker;    // Contagem de alterações de estado do LED
uint16_t  enca;
uint16_t  encb;

TMotCtrl motor;

/* ******************************************************************* */
/* *** Protótipos das funções **************************************** */
//
// Obs: Este bloco não é necessário para compilação mas é útil como
//      referência durante o processo de desenvolvimento.

void     tasks_10ms( void );
void     tasks_100ms( void );
void     tasks_800ms( void );
void     tasks_1000ms( void );
//pt1
void     led(/*uint16_t ms*/);
uint8_t get_node_addr( void );
uint16_t get_volt_bat( void );
void status_encoders( uint16_t *count_enc_a, uint16_t *count_enc_b);
//pt2
void set_motor_status( uint32_t );
/* ******************************************************************* */
/* *** SETUP ********************************************************* */

void setup() {

  Serial.begin(115200);               // Inicialização da com. serial

  // Inicialização do pino do LED
  pinMode(LED, OUTPUT);               // Pino do LED como saída digital
  digitalWrite(LED, LOW);

  analogReference(INTERNAL);          // Referência dos ADCs -> 1.1V

  SPI.begin();                        // Inicializa a interface SPI

  blinker = 0;            // Inicialização da variável

  pinMode(RADIO_A0, INPUT_PULLUP);    // Endereço deste nó: bit 0
  pinMode(RADIO_A1, INPUT_PULLUP);    // Endereço deste nó: bit 1

  //pinMode(IRQ_ENQ_A, INPUT_PULLUP);
  //pinMode(IRQ_ENQ_B, INPUT_PULLUP);

}


/* ******************************************************************* */
/* *** LOOP PRINCIPAL ************************************************ */

void loop() {

  tasks_10ms();                // Tarefas executadas a cada 10ms
  tasks_100ms();               // Tarefas executadas a cada 100ms
  tasks_1000ms();              // Tarefas executadas a cada 1000ms
  // pt1
  led();
  get_node_addr();
  get_volt_bat();
  status_encoders(enca, encb);
  //pt2
  void set_motor_status( uint32_t );


}


/* ******************************************************************* */
/* *** FUNÇÕES (implementações) ************************************** */

/* *********************************************************************
   Tarefas que devem ser executadas em intervalos de 10ms
*/
void tasks_10ms( void ) {

  if ( (millis() - tasks.last_10ms) > 10 ) {
    tasks.last_10ms = millis();



  }
}

/* *********************************************************************
   Tarefas que devem ser executadas em intervalos de 100ms
*/
void tasks_100ms( void ) {

  if ( (millis() - tasks.last_100ms) > 100 ) {
    tasks.last_100ms = millis();



  }
}

/* *********************************************************************
   Tarefas que devem ser executadas em intervalos de 800ms
*/
void led(/*uint16_t ms*/) {
  uint16_t ms = 800;
  if ( (millis() - tasks.last_v) > ms) {
    tasks.last_v = millis();

    blinker++;

    if ( (blinker % 2 ) == 0)
      digitalWrite(LED, LOW);
    else
      digitalWrite(LED, HIGH);
  }
}

/* *********************************************************************
   Tarefas que devem ser executadas em intervalos de 1000ms
*/
void tasks_1000ms( void ) {

  if ( (millis() - tasks.last_1000ms) > 1000 ) {
    tasks.last_1000ms = millis();
    /*
      // Conta execuções
      blinker++;
      // Escreve "bit 0" de "blinker" para o LED
      //digitalWrite(LED, bitRead(blinker, 0));
      if(blinker % 2 == 0)
      digitalWrite(LED,HIGH);
      else
      digitalWrite(LED,LOW);
      // Serial.write(blinker);  // Escreve dados binários na porta serial
      // Serial.print(blinker);  // Imprime dados na porta serial como texto ASCII
      Serial.println(blinker);   // Idem ao "print", adicionando EOL ao string
    */
  }
}

uint8_t get_node_addr( void ) {

  uint8_t addr = 0;
  uint8_t addr2 = 0;
  addr = digitalRead(RADIO_A0);
  addr2 = digitalRead(RADIO_A1);

  addr = ((addr << 1) | addr2);
  Serial.println(addr);

  return (addr);

}

uint16_t get_volt_bat( void ) {

  uint16_t volt = analogRead(VOLT_BAT);
  volt = (((volt * 10) * 1.1) / 1023.0) * 1000.0;
  Serial.println(volt);

  return volt;

}

void status_encoders( uint16_t count_enc_a, uint16_t count_enc_b) {

  attachInterrupt(digitalPinToInterrupt(count_enc_a), counta, RISING);

  attachInterrupt(digitalPinToInterrupt(count_enc_b), countb, RISING);

}

void counta () {
  enca++;
  Serial.println(enca);
}

void countb () {
  encb++;
  Serial.println(encb);
}

void set_motor_status( uint32_t msg) {
    
    motor.status = msg;
    
    digitalWrite(HBRID_EN, LOW);

    digitalWrite(MTR_AIN1, bitRead(motor.config.dir_motor_A, 0));
    digitalWrite(MTR_AIN2, bitRead(motor.config.dir_motor_A, 1));
    digitalWrite(MTR_BIN1, bitRead(motor.config.dir_motor_B, 0));
    digitalWrite(MTR_BIN2, bitRead(motor.config.dir_motor_B, 1));

    analogWrite(MTR_PWMA, motor.config.pwm_motor_A);
    analogWrite(MTR_PWMB, motor.config.pwm_motor_B);

    digitalWrite(HBRID_EN, HIGH);
}

uint32_t get_motor_status(){
  
    return motor.status;
}

bool is_motor_locked( uint8_t engine) {
  
  if (engine)
     return !(bitRead(motor.config.dir_motor_A,0) ^ bitRead(motor.config.dir_motor_A,1));
  else
     return !(bitRead(motor.config.dir_motor_B,0) ^ bitRead(motor.config.dir_motor_B,1));
  }

 uint8_t set_pwm_max( void ) {
  
  
  
  }
/* ****************************************************************** */
/* ****************************************************************** */
