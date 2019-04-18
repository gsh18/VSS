
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


typedef struct {
    uint32_t last_10ms   = 0;   // Controle das tarefas executadas a cada 10ms
    uint32_t last_100ms  = 0;   // Controle das tarefas executadas a cada 100ms
    uint32_t last_800ms = 0;
    uint32_t last_1000ms = 0;   // Controle das tarefas executadas a cada 1000ms
} TasksTCtr;

/* ******************************************************************* */
/* *** Variáveis globais e instanciações ***************************** */


TasksTCtr tasks;		// Contagem de tempo para execução de tarefas

uint32_t  blinker;    // Contagem de alterações de estado do LED
uint32_t  blinker2;    // Contagem de alterações de estado do LED
uint8_t  bit1;    // Contagem de alterações de estado do LED
uint8_t  bit2;    // Contagem de alterações de estado do LED
uint16_t enca;
uint16_t encb;

/* ******************************************************************* */
/* *** Protótipos das funções **************************************** */
//
// Obs: Este bloco não é necessário para compilação mas é útil como
//      referência durante o processo de desenvolvimento.

void     tasks_10ms( void );
void     tasks_100ms( void );
void     tasks_800ms( void );
void     tasks_1000ms( void );
uint8_t get_node_addr( void );
uint16_t get_volt_bat( void );
void status_encoders( uint16_t *count_enc_a, uint16_t *count_enc_b);

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
    blinker2 = 0;            // Inicialização da variável

    pinMode(RADIO_A0, INPUT_PULLUP);    // Endereço deste nó: bit 0
    pinMode(RADIO_A1, INPUT_PULLUP);    // Endereço deste nó: bit 1
       
}


/* ******************************************************************* */
/* *** LOOP PRINCIPAL ************************************************ */

void loop() {
    
    tasks_10ms();                // Tarefas executadas a cada 10ms
    tasks_100ms();               // Tarefas executadas a cada 100ms
    tasks_800ms();              // Tarefas executadas a cada 800ms
    tasks_1000ms();              // Tarefas executadas a cada 1000ms
    get_node_addr();
    get_volt_bat();
    status_encoders(enca, encb);

}


/* ******************************************************************* */
/* *** FUNÇÕES (implementações) ************************************** */

/* *********************************************************************
 * Tarefas que devem ser executadas em intervalos de 10ms
 */
void tasks_10ms( void ) {

     if( (millis() - tasks.last_10ms) > 10 ){
        tasks.last_10ms = millis();



    }
}

/* *********************************************************************
 * Tarefas que devem ser executadas em intervalos de 100ms
 */
void tasks_100ms( void ) {

    if( (millis() - tasks.last_100ms) > 100 ){
        tasks.last_100ms = millis();



    }
}

/* *********************************************************************
 * Tarefas que devem ser executadas em intervalos de 800ms
 */
void tasks_800ms( void ) {

    if( (millis() - tasks.last_800ms) > 800 ){
        tasks.last_100ms = millis();

        blinker2++;

        // ciclo 0.8s
        if ( (blinker2 % 2 ) == 0)
          digitalWrite(LED,HIGH);
        else 
          digitalWrite(LED,LOW);
/*
        // ciclo 1.6s
       if ( (blinker2 % 4 ) == 0)
          digitalWrite(LED,HIGH);
        else 
          digitalWrite(LED,LOW);

        // ciclo 3.2s
       if ( (blinker2 % 8 ) == 0)
          digitalWrite(LED,HIGH);
        else 
          digitalWrite(LED,LOW);
          
        // ciclo 4s
       if ( (blinker2 % 10 ) == 0)
          digitalWrite(LED,HIGH);
        else 
          digitalWrite(LED,LOW);

        // ciclo 8s
       if ( (blinker2 % 20 ) == 0)
          digitalWrite(LED,HIGH);
        else 
          digitalWrite(LED,LOW);
*/
    }
}

/* *********************************************************************
 * Tarefas que devem ser executadas em intervalos de 1000ms
 */
void tasks_1000ms( void ) {

    if( (millis() - tasks.last_1000ms) > 1000 ){
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
/*
    bit1 = digitalRead(RADIO_A0);
    bit2 = digitalRead(RADIO_A1);    
  
    bit1 = ((bit2<<1) + bit1);
    Serial.println(bit1);

    //return();????
*/
}

uint16_t get_volt_bat( void ) {
  
   bit1 = digitalRead(VOLT_BAT);

  
}

void status_encoders( uint16_t *count_enc_a, uint16_t *count_enc_b) {

    

      
}

/* ****************************************************************** */
/* ****************************************************************** */
