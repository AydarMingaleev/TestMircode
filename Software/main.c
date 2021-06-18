#include "nrf52.h"
#include "stdbool.h"
#include "string.h"

#define UART_BAUDRATE (uint32_t) (0x1D60000) //BAUDRATE 115200 for UART
#define UART_TX_PIN (uint32_t) (0x03) //Pin P0_03
#define UART_RX_PIN (uint32_t) (0x05) //Pin P0_05
#define UART_CTS_PIN (uint32_t) (0x10) //Pin P0_16
#define UART_RTS_PIN (uint32_t) (0x08) //Pin P0_8
#define UART_ENABLE (uint32_t) (0x08) //Enable UART
#define UART_DISABLE (uint32_t) (0x00) //Disable UART

#define COUNT_DELAY 10000

#define TURN_OFF 0
#define TURN_ON 1

#define SDN_PIN 0x0C //P0_12
#define LOD_PIN 0x04 //P0_04
#define OUTPUT_PIN 0x03 //P0_03 AIN1
#define CONTROL_PIN 0x05 //P0_05

static uint32_t bytewise_bitswap(uint32_t inp);
static uint32_t swap_bits(uint32_t inp);

//Инициализация тактирования
static void clock_initialization(void);
//Инициализация UART
static void UART_initialization(void);
//Отправка байта
uint8_t UART_send_byte(char buffer);
//Отправка текста
void UART_send_text(char* buffer);

//Инициализация BLE
static void BLE_initialization(void);
//Отправка сообщение BLE
void BLE_send_packet(void);

//Инициализация портов для AD8233
static void initialization_gpio(void);
//Включение/отключение микросхемы AD8233
static void control_AD8233(uint8_t state);
//Прием сигнала
static void give_signal(void);

//Обработка индикация ошибок
static void error_signal(void);

int main()
{
  clock_initialization();
  initialization_gpio();
  UART_initialization();
  BLE_initialization();
  while (1){
    control_AD8233(TURN_ON);
    give_signal();
  }
}

void clock_initialization(void){
  uint16_t count = 0;
  NRF_CLOCK->EVENTS_HFCLKSTARTED |= (uint32_t) 0x00;
  NRF_CLOCK->TASKS_HFCLKSTART |= (uint32_t) 0x01;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED != 0x01 || count<COUNT_DELAY){}; 
  if (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0 && count>=COUNT_DELAY){
    error_signal();
  }
  
  count = 0;
  NRF_CLOCK->LFCLKSRC |= (uint32_t) 0x01; //Enable extern crystall
  NRF_CLOCK->EVENTS_LFCLKSTARTED |= (uint32_t) 0x00;
  NRF_CLOCK->TASKS_LFCLKSTART |= (uint32_t) 0x01;
  while (NRF_CLOCK->EVENTS_LFCLKSTARTED  != 0x01 || count<COUNT_DELAY){}; 
  if (NRF_CLOCK->EVENTS_LFCLKSTARTED  == 0 && count>=COUNT_DELAY){
    error_signal();
  }
}

void UART_initialization(void){
  //Инициализация GPIO
  NRF_P0->PIN_CNF[3] |= (uint32_t) 0x03; //TX -output 
  NRF_P0->PIN_CNF[5] |= (uint32_t) 0x0C; //RX - input PullUP
  NRF_P0->PIN_CNF[16] |= (uint32_t) 0x0C; //CTS - input Pullup
  NRF_P0->PIN_CNF[8] |= (uint32_t) 0x03; //RTS - output 
  //baudrate 115200
  NRF_UART0->BAUDRATE |= UART_BAUDRATE;
  //Конфигурация 
    //HWFC -enable
    //Parity - include event parity
    //STOP - 1 stop bit
  NRF_UART0->CONFIG |= (uint32_t)0x1E;
  //Настройка пинов
  NRF_UART0->PSELTXD |= UART_TX_PIN;
  NRF_UART0->PSELRXD |= UART_RX_PIN;
  NRF_UART0->PSELCTS |= UART_CTS_PIN;
  NRF_UART0->PSELRTS |= UART_RTS_PIN;
  //Включить uart
  NRF_UART0->ENABLE |= UART_ENABLE;
}

uint8_t UART_send_byte(char buffer){
  NRF_UART0->TXD = (uint32_t)buffer;
  NRF_P0->OUT |= UART_RTS_PIN;
  if (!NRF_UART0->EVENTS_CTS && NRF_UART0->EVENTS_TXDRDY){
        NRF_UART0->TASKS_STARTTX;
  }
  else{
    NRF_UART0->TASKS_STOPTX;
    return 1;
  }
  NRF_P0->OUT &= ~UART_RTS_PIN;
  return 0;
}

void UART_send_text(char* buffer){
  uint8_t count = 0;
  while(*(buffer+count)){
    UART_send_byte(*(buffer + count));
    count++;
  }
}

void BLE_initialization(void){
   // Radio config
   NRF_RADIO->TXPOWER  |= (uint32_t) 0x00;
   NRF_RADIO->FREQUENCY = 7UL;  // Frequency bin 7, 2407MHz

    NRF_RADIO->PREFIX0 =
        ((uint32_t)swap_bits(0xC3) << 24) // Prefix byte of address 3 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC2) << 16) // Prefix byte of address 2 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC1) << 8)  // Prefix byte of address 1 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC0) << 0); // Prefix byte of address 0 converted to nRF24L series format

    NRF_RADIO->PREFIX1 =
        ((uint32_t)swap_bits(0xC7) << 24) // Prefix byte of address 7 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC6) << 16) // Prefix byte of address 6 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC4) << 0); // Prefix byte of address 4 converted to nRF24L series format

    NRF_RADIO->BASE0 = bytewise_bitswap(0x01234567UL);  // Base address for prefix 0 converted to nRF24L series format
    NRF_RADIO->BASE1 = bytewise_bitswap(0x89ABCDEFUL);  // Base address for prefix 1-7 converted to nRF24L series format

    NRF_RADIO->TXADDRESS   = 0x00UL;  // Set device address 0 to use when transmitting
    NRF_RADIO->RXADDRESSES = 0x01UL;  // Enable device address 0 to use to select which addresses to receive

    NRF_RADIO->PCNF0 = (uint32_t) 0x00;

    NRF_RADIO->PCNF1 = (uint32_t) 0xD2;

    // CRC
    NRF_RADIO->CRCCNF = (uint32_t) 0x02; // Number of checksum bits
    if ((NRF_RADIO->CRCCNF & 0x03UL) == (0x02UL))
    {
        NRF_RADIO->CRCINIT = 0xFFFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x11021UL;  // CRC poly: x^16 + x^12^x^5 + 1
    }
    else if ((NRF_RADIO->CRCCNF & 0x03UL) == (0x02UL))
    {
        NRF_RADIO->CRCINIT = 0xFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x107UL;  // CRC poly: x^8 + x^2^x^1 + 1
    }
}

void BLE_send_packet(void){
  uint16_t count = 0;
  NRF_RADIO->EVENTS_READY = 0U;
  NRF_RADIO->TASKS_TXEN   = 1;
  while (NRF_RADIO->EVENTS_READY != 0x01 || count<COUNT_DELAY){}; 
  if (NRF_RADIO->EVENTS_READY == 0 && count>=COUNT_DELAY){
    error_signal();
  }

  NRF_RADIO->EVENTS_END  = (uint32_t) 0x00;
  NRF_RADIO->TASKS_START = (uint32_t) 0x01;
  while (NRF_RADIO->EVENTS_END != 0x01 || count<COUNT_DELAY){}; 
  if (NRF_RADIO->EVENTS_READY == 0 && count>=COUNT_DELAY){
    error_signal();
  }

  NRF_RADIO->EVENTS_DISABLED = (uint32_t) 0x00;
  NRF_RADIO->TASKS_DISABLE = (uint32_t) 0x01;

  while (NRF_RADIO->EVENTS_DISABLED != 0x01 || count<COUNT_DELAY){}; 
  if (NRF_RADIO->EVENTS_DISABLED == 0 && count>=COUNT_DELAY){
    error_signal();
  }
}


static uint32_t swap_bits(uint32_t inp)
{
    uint32_t i;
    uint32_t retval = 0;

    inp = (inp & 0x000000FFUL);

    for (i = 0; i < 8; i++)
    {
        retval |= ((inp >> i) & 0x01) << (7 - i);
    }

    return retval;
}


static uint32_t bytewise_bitswap(uint32_t inp)
{
      return (swap_bits(inp >> 24) << 24)
           | (swap_bits(inp >> 16) << 16)
           | (swap_bits(inp >> 8) << 8)
           | (swap_bits(inp));
}

void initialization_gpio(void){
  NRF_P0->PIN_CNF[OUTPUT_PIN] |= (uint32_t) 0x04; //In, pull down
  NRF_P0->PIN_CNF[SDN_PIN] |= (uint32_t) 0x03; //Output
  NRF_P0->PIN_CNF[LOD_PIN] |= (uint32_t) 0x04; //In, pull down
  NRF_P0->PIN_CNF[CONTROL_PIN] |= (uint32_t) 0x03; //Output
}


void control_AD8233(uint8_t state){
  switch (state){
    case 0: {
      NRF_P0->OUT |= (uint32_t) 0x00; //Отключить AD8233
      break;
    }
    case 1: {
      NRF_P0->OUT |= (uint32_t) 0x01; //Включить AD8233
      break;
    }
  default:{
    break;
  }
  }
}

//Проверка подключение электродов
uint8_t check_connect(void){
  uint8_t signal = NRF_P0->IN;
  if (signal > 0){
    error_signal();
  } 
  return 0;
}

//Настройка АЦП
void initization_adc(void){
  NRF_SAADC->CH[1].PSELP |= (uint32_t) 0x02; //AIN 1
  NRF_SAADC->CH[1].CONFIG |= (uint32_t) 0x95; //Pulldown, bypass,gain1, internal, 10us, SE, Disable
  NRF_SAADC->ENABLE |= (uint32_t) 0x01;
}

//Считывание сигнала с 
void give_signal(void){
  uint16_t signal = 0;
  for (uint16_t count = 0; count <100; count++){
    signal += NRF_SAADC->RESULT.PTR;
  }
}

//Обработка ошибок
void error_signal(void){
    //Включить светодиод или другое сообщение об ошибке
  while(1){
    
  }
}

