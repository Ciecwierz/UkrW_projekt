#include <stdio.h>

#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/spi.h"
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

#define STATUS_REGISTER 0xF3
#define PRESS_MSB 0xF7

void ukrw_uart0_init(void);
uint8_t get_bmp_id(void);
void bmp_spi_setup(void);
static inline void myPrint(const uint8_t* ptr);
static inline void myPrintValue(uint8_t value);
void pico_set_led(bool led_on);
int pico_led_init(void);
void board_test(void);

typedef struct 
{
    
    uint8_t temp_compensation[6];
    uint8_t press_compensation[18];
    int32_t temperature_raw;
    int32_t pressure_raw;

    union
    {
        uint8_t all;
        struct
        {
            uint8_t t_sb : 3;
            uint8_t filter : 3;
            uint8_t reserved : 1;
            uint8_t spi3w_en : 1;
        }bit;
    }config;

    union 
    {
        uint8_t all;
        struct
        {
            uint8_t osrs_t : 3;
            uint8_t osrs_p : 3;
            uint8_t mode : 2;
            
        }bit;
    }ctrl_meas;

}bmp_data_t;

void weather_measurement_setup(bmp_data_t* const bmp);
void weather_measurement(bmp_data_t* const bmp);
void presentData(const bmp_data_t* const bmp, char* output_buffer, const size_t buffer_size);
void read_compensation_values(bmp_data_t* const bmp);
int32_t bmp280_compensate_temperature(const bmp_data_t* const bmp);
uint32_t bmp280_compensate_pressure(const bmp_data_t* const bmp);















char buffer[128];
int32_t t_fine;

int main(void)
{
    stdio_init_all();
    
    bmp_data_t my_bmp;
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    bmp_spi_setup();
    ukrw_uart0_init();

   

    weather_measurement_setup(&my_bmp); 

    uart_puts(UART_ID, "Testing UART\n");
    uart_putc(UART_ID, 'A');
    uart_putc(UART_ID, '\n');
    myPrint((const uint8_t*)"Hello, UART!");

    read_compensation_values(&my_bmp);
   
  
   
    while(true)
    {
        weather_measurement(&my_bmp);
        sleep_ms(20*1000);
    }





/*
    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }
*/

/*
    // Get a free channel, panic() if there are none
    int chan = dma_claim_unused_channel(true);
    
    // 8 bit transfers. Both read and write address increment after each
    // transfer (each pointing to a location in src or dst respectively).
    // No DREQ is selected, so the DMA transfers as fast as it can.
    
    dma_channel_config c = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    
    dma_channel_configure(
        chan,          // Channel to be configured
        &c,            // The configuration we just created
        dst,           // The initial write address
        src,           // The initial read address
        count_of(src), // Number of transfers; in this case each is 1 byte.
        true           // Start immediately.
    );
    
    // We could choose to go and do something else whilst the DMA is doing its
    // thing. In this case the processor has nothing else to do, so we just
    // wait for the DMA to finish.
    dma_channel_wait_for_finish_blocking(chan);
    
    // The DMA has now copied our text from the transmit buffer (src) to the
    // receive buffer (dst), so we can print it out from there.
    puts(dst);

    // Example to turn on the Pico W LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
    */
}



// Perform initialisation
int pico_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // For Pico W devices we need to initialise the driver etc
    return cyw43_arch_init();
#endif
}

// Turn the led on or off
void pico_set_led(bool led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}
static inline void myPrint(const uint8_t* ptr)
{
    uart_puts(UART_ID,ptr);
    uart_putc(UART_ID,'\n');
   // uart_puts(UART_ID,"Send to terminal...\n");
}

static inline void  myPrintValue(uint8_t value)
{
    char buffer[10];
    sprintf(buffer, "%u", value);
    myPrint(buffer);
}

void bmp_spi_setup()
{
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);  
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
}

  uint8_t get_bmp_id()
{
    const uint8_t readID = 0xd0 | 0x80;
    uint8_t data = 0;


    gpio_put(PIN_CS,0);
    spi_write_blocking(SPI_PORT, &readID,1);
    spi_read_blocking(SPI_PORT,0x00,&data, 1);
    gpio_put(PIN_CS,1);
    return data;

}


void ukrw_uart0_init()
{
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);
    myPrint("START UART");
    
}


void board_test(void)
{
    char buffer[25];

    myPrint("Running...");
    uint8_t id = get_bmp_id();
    snprintf(buffer, sizeof(buffer), "%02X", id);
    sleep_ms(1500);
    myPrint(buffer);


    pico_set_led(true);
    sleep_ms(250);
    pico_set_led(false);
    sleep_ms(150);
}


void weather_measurement_setup(bmp_data_t*const bmp)
{
    bmp->ctrl_meas.bit.osrs_t= 0b001;
    bmp->ctrl_meas.bit.osrs_p = 0b001;
    const uint16_t data = (0xF4 << 8) | (bmp->ctrl_meas.all);
    gpio_put(PIN_CS,0);
    spi_write_blocking(SPI_PORT,(const uint8_t*)&data,2);
    gpio_put(PIN_CS,1);
    myPrint("Setup done.");
  
}
void read_compensation_values(bmp_data_t* const bmp)
{
      sleep_ms(1);

    gpio_put(PIN_CS,0);
    spi_write_blocking(SPI_PORT, (const uint8_t[]){0x88 | 0x80},1);
    spi_read_blocking(SPI_PORT,0x00,bmp->temp_compensation, 6);
    gpio_put(PIN_CS,1);

    sleep_ms(1);

    gpio_put(PIN_CS,0);
    spi_write_blocking(SPI_PORT, (const uint8_t[]){0x8E | 0x80},1);
    spi_read_blocking(SPI_PORT,0x00,bmp->press_compensation, 18);
    gpio_put(PIN_CS,1);

    sleep_ms(1);
    myPrint("Compensation values read.");
    myPrintValue(3);
    for(int i = 0; i < 6; i++)
        myPrintValue(bmp->temp_compensation[i]);
    for(int j = 0; j < 18; j++)
        myPrintValue(bmp->press_compensation[j]);


}
 void weather_measurement(bmp_data_t* const bmp)
{
    uint8_t status =255;
    bmp->ctrl_meas.bit.mode = 0b01;
    const uint8_t start[2] ={ 0xF4  , (bmp->ctrl_meas.all)};

    gpio_put(PIN_CS,0);
    spi_write_blocking(SPI_PORT,start,2);
    gpio_put(PIN_CS,1);

    sleep_ms(6);

    gpio_put(PIN_CS,0);
    spi_write_blocking(SPI_PORT, (const uint8_t[]){STATUS_REGISTER | 0x80},1);
    spi_read_blocking(SPI_PORT,0x00,&status, 1);
    gpio_put(PIN_CS,1);

   

    if((status & 0x80) == 0)
    {
        uint8_t press_temp[6] = {0,0,0,0,0,0};
        
        gpio_put(PIN_CS,0);
        spi_write_blocking(SPI_PORT,(const uint8_t[]){PRESS_MSB | 0x80},1);
        spi_read_blocking(SPI_PORT, 0x00, press_temp,6);
        gpio_put(PIN_CS,1);

       //tutaj i w dół
       
         bmp->temperature_raw = (int32_t)((uint32_t)press_temp[0] << 12) |
                                ((uint32_t)press_temp[1] <<4) |
                                (press_temp[2] >> 4);
       
         bmp->pressure_raw = (int32_t)((uint32_t)press_temp[3] << 12) |
                                ((uint32_t)press_temp[4] <<4) |
                                (press_temp[5] >> 4);
       
      
        presentData(bmp, buffer, sizeof(buffer));

    }
    else
    {
        myPrint("Measuring...?");
    }
}
int32_t bmp280_compensate_temperature(const bmp_data_t* const bmp)
{
   
    int32_t T;
    int32_t dig1 = (int32_t)(bmp->temp_compensation[0] | bmp->temp_compensation[1]);
    int32_t dig2 = (int32_t)(bmp->temp_compensation[2] | bmp->temp_compensation[3]);
    int32_t dig3 = (int32_t)(bmp->temp_compensation[4] | bmp->temp_compensation[5]);
    
    int32_t var1 = ((((bmp->temperature_raw >> 3) - (dig1<<1))) * (dig2)) >> 11;
    int32_t var2 = (((((bmp->temperature_raw>>4) - (dig1)) * ((bmp->temperature_raw>>4) - (dig1))) >> 12) *
                                                            (dig3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;

}
uint32_t bmp280_compensate_pressure(const bmp_data_t* const bmp)
{
    int32_t dig1, dig2, dig3, dig4, dig5, dig6 ,dig7, dig8, dig9;
    int64_t  p, var1, var2;

    dig1 = (int32_t)((bmp->press_compensation[0] << 8) | bmp->press_compensation[1]);
    dig2 = (int32_t)((bmp->press_compensation[2] << 8) | bmp->press_compensation[3]);
    dig3 = (int32_t)((bmp->press_compensation[4] << 8) | bmp->press_compensation[5]);
    dig4 = (int32_t)((bmp->press_compensation[6] << 8)| bmp->press_compensation[7]);
    dig5 = (int32_t)((bmp->press_compensation[8] << 8)| bmp->press_compensation[9]);
    dig6 = (int32_t)((bmp->press_compensation[10] << 8)| bmp->press_compensation[11]);
    dig7 = (int32_t)((bmp->press_compensation[12] << 8) | bmp->press_compensation[13]);
    dig8 = (int32_t)((bmp->press_compensation[14] << 8) | bmp->press_compensation[15]);
    dig9 = (int32_t)((bmp->press_compensation[16] << 8) | bmp->press_compensation[17]);
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig6;
    var2 = var2 + ((var1*(int64_t)dig5)<<17);
    var2 = var2 + (((int64_t)dig4)<<35);
    var1 = ((var1 * var1 * (int64_t)dig3)>>8) + ((var1 * (int64_t)dig2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig1)>>33;
    if (var1 == 0)
    {
            return 0; // avoid exception caused by division by zero
    }
    p = 1048576-bmp->pressure_raw;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)dig9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)dig8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig7)<<4);
    return (uint32_t)p;

}
void presentData( const bmp_data_t* const bmp, char* output_buffer, const size_t buffer_size)
{
    
    int32_t temperature = bmp280_compensate_temperature(bmp);
    uint32_t pressure = bmp280_compensate_pressure(bmp);
    
   
    
    memset(output_buffer, 0, buffer_size);
    snprintf(output_buffer, buffer_size, "Temp: %ld.%02ld °C, Pressure: %ld.%02ld hPa\n", 
        (int)(temperature/ 100), abs((int)(temperature % 100)),
        (int)(pressure /256 /100), abs((int)(pressure / 256) % 100));
    myPrint(output_buffer);
    
} 





// measurement/oversampling enabled thru osrs_p[2:0]  in 0xF4 register
// pressure stored in 0xF9 register
// temperature stored in 0xFC register
//iir filter config  in filter[2:0] bits of 0xF5 register

//weather monitoring settings
// ultra-low power oversampling setting
// x1 osrs_p & osrs_t
//IIR coeff off
//ODR 1/60 Hz
//probably best to use forced mode

//sleep mode for bmp280  in mode[1:0] of oxF4 register