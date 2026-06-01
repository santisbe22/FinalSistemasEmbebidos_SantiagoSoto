#include "driver/spi_master.h"
#include "esp_err.h"

#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_CLK  18


// PUNTO 3A
esp_err_t spi_bus_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,   
        .quadhd_io_num = -1,   
        .max_transfer_sz = 2   
    };

    
    return spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
}

// PUNTO 3B
#include <stdint.h>
#include "driver/spi_master.h"
#include "esp_err.h"
esp_err_t mcp4132_write_register(spi_device_handle_t spi,
                                 uint8_t address,
                                 uint8_t value)
{
   
    uint8_t tx_data[2];

   
    tx_data[0] = (address << 4) | (0x00 << 2);

   
    tx_data[1] = value & 0x7F;

    spi_transaction_t t = {
        .length = 16,          
        .tx_buffer = tx_data,
        .rx_buffer = NULL      
    };

    return spi_device_transmit(spi, &t);
}

//PUNTO 3C
#include <stdint.h>
#include "driver/spi_master.h"
#include "esp_err.h"
uint16_t mcp4132_read_register(uint8_t address)
{
    uint8_t tx_data[2];
    uint8_t rx_data[2];
 
    tx_data[0] = mcp4132_build_command(address, MCP_CMD_READ, 0);
    tx_data[1] = 0x00;  
 
    memset(rx_data, 0, sizeof(rx_data));
 
    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));
 
    transaction.length = 16;
    transaction.tx_buffer = tx_data;
    transaction.rx_buffer = rx_data;
 
    esp_err_t ret = spi_device_transmit(mcp4132_handle, &transaction);
 
    if (ret != ESP_OK) {
        return 0xFFFF;
    }
 
    uint16_t value = ((rx_data[0] & 0x01) << 8) | rx_data[1];
 
    return value;
}


//PUNTO 4A

#include "esp_err.h"
#include <stdint.h>

#define MCP4132_WIPER0_ADDR 0x00

esp_err_t mcp4132_set_wiper(spi_device_handle_t spi, uint8_t n)
{
    // Validación de rango (0 - 128)
    if (n > 128)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Ajuste para 7 bits (0 - 127)
    uint8_t value;

    if (n == 128)
    {
        value = 127;  
    }
    else
    {
        value = n;
    }

    //funcion escritura
    return mcp4132_write_register(spi, MCP4132_WIPER0_ADDR, value);
}

//PUNTO 4B

#include <math.h>
#include "esp_err.h"

#define MCP4132_RAB   10000.0f   // 10kΩ
#define MCP4132_RW    75.0f      // resistencia wiper
#define MCP4132_STEPS 128.0f

#define FILTER_C      10e-6f      // 10 µF 

esp_err_t mcp4132_set_cutoff_frequency(spi_device_handle_t spi, float fc)
{
    
    if (fc <= 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Calcular R necesaria
    float Rwb = 1.0f / (2.0f * M_PI * fc * FILTER_C);

    // Calcular n
    float n_float = ((Rwb - MCP4132_RW) * MCP4132_STEPS) / MCP4132_RAB;

    // Redondeo al entero más cercano
    int n = (int)(n_float + 0.5f);

    //  Saturación al rango permitido
    if (n < 0)
        n = 0;
    if (n > 128)
        n = 128;

    //  Usar función anterior
    return mcp4132_set_wiper(spi, (uint8_t)n);
}
