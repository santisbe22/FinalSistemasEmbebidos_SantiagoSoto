/* ====== LIBRERIAS ====== */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
static const char *TAG = "LAB4";

/* ====== DEFINES ====== */
#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_SDA_IO   21
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  50000

#define DS1307_ADDR 0x68
#define LCD_ADDR    0x27

#define LCD_RS          (1 << 0)
#define LCD_EN          (1 << 2)
#define LCD_BACKLIGHT   (1 << 3)

#define LCD_CMD_CLEAR       0x01
#define LCD_CMD_ENTRY_MODE  0x06
#define LCD_CMD_DISPLAY_ON  0x0C
#define LCD_CMD_FUNCTION_SET 0x28
#define LCD_CMD_LINE1       0x80
#define LCD_CMD_LINE2       0xC0

// ✅ FIX 1: 2004A tiene 20 columnas, no 16
#define LCD_COLS 20

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS    5
#define PIN_NUM_RST   4

#define LED_ROJO   2
#define LED_VERDE 12
#define LED_AZUL  13
#define BUZZER    14

#define REG_COMMAND     0x01
#define REG_COM_IEN     0x02
#define REG_COM_IRQ     0x04
#define REG_ERROR       0x06
#define REG_FIFO_DATA   0x09
#define REG_FIFO_LEVEL  0x0A
#define REG_BIT_FRAMING 0x0D
#define REG_MODE        0x11
#define REG_TX_CONTROL  0x14
#define REG_TX_ASK      0x15
#define REG_T_MODE      0x2A
#define REG_T_PRESCALER 0x2B
#define REG_T_RELOAD_H  0x2C
#define REG_T_RELOAD_L  0x2D
#define REG_VERSION     0x37

#define CMD_IDLE        0x00
#define CMD_TRANSCEIVE  0x0C
#define CMD_SOFT_RESET  0x0F

#define PICC_REQA     0x26
#define PICC_ANTICOLL 0x93

/* ====== ESTADOS ====== */
typedef enum {
    ESTADO_BLOQUEADO,
    ESTADO_VALIDANDO,
    ESTADO_ACTIVO
} estado_t;

static estado_t estado_actual   = ESTADO_BLOQUEADO;
static int      estado_anterior = -1;

/* ====== VARIABLES ====== */
static uint8_t h, m, s;

// ✅ FIX 1: buffer de linea ahora es LCD_COLS+1
static char linea[LCD_COLS + 1];
static spi_device_handle_t spi_handle;

// BLE: mensaje recibido
// ✅ FIX 1: buffer de mensaje también es LCD_COLS+1
static char             ble_last_msg[LCD_COLS + 1] = "Sin mensajes";
static volatile bool    ble_has_msg = false;
static portMUX_TYPE     ble_mux = portMUX_INITIALIZER_UNLOCKED;

/* ====== BLE NUS ====== */
static const ble_uuid128_t nus_svc_uuid =
    BLE_UUID128_INIT(0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,
                     0x93,0xF3,0xA3,0xB5,0x01,0x00,0x40,0x6E);

static const ble_uuid128_t nus_rx_uuid =
    BLE_UUID128_INIT(0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,
                     0x93,0xF3,0xA3,0xB5,0x02,0x00,0x40,0x6E);

static int nus_rx_access(uint16_t conn, uint16_t attr,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (estado_actual != ESTADO_ACTIVO) {
        ESP_LOGW("BLE", "Rechazado: no estoy en ACTIVO");
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    int len = ctxt->om->om_len;
    if (len <= 0) return 0;

    char tmp[LCD_COLS + 1];
    // ✅ FIX 1: máximo LCD_COLS caracteres (20)
    int copy_len = (len > LCD_COLS) ? LCD_COLS : len;
    memcpy(tmp, ctxt->om->om_data, copy_len);
    tmp[copy_len] = '\0';

    portENTER_CRITICAL(&ble_mux);
    memset(ble_last_msg, ' ', LCD_COLS);
    ble_last_msg[LCD_COLS] = '\0';
    memcpy(ble_last_msg, tmp, strlen(tmp));
    ble_has_msg = true;
    portEXIT_CRITICAL(&ble_mux);

    ESP_LOGI("BLE", "RX: %s", tmp);
    return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid      = &nus_rx_uuid.u,
                .access_cb = nus_rx_access,
                .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {0}
        }
    },
    {0}
};

static void ble_advertise(void)
{
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields fields      = {0};

    const char *name  = "PanelHMI";
    fields.name       = (uint8_t *)name;
    fields.name_len   = strlen(name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      &adv_params, NULL, NULL);
}

static void ble_on_sync(void)
{
    ESP_LOGI(TAG, "BLE sync OK");
    ble_svc_gap_device_name_set("PanelHMI");
    ble_advertise();
    ESP_LOGI(TAG, "Advertising como PanelHMI");
}

static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE host task iniciada");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init FALLO: %d", ret);
        return;
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_hs_cfg.sync_cb = ble_on_sync;

    nimble_port_freertos_init(ble_host_task);
}

/* ====== RTC ====== */
static uint8_t bcd_to_dec(uint8_t val) {
    return (val >> 4) * 10 + (val & 0x0F);
}

static uint8_t dec_to_bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

static void rtc_set_time(uint8_t hour, uint8_t min, uint8_t sec) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS1307_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_write_byte(cmd, dec_to_bcd(sec) & 0x7F, true);
    i2c_master_write_byte(cmd, dec_to_bcd(min), true);
    i2c_master_write_byte(cmd, dec_to_bcd(hour), true);
    i2c_master_write_byte(cmd, 0x01, true);
    i2c_master_write_byte(cmd, 0x01, true);
    i2c_master_write_byte(cmd, 0x01, true);
    i2c_master_write_byte(cmd, 0x23, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

static void rtc_get_time(uint8_t *hour, uint8_t *min, uint8_t *sec) {
    uint8_t data[3];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS1307_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS1307_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[2], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    *sec  = bcd_to_dec(data[0] & 0x7F);
    *min  = bcd_to_dec(data[1]);
    *hour = bcd_to_dec(data[2]);
}

/* ====== LCD ====== */
static void lcd_write(uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD I2C FALLO: %s", esp_err_to_name(ret));
    }
}

static void lcd_pulse(uint8_t data) {
    lcd_write(data | LCD_EN | LCD_BACKLIGHT);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write((data & ~LCD_EN) | LCD_BACKLIGHT);
}

static void lcd_send_nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = (nibble & 0xF0) | mode | LCD_BACKLIGHT;
    lcd_pulse(data);
}

static void lcd_cmd(uint8_t cmd) {
    lcd_send_nibble(cmd & 0xF0, 0);
    lcd_send_nibble((cmd << 4) & 0xF0, 0);
    if (cmd == LCD_CMD_CLEAR) vTaskDelay(pdMS_TO_TICKS(2));
}

static void lcd_data(uint8_t data) {
    lcd_send_nibble(data & 0xF0, LCD_RS);
    lcd_send_nibble((data << 4) & 0xF0, LCD_RS);
}

static void lcd_init(void) {
    vTaskDelay(pdMS_TO_TICKS(200));   // más tiempo de arranque

    // secuencia de reset en 4 bits
    lcd_send_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_send_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_send_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_send_nibble(0x20, 0);         // pasar a modo 4 bits
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_cmd(LCD_CMD_FUNCTION_SET);    // 0x28: 4 bits, 2 líneas
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_cmd(LCD_CMD_DISPLAY_ON);      // 0x0C
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_cmd(LCD_CMD_CLEAR);           // 0x01
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_cmd(LCD_CMD_ENTRY_MODE);      // 0x06
    vTaskDelay(pdMS_TO_TICKS(5));
}

static void lcd_set_cursor(uint8_t row, uint8_t col) {
    lcd_cmd((row == 0 ? LCD_CMD_LINE1 : LCD_CMD_LINE2) + col);
}

// ✅ FIX 1: buffer y loop ahora usan LCD_COLS (20)
static void lcd_print_line(uint8_t row, const char *text) {
    char buffer[LCD_COLS + 1];
    memset(buffer, ' ', LCD_COLS);
    buffer[LCD_COLS] = '\0';
    int len = strlen(text);
    if (len > LCD_COLS) len = LCD_COLS;
    memcpy(buffer, text, len);
    lcd_set_cursor(row, 0);
    vTaskDelay(pdMS_TO_TICKS(1));   // ← pequeño delay antes de escribir
    for (int i = 0; i < LCD_COLS; i++)
        lcd_data(buffer[i]);
}
/* ====== I2C ====== */
static void i2c_init(void) {
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .sda_pullup_en    = 1,
        .scl_pullup_en    = 1,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    // ← SCAN TEMPORAL
    ESP_LOGI(TAG, "Escaneando bus I2C...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Dispositivo encontrado en 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "Scan terminado");
}

/* ====== LEDs + Buzzer ====== */
static void gpio_init_outputs(void) {
    gpio_set_direction(LED_ROJO,  GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_VERDE, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_AZUL,  GPIO_MODE_OUTPUT);
    gpio_set_direction(BUZZER,    GPIO_MODE_OUTPUT);
    gpio_set_level(LED_ROJO,  0);
    gpio_set_level(LED_VERDE, 0);
    gpio_set_level(LED_AZUL,  0);
    gpio_set_level(BUZZER,    0);
}

static void leds_apagar_todos(void) {
    gpio_set_level(LED_ROJO,  0);
    gpio_set_level(LED_VERDE, 0);
    gpio_set_level(LED_AZUL,  0);
}

static void buzzer_corto(void) {
    gpio_set_level(BUZZER, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(BUZZER, 0);
}

static void buzzer_largo(void) {
    gpio_set_level(BUZZER, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    gpio_set_level(BUZZER, 0);
}

static void led_rojo_parpadeo(void) {
    for (int i = 0; i < 3; i++) {
        gpio_set_level(LED_ROJO, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED_ROJO, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ====== MFRC522 ====== */
static void mfrc522_write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { (reg << 1) & 0x7E, val };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    spi_device_transmit(spi_handle, &t);
}

static uint8_t mfrc522_read_reg(uint8_t reg) {
    uint8_t tx[2] = { ((reg << 1) & 0x7E) | 0x80, 0x00 };
    uint8_t rx[2] = {0};
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_transmit(spi_handle, &t);
    return rx[1];
}

static void mfrc522_set_bits(uint8_t reg, uint8_t mask) {
    mfrc522_write_reg(reg, mfrc522_read_reg(reg) | mask);
}

static void mfrc522_clear_bits(uint8_t reg, uint8_t mask) {
    mfrc522_write_reg(reg, mfrc522_read_reg(reg) & ~mask);
}

static void mfrc522_init(void) {
    spi_bus_config_t bus = {
        .miso_io_num   = PIN_NUM_MISO,
        .mosi_io_num   = PIN_NUM_MOSI,
        .sclk_io_num   = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 1000000,
        .mode           = 0,
        .spics_io_num   = PIN_NUM_CS,
        .queue_size     = 5
    };
    spi_bus_add_device(SPI2_HOST, &dev, &spi_handle);

    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    mfrc522_write_reg(REG_COMMAND, CMD_SOFT_RESET);
    vTaskDelay(pdMS_TO_TICKS(50));

    mfrc522_write_reg(REG_T_MODE,      0x80);
    mfrc522_write_reg(REG_T_PRESCALER, 0xA9);
    mfrc522_write_reg(REG_T_RELOAD_H,  0x03);
    mfrc522_write_reg(REG_T_RELOAD_L,  0xE8);
    mfrc522_write_reg(REG_TX_ASK,      0x40);
    mfrc522_write_reg(REG_MODE,        0x3D);
    mfrc522_set_bits(REG_TX_CONTROL,   0x03);

    uint8_t ver = mfrc522_read_reg(REG_VERSION);
    ESP_LOGI(TAG, "MFRC522 version: 0x%02X", ver);
}

static int mfrc522_transceive(uint8_t *send, uint8_t send_len,
                               uint8_t *recv, uint8_t *recv_len) {
    mfrc522_clear_bits(REG_COM_IRQ,    0x80);
    mfrc522_set_bits(REG_FIFO_LEVEL,   0x80);
    mfrc522_write_reg(REG_COMMAND,     CMD_IDLE);

    for (int i = 0; i < send_len; i++)
        mfrc522_write_reg(REG_FIFO_DATA, send[i]);

    mfrc522_write_reg(REG_COMMAND, CMD_TRANSCEIVE);
    mfrc522_set_bits(REG_BIT_FRAMING, 0x80);

    int timeout = 2000;
    uint8_t irq;
    do {
        irq = mfrc522_read_reg(REG_COM_IRQ);
        timeout--;
    } while (timeout > 0 && !(irq & 0x30) && !(irq & 0x01));

    mfrc522_clear_bits(REG_BIT_FRAMING, 0x80);

    if (timeout <= 0 || (irq & 0x01)) return -1;
    uint8_t err = mfrc522_read_reg(REG_ERROR);
    if (err & 0x1B) return -2;

    if (recv && recv_len) {
        uint8_t n = mfrc522_read_reg(REG_FIFO_LEVEL);
        if (n > *recv_len) n = *recv_len;
        *recv_len = n;
        for (int i = 0; i < n; i++)
            recv[i] = mfrc522_read_reg(REG_FIFO_DATA);
    }
    return 0;
}

static bool mfrc522_request(void) {
    uint8_t send = PICC_REQA;
    uint8_t recv[2];
    uint8_t recv_len = 2;
    mfrc522_write_reg(REG_BIT_FRAMING, 0x07);
    int ret = mfrc522_transceive(&send, 1, recv, &recv_len);
    return (ret == 0 && recv_len == 2);
}

static bool mfrc522_anticoll(uint8_t *uid, uint8_t *uid_len) {
    uint8_t send[2]  = { PICC_ANTICOLL, 0x20 };
    uint8_t recv[5];
    uint8_t recv_len = 5;
    mfrc522_write_reg(REG_BIT_FRAMING, 0x00);
    int ret = mfrc522_transceive(send, 2, recv, &recv_len);

    if (ret == 0 && recv_len == 5) {
        uint8_t bcc = 0;
        for (int i = 0; i < 4; i++) bcc ^= recv[i];
        if (bcc == recv[4]) {
            memcpy(uid, recv, 4);
            *uid_len = 4;
            return true;
        }
    }
    return false;
}

static bool rfid_leer_uid(uint8_t *uid, uint8_t *uid_len) {
    if (!mfrc522_request()) return false;
    return mfrc522_anticoll(uid, uid_len);
}

/* ====== AUTORIZACION ====== */
static const uint8_t UID_REAL[] = {0x57, 0x29, 0x2C, 0x17};

static bool uid_es_autorizado(const uint8_t *uid, uint8_t len) {
    if (len == 4 && memcmp(uid, UID_REAL, 4) == 0) return true;
    return false;
}

/* ====== MAIN ====== */
void app_main(void) {
    i2c_init();
    lcd_init();
    mfrc522_init();
    gpio_init_outputs();
    ble_init();

    rtc_set_time(12, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(300));

    uint8_t uid[10];
    uint8_t uid_len = 0;

    while (1) {
        // ── Cambio de estado: pintar LCD una sola vez ──────────
        if ((int)estado_actual != estado_anterior) {
            estado_anterior = (int)estado_actual;

            switch (estado_actual) {

          case ESTADO_BLOQUEADO:
    leds_apagar_todos();
    gpio_set_level(LED_ROJO, 1);
    lcd_cmd(LCD_CMD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_print_line(0, "Panel bloqueado");
    lcd_print_line(1, "Acerque credencial");
    ESP_LOGI(TAG, "Estado: BLOQUEADO");
    break;

            case ESTADO_VALIDANDO:
                break;

           case ESTADO_ACTIVO:
             leds_apagar_todos();
              gpio_set_level(LED_AZUL, 1);
             lcd_cmd(LCD_CMD_CLEAR);          // ← agrega esta línea
             vTaskDelay(pdMS_TO_TICKS(2));    // ← y esta
    {
        char msg[LCD_COLS + 1];
        portENTER_CRITICAL(&ble_mux);
        memcpy(msg, ble_last_msg, LCD_COLS + 1);
        portEXIT_CRITICAL(&ble_mux);
        lcd_print_line(0, msg);
    }
    rtc_get_time(&h, &m, &s);
    snprintf(linea, sizeof(linea), "%02d:%02d:%02d", h, m, s);
    lcd_print_line(1, linea);
    ESP_LOGI(TAG, "Estado: ACTIVO");
    break;
            }
        }

        // ── Lógica de cada estado ──────────────────────────────
        switch (estado_actual) {

         case ESTADO_BLOQUEADO:
              if (rfid_leer_uid(uid, &uid_len)) {
         ESP_LOGI(TAG, "UID leido: %02X %02X %02X %02X",
                 uid[0], uid[1], uid[2], uid[3]);
             estado_actual = ESTADO_VALIDANDO;
    }
            vTaskDelay(pdMS_TO_TICKS(200));
            break;

        case ESTADO_VALIDANDO:
            if (uid_es_autorizado(uid, uid_len)) {
                leds_apagar_todos();
                gpio_set_level(LED_VERDE, 1);
                lcd_print_line(0, "Acceso concedido");

                rtc_get_time(&h, &m, &s);
                snprintf(linea, sizeof(linea), "%02d:%02d:%02d", h, m, s);
                lcd_print_line(1, linea);

                buzzer_corto();
                ESP_LOGI(TAG, "Acceso Concedido - %s", linea);
                vTaskDelay(pdMS_TO_TICKS(100));

                portENTER_CRITICAL(&ble_mux);
                strcpy(ble_last_msg, "Sin mensajes");
                ble_has_msg = false;
                portEXIT_CRITICAL(&ble_mux);

                estado_actual = ESTADO_ACTIVO;

            } else {
                lcd_print_line(0, "Acceso denegado");
                // ✅ FIX 2: string completo, cabe en 20 cols
                lcd_print_line(1, "UID no registrado");
                ESP_LOGI(TAG, "Acceso DENEGADO");
                led_rojo_parpadeo();
                buzzer_largo();
                estado_actual = ESTADO_BLOQUEADO;
            }
            break;

        case ESTADO_ACTIVO:
            // Actualizar LCD cada 1 segundo
            {
                char msg[LCD_COLS + 1];
                portENTER_CRITICAL(&ble_mux);
                memcpy(msg, ble_last_msg, LCD_COLS + 1);
                portEXIT_CRITICAL(&ble_mux);
                lcd_print_line(0, msg);
            }

            rtc_get_time(&h, &m, &s);
            snprintf(linea, sizeof(linea), "%02d:%02d:%02d", h, m, s);
            lcd_print_line(1, linea);

            // Cerrar sesión con RFID
            if (rfid_leer_uid(uid, &uid_len)) {
                if (uid_es_autorizado(uid, uid_len)) {
                    ESP_LOGI(TAG, "Cerrando sesion");
                    leds_apagar_todos();
                    buzzer_corto();

                    portENTER_CRITICAL(&ble_mux);
                    strcpy(ble_last_msg, "Sin mensajes");
                    ble_has_msg = false;
                    portEXIT_CRITICAL(&ble_mux);

                    estado_actual = ESTADO_BLOQUEADO;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            break;
        }
    }
}   