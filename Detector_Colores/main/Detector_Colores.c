#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "esp_log.h"


// LEDs de iluminación tricromática
#define LED_R_GPIO       GPIO_NUM_25    // Rojo
#define LED_G_GPIO       GPIO_NUM_26   // Verde
#define LED_B_GPIO       GPIO_NUM_27   // Azul
#define LED_STATUS_GPIO  GPIO_NUM_19   // Estado

// Botones
#define BTN1_GPIO        GPIO_NUM_32   // Modo siguiente
#define BTN2_GPIO        GPIO_NUM_33   // Modo anterior
#define BTN3_GPIO        GPIO_NUM_13   // MEDIR
#define BTN4_GPIO        GPIO_NUM_12   // HOME / Reset

// ADC - LDR (fotoresistencia)
#define ADC_CHANNEL      ADC1_CHANNEL_6   // GPIO34
#define ADC_ATTEN        ADC_ATTEN_DB_11  
#define ADC_WIDTH_BITS   ADC_WIDTH_BIT_12 
#define ADC_SAMPLES      32               // promedio de muestras

// OLED I2C
#define OLED_SDA         GPIO_NUM_21
#define OLED_SCL         GPIO_NUM_22
#define OLED_PORT        I2C_NUM_0
#define OLED_SPEED       100000
#define OLED_ADDR        0x3C

// Antirrebote botones
#define DEBOUNCE_MS      120


//  TIPOS Y VARIABLES GLOBALES

typedef enum {
    EV_BTN1 = 1,
    EV_BTN2,
    EV_BTN3,
    EV_BTN4
} event_t;

typedef enum {
    COLOR_ROJO,
    COLOR_VERDE,
    COLOR_AZUL,
    COLOR_AMARILLO,
    COLOR_CIAN,
    COLOR_MAGENTA,
    COLOR_BLANCO,
    COLOR_NEGRO,
    COLOR_GRIS,
    COLOR_DESCONOCIDO
} color_t;

typedef enum {
    MODO_MEDICION = 0,
    MODO_PORCENTAJES,
    MODO_RESULTADO,
    MODO_MAX
} modo_t;

static QueueHandle_t event_queue;
static volatile TickType_t last_tick_btn1 = 0;
static volatile TickType_t last_tick_btn2 = 0;
static volatile TickType_t last_tick_btn3 = 0;
static volatile TickType_t last_tick_btn4 = 0;

static modo_t   modo_actual     = MODO_MEDICION;
static uint32_t val_rojo        = 0;
static uint32_t val_verde       = 0;
static uint32_t val_azul        = 0;
static float    porc_rojo       = 0;
static float    porc_verde      = 0;
static float    porc_azul       = 0;
static color_t  color_detectado = COLOR_DESCONOCIDO;
static int      medicion_lista  = 0;

static const char *TAG = "COLOR_SENSOR";

// Valores de calibracion
static uint32_t fondo_rojo = 0;
static uint32_t fondo_verde = 0;
static uint32_t fondo_azul = 0;
static int calibrado = 0;

typedef struct {
    const char *nombre;
    float r, g, b;
} color_ref_t;

static color_ref_t colores[] = {
    {"ROJO",100,0,0},{"VERDE",0,100,0},{"AZUL",0,0,100},
    {"AMARILLO",50,50,0},{"CIAN",0,50,50},{"MAGENTA",50,0,50},
    {"NARANJA",70,30,0},{"ROSA",70,20,10},{"MORADO",40,0,60},
    {"BLANCO",33,33,33},{"GRIS",33,33,33},{"NEGRO",0,0,0}
};

#define NUM_COLORES (sizeof(colores)/sizeof(colores[0]))


//  PROTOTIPOS

static void     init_leds(void);
static void     init_adc(void);
static void     init_botones(void);
static void     init_oled(void);
static uint32_t leer_adc_promedio(void);
static void     medir_colores(void);
static void     calcular_porcentajes(void);
static color_t  clasificar_color(void);
static void     actualizar_led_status(color_t color);
static void     oled_cmd(uint8_t cmd);
static void     oled_enviar(uint8_t *datos, size_t tam);
static void     oled_inicio(void);
static void     limpiar_oled(void);
static void     posicion_oled(uint8_t linea, uint8_t columna);
static void     fuente(char c, uint8_t f[5]);
static void     escribir_char(char c);
static void     escribir_texto(const char *txt);
static void     mostrar_linea(uint8_t linea, const char *txt);
static void     mostrar_pantalla(void);


//  ISR DE BOTONES

void IRAM_ATTR isr_btn1(void *arg) {
    TickType_t now = xTaskGetTickCountFromISR();
    if ((now - last_tick_btn1) >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
        last_tick_btn1 = now;
        event_t ev = EV_BTN1;
        xQueueSendFromISR(event_queue, &ev, NULL);
    }
}

void IRAM_ATTR isr_btn2(void *arg) {
    TickType_t now = xTaskGetTickCountFromISR();
    if ((now - last_tick_btn2) >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
        last_tick_btn2 = now;
        event_t ev = EV_BTN2;
        xQueueSendFromISR(event_queue, &ev, NULL);
    }
}

void IRAM_ATTR isr_btn3(void *arg) {
    TickType_t now = xTaskGetTickCountFromISR();
    if ((now - last_tick_btn3) >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
        last_tick_btn3 = now;
        event_t ev = EV_BTN3;
        xQueueSendFromISR(event_queue, &ev, NULL);
    }
}

void IRAM_ATTR isr_btn4(void *arg) {
    TickType_t now = xTaskGetTickCountFromISR();
    if ((now - last_tick_btn4) >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
        last_tick_btn4 = now;
        event_t ev = EV_BTN4;
        xQueueSendFromISR(event_queue, &ev, NULL);
    }
}


//  INICIALIZACIONES

static void init_leds(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LED_R_GPIO)      |
                        (1ULL << LED_G_GPIO)      |
                        (1ULL << LED_B_GPIO)      |
                        (1ULL << LED_STATUS_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    gpio_set_level(LED_R_GPIO,      0);
    gpio_set_level(LED_G_GPIO,      0);
    gpio_set_level(LED_B_GPIO,      0);
    gpio_set_level(LED_STATUS_GPIO, 0);
}

static void init_adc(void) {
    adc1_config_width(ADC_WIDTH_BITS);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
}

static void init_botones(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BTN1_GPIO) |
                        (1ULL << BTN2_GPIO) |
                        (1ULL << BTN3_GPIO) |
                        (1ULL << BTN4_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN1_GPIO, isr_btn1, NULL);
    gpio_isr_handler_add(BTN2_GPIO, isr_btn2, NULL);
    gpio_isr_handler_add(BTN3_GPIO, isr_btn3, NULL);
    gpio_isr_handler_add(BTN4_GPIO, isr_btn4, NULL);
}


//  OLED - I2C

static void init_oled(void) {
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = OLED_SDA,
        .scl_io_num       = OLED_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = OLED_SPEED
    };
    i2c_param_config(OLED_PORT, &cfg);
    i2c_driver_install(OLED_PORT, cfg.mode, 0, 0, 0);
}

static void oled_cmd(uint8_t cmd) {
    uint8_t paquete[2] = {0x00, cmd};
    i2c_master_write_to_device(OLED_PORT, OLED_ADDR, paquete, 2,
                               pdMS_TO_TICKS(1000));
}

static void oled_enviar(uint8_t *datos, size_t tam) {
    uint8_t buffer[129];
    buffer[0] = 0x40;
    memcpy(&buffer[1], datos, tam);
    i2c_master_write_to_device(OLED_PORT, OLED_ADDR, buffer, tam + 1,
                               pdMS_TO_TICKS(1000));
}

static void oled_inicio(void) {
    vTaskDelay(pdMS_TO_TICKS(100));
    oled_cmd(0xAE);           // Display OFF
    oled_cmd(0xD5); oled_cmd(0x80); // Clock divide
    oled_cmd(0xA8); oled_cmd(0x1F); // Multiplex 32 lineas
    oled_cmd(0xD3); oled_cmd(0x00); // Display offset
    oled_cmd(0x40);           // Start line
    oled_cmd(0x8D); oled_cmd(0x14); // Charge pump ON
    oled_cmd(0x20); oled_cmd(0x00); // Horizontal addressing
    oled_cmd(0xA1);           // Segment remap
    oled_cmd(0xC8);           // COM scan direction
    oled_cmd(0xDA); oled_cmd(0x02); // COM pins 128x32
    oled_cmd(0x81); oled_cmd(0xCF); // Contrast
    oled_cmd(0xD9); oled_cmd(0xF1); // Pre-charge
    oled_cmd(0xDB); oled_cmd(0x40); // VCOMH deselect
    oled_cmd(0xA4);           // Display RAM
    oled_cmd(0xA6);           // Normal display
    oled_cmd(0xAF);           // Display ON
}

static void limpiar_oled(void) {
    for (int pag = 0; pag < 4; pag++) {
        oled_cmd(0xB0 + pag);
        oled_cmd(0x00);
        oled_cmd(0x10);
        uint8_t vacio[128];
        memset(vacio, 0x00, 128);
        oled_enviar(vacio, 128);
    }
}

static void posicion_oled(uint8_t linea, uint8_t columna) {
    oled_cmd(0xB0 + linea);
    oled_cmd(0x00 + (columna & 0x0F));
    oled_cmd(0x10 + ((columna >> 4) & 0x0F));
}

static void fuente(char c, uint8_t f[5]) {
    memset(f, 0x00, 5);
    switch (c) {
        case '0': {uint8_t x[5]={0x3E,0x51,0x49,0x45,0x3E}; memcpy(f,x,5);} break;
        case '1': {uint8_t x[5]={0x00,0x42,0x7F,0x40,0x00}; memcpy(f,x,5);} break;
        case '2': {uint8_t x[5]={0x42,0x61,0x51,0x49,0x46}; memcpy(f,x,5);} break;
        case '3': {uint8_t x[5]={0x21,0x41,0x45,0x4B,0x31}; memcpy(f,x,5);} break;
        case '4': {uint8_t x[5]={0x18,0x14,0x12,0x7F,0x10}; memcpy(f,x,5);} break;
        case '5': {uint8_t x[5]={0x27,0x45,0x45,0x45,0x39}; memcpy(f,x,5);} break;
        case '6': {uint8_t x[5]={0x3C,0x4A,0x49,0x49,0x30}; memcpy(f,x,5);} break;
        case '7': {uint8_t x[5]={0x01,0x71,0x09,0x05,0x03}; memcpy(f,x,5);} break;
        case '8': {uint8_t x[5]={0x36,0x49,0x49,0x49,0x36}; memcpy(f,x,5);} break;
        case '9': {uint8_t x[5]={0x06,0x49,0x49,0x29,0x1E}; memcpy(f,x,5);} break;
        case 'A': {uint8_t x[5]={0x7E,0x11,0x11,0x11,0x7E}; memcpy(f,x,5);} break;
        case 'B': {uint8_t x[5]={0x7F,0x49,0x49,0x49,0x36}; memcpy(f,x,5);} break;
        case 'C': {uint8_t x[5]={0x3E,0x41,0x41,0x41,0x22}; memcpy(f,x,5);} break;
        case 'D': {uint8_t x[5]={0x7F,0x41,0x41,0x22,0x1C}; memcpy(f,x,5);} break;
        case 'E': {uint8_t x[5]={0x7F,0x49,0x49,0x49,0x41}; memcpy(f,x,5);} break;
        case 'F': {uint8_t x[5]={0x7F,0x09,0x09,0x09,0x01}; memcpy(f,x,5);} break;
        case 'G': {uint8_t x[5]={0x3E,0x41,0x49,0x49,0x7A}; memcpy(f,x,5);} break;
        case 'I': {uint8_t x[5]={0x00,0x41,0x7F,0x41,0x00}; memcpy(f,x,5);} break;
        case 'J': {uint8_t x[5]={0x20,0x40,0x41,0x3F,0x01}; memcpy(f,x,5);} break;
        case 'L': {uint8_t x[5]={0x7F,0x40,0x40,0x40,0x40}; memcpy(f,x,5);} break;
        case 'M': {uint8_t x[5]={0x7F,0x02,0x0C,0x02,0x7F}; memcpy(f,x,5);} break;
        case 'N': {uint8_t x[5]={0x7F,0x04,0x08,0x10,0x7F}; memcpy(f,x,5);} break;
        case 'O': {uint8_t x[5]={0x3E,0x41,0x41,0x41,0x3E}; memcpy(f,x,5);} break;
        case 'P': {uint8_t x[5]={0x7F,0x09,0x09,0x09,0x06}; memcpy(f,x,5);} break;
        case 'R': {uint8_t x[5]={0x7F,0x09,0x19,0x29,0x46}; memcpy(f,x,5);} break;
        case 'S': {uint8_t x[5]={0x46,0x49,0x49,0x49,0x31}; memcpy(f,x,5);} break;
        case 'T': {uint8_t x[5]={0x01,0x01,0x7F,0x01,0x01}; memcpy(f,x,5);} break;
        case 'U': {uint8_t x[5]={0x3F,0x40,0x40,0x40,0x3F}; memcpy(f,x,5);} break;
        case 'V': {uint8_t x[5]={0x1F,0x20,0x40,0x20,0x1F}; memcpy(f,x,5);} break;
        case 'W': {uint8_t x[5]={0x7F,0x20,0x18,0x20,0x7F}; memcpy(f,x,5);} break;
        case 'Z': {uint8_t x[5]={0x61,0x51,0x49,0x45,0x43}; memcpy(f,x,5);} break;
        case ':': {uint8_t x[5]={0x00,0x36,0x36,0x00,0x00}; memcpy(f,x,5);} break;
        case '-': {uint8_t x[5]={0x08,0x08,0x08,0x08,0x08}; memcpy(f,x,5);} break;
        case '.': {uint8_t x[5]={0x00,0x60,0x60,0x00,0x00}; memcpy(f,x,5);} break;
        case '%': {uint8_t x[5]={0x23,0x13,0x08,0x64,0x62}; memcpy(f,x,5);} break;
        case ' ': default: break;
    }
}

static void escribir_char(char c) {
    uint8_t f[5];
    fuente(c, f);
    uint8_t dato[6] = {f[0], f[1], f[2], f[3], f[4], 0x00};
    oled_enviar(dato, 6);
}

static void escribir_texto(const char *txt) {
    while (*txt) { escribir_char(*txt++); }
}

static void mostrar_linea(uint8_t linea, const char *txt) {
    posicion_oled(linea, 0);
    uint8_t borrar[128];
    memset(borrar, 0x00, 128);
    oled_enviar(borrar, 128);
    posicion_oled(linea, 0);
    escribir_texto(txt);
}


//  LECTURA ADC

static uint32_t leer_adc_promedio(void) {
    uint32_t acc = 0;

    for (int i = 0; i < ADC_SAMPLES; i++) {
        acc += adc1_get_raw(ADC_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(2)); // LDR necesita tiempo
    }

    return acc / ADC_SAMPLES;
}

//Funcion de distancia
static float distancia_color(float r, float g, float b, color_ref_t c) {
    return (r-c.r)*(r-c.r) + (g-c.g)*(g-c.g) + (b-c.b)*(b-c.b);
}

// Calibracion
static void calibrar_ambiente(void) {

    gpio_set_level(LED_R_GPIO, 0);
    gpio_set_level(LED_G_GPIO, 0);
    gpio_set_level(LED_B_GPIO, 0);

    mostrar_linea(0, "CALIBRANDO");
    mostrar_linea(1, "SIN OBJETO");
    mostrar_linea(2, "ESPERE...");
    mostrar_linea(3, "");

    vTaskDelay(pdMS_TO_TICKS(500));

    fondo_rojo  = leer_adc_promedio();
    fondo_verde = leer_adc_promedio();
    fondo_azul  = leer_adc_promedio();

    calibrado = 1;

    mostrar_linea(0, "CALIBRADO");
    mostrar_linea(1, "OK");
    vTaskDelay(pdMS_TO_TICKS(800));
}

//Top 3 colores mas cercanos
static void clasificar_top3(char *c1, char *c2, char *c3) {

    float d1=1e9,d2=1e9,d3=1e9;
    int i1=-1,i2=-1,i3=-1;

    for(int i=0;i<NUM_COLORES;i++){
        float d = distancia_color(porc_rojo, porc_verde, porc_azul, colores[i]);

        if(d<d1){ d3=d2; i3=i2; d2=d1; i2=i1; d1=d; i1=i; }
        else if(d<d2){ d3=d2; i3=i2; d2=d; i2=i; }
        else if(d<d3){ d3=d; i3=i; }
    }

    strcpy(c1,colores[i1].nombre);
    strcpy(c2,colores[i2].nombre);
    strcpy(c3,colores[i3].nombre);
}

//  MEDICIÓN TRICROMÁTICA

static void medir_colores(void) {

    gpio_set_level(LED_R_GPIO, 0);
    gpio_set_level(LED_G_GPIO, 0);
    gpio_set_level(LED_B_GPIO, 0);

    mostrar_linea(0, "MIDIENDO");
    mostrar_linea(1, "ESPERE...");
    mostrar_linea(2, "");
    mostrar_linea(3, "");

    vTaskDelay(pdMS_TO_TICKS(200));

    // ===== ROJO =====
    mostrar_linea(1, "LED ROJO");
    gpio_set_level(LED_R_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    val_rojo = leer_adc_promedio();
    val_rojo = (val_rojo > fondo_rojo) ? val_rojo - fondo_rojo : 0;  
    gpio_set_level(LED_R_GPIO, 0);

    vTaskDelay(pdMS_TO_TICKS(150));

    // ===== VERDE =====
    mostrar_linea(1, "LED VERDE");
    gpio_set_level(LED_G_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    val_verde = leer_adc_promedio();
    val_verde = (val_verde > fondo_verde) ? val_verde - fondo_verde : 0;
    gpio_set_level(LED_G_GPIO, 0);

    vTaskDelay(pdMS_TO_TICKS(150));

    // ===== AZUL =====
    mostrar_linea(1, "LED AZUL");
    gpio_set_level(LED_B_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    val_azul = leer_adc_promedio();
    val_azul = (val_azul > fondo_azul) ? val_azul - fondo_azul : 0;
    gpio_set_level(LED_B_GPIO, 0);

    medicion_lista = 1;

    ESP_LOGI(TAG, "R=%lu G=%lu B=%lu",
             (unsigned long)val_rojo,
             (unsigned long)val_verde,
             (unsigned long)val_azul);
}



//  PORCENTAJES RGB

static void calcular_porcentajes(void) {
    float total = (float)(val_rojo + val_verde + val_azul);
    if (total <= 0) total = 1;
    porc_rojo  = (val_rojo  * 100.0f) / total;
    porc_verde = (val_verde * 100.0f) / total;
    porc_azul  = (val_azul  * 100.0f) / total;
}


//  CLASIFICACIÓN DE COLOR

static color_t clasificar_color(void) {

    // ===== NEGRO =====
    if (val_rojo < 200 && val_verde < 200 && val_azul < 200)
        return COLOR_NEGRO;

    // ===== BLANCO =====
    if (val_rojo > 3000 && val_verde > 3000 && val_azul > 3000)
        return COLOR_BLANCO;

    // ===== GRIS ===== (valores similares)
    if (abs((int)porc_rojo - (int)porc_verde) < 10 &&
        abs((int)porc_rojo - (int)porc_azul) < 10)
        return COLOR_GRIS;

    // ===== COLORES PRIMARIOS =====
    if (porc_rojo > 60 && porc_verde < 25 && porc_azul < 25)
        return COLOR_ROJO;

    if (porc_verde > 60 && porc_rojo < 25 && porc_azul < 25)
        return COLOR_VERDE;

    if (porc_azul > 60 && porc_rojo < 25 && porc_verde < 25)
        return COLOR_AZUL;

    // ===== COLORES SECUNDARIOS =====
    // Amarillo = rojo + verde
    if (porc_rojo > 35 && porc_verde > 35 && porc_azul < 20)
        return COLOR_AMARILLO;

    // Cian = verde + azul
    if (porc_verde > 35 && porc_azul > 35 && porc_rojo < 20)
        return COLOR_CIAN;

    // Magenta = rojo + azul
    if (porc_rojo > 35 && porc_azul > 35 && porc_verde < 20)
        return COLOR_MAGENTA;

    return COLOR_DESCONOCIDO;
}


//  LED STATUS

static void actualizar_led_status(color_t color) {
    gpio_set_level(LED_STATUS_GPIO, (color != COLOR_DESCONOCIDO) ? 1 : 0);
}


//  MOSTRAR PANTALLA OLED

static void mostrar_pantalla(void) {
    char l1[22], l2[22], l3[22], l4[22];
    const char *nombres[] = {
        "ROJO",
        "VERDE",
        "AZUL",
        "AMARILLO",
        "CIAN",
        "MAGENTA",
        "BLANCO",
        "NEGRO",
        "GRIS",
        "DESCONOCIDO"
    };

    switch (modo_actual) {

        // MODO 0: Valores RAW del ADC
        case MODO_MEDICION:
            sprintf(l1, "LECTURA RAW");
            sprintf(l2, "R:%lu", (unsigned long)val_rojo);
            sprintf(l3, "V:%lu", (unsigned long)val_verde);
            sprintf(l4, "A:%lu", (unsigned long)val_azul);
            break;

        // MODO 1: Porcentajes RGB
        case MODO_PORCENTAJES:
            sprintf(l1, "PORCENTAJES");
            sprintf(l2, "R:%.1f%%", porc_rojo);
            sprintf(l3, "V:%.1f%%", porc_verde);
            sprintf(l4, "A:%.1f%%", porc_azul);
            break;

        // MODO 2: Color detectado
        case MODO_RESULTADO:
            sprintf(l1, "COLOR:");
            sprintf(l2, "%s", medicion_lista ?
                    nombres[color_detectado] : "SIN MEDIR");
            sprintf(l3, medicion_lista ? "" : "BTN3-MEDIR");
            l4[0] = '\0';
            break;

        default: break;
    }

    mostrar_linea(0, l1);
    mostrar_linea(1, l2);
    mostrar_linea(2, l3);
    mostrar_linea(3, l4);
}


//  APP MAIN

void app_main(void) {
    event_queue = xQueueCreate(8, sizeof(event_t));

    init_leds();
    init_adc();
    init_botones();
    init_oled();
    oled_inicio();
    limpiar_oled();

    // Pantalla de bienvenida
    mostrar_linea(0, "DETECTOR");
    mostrar_linea(1, "DE COLOR");
    mostrar_linea(2, "ESP32");
    mostrar_linea(3, "INICIANDO");
    vTaskDelay(pdMS_TO_TICKS(2000));
    limpiar_oled();

    // Instrucciones iniciales en OLED
    mostrar_linea(0, "BTN1-MODO");
    mostrar_linea(1, "BTN2-MODO");
    mostrar_linea(2, "BTN3-MEDIR");
    mostrar_linea(3, "BTN4-RESET");
    vTaskDelay(pdMS_TO_TICKS(2000));
    limpiar_oled();

    ESP_LOGI(TAG, "=== Sistema de Deteccion de Color ===");
    ESP_LOGI(TAG, "BTN1=Modo+  BTN2=Modo-  BTN3=MEDIR  BTN4=Reset");

    // Mostrar pantalla inicial
    mostrar_pantalla();

    while (1) {
        event_t ev;
        if (xQueueReceive(event_queue, &ev, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (ev) {

                case EV_BTN1:
                    // Modo siguiente
                    modo_actual = (modo_actual + 1) % MODO_MAX;
                    ESP_LOGI(TAG, "Modo: %d", modo_actual);
                    limpiar_oled();
                    mostrar_pantalla();
                    break;

                case EV_BTN2:
                    // Modo anterior
                    modo_actual = (modo_actual == 0) ?
                                  (MODO_MAX - 1) : (modo_actual - 1);
                    ESP_LOGI(TAG, "Modo: %d", modo_actual);
                    limpiar_oled();
                    mostrar_pantalla();
                    break;

                    case EV_BTN3:

                    if (!calibrado) {
                        calibrar_ambiente();
                    } else {
                        medir_colores();
                        calcular_porcentajes();

                        char c1[12], c2[12], c3[12];
                        clasificar_top3(c1, c2, c3);

                        mostrar_linea(0, "COLOR:");
                        mostrar_linea(1, c1);
                        mostrar_linea(2, c2);
                        mostrar_linea(3, c3);
                    }
                    break;

                case EV_BTN4:
                    // Reset total
                    val_rojo        = 0;
                    val_verde       = 0;
                    val_azul        = 0;
                    porc_rojo       = 0;
                    porc_verde      = 0;
                    porc_azul       = 0;
                    color_detectado = COLOR_DESCONOCIDO;
                    medicion_lista  = 0;
                    modo_actual     = MODO_MEDICION;
                    calibrado = 0;
                    actualizar_led_status(color_detectado);
                    ESP_LOGI(TAG, "Reset realizado");
                    limpiar_oled();
                    mostrar_linea(0, "RESET");
                    mostrar_linea(1, "LISTO");
                    vTaskDelay(pdMS_TO_TICKS(800));
                    limpiar_oled();
                    mostrar_pantalla();
                    break;

                default: break;
            }
        }
    }
}