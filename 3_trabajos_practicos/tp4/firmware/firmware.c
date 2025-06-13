#include <stdio.h>
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "helper.h"   //libreria generador de pwm para simular
#include "lcd.h"      //libreria de LCD por I2C
#include "bmp280.h"   //libreria del sensor de presion y temperatura bmp280

// Eleccion de I2C a usar
#define I2C         i2c0
// Eleccion de GPIO para SDA
#define SDA_GPIO    16
// Eleccion de GPIO para SCL
#define SCL_GPIO    17
// Direccion de 7 bits del adaptador del LCD
#define ADDR_LCD        0x27
//#define ADDR_BMP280     0x76

TaskHandle_t handle_Task_BMP280 = NULL;
TaskHandle_t handle_Task_LCD = NULL;

SemaphoreHandle_t xMutex_I2C;


// Cola para datos del sensor
QueueHandle_t queue_datos;

typedef struct {
    float temp;
    float presion;
} sensor_data_t;

/*
// Semaforo para CONTAR LOS FLANCOS
SemaphoreHandle_t semphrCounting;

#define PIN_ENT_SEÑAL 2     //GPIO 2 COMO ENTRADA
#define PIN_GEN_SEÑAL 3     //GPIO 3 COMO SALIDA PWM

void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (gpio == PIN_ENT_SEÑAL && (events & GPIO_IRQ_EDGE_RISE)) {
        xSemaphoreGiveFromISR(semphrCounting, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
*/
//TAREA LEER SENSOR TEMPERATURA Y PRESION

sensor_data_t struct_datos;

void task_BMP280(void *params) {
    
    // Obtiene los parámetros de calibración
    struct bmp280_calib_param struct_calib_params;
    xSemaphoreTake(xMutex_I2C ,portMAX_DELAY);
    bmp280_get_calib_params(&struct_calib_params);
    xSemaphoreGive(xMutex_I2C);

    while (1) {
        int32_t raw_temp, raw_presion;
        xSemaphoreTake(xMutex_I2C ,portMAX_DELAY);
        bmp280_read_raw(&raw_temp, &raw_presion); // Lee valores crudos
        xSemaphoreGive(xMutex_I2C);

        struct_datos.temp = bmp280_convert_temp(raw_temp, &struct_calib_params); // Convierte temperatura
        struct_datos.presion = bmp280_convert_pressure(raw_presion, raw_temp, &struct_calib_params) / 100.00; // Convierte presion
        printf("Temperatura: %.2f °C \nPresion: %.2f hPa\n", struct_datos.temp, struct_datos.presion);

        xQueueOverwrite(queue_datos, &struct_datos);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//RECIBIR DATOS DE LA CoLA E IMPRIMIRLOS POR EL DisPLAY LCD
void task_LCD(void *params){
    
    sensor_data_t datos_LCD;
    // Variable para imprimir el mensaje
    char str[16];

    while (1) {

        xQueuePeek(queue_datos, &datos_LCD, portMAX_DELAY);
        xSemaphoreTake(xMutex_I2C ,portMAX_DELAY);
        // Armo un string con la variable de contador y la incremento
        lcd_set_cursor(0, 0);
        sprintf(str, "Temp: %.2f C", datos_LCD.temp);
        lcd_string(str);
        // Muevo el cursor al comienzo de la segunda fila
        lcd_set_cursor(1, 0);
        // Imprimo el mensaje
        sprintf(str,"Presion: %.2f hPa", datos_LCD.presion);
        lcd_string(str);
        xSemaphoreGive(xMutex_I2C);
        // Demora
        vTaskDelay(pdMS_TO_TICKS(100));
    }

}

/**
 * @brief Programa principal
 */
int main(void) {
    stdio_init_all();

    // Inicializacion de GPIO ENTRADA SEÑAL
    //gpio_init(PIN_ENT_SEÑAL);
    //gpio_set_dir(PIN_ENT_SEÑAL, false);
    //gpio_pull_down(PIN_ENT_SEÑAL);

    // Generador de PWM para comprobar contador de flancos
    //pwm_user_init(PIN_GEN_SEÑAL, 10000);
    
    //Creacion de Semaforo Counting
    //semphrCounting = xSemaphoreCreateCounting( 10000, 0 );  
    
    // Creacion de tareas
    xTaskCreate(task_BMP280, "Task_BMP280", 2*configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(task_LCD, "Task_LCD", 2*configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    //Creacion de la cola que enviara datos de Task_BMP280 a Task_LCD
    queue_datos = xQueueCreate(1, sizeof(sensor_data_t));
    xMutex_I2C = xSemaphoreCreateMutex();
    // Inicializo el I2C con un clock de 100 KHz
    i2c_init(I2C, 100000);
    // Habilito la funcion de I2C en los GPIOs
    gpio_set_function(SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(SCL_GPIO, GPIO_FUNC_I2C);
    // Habilito pull-ups
    gpio_pull_up(SDA_GPIO);
    gpio_pull_up(SCL_GPIO);
    // Inicializo LCD
    lcd_init(I2C, ADDR_LCD);
    // Limpio el LCD
    lcd_clear();

    // Inicializa el sensor BMP280
    bmp280_init(i2c0);

    // Arranca el scheduler
    vTaskStartScheduler();
    while(1);
}