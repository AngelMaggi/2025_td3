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
#define SETPOINT 25.0

//Handles de las tareas
TaskHandle_t handle_Task_BMP280 = NULL;
TaskHandle_t handle_Task_LCD = NULL;
TaskHandle_t handle_Task_LED = NULL;

//handle del semaforo mutex que manejara en bus I2C
SemaphoreHandle_t xMutex_I2C;


// Handles de las colas que se usaran (datos y numero de pagina)
QueueHandle_t queue_datos;
QueueHandle_t queue_pages;

//structura para manejo de los datos del sensor
typedef struct {
    float temp;
    float presion;
} sensor_data_t;

//variable auxiliar
bool change_page = 0;

#define PIN_BOTON 13     //GPIO 15 COMO ENTRADA
#define PIN_LED 14     //GPIO 14 COMO SALIDA PWM

/*interrupcion que se ejecuta al presionar el boton, por flanco descendete, modificando un dato de cola
 que cambiara la pagina a mostrar en el lcd, */
void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    bool page;
    if (gpio == PIN_BOTON && (events & GPIO_IRQ_EDGE_FALL)) {
        xQueuePeekFromISR(queue_pages, &page);
        page = !page;
        change_page = true;
        sleep_ms(200);
        xQueueOverwriteFromISR(queue_pages, &page, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

//strucutura de datos, necesaria para la calibracion del sensor y calculos correctos
sensor_data_t struct_datos;

//TAREA LEER SENSOR TEMPERATURA Y PRESION
void task_BMP280(void *params) {
    
    // Obtiene los parámetros de calibración
    struct bmp280_calib_param struct_calib_params;
    xSemaphoreTake(xMutex_I2C ,portMAX_DELAY);          //tomo el semaforo para que ningun otra tarea pueda utilizar el bus I2C
    bmp280_get_calib_params(&struct_calib_params);      //obtengo los parametros de calibracion del sensor
    xSemaphoreGive(xMutex_I2C);                         //devuelvo el semaforo I2C

    while (1) {
        int32_t raw_temp, raw_presion;
        xSemaphoreTake(xMutex_I2C ,portMAX_DELAY);      //vuelvo a tomar el semaforo ahora para obtener los datos RAW del sensor
        bmp280_read_raw(&raw_temp, &raw_presion); // Lee valores crudos
        xSemaphoreGive(xMutex_I2C);                //devuelvo semaforo

        struct_datos.temp = bmp280_convert_temp(raw_temp, &struct_calib_params); // Convierte temperatura
        struct_datos.presion = bmp280_convert_pressure(raw_presion, raw_temp, &struct_calib_params) / 100.00; // Convierte presion
        printf("Temperatura: %.2f °C \nPresion: %.2f hPa\n", struct_datos.temp, struct_datos.presion);  //imprimo datos por consola serial para debuggear

        xQueueOverwrite(queue_datos, &struct_datos);    //sobreescribo datos del sensor en cola de datos

        vTaskDelay(pdMS_TO_TICKS(100));         //demora de 100ms hasta proxima lectura
    }
}

//RECIBIR DATOS DE LA CoLA E IMPRIMIRLOS POR EL DisPLAY LCD
void task_LCD(void *params){
    
    sensor_data_t datos_LCD;
    bool page;
    // Variable para imprimir el mensaje
    char str[16];
    float error_prueba;

    while (1) {
        xQueuePeek(queue_datos, &datos_LCD, portMAX_DELAY);         //hago peek de datos para posterior manejo
        xSemaphoreTake(xMutex_I2C ,portMAX_DELAY);                  //tomo el semaforo mutex para poder imprimir en LCD y no ser interrumpido por lectura de datos del sensor
        xQueuePeek(queue_pages, &page, portMAX_DELAY);              //hago peek de numero de pagina a imprimir

        if(change_page) {lcd_clear(); change_page = false;}         //verifico si es necesario un borrado de pantalla
        if(!page){
            lcd_set_cursor(0, 0);
            sprintf(str, "Temp: %.2f C", datos_LCD.temp);               //imprimo datos de TEMP y PRESION en la primer
            lcd_string(str);                                            //pagina del display LCD
            // Muevo el cursor al comienzo de la segunda fila
            lcd_set_cursor(1, 0);
            // Imprimo el mensaje
            sprintf(str,"Presion: %.2f hPa", datos_LCD.presion);
            lcd_string(str);
        }else{                                                           //imprimo datos de TEMP, SETPOINT Y ERROR
            lcd_set_cursor(0, 0);                                        //en la segunda pagina del DISPLAY LCD
            sprintf(str, "Temp: %.2f C", datos_LCD.temp);
            lcd_string(str);
            lcd_set_cursor(1, 0);
            sprintf(str, "SetPoint: %.2f C", SETPOINT);
            lcd_string(str);
            // Muevo el cursor al comienzo de la segunda fila
            lcd_set_cursor(2, 0);
            // Imprimo el mensaje
            error_prueba = datos_LCD.temp-SETPOINT; if (error_prueba < 0 ) error_prueba = - error_prueba;  //obtengo valor absoluto del error
            sprintf(str,"Error: %.2f C", error_prueba);
            lcd_string(str);
        }
        xSemaphoreGive(xMutex_I2C);                 //devuelvo semaforo mutex I2C para libre utilizacion del sensor, si fuese necesario
        // Demora
        vTaskDelay(pdMS_TO_TICKS(100));             //demora de 100ms hasta proxima secuencia de impresion
    }

}

//tarea de encendido de LED mediante el uso de PWM
void task_LED(void *params){
    sensor_data_t datos_LCD;
    float error_prueba;
    uint16_t pwm_duty;
    
    while(1){
    
        xQueuePeek(queue_datos, &datos_LCD, portMAX_DELAY);         //hago peek de los datos de sensor (solo utilizo la TEMP)
        error_prueba = datos_LCD.temp-SETPOINT;                     //calculo ERROR
        if (error_prueba < 0 ) error_prueba = - error_prueba;       //obtengo valor absoluto del ERROR
        pwm_duty = (uint16_t)(error_prueba * 100.0 / 5.0);          //hago escalado de ERROR -> PWM ------ (0 - 5 °C) -> (0 - 100) 
        if(pwm_duty > 100) pwm_duty = 100;                          //limito maximo dutycicle a 100
        if(pwm_duty < 0 ) pwm_duty = 0;                             //limito minimo dutycicle a 0 
        pwm_set_gpio_level(PIN_LED, pwm_duty);                      //establezco dutycicle del PWM
        vTaskDelay(pdMS_TO_TICKS(100));                             //demora de 100ms hasta proxima actualizacion del LED
    }
}

/**
 * @brief Programa principal
 */
int main(void) {
    stdio_init_all();

    // Generador de PWM para encender el LED
    pwm_user_init(PIN_LED, 10000);
    
    // Creacion de tareas
    xTaskCreate(task_BMP280, "Task_BMP280", 4*configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(task_LCD, "Task_LCD", 4*configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(task_LED, "Task_LED", 4*configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    //Creacion de la cola que enviara datos de Task_BMP280 a Task_LCD
    queue_datos = xQueueCreate(1, sizeof(sensor_data_t));
    bool page_number = 0;
    queue_pages = xQueueCreate(1, sizeof(bool));
    xQueueOverwrite(queue_pages, &page_number);

    //creacion del semaforo mutex
    xMutex_I2C = xSemaphoreCreateMutex();

    // Inicializo el I2C con un clock de 100 KHz
    i2c_init(I2C, 100000);
    
    // Habilito la funcion de I2C en los GPIOs
    gpio_set_function(SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(SCL_GPIO, GPIO_FUNC_I2C);
    
    // Habilito pull-ups
    gpio_pull_up(SDA_GPIO);
    gpio_pull_up(SCL_GPIO);

    gpio_init(PIN_BOTON);
    gpio_set_dir(PIN_BOTON, false);
    gpio_pull_up(PIN_BOTON);
   
    // Inicializo LCD
    lcd_init(I2C, ADDR_LCD);
    // Limpio el LCD
    lcd_clear();

    // Inicializa el sensor BMP280
    bmp280_init(i2c0);

    //Habilito las interrupcion del pulsador po flanco ascendente
    gpio_set_irq_enabled_with_callback(PIN_BOTON, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Arranca el scheduler
    vTaskStartScheduler();
    while(1);
}


