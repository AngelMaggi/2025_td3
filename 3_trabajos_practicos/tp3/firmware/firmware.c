#include <stdio.h>
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "helper.h"   //libreria generador de pwm para simular
#include "lcd.h"      //libreria de LCD por I2C

// Eleccion de I2C a usar
#define I2C         i2c0
// Eleccion de GPIO para SDA
#define SDA_GPIO    16
// Eleccion de GPIO para SCL
#define SCL_GPIO    17
// Direccion de 7 bits del adaptador del LCD
#define ADDR        0x27

TaskHandle_t handle_Task_Frecuencimetro = NULL;

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

//TAREA FRECUENCIMETRO, DETECTOR DE FLANCOS ASCENDENTES
void task_frecuencimetro(void *params) {
    
    TickType_t tiempo_ms = xTaskGetTickCount();
    int contador = 0;
    char str1[20]="La frecuencia es:";
    char str2[20]="";

    //HABILITO LA INTERRUPCION DEL PIN 2 POR FLANCOS ASCENDENTES
    gpio_set_irq_enabled_with_callback(PIN_ENT_SEÑAL, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

        while(1) {

            if(xTaskGetTickCount() - tiempo_ms > 1000){
                xSemaphoreTake( semphrCounting, 0 );
                contador = uxSemaphoreGetCount(semphrCounting);
                lcd_clear();
                lcd_set_cursor(0,0);
                lcd_string(str1);
                lcd_set_cursor(1, 0);
                sprintf(str2, "%i Hz", contador);
                lcd_string(str2);
                tiempo_ms = xTaskGetTickCount();
                xQueueReset(semphrCounting);
            }
            vTaskDelay(pdMS_TO_TICKS(10));  // Pequeño delay para no saturar la CPU
        }
    }


/**
 * @brief Programa principal
 */
int main(void) {
    stdio_init_all();

    // Inicializacion de GPIO ENTRADA SEÑAL
    gpio_init(PIN_ENT_SEÑAL);
    gpio_set_dir(PIN_ENT_SEÑAL, false);
    gpio_pull_down(PIN_ENT_SEÑAL);

    // Generador de PWM para comprobar contador de flancos
    pwm_user_init(PIN_GEN_SEÑAL, 10000);
    
    //Creacion de Semaforo Counting
    semphrCounting = xSemaphoreCreateCounting( 10000, 0 );  
    
    // Creacion de tareas
    xTaskCreate(task_frecuencimetro, "Task_Frecuencimetro", 4*configMINIMAL_STACK_SIZE, NULL, 2, NULL);

    // Inicializo el I2C con un clock de 100 KHz
    i2c_init(I2C, 100000);
    // Habilito la funcion de I2C en los GPIOs
    gpio_set_function(SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(SCL_GPIO, GPIO_FUNC_I2C);
    // Habilito pull-ups
    gpio_pull_up(SDA_GPIO);
    gpio_pull_up(SCL_GPIO);
    // Inicializo LCD
    lcd_init(I2C, ADDR);
    // Limpio el LCD
    lcd_clear();

    // Arranca el scheduler
    vTaskStartScheduler();
    while(1);
}