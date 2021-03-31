#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

//static void blink(void* arg);
//static IRAM_ATTR void blink(void* arg);//IRAM_ATTR 强制将函数的属性改为.text。将定义的中断回调函数定义在iram区
//ISR处理程序不再需要用IRAM_ATTR来声明，除非你在gpio_install_isr_service()中分配ISR时传递了ESP_INTR_FLAG_IRAM标志。

//static void gpio_task_example(void* arg);//必须具备Freertos的线程
//static xQueueHandle gpio_evt_queue = NULL;
void * relay_main(void * p)
{
    /*
	gpio_config_t led_config;
	led_config.intr_type = GPIO_INTR_DISABLE;
	led_config.pin_bit_mask = GPIO_SEL_2;
	led_config.mode = GPIO_MODE_OUTPUT;
	led_config.pull_up_en = GPIO_PULLUP_DISABLE;
	led_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpio_config(&led_config);
    */

   /*
	gpio_config_t button_config;
	button_config.intr_type = GPIO_INTR_NEGEDGE;//按键另外一端接地，使用下降沿触发
	button_config.pin_bit_mask = GPIO_SEL_0;
	button_config.mode = GPIO_MODE_INPUT;
	button_config.pull_up_en = GPIO_PULLUP_ENABLE;//按键另外一端接地，使用下降沿触发，必须拉高
	button_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpio_config(&button_config);//配置完默认是开启中断

 	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));//创建消息队列
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

	static int count = 1;//计入进入中断的次数,注意野指针
	gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);//设置中断优先级最低
	gpio_isr_handler_add(GPIO_NUM_0, blink, &count);//注册中断处理程序
    */
    gpio_pad_select_gpio(GPIO_NUM_5);//选择一个GPIO
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);//把这个GPIO作为输出
    printf("开始继电器任务...\n");

	while (1)
	{
        /*
		vTaskDelay(20000/portTICK_RATE_MS);
		gpio_intr_disable(GPIO_NUM_0);//20秒后关闭中断
		vTaskDelay(20000/portTICK_RATE_MS);
		gpio_intr_enable(GPIO_NUM_0);//20秒后开启中断
        */
       //printf("5输出高电平...\n");
       gpio_set_level(GPIO_NUM_5, 0);//把这个GPIO输出高电平       
       vTaskDelay(5000/portTICK_RATE_MS);       
       //printf("5输出低电平...\n");
       gpio_set_level(GPIO_NUM_5, 1);//把这个GPIO输出低电平
       vTaskDelay(5000/portTICK_RATE_MS);
	}
}

/*
void blink(void* args)
{
	gpio_set_level(GPIO_NUM_2,(*(int*)args)%2);
	(*(int*)args)++;  
	xQueueSendFromISR(gpio_evt_queue, args, NULL);//允许消息队列。注意野指针
	//printf("args is %d.\r\n",(*(int*)args));//中断中不允许存在打印等,较大的代码
}
void gpio_task_example(void* arg)
{
    for(;;) {
    	int args;
	    if(xQueueReceive(gpio_evt_queue, &args, portMAX_DELAY)) {
		 	printf("args is %d.\r\n",args);
	    }
		vTaskDelay(10);
    }
}
*/

