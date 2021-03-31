#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"


//定义2个GPIO作为PWM输出
#define  LEDC_TOTAL_NUM 1
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           0
#define LEDC_HS_CH0_GPIO       1
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#ifdef CONFIG_IDF_TARGET_ESP32S2
#define LEDC_LS_CH0_GPIO       (1)
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0
#endif

//渐变的变大最终目标占空比
#define LEDC_TEST_DUTY         8000
//渐变的时间
#define LEDC_TEST_FADE_TIME    3000

void * ledc_main(void * p) {
	
	//printf("呼吸灯进程开启...\n");

	ledc_timer_config_t ledc_timer = { .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
			.freq_hz = 5000,                      // frequency of PWM signal
			.speed_mode = LEDC_HS_MODE,           // timer mode
			.timer_num = LEDC_HS_TIMER            // timer index
			};

	// Set configuration of timer0 for high speed channels
	ledc_timer_config(&ledc_timer);

	ledc_channel_config_t ledc_channel[LEDC_TOTAL_NUM] = { { .channel =
	LEDC_HS_CH0_CHANNEL, .duty = 0, .gpio_num = LEDC_HS_CH0_GPIO, .speed_mode =
	LEDC_HS_MODE, .timer_sel = LEDC_HS_TIMER }
	};

	int ch;
	// Set LED Controller with previously prepared configuration
	for (ch = 0; ch < LEDC_TOTAL_NUM; ch++) {
		ledc_channel_config(&ledc_channel[ch]);
	}

	// Initialize fade service.
	ledc_fade_func_install(0);

    while (true) {

		//printf("1. PWM逐渐变大的周期目标 = %d\n", LEDC_TEST_DUTY);

		for (ch = 0; ch < LEDC_TOTAL_NUM; ch++) {
			ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
					ledc_channel[ch].channel, LEDC_TEST_DUTY,
					LEDC_TEST_FADE_TIME);
			ledc_fade_start(ledc_channel[ch].speed_mode,
					ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
		}
		vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

		//printf("2.  PWM逐渐变小的周期目标 = 0\n");

		for (ch = 0; ch < LEDC_TOTAL_NUM; ch++) {
			ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
					ledc_channel[ch].channel, 0, LEDC_TEST_FADE_TIME);
			ledc_fade_start(ledc_channel[ch].speed_mode,
					ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
		}

		vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

	}

}

