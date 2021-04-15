/*
参考链接
https://blog.csdn.net/qq_24550925/article/details/85852672?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522161823656316780366561643%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=161823656316780366561643&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_v2~rank_v29-6-85852672.first_rank_v2_pc_rank_v29&utm_term=ESP32+AHT10
*/

/* 
=============
头文件包含
=============
*/
#include <stdio.h>
#include "string.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"


/*
===========================
宏定义
=========================== 
*/
//I2C 
#define I2C_SCL_IO          19                  //SCL->IO19
#define I2C_SDA_IO          18                  //SDA->IO18
#define I2C_MASTER_NUM      I2C_NUM_1           //I2C_1
#define WRITE_BIT           I2C_MASTER_WRITE    //写:0
#define READ_BIT            I2C_MASTER_READ     //读:1
#define ACK_CHECK_EN        0x1                 //主机检查从机的ACK
#define ACK_CHECK_DIS       0x0                 //主机不检查从机的ACK
#define ACK_VAL             0x0                 //应答
#define NACK_VAL            0x1                 //不应答

//aht10
#define aht10_WRITE_ADDR    0x38                //地址 
#define CMD_FETCH_DATA_H    0x22                //循环采样，参考aht10 datasheet
#define CMD_FETCH_DATA_L    0x36


/*
===========================
全局变量定义
=========================== 
*/
unsigned char aht10_buf[6]={0}; 
float g_temp=0.0, g_rh=0.0;


/*
===========================
函数声明
=========================== 
*/

esp_err_t i2c_init(void)
{
	//i2c配置结构体
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                    //I2C模式
    conf.sda_io_num = I2C_SDA_IO;                   //SDA IO映射
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;        //SDA IO模式
    conf.scl_io_num = I2C_SCL_IO;                   //SCL IO映射
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;        //SCL IO模式
    conf.master.clk_speed = 100000;                 //I2C CLK频率
    i2c_param_config(I2C_MASTER_NUM, &conf);        //设置I2C
    //注册I2C服务即使能
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode,0,0,0);
}

int aht10_init(void)
{
    int ret;
    //配置aht10的寄存器
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();                                   //新建操作I2C句柄
    i2c_master_start(cmd);                                                          //启动I2C
    i2c_master_write_byte(cmd, aht10_WRITE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);    //发地址+写+检查ack
    //i2c_master_write_byte(cmd, CMD_FETCH_DATA_H, ACK_CHECK_EN);                     //发数据高8位+检查ack
    //i2c_master_write_byte(cmd, CMD_FETCH_DATA_L, ACK_CHECK_EN);                     //发数据低8位+检查ack
    //i2c_master_write_byte(cmd, 0x70, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0xe1, ACK_CHECK_EN);	
	i2c_master_write_byte(cmd, 0x08, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
    i2c_master_stop(cmd);                                                           //停止I2C
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);        //I2C发送
    i2c_cmd_link_delete(cmd);                                                       //删除I2C句柄
    printf("初始化aht10,结果返回ret:%d\r\n",ret);
    return ret;
}

unsigned char AHT10_CalcCrc(unsigned char *data, unsigned char nbrOfBytes)
{
	unsigned char bit;        // bit mask
    unsigned char crc = 0xFF; // calculated checksum
    unsigned char byteCtr;    // byte counter
    unsigned int POLYNOMIAL =  0x131;           // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

    // calculates 8-Bit checksum with given polynomial
    for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
        crc ^= (data[byteCtr]);
        for(bit = 8; bit > 0; --bit) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ POLYNOMIAL;
            }  else {
                crc = (crc << 1);
            }
        }
    }
	return crc;
}

unsigned char AHT10_CheckCrc(unsigned char *pdata, unsigned char nbrOfBytes, unsigned char checksum)
{
    unsigned char crc;
	crc = AHT10_CalcCrc(pdata, nbrOfBytes);// calculates 8-Bit checksum
    if(crc != checksum) 
    {   
        return 1;           
    }
    return 0;              
}

int aht10_get_value(void)
{
    int ret;
    ESP_LOGI("aht10", "读取温湿度了....\n");
    
    vTaskDelay(80/portTICK_RATE_MS);

    //配置aht10的寄存器
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();                                   //新建操作I2C句柄
    i2c_master_start(cmd);                                                          //启动I2C
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, aht10_WRITE_ADDR << 1 | READ_BIT, ACK_CHECK_EN));

    i2c_master_read(cmd, &aht10_buf,5, ACK_VAL);                               //读取数据+回复ack
    i2c_master_read_byte(cmd, &aht10_buf[5],NACK_VAL);                              //读取数据+不回复ack
    
    i2c_master_stop(cmd);                                                           //停止I2C
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);        //I2C发送

    i2c_cmd_link_delete(cmd);                          

    cmd = i2c_cmd_link_create();           
    i2c_master_start(cmd);                          
    if((aht10_buf[0] & 0x08)==0)
	{
        i2c_master_write_byte(cmd, aht10_WRITE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);    //发地址+写+检查ack
        i2c_master_write_byte(cmd, 0xe1, ACK_CHECK_EN);	
        i2c_master_write_byte(cmd, 0x08, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);        
    }
    else
    {
        i2c_master_write_byte(cmd, aht10_WRITE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);    //发地址+读+检查ack
        i2c_master_write_byte(cmd, 0xac, ACK_CHECK_EN); 
        i2c_master_write_byte(cmd, 0x33, ACK_CHECK_EN); 
        i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); 
    }
    
    i2c_master_stop(cmd);                                                           //停止I2C
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);        //I2C发送
    i2c_cmd_link_delete(cmd);                          
    
    //ESP_LOGI("aht10", "结束读取温湿度了....\r\n");
    vTaskDelay(500/portTICK_RATE_MS);                             //删除I2C句柄
    if(ret!=ESP_OK)
    {
        return ret;
    }
    return ESP_OK;
}

void app_main()
{
    ESP_LOGI("aht10", "已经开始尝试读取温湿度了....\r\n");
    ESP_ERROR_CHECK(i2c_init());                         //I2C初始化
    //ESP_ERROR_CHECK(aht10_init());                       //aht10初始化
    vTaskDelay(100/portTICK_RATE_MS);   //延时100ms
	int32_t i,result_t,result_h;
    while(1)
    {
        if(aht10_get_value()==ESP_OK)   //获取温湿度
        {
            /*
            for(i=0;i<6;i++)
            {
                printf("i2c_master_readByte:0x%x \n",aht10_buf[i]);
            }
            */
            result_t = ((aht10_buf[3] & 0x0F) << 16) | (aht10_buf[4] << 8) | aht10_buf[5];
            int32_t t = (((200.0 * (float)result_t) / 1048576.0) - 50.0)*100.0;
            result_h = ((aht10_buf[1] << 16) | (aht10_buf[2] << 8) | aht10_buf[3]) >> 4;
            int32_t h = (float)result_h * 10000.0 / 1048576.0;
            printf("t:%u.%u,h:%u.%u\n",t/100,t%100,h/100,h%100);	
        }            
        vTaskDelay(30000/portTICK_RATE_MS);
    }
}