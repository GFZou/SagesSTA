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

//ags02ma
#define ags02ma_WRITE_ADDR    0x1A                //地址 
#define CMD_FETCH_DATA_H    0x22                //循环采样，参考ags02ma datasheet
#define CMD_FETCH_DATA_L    0x36


/*
===========================
全局变量定义
=========================== 
*/
unsigned char ags02ma_buf[5]={0}; 
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
    conf.master.clk_speed = 30000;                 //I2C CLK频率
    i2c_param_config(I2C_MASTER_NUM, &conf);        //设置I2C
    //注册I2C服务即使能
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode,0,0,0);
}

//传感器零点校准:上电后5分钟放置于清新空气中5分钟发送
//0x1A 0x01 0x00 0x0C 0xFF 0xF3 0xFC
int ags02ma_init(void)
{
    int ret;
    //配置ags02ma的寄存器
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();                                   //新建操作I2C句柄
    i2c_master_start(cmd);                                                          //启动I2C
    i2c_master_write_byte(cmd, ags02ma_WRITE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);    //发地址+写+检查ack
    i2c_master_write_byte(cmd, 0x01, ACK_CHECK_EN);                     //发数据高8位+检查ack
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);                     //发数据低8位+检查ack
    i2c_master_write_byte(cmd, 0x0C, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0xFF, ACK_CHECK_EN);	
	i2c_master_write_byte(cmd, 0xF3, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0xFC, ACK_CHECK_EN);
    i2c_master_stop(cmd);                                                           //停止I2C
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);        //I2C发送
    i2c_cmd_link_delete(cmd);                                                       //删除I2C句柄
    printf("零点校准ags02ma,结果返回ret:%d\r\n",ret);
    return ret;
}

bool checkCRC8(uint8_t *data, uint8_t Num) 
{
  uint8_t bit, byte, crc = 0xFF;
  // the data of the converted data byte should be identical with the data got from the check function.
  for (byte = 0; byte < Num; byte++)
  {
    crc ^= (data[byte]);
    for (bit = 8; bit > 0; --bit)
    {
      if (crc & 0x80) 
          crc = (crc << 1) ^ 0x31;
      else 
          crc = (crc << 1);
    }
  }
  //compare the caculated crc with the read crc to determine if the reading is correct
  if (crc == data[Num]) { 
    return true;
  } else {
    return false;
  }
}

//**********************************************************
//函数名称：Calc_CRC8
//功能 ：CRC8 计算，初值：0xFF，多项式：0x31(x8 + x5 + x4 +1 )
//参数 ：u8 *dat：需要校验数据的首地址；u8 Num：CRC 校验数据长度
//返回 ：crc：计算出的校验值
//**********************************************************
unsigned char Calc_CRC8(unsigned char *dat, unsigned char Num)
{
    unsigned char i,byte,crc=0xFF;
    for(byte=0; byte<Num; byte++)
    {
        crc^=(dat[byte]);
        for(i=0;i<8;i++)
        {
            if(crc & 0x80) crc=(crc<<1)^0x31;
            else crc=(crc<<1);
        }
    }
    return crc;
}

int ags02ma_get_value(void)
{
    int ret;
    ESP_LOGI("ags02ma", "读取空气挥发性含量了....\n");
    
    vTaskDelay(80/portTICK_RATE_MS);

    //配置ags02ma的寄存器
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();                                   //新建操作I2C句柄
    i2c_master_start(cmd);                                                          //启动I2C
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ags02ma_WRITE_ADDR << 1 | READ_BIT, ACK_CHECK_EN));

    i2c_master_read(cmd, &ags02ma_buf,4, ACK_VAL);                               //读取数据+回复ack
    i2c_master_read_byte(cmd, &ags02ma_buf[5],NACK_VAL);                              //读取数据+不回复ack
    
    i2c_master_stop(cmd);                                                           //停止I2C
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);        //I2C发送
    i2c_cmd_link_delete(cmd);                          
    /*
    cmd = i2c_cmd_link_create();           
    i2c_master_start(cmd);                          
    if((ags02ma_buf[0] & 0x08)==0)
	{
        i2c_master_write_byte(cmd, ags02ma_WRITE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);    //发地址+写+检查ack
        i2c_master_write_byte(cmd, 0xe1, ACK_CHECK_EN);	
        i2c_master_write_byte(cmd, 0x08, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);        
    }
    else
    {
        i2c_master_write_byte(cmd, ags02ma_WRITE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);    //发地址+读+检查ack
        i2c_master_write_byte(cmd, 0xac, ACK_CHECK_EN); 
        i2c_master_write_byte(cmd, 0x33, ACK_CHECK_EN); 
        i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN); 
    }
    
    i2c_master_stop(cmd);                                                           //停止I2C
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);        //I2C发送
    i2c_cmd_link_delete(cmd);                          
    */

    ESP_LOGI("ags02ma", "结束读取数据了....\r\n");
    vTaskDelay(500/portTICK_RATE_MS);                             //删除I2C句柄
    if(ret!=ESP_OK)
    {
        return ret;
    }
    return ESP_OK;
}

void app_main()
{
    ESP_LOGI("ags02ma", "已经开始尝试读取温湿度了....\r\n");
    ESP_ERROR_CHECK(i2c_init());                         //I2C初始化
    //ESP_ERROR_CHECK(ags02ma_init());                       //ags02ma初始化
    vTaskDelay(100/portTICK_RATE_MS);   //延时100ms
    int voc=-10.0;
    while(1)
    {
        vTaskDelay(3000/portTICK_RATE_MS);
        if(ags02ma_get_value()==ESP_OK)   //获取温湿度
        {            
            for(int i=0;i<5;i++)
            {
                printf("i2c_master_readByte:0x%x \n",ags02ma_buf[i]);
            }
            
            // when the returned data is wrong, request to get data again until the data is correct. 
            //read data from the sensor, check if VOC data got from the check function is correct. 
            //if it is correct, retrun VOC concentration 
            if (checkCRC8(ags02ma_buf, 4) == 1) {
                voc = ags02ma_buf[1];
                voc <<= 8;
                voc |= ags02ma_buf[2];
                voc <<= 8;
                voc |= ags02ma_buf[3];
                printf("读出来的空气挥发性浓度值是:%2lfPPB\n",voc / 10.0);;
            }
            else 
            {
                voc = ags02ma_buf[1];
                voc <<= 8;
                voc |= ags02ma_buf[2];
                voc <<= 8;
                voc |= ags02ma_buf[3];
                printf("检验出错,读出来的空气挥发性浓度值是:%2lfPPB\n",voc / 10.0);
            }
        }
    }
}