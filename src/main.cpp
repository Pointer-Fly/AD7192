#include <SPI.h>
#include <Arduino.h>
#include <AD7192.h>

void single_conversion_voltage(void *arg);
void difference_continuous_conversion_voltage(void *arg);
void continuous_conversion_voltage(void *arg);

void setup()
{

  Serial.begin(115200);
  AD7192_Init();  // 初始化AD7192
  AD7192SoftwareReset();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  ReadFromAD7192ViaSPI(REG_ID, 1, AD7192Registers, REG_ID);

  Serial.println(AD7192Registers[REG_ID], HEX);

  if ((AD7192Registers[REG_ID] & 0x0F) != 0)
  {
    Serial.println("AD7192 初始化失败请检查连接！");
    while(1);
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  // 下面三个task不能同时启动，只能启动一个
  // 4路单端单次转换输出
  xTaskCreate(single_conversion_voltage, "single_conversion_voltage", 4096, NULL, 5, NULL);
  // 两路差分连续读
  // xTaskCreate(difference_continuous_conversion_voltage, "difference_continuous_conversion_voltage", 4096, NULL, 5, NULL);
  // 4路单端连续转换输出
  // xTaskCreate(continuous_conversion_voltage, "continuous_conversion_voltage", 4096, NULL, 5, NULL);

}

// The loop function runs over and over again until power down or reset
void loop()
{
  // delay(1000); //  Refresh watchdog?
}

/**
  * @brief  4路单端连续转换电压实验
  * @param  无
  * @retval 无
  */
void continuous_conversion_voltage(void *arg)
{
  uint32_t ad_data = 0;
  float v = 0.0;
  uint32_t mode = 0, cfg = 0;
  
  Serial.print("野火 AD9172 4 路连续转换读电压实验\r\n");
  
  /* 读 AD7192 寄存器 */
  ReadFromAD7192ViaSPI(REG_COM_STA, 8, AD7192Registers, REG_COM_STA);
  for(int i=0; i < 8; i++)
  {
    Serial.print("AD7192Register[");
    Serial.print(i+REG_COM_STA);
    Serial.print("] = 0x");
    Serial.println(AD7192Registers[i+REG_COM_STA], HEX);

  }
  
       /* 单次转换|使能状态传输|外部时钟|sinc4滤波器|禁用奇偶校验|时钟不分频|禁用单周期转换|禁用60Hz陷波|128 */
  mode = MODE_SING|DAT_STA_EN|EXT_XTAL|SINC_4|ENPAR_DIS|CLK_DIV_DIS|SINGLECYCLE_DIS|REJ60_DIS|1023;
  cfg  = CHOP_DIS|REF_IN1|AIN1_AIN2|BURN_DIS|REFDET_DIS|BUF_DIS|UB_BI|GAIN_1;
       /*禁用斩波|外部基准电压1|12差分通道(单)|禁用激励电流|禁用基准电压检测|禁用模拟输入缓冲|双极性模式|增益为128 */
  
  ad7192_mode_cfg_reg(mode, cfg);    // 配置模式寄存器和配置寄存器

  /* 校准 */
  AD7192InternalZeroScaleCalibration();
  AD7192InternalFullScaleCalibration();	
  
  AD7192StartContinuousConvertion(AIN1_COM|AIN2_COM|AIN3_COM|AIN4_COM);    // 启动连续转换
  
	while(1)
	{
    // Serial.println("Read AIN1_COM");
    ad_data = AD7192ReadConvertingData();
    
    switch(ad_data & 7)
    {
      case 4:
        v = ((ad_data >> 8) / 8388608.0 - 1)*3.3;
        Serial.print("AIN1_COM = ");
        Serial.print(v, 6);
        Serial.println("V");
      break;
      
      case 5:
        v = ((ad_data >> 8) / 8388608.0 - 1)*3.3;
        Serial.print("AIN2_COM = ");
        Serial.print(v, 6);
        Serial.println("V");
      break;
      
      case 6:
        v = ((ad_data >> 8) / 8388608.0 - 1)*3.3;
        Serial.print("AIN3_COM = ");
        Serial.print(v, 6);
        Serial.println("V");
      break;
      
      case 7:
        v = ((ad_data >> 8) / 8388608.0 - 1)*3.3;
        Serial.print("AIN4_COM = ");
        Serial.print(v, 6);
        Serial.println("V");
      break;
    }
    // vTaskDelay(1000 / portTICK_PERIOD_MS); // 可以在这里加延时
	}
}



/**
  * @brief  2路差分连续转换电压实验
  * @param  无
  * @retval 无
  */
void difference_continuous_conversion_voltage(void *arg)
{
  uint32_t ad_data = 0;
  float v = 0.0;
  uint32_t mode = 0, cfg = 0;
  
  Serial.print("野火 AD9172 2路 差分连续转换读电压实验\r\n");
  
  /* 读 AD7192 寄存器 */
  ReadFromAD7192ViaSPI(REG_COM_STA, 8, AD7192Registers, REG_COM_STA);
  for(int i=0; i < 8; i++)
  {
    Serial.print("AD7192Register[");
    Serial.print(i+REG_COM_STA);
    Serial.print("] = 0x");
    Serial.println(AD7192Registers[i+REG_COM_STA], HEX);
  }
  
       /* 单次转换|使能状态传输|外部时钟|sinc4滤波器|禁用奇偶校验|时钟不分频|禁用单周期转换|禁用60Hz陷波|128 */
  mode = MODE_SING|DAT_STA_EN|EXT_XTAL|SINC_4|ENPAR_DIS|CLK_DIV_DIS|SINGLECYCLE_DIS|REJ60_DIS|1023;
  cfg  = CHOP_DIS|REF_IN1|AIN1_AIN2|BURN_DIS|REFDET_DIS|BUF_DIS|UB_BI|GAIN_1;
       /*禁用斩波|外部基准电压1|12差分通道(单)|禁用激励电流|禁用基准电压检测|禁用模拟输入缓冲|双极性模式|增益为128 */
  
  ad7192_mode_cfg_reg(mode, cfg);    // 配置模式寄存器和配置寄存器

  /* 校准 */
  AD7192InternalZeroScaleCalibration();
  AD7192InternalFullScaleCalibration();	
  
  AD7192StartContinuousConvertion(AIN1_AIN2|AIN3_AIN4);    // 启动连续转换
  AD7192StartContinuousRead();	
	while(1)
	{
    Serial.println("Read AIN1_COM");
    ad_data = AD7192ReadConvertingData();

    switch(ad_data & 7)
    {
      case 0:
        v = ((ad_data >> 8) / 8388608.0 - 1)*3.3;
        Serial.print("AIN1_AIN2 = ");
        Serial.print(v, 6);
        Serial.println("V");
      break;
      
      case 1:
        v = ((ad_data >> 8) / 8388608.0 - 1)*3.3;
        Serial.print("AIN3_AIN4 = ");
        Serial.print(v, 6);
        Serial.println("V");
      break;
      
    }
    // vTaskDelay(1000 / portTICK_PERIOD_MS); //可以在这里加延时
	}
}


/**
 * @brief  单次转换电压实验
 * @param  无
 * @retval 无
 */
void single_conversion_voltage(void *arg)
{
  uint32_t ad_data = 0;
  float v = 0.0;
  uint32_t mode = 0, cfg = 0;

  Serial.println("野火 AD9172 4 路单次转换读电压实验\r\n");

  /* 读 AD7192 寄存器 */
  ReadFromAD7192ViaSPI(REG_COM_STA, 8, AD7192Registers, REG_COM_STA);
  for (int i = 0; i < 8; i++)
  {
    Serial.print("AD7192Register[");
    Serial.print(i + REG_COM_STA);
    Serial.print("] = 0x");
    Serial.println(AD7192Registers[i + REG_COM_STA], HEX);
  }

  /* 单次转换|禁用状态传输|外部时钟|sinc4滤波器|禁用奇偶校验|时钟不分频|禁用单周期转换|禁用60Hz陷波|128 */
  mode = MODE_SING | DAT_STA_DIS | EXT_XTAL | SINC_4 | ENPAR_DIS | CLK_DIV_DIS | SINGLECYCLE_DIS | REJ60_DIS | 1023;
  cfg = CHOP_DIS | REF_IN1 | AIN1_AIN2 | BURN_DIS | REFDET_DIS | BUF_DIS | UB_BI | GAIN_1;
  /*禁用斩波|外部基准电压1|12差分通道(单)|禁用激励电流|禁用基准电压检测|禁用模拟输入缓冲|双极性模式|增益为128 */

  ad7192_mode_cfg_reg(mode, cfg); // 配置模式寄存器和配置寄存器

  /* 校准 */
  Serial.println("内部校准中\r\n");
  AD7192InternalZeroScaleCalibration();
  AD7192InternalFullScaleCalibration();
  Serial.println("内部校准完成\r\n");

  while (1)
  {
    /* 读通道 1 转换数据 */
    // Serial.println("Read AIN1_COM");
    AD7192StartSingleConvertion(AIN1_COM);
    ad_data = AD7192ReadConvertingData();
    v = (ad_data / 8388608.0 - 1) * 3.3;
    Serial.print("AIN1_COM = ");
    Serial.print(v,6);
    Serial.println("V");

    /* 读通道 2 转换数据 */
    AD7192StartSingleConvertion(AIN2_COM);
    ad_data = AD7192ReadConvertingData();
    v = (ad_data / 8388608.0 - 1)*3.3;
    Serial.print("AIN2_COM = ");
    Serial.print(v, 6);
    Serial.println("V");

    /* 读通道 3 转换数据 */
    AD7192StartSingleConvertion(AIN3_COM);
    ad_data = AD7192ReadConvertingData();
    v = (ad_data / 8388608.0 - 1)*3.3;
    Serial.print("AIN3_COM = ");
    Serial.print(v, 6);
    Serial.println("V");

    /* 读通道 4 转换数据 */
    AD7192StartSingleConvertion(AIN4_COM);
    ad_data = AD7192ReadConvertingData();
    v = (ad_data / 8388608.0 - 1)*3.3;
    Serial.print("AIN4_COM = ");
    Serial.print(v, 6);
    Serial.println("V");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}