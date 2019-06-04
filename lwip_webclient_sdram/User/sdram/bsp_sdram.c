/**
  ******************************************************************************
  * @file    bsp_sdram.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   sdram应用函数接口
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 STM32F767 开发板  
  * 论坛    :http://www.chuxue123.com
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./sdram/bsp_sdram.h"  
#include "main.h"

/**
  * @brief  延迟一段时间
  * @param  延迟的时间长度
  * @retval None
  */
static void SDRAM_delay(__IO uint32_t nCount)
{
  __IO uint32_t index = 0; 
  for(index = (100000 * nCount); index != 0; index--)
  {
  }
}

/**
  * @brief  初始化控制SDRAM的IO
  * @param  无
  * @retval 无
  */
static void SDRAM_GPIO_Config(void)
{		
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* 使能SDRAM相关的GPIO时钟 */

  /*地址信号线*/
  FMC_A0_GPIO_CLK;  FMC_A1_GPIO_CLK;  FMC_A2_GPIO_CLK;
  FMC_A3_GPIO_CLK;FMC_A4_GPIO_CLK;FMC_A5_GPIO_CLK;
  FMC_A6_GPIO_CLK; FMC_A7_GPIO_CLK; FMC_A8_GPIO_CLK;
  FMC_A9_GPIO_CLK; FMC_A10_GPIO_CLK;FMC_A11_GPIO_CLK; 
  FMC_A12_GPIO_CLK;
  /*数据信号线*/
  FMC_D0_GPIO_CLK; FMC_D1_GPIO_CLK ; FMC_D2_GPIO_CLK ; 
  FMC_D3_GPIO_CLK ; FMC_D4_GPIO_CLK ; FMC_D5_GPIO_CLK ;
  FMC_D6_GPIO_CLK; FMC_D7_GPIO_CLK ; FMC_D8_GPIO_CLK ;
  FMC_D9_GPIO_CLK ;FMC_D10_GPIO_CLK; FMC_D11_GPIO_CLK;
  FMC_D12_GPIO_CLK; FMC_D13_GPIO_CLK; FMC_D14_GPIO_CLK;
  FMC_D15_GPIO_CLK;  
  /*控制信号线*/
  FMC_CS_GPIO_CLK ; FMC_BA0_GPIO_CLK; FMC_BA1_GPIO_CLK ;
  FMC_WE_GPIO_CLK ; FMC_RAS_GPIO_CLK ; FMC_CAS_GPIO_CLK;
  FMC_CLK_GPIO_CLK ; FMC_CKE_GPIO_CLK; FMC_UDQM_GPIO_CLK;
  FMC_LDQM_GPIO_CLK;

  
  /*-- GPIO 配置 -----------------------------------------------------*/
  /* 通用 GPIO 配置 */       
  GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;//配置为复用功能
  GPIO_InitStructure.Pull      = GPIO_PULLUP;
  GPIO_InitStructure.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStructure.Alternate = GPIO_AF12_FMC;
  
  /*地址信号线 针对引脚配置*/
  GPIO_InitStructure.Pin = FMC_A0_GPIO_PIN; 
  HAL_GPIO_Init(FMC_A0_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = FMC_A1_GPIO_PIN; 
  HAL_GPIO_Init(FMC_A1_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = FMC_A2_GPIO_PIN; 
  HAL_GPIO_Init(FMC_A2_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = FMC_A3_GPIO_PIN; 
  HAL_GPIO_Init(FMC_A3_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = FMC_A4_GPIO_PIN; 
  HAL_GPIO_Init(FMC_A4_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = FMC_A5_GPIO_PIN; 
  HAL_GPIO_Init(FMC_A5_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = FMC_A6_GPIO_PIN; 
  HAL_GPIO_Init(FMC_A6_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = FMC_A7_GPIO_PIN; 
  HAL_GPIO_Init(FMC_A7_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = FMC_A8_GPIO_PIN; 
  HAL_GPIO_Init(FMC_A8_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = FMC_A9_GPIO_PIN; 
  HAL_GPIO_Init(FMC_A9_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = FMC_A10_GPIO_PIN; 
  HAL_GPIO_Init(FMC_A10_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = FMC_A11_GPIO_PIN; 
  HAL_GPIO_Init(FMC_A11_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = FMC_A12_GPIO_PIN; 
  HAL_GPIO_Init(FMC_A12_GPIO_PORT, &GPIO_InitStructure);
  
  
  /*数据信号线 针对引脚配置*/
  GPIO_InitStructure.Pin = FMC_D0_GPIO_PIN; 
  HAL_GPIO_Init(FMC_D0_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_D1_GPIO_PIN; 
  HAL_GPIO_Init(FMC_D1_GPIO_PORT, &GPIO_InitStructure);

    
  GPIO_InitStructure.Pin = FMC_D2_GPIO_PIN; 
  HAL_GPIO_Init(FMC_D2_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_D3_GPIO_PIN; 
  HAL_GPIO_Init(FMC_D3_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_D4_GPIO_PIN; 
  HAL_GPIO_Init(FMC_D4_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_D5_GPIO_PIN; 
  HAL_GPIO_Init(FMC_D5_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_D6_GPIO_PIN; 
  HAL_GPIO_Init(FMC_D6_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_D7_GPIO_PIN; 
  HAL_GPIO_Init(FMC_D7_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_D8_GPIO_PIN; 
  HAL_GPIO_Init(FMC_D8_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_D9_GPIO_PIN; 
  HAL_GPIO_Init(FMC_D9_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_D10_GPIO_PIN; 
  HAL_GPIO_Init(FMC_D10_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_D11_GPIO_PIN; 
  HAL_GPIO_Init(FMC_D11_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_D12_GPIO_PIN; 
  HAL_GPIO_Init(FMC_D12_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_D13_GPIO_PIN; 
  HAL_GPIO_Init(FMC_D13_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_D14_GPIO_PIN; 
  HAL_GPIO_Init(FMC_D14_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_D15_GPIO_PIN; 
  HAL_GPIO_Init(FMC_D15_GPIO_PORT, &GPIO_InitStructure);

  
  /*控制信号线*/
  GPIO_InitStructure.Pin = FMC_CS_GPIO_PIN; 
  HAL_GPIO_Init(FMC_CS_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_BA0_GPIO_PIN; 
  HAL_GPIO_Init(FMC_BA0_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_BA1_GPIO_PIN;
  HAL_GPIO_Init(FMC_BA1_GPIO_PORT, &GPIO_InitStructure);

    
  GPIO_InitStructure.Pin = FMC_WE_GPIO_PIN; 
  HAL_GPIO_Init(FMC_WE_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_RAS_GPIO_PIN; 
  HAL_GPIO_Init(FMC_RAS_GPIO_PORT, &GPIO_InitStructure);

    
  GPIO_InitStructure.Pin = FMC_CAS_GPIO_PIN; 
  HAL_GPIO_Init(FMC_CAS_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_CLK_GPIO_PIN; 
  HAL_GPIO_Init(FMC_CLK_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_CKE_GPIO_PIN; 
  HAL_GPIO_Init(FMC_CKE_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_UDQM_GPIO_PIN; 
  HAL_GPIO_Init(FMC_UDQM_GPIO_PORT, &GPIO_InitStructure);

  
  GPIO_InitStructure.Pin = FMC_LDQM_GPIO_PIN; 
  HAL_GPIO_Init(FMC_LDQM_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  对SDRAM芯片进行初始化配置
  * @param  None. 
  * @retval None.
  */
static void SDRAM_InitSequence(void)
{
  /* Step 1 --------------------------------------------------------------------*/
  /* 配置命令：开启提供给SDRAM的时钟 */
	FMC_Bank5_6->SDCMR = 0x00000009;
  /* Step 2: 延时100us */ 
  SDRAM_delay(1);	
	/* Step 3 --------------------------------------------------------------------*/
  /* 配置命令：对所有的bank预充电 */ 
  FMC_Bank5_6->SDCMR = 0x0000000A;    
  /* Step 4 --------------------------------------------------------------------*/
  /* 配置命令：自动刷新 */   
  FMC_Bank5_6->SDCMR = 0x0000008B;  
  /* Step 5 --------------------------------------------------------------------*/
  /* 设置sdram寄存器配置 */
  FMC_Bank5_6->SDCMR = 0x0004600C;
  /* Step 6 --------------------------------------------------------------------*/
	FMC_Bank5_6->SDRTR |= (824<<1);
}


/**
  * @brief  初始化配置使用SDRAM的FMC及GPIO接口，
  *         本函数在SDRAM读写操作前需要被调用
  * @param  None
  * @retval None
  */
void SDRAM_Init(void)
{
	//使能HSE
  RCC->CR |= RCC_CR_HSEON;
  while(!(RCC->CR&(1<<17)))
	{;}
	//使能PLL
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR&(1<<25)))
	{;}
	//选择HSE作为PLL时钟源
	RCC->PLLCFGR |= (1 << 22);
	//设置分频系数 PLLCLK = HSE/M*N/P = 25M / 25 *432 / 2 = 216M
  RCC->PLLCFGR |= (25 << 0);//PLLM
	RCC->PLLCFGR |= (432 << 6);//PLLN
	RCC->PLLCFGR |= (2 << 16);//PLLP
	RCC->PLLCFGR |= (9 << 24);//PLLQ
	/* 激活 OverDrive 模式以达到216M频率  */ 
	HAL_PWREx_EnableOverDrive();

	/* 配置FMC接口相关的 GPIO*/
  SDRAM_GPIO_Config();
	
	/* 使能 FMC 时钟 */
	__HAL_RCC_FMC_CLK_ENABLE();

	/** Perform the SDRAM1 memory initialization sequence
  */
	FMC_Bank5_6->SDCR[FMC_SDRAM_BANK1] = 0x000039D0;
	FMC_Bank5_6->SDCR[FMC_SDRAM_BANK2] = 0x000001D8;
	
	FMC_Bank5_6->SDTR[FMC_SDRAM_BANK1] = 0x0F1F7FFF;
	FMC_Bank5_6->SDTR[FMC_SDRAM_BANK2] = 0x01010471;
	
  /* FMC SDRAM device initialization sequence */
  SDRAM_InitSequence(); 
}

/**
  * @brief  以“字”为单位向sdram写入数据 
  * @param  pBuffer: 指向数据的指针 
  * @param  uwWriteAddress: 要写入的SDRAM内部地址
  * @param  uwBufferSize: 要写入数据大小
  * @retval None.
  */
void SDRAM_WriteBuffer(uint32_t* pBuffer, uint32_t uwWriteAddress, uint32_t uwBufferSize)
{
  __IO uint32_t write_pointer = (uint32_t)uwWriteAddress;

//  /* 禁止写保护 */
//  HAL_SDRAM_WriteProtection_Disable(&hsdram1);
	FMC_Bank5_6->SDCR[1] &= ~FMC_SDRAM_WRITE_PROTECTION_ENABLE;
//  /* 检查SDRAM标志，等待至SDRAM空闲 */ 
//  while(HAL_SDRAM_GetState(&hsdram1) != RESET)
//  {
//  }

  /* 循环写入数据 */
  for (; uwBufferSize != 0; uwBufferSize--) 
  {
    /* 发送数据到SDRAM */
    *(uint32_t *) (SDRAM_BANK_ADDR + write_pointer) = *pBuffer++;

    /* 地址自增*/
    write_pointer += 4;
  }  
}

/**
  * @brief  从SDRAM中读取数据 
  * @param  pBuffer: 指向存储数据的buffer
  * @param  ReadAddress: 要读取数据的地十
  * @param  uwBufferSize: 要读取的数据大小
  * @retval None.
  */
void SDRAM_ReadBuffer(uint32_t* pBuffer, uint32_t uwReadAddress, uint32_t uwBufferSize)
{
  __IO uint32_t write_pointer = (uint32_t)uwReadAddress;
  
   
//  /* 检查SDRAM标志，等待至SDRAM空闲 */  
//  while ( HAL_SDRAM_GetState(&hsdram1) != RESET)
//  {
//  }
  
  /*读取数据 */
  for(; uwBufferSize != 0x00; uwBufferSize--)
  {
   *pBuffer++ = *(__IO uint32_t *)(SDRAM_BANK_ADDR + write_pointer );
    
   /* 地址自增*/
    write_pointer += 4;
  } 
}


/**
  * @brief  测试SDRAM是否正常 
  * @param  None
  * @retval 正常返回1，异常返回0
  */
uint8_t SDRAM_Test(void)
{
  /*写入数据计数器*/
  uint32_t counter=0;
  
  /* 8位的数据 */
  uint8_t ubWritedata_8b = 0, ubReaddata_8b = 0;  
  
  /* 16位的数据 */
  uint16_t uhWritedata_16b = 0, uhReaddata_16b = 0; 
  
  SDRAM_INFO("正在检测SDRAM，以8位、16位的方式读写sdram...");

  /*按8位格式读写数据，并校验*/
  
  /* 把SDRAM数据全部重置为0 ，W9825G6KH_SIZE是以8位为单位的 */
  for (counter = 0x00; counter < W9825G6KH_SIZE; counter++)
  {
    *(__IO uint8_t*) (SDRAM_BANK_ADDR + counter) = (uint8_t)0x0;
  }
  
  /* 向整个SDRAM写入数据  8位 */
  for (counter = 0; counter < W9825G6KH_SIZE; counter++)
  {
    *(__IO uint8_t*) (SDRAM_BANK_ADDR + counter) = (uint8_t)(ubWritedata_8b + counter);
  }
  
  /* 读取 SDRAM 数据并检测*/
  for(counter = 0; counter<W9825G6KH_SIZE;counter++ )
  {
    ubReaddata_8b = *(__IO uint8_t*)(SDRAM_BANK_ADDR + counter);  //从该地址读出数据
    
    if(ubReaddata_8b != (uint8_t)(ubWritedata_8b + counter))      //检测数据，若不相等，跳出函数,返回检测失败结果。
    {
      SDRAM_ERROR("8位数据读写错误！");
      return 0;
    }
  }
	
  /*按16位格式读写数据，并检测*/
  
  /* 把SDRAM数据全部重置为0 */
  for (counter = 0x00; counter < W9825G6KH_SIZE/2; counter++)
  {
    *(__IO uint16_t*) (SDRAM_BANK_ADDR + 2*counter) = (uint16_t)0x00;
  }
  
  /* 向整个SDRAM写入数据  16位 */
  for (counter = 0; counter < W9825G6KH_SIZE/2; counter++)
  {
    *(__IO uint16_t*) (SDRAM_BANK_ADDR + 2*counter) = (uint16_t)(uhWritedata_16b + counter);
  }
  
    /* 读取 SDRAM 数据并检测*/
  for(counter = 0; counter<W9825G6KH_SIZE/2;counter++ )
  {
    uhReaddata_16b = *(__IO uint16_t*)(SDRAM_BANK_ADDR + 2*counter);  //从该地址读出数据
    
    if(uhReaddata_16b != (uint16_t)(uhWritedata_16b + counter))      //检测数据，若不相等，跳出函数,返回检测失败结果。
    {
      SDRAM_ERROR("16位数据读写错误！");

      return 0;
    }
  }
  SDRAM_INFO("SDRAM读写测试正常！"); 
  /*检测正常，return 1 */
  return 1;
}


/*********************************************END OF FILE**********************/

