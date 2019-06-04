/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LAN8720A_H
#define __LAN8720A_H

#include "stm32f7xx.h"

/*
	ETH_MDIO -------------------------> PA2
	ETH_MDC --------------------------> PC1
	ETH_MII_RX_CLK/ETH_RMII_REF_CLK---> PA1
	ETH_MII_RX_DV/ETH_RMII_CRS_DV ----> PA7
	ETH_MII_RXD0/ETH_RMII_RXD0 -------> PC4
	ETH_MII_RXD1/ETH_RMII_RXD1 -------> PC5
	ETH_MII_TX_EN/ETH_RMII_TX_EN -----> PB11
	ETH_MII_TXD0/ETH_RMII_TXD0 -------> PG13
	ETH_MII_TXD1/ETH_RMII_TXD1 -------> PG14
																						*/
/* ETH_MDIO */
#define ETH_MDIO_GPIO_CLK_ENABLE()          __GPIOA_CLK_ENABLE()
#define ETH_MDIO_PORT                       GPIOA
#define ETH_MDIO_PIN                        GPIO_PIN_2
#define ETH_MDIO_AF                         GPIO_AF11_ETH

/* ETH_MDC */
#define ETH_MDC_GPIO_CLK_ENABLE()           __GPIOC_CLK_ENABLE();
#define ETH_MDC_PORT                        GPIOC
#define ETH_MDC_PIN                         GPIO_PIN_1
#define ETH_MDC_AF                          GPIO_AF11_ETH

/* ETH_RMII_REF_CLK */
#define ETH_RMII_REF_CLK_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE();
#define ETH_RMII_REF_CLK_PORT               GPIOA
#define ETH_RMII_REF_CLK_PIN                GPIO_PIN_1
#define ETH_RMII_REF_CLK_AF                 GPIO_AF11_ETH

/* ETH_RMII_CRS_DV */
#define ETH_RMII_CRS_DV_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE();
#define ETH_RMII_CRS_DV_PORT                GPIOA
#define ETH_RMII_CRS_DV_PIN                 GPIO_PIN_7
#define ETH_RMII_CRS_DV_AF                  GPIO_AF11_ETH

/* ETH_RMII_RXD0 */
#define ETH_RMII_RXD0_GPIO_CLK_ENABLE()     __GPIOC_CLK_ENABLE();
#define ETH_RMII_RXD0_PORT                  GPIOC
#define ETH_RMII_RXD0_PIN                   GPIO_PIN_4
#define ETH_RMII_RXD0_AF                    GPIO_AF11_ETH

/* ETH_RMII_RXD1 */
#define ETH_RMII_RXD1_GPIO_CLK_ENABLE()     __GPIOC_CLK_ENABLE();
#define ETH_RMII_RXD1_PORT                  GPIOC
#define ETH_RMII_RXD1_PIN                   GPIO_PIN_5
#define ETH_RMII_RXD1_AF                    GPIO_AF11_ETH

/* ETH_RMII_TX_EN */
#define ETH_RMII_TX_EN_GPIO_CLK_ENABLE()    __GPIOB_CLK_ENABLE();
#define ETH_RMII_TX_EN_PORT                 GPIOB
#define ETH_RMII_TX_EN_PIN                  GPIO_PIN_11
#define ETH_RMII_TX_EN_AF                   GPIO_AF11_ETH

/* ETH_RMII_TXD0 */
#define ETH_RMII_TXD0_GPIO_CLK_ENABLE()     __GPIOG_CLK_ENABLE();
#define ETH_RMII_TXD0_PORT                  GPIOG
#define ETH_RMII_TXD0_PIN                   GPIO_PIN_13
#define ETH_RMII_TXD0_AF                    GPIO_AF11_ETH

/* ETH_RMII_TXD1 */
#define ETH_RMII_TXD1_GPIO_CLK_ENABLE()     __GPIOG_CLK_ENABLE();
#define ETH_RMII_TXD1_PORT                  GPIOG
#define ETH_RMII_TXD1_PIN                   GPIO_PIN_14
#define ETH_RMII_TXD1_AF                    GPIO_AF11_ETH

void ETH_GPIO_Config(void);


#endif /* __LAN8720A_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
