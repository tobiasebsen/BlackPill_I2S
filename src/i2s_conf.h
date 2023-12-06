#pragma once

#define I2S_INSTANCE	3
#define I2S_USE_DMA		1

#if I2S_INSTANCE == 1
#define I2S_SPI			SPI1
#define I2S_DMAn		2
#define I2S_RX_STREAM	0
#define I2S_RX_CHANNEL	3
#define I2S_TX_STREAM	2
#define I2S_TX_CHANNEL	2
#endif

#if I2S_INSTANCE == 2
#define I2S_SPI			SPI2
#define I2S_DMAn		1
#define I2S_RX_STREAM	3
#define I2S_RX_CHANNEL	3
#define I2S_TX_STREAM	4
#define I2S_TX_CHANNEL	0
#endif

#if I2S_INSTANCE == 3
#define I2S_SPI			SPI3
#define I2S_DMAn		1
#define I2S_RX_STREAM	0
#define I2S_RX_CHANNEL	3
#define I2S_TX_STREAM	5
#define I2S_TX_CHANNEL	0
#endif