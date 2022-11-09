/*
 * ILI9341.c
 *
 *  Created on: May 3, 2021
 *      Author: nunokawa
 */

#include "main.h"
#include "stm32f04xx_hal.h"
#include "ILI9341.h"

extern SPI_HandleTypeDef hspi1;

#define BACKLIGHT_GPIO_PORT LCD_BACKLIGHT_GPIO_Port
#define BACKLIGHT_PIN       LCD_BACKLIGHT_Pin

#define RESET_GPIO_PORT     LCD_RESET_GPIO_Port
#define RESET_PIN           LCD_RESET_Pin

#define DC_GPIO_PORT        LCD_DC_GPIO_Port
#define DC_PIN              LCD_DC_Pin

#define CS_GPIO_PORT        LCD_CS_GPIO_Port
#define CS_PIN              LCD_CS_Pin

#define MIN(a, b) (((a)>(b)) ? (b) : (a))
#define bitcheck(a,b)   (a >> b) & 1

static __IO uint8_t flg_done; // DMA Transfer complete flag

void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef *hspi) {
	flg_done = 1;
}

void ILI9341_setBackLight(uint8_t v) {
	HAL_GPIO_WritePin(BACKLIGHT_GPIO_PORT, BACKLIGHT_PIN, (v)? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void setReset(uint8_t v) {
	HAL_GPIO_WritePin(RESET_GPIO_PORT, RESET_PIN, (v)? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void setDC(uint8_t v) {
	HAL_GPIO_WritePin(DC_GPIO_PORT, DC_PIN, (v)? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void setCS(uint8_t v) {
	HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, (v)? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void writeCmd(uint8_t c){
	setDC(0);
	HAL_SPI_Transmit(&hspi1, &c, 1, 1);
}

static inline void writeData(uint8_t d){
  setDC(1);
  HAL_SPI_Transmit(&hspi1, &d, 1, 1);
}

static inline void writeData16(uint16_t d){
  uint8_t d2[2] = {(d >> 8) & 0xff, d & 0xff};
  setDC(1);
  HAL_SPI_Transmit(&hspi1, d2, 2, 1);
}


void ILI9341_Init(void)
{
  setReset(0);
  HAL_Delay(20);
  setReset(1);
  HAL_Delay(20);

  ILI9341_MemoryAccessControl_t mac = {
    .MY  = 1,
    .MX  = 1,
    .MV  = 1,
    .ML  = 0,
    .BGR = 1,
    .MH  = 0
  };

  ILI9341_setMemoryAccessControl(&mac);

  ILI9341_setPixelFormat(0x05, 0x05);

  ILI9341_sleepOut();

  ILI9341_setDisplayOn();
}


void ILI9341_readDisplayStatus(ILI9341_DisplayStatus_t *status) {
  if (!status) {
    return;
  }
  
  uint8_t rbuf[5];

  setCS(0);
  writeCmd(0x09);
  setDC(1);
  HAL_SPI_Receive(&hspi1, rbuf, 5, 100);
  setCS(1);

  status->boosterVoltageStatus   = !! bitcheck(rbuf[1], 7);
  status->rowAddressOrder        = !! bitcheck(rbuf[1], 6);
  status->columnAddressOrder     = !! bitcheck(rbuf[1], 5);
  status->rowColumnExchange      = !! bitcheck(rbuf[1], 4);
  status->verticalRefresh        = !! bitcheck(rbuf[1], 3);
  status->rgbOrder               = !! bitcheck(rbuf[1], 2);
  status->horizontalRefreshOrder = !! bitcheck(rbuf[1], 1);

  status->interfaceColorPixelFormatDefinition = (rbuf[2] >> 4) & 0x07;
  status->idleMode                            = !! bitcheck(rbuf[2], 3);
  status->partialMode                         = !! bitcheck(rbuf[2], 2);
  status->sleepInOut                          = !! bitcheck(rbuf[2], 1);
  status->displayNormalMode                   = !! bitcheck(rbuf[2], 0);

//  status->verticalScrollingStatus = !! bitcheck(rbuf[3], );
//  status->inversionStatus
//  status->allPixelOn
//  status->allPixelOff
  status->displayOnOff            = !! bitcheck(rbuf[3], 2);
  status->tearingEffectLineOnOff  = !! bitcheck(rbuf[3], 1);
  status->gammaCurveSelection     = !! bitcheck(rbuf[3], 0);
  status->gammaCurveSelection    |= (rbuf[4] >> 6) & 0x03;

  status->tearingEffectLineMode = !! bitcheck(rbuf[4], 5);
}

void ILI9341_reasdDisplayIdentificationInformation(ILI9341_DisplayIdentificationInformation_t *info) {

  if (!info) {
    return;
  }
  
  uint8_t rbuf[4];

  setCS(0);
  writeCmd(0x04);
  
  setDC(1);
  HAL_SPI_Receive(&hspi1, rbuf, 4, 100);
  setCS(1);

  info->id1 = rbuf[1];
  info->id2 = rbuf[2];
  info->id3 = rbuf[3];
}

void ILI9341_enterSleepMode() {
  setCS(0);
  writeCmd(0x10);
  setCS(1);
}

void ILI9341_sleepOut() {
  setCS(0);
  writeCmd(0x11);
  setCS(1);

  HAL_Delay(60);
}


void ILI9341_setPixelFormat(uint8_t dpi, uint8_t dbi)
{
  dpi &= 0x07;
  dbi &= 0x07;

  uint8_t v = 0;

  v |= dpi << 4;
  v |= dbi;

  setCS(0);
  writeCmd(0x3A);
  writeData(v);
  setCS(1);
}

void ILI9341_setDisplayOff(){
  setCS(0);
  writeCmd(0x28);
  setCS(1);
}

void ILI9341_setDisplayOn(){
  setCS(0);
  writeCmd(0x29);
  setCS(1);
}

void ILI9341_setColumnAddress(uint16_t sc, uint16_t ec){
  setCS(0);
  writeCmd(0x2A);
  writeData16(sc);
  writeData16(ec);
  setCS(1);
}

void ILI9341_setPageAddress(uint16_t sp, uint16_t ep){
  setCS(0);
  writeCmd(0x2B);
  writeData16(sp);
  writeData16(ep);
  setCS(1);
}

void ILI9341_MemoryWrite(const void *buffer, uint32_t size) {

  setCS(0);
  writeCmd(0x2C);
  setDC(1);

  uint8_t *p = (uint8_t*)buffer;

  while (0 < size) {
    uint16_t sz = MIN(65536 - 8, size);

    flg_done = 0;
    HAL_SPI_Transmit_DMA(&hspi1, p, sz);

    while (!flg_done) {
      __NOP();
    }

    //    HAL_SPI_DMAStop (&hspi1);

    p += sz;
    size -= sz;
  }

  setCS(1);
}

void ILI9341_setMemoryAccessControl(const ILI9341_MemoryAccessControl_t *control) 
{
  if (control == NULL) {
    return;
  }

  uint8_t v = 0;

  v |= (!!control->MY)  << 7;
  v |= (!!control->MX)  << 6;
  v |= (!!control->MV)  << 5;
  v |= (!!control->ML)  << 4;
  v |= (!!control->BGR) << 3;
  v |= (!!control->MH)  << 2;

  setCS(0);
  writeCmd(0x36);
  writeData(v);
  setCS(1);
}

