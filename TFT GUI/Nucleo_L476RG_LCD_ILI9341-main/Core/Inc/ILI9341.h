/*
 * ILI9341.h
 *
 *  Created on: May 3, 2021
 *      Author: nunokawa
 */

#ifndef INC_ILI9341_H_
#define INC_ILI9341_H_

typedef struct {
  uint8_t id1;
  uint8_t id2;
  uint8_t id3;
} ILI9341_DisplayIdentificationInformation_t;

typedef struct {
  uint8_t boosterVoltageStatus : 1;
  uint8_t rowAddressOrder : 1;
  uint8_t columnAddressOrder : 1;
  uint8_t rowColumnExchange : 1;
  uint8_t verticalRefresh : 1;
  uint8_t rgbOrder : 1;
  uint8_t horizontalRefreshOrder : 1;
  uint8_t interfaceColorPixelFormatDefinition : 3;
  uint8_t idleMode : 1;
  uint8_t partialMode : 1;
  uint8_t sleepInOut : 1;
  uint8_t displayNormalMode : 1;
//  uint8_t verticalScrollingStatus : 1;
//  uint8_t inversionStatus : 1;
//  uint8_t allPixelOn : 1;
//  uint8_t allPixelOff : 1;
  uint8_t displayOnOff : 1;
  uint8_t tearingEffectLineOnOff : 1;
  uint8_t gammaCurveSelection : 3;
  uint8_t tearingEffectLineMode : 1;
} ILI9341_DisplayStatus_t;


typedef struct {
  uint8_t MY:1;  // Row Address Order
  uint8_t MX:1;  // Column Address Order
  uint8_t MV:1;  // Row / Column Exchange
  uint8_t ML:1;  // Vertical Refresh Order
  uint8_t BGR:1; //  RGB - BGR Order
  uint8_t MH:1;  // Horizontal Refresh Order1;
} ILI9341_MemoryAccessControl_t;





/**
 *
 */
void ILI9341_Init();


/**
 *
 */
void ILI9341_readDisplayStatus(ILI9341_DisplayStatus_t *status);

/**
 *
 */
void ILI9341_reasdDisplayIdentificationInformation(ILI9341_DisplayIdentificationInformation_t *info);

/**
 *
 */
void ILI9341_enterSleepMode();

/**
 *
 */
void ILI9341_sleepOut();


/**
 *
 */
void ILI9341_setBackLight(uint8_t v);

/**
 *
 */
void ILI9341_setColumnAddress(uint16_t st, uint16_t ed);

/**
 *
 */
void ILI9341_setPageAddress(uint16_t st, uint16_t ed);


/**
 *
 */
void ILI9341_setPixelFormat(uint8_t dpi, uint8_t dbi);

/**
 *
 */
void ILI9341_setMemoryAccessControl(const ILI9341_MemoryAccessControl_t *control);


/**
 *
 */
void ILI9341_setDisplayOff();

/**
 *
 */
void ILI9341_setDisplayOn();


/**
 *
 */
void ILI9341_MemoryWrite(const void *buffer, uint32_t size);

#endif /* INC_ILI9341_H_ */
