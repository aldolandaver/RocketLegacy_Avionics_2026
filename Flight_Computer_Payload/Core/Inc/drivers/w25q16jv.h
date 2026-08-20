/**
 ******************************************************************************
 * @file    w25q16jv.h
 * @brief   Driver SPI para memoria NOR Winbond W25Q16JV (2 MB / 16 Mbit),
 *          usada como bitácora de vuelo (DAQ log) en escritura puramente
 *          secuencial página a página, sin sistema de archivos.
 ******************************************************************************
 */
#ifndef W25Q16JV_H
#define W25Q16JV_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define W25Q_CMD_WRITE_ENABLE     0x06u
#define W25Q_CMD_WRITE_DISABLE    0x04u
#define W25Q_CMD_READ_STATUS1     0x05u
#define W25Q_CMD_PAGE_PROGRAM     0x02u
#define W25Q_CMD_READ_DATA        0x03u
#define W25Q_CMD_SECTOR_ERASE_4K  0x20u
#define W25Q_CMD_CHIP_ERASE       0xC7u
#define W25Q_CMD_JEDEC_ID         0x9Fu

#define W25Q_JEDEC_MANUFACTURER   0xEFu  /* Winbond              */
#define W25Q_JEDEC_MEMTYPE        0x40u  /* Familia W25Q          */
#define W25Q_JEDEC_CAPACITY       0x15u  /* 16 Mbit -> W25Q16JV   */

#define W25Q_PAGE_SIZE            256u
#define W25Q_SECTOR_SIZE          4096u
#define W25Q_TOTAL_SIZE_BYTES     (2u * 1024u * 1024u)

#define W25Q_STATUS1_BUSY_BIT     0x01u

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t      cs_pin;
    uint32_t      write_cursor; /* Próxima dirección libre, avanza monótonamente */
} w25q16jv_handle_t;

bool     W25Q16JV_Init(w25q16jv_handle_t *dev);
bool     W25Q16JV_EraseChip(w25q16jv_handle_t *dev);
/**
 * @brief Escribe hasta 256 bytes de forma segura respetando el límite de
 *        página. El llamador (tarea DAQ) es responsable de trocear buffers
 *        más grandes en múltiplos de página. No hace erase automático:
 *        se asume que el sector fue borrado antes del vuelo.
 */
bool     W25Q16JV_WritePage(w25q16jv_handle_t *dev, uint32_t addr, const uint8_t *data, uint16_t len);
bool     W25Q16JV_ReadData(w25q16jv_handle_t *dev, uint32_t addr, uint8_t *out, uint32_t len);
bool     W25Q16JV_IsBusy(w25q16jv_handle_t *dev);

#endif /* W25Q16JV_H */
