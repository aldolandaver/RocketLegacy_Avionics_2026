/**
 ******************************************************************************
 * @file    w25q16jv.c
 * @brief   Operaciones básicas NOR: identificación JEDEC, write-enable,
 *          programación de página y lectura secuencial. El borrado de chip
 *          se ejecuta una única vez en tierra antes del vuelo (Estado 1),
 *          nunca durante el vuelo activo.
 ******************************************************************************
 */
#include "drivers/w25q16jv.h"

#define SPI_TIMEOUT_MS   50u
#define ERASE_TIMEOUT_MS 30000u /* Chip erase del W25Q16JV puede tardar hasta ~25 s */

static inline void cs_low(w25q16jv_handle_t *dev)  { HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET); }
static inline void cs_high(w25q16jv_handle_t *dev) { HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET); }

static bool write_enable(w25q16jv_handle_t *dev)
{
    uint8_t cmd = W25Q_CMD_WRITE_ENABLE;
    cs_low(dev);
    bool ok = (HAL_SPI_Transmit(dev->hspi, &cmd, 1, SPI_TIMEOUT_MS) == HAL_OK);
    cs_high(dev);
    return ok;
}

static uint8_t read_status1(w25q16jv_handle_t *dev)
{
    uint8_t tx[2] = { W25Q_CMD_READ_STATUS1, 0x00u };
    uint8_t rx[2] = { 0 };

    cs_low(dev);
    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, SPI_TIMEOUT_MS);
    cs_high(dev);
    return rx[1];
}

bool W25Q16JV_IsBusy(w25q16jv_handle_t *dev)
{
    return (read_status1(dev) & W25Q_STATUS1_BUSY_BIT) != 0u;
}

static void wait_ready(w25q16jv_handle_t *dev, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (W25Q16JV_IsBusy(dev)) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            break; /* el llamador debe revalidar; evita bloqueo infinito del RTOS */
        }
    }
}

bool W25Q16JV_Init(w25q16jv_handle_t *dev)
{
    cs_high(dev);

    uint8_t tx[4] = { W25Q_CMD_JEDEC_ID, 0, 0, 0 };
    uint8_t rx[4] = { 0 };

    cs_low(dev);
    bool ok = (HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 4, SPI_TIMEOUT_MS) == HAL_OK);
    cs_high(dev);

    if (!ok) return false;

    if (rx[1] != W25Q_JEDEC_MANUFACTURER || rx[2] != W25Q_JEDEC_MEMTYPE || rx[3] != W25Q_JEDEC_CAPACITY) {
        return false;
    }

    dev->write_cursor = 0u;
    return true;
}

bool W25Q16JV_EraseChip(w25q16jv_handle_t *dev)
{
    if (!write_enable(dev)) return false;

    uint8_t cmd = W25Q_CMD_CHIP_ERASE;
    cs_low(dev);
    bool ok = (HAL_SPI_Transmit(dev->hspi, &cmd, 1, SPI_TIMEOUT_MS) == HAL_OK);
    cs_high(dev);
    if (!ok) return false;

    wait_ready(dev, ERASE_TIMEOUT_MS);
    dev->write_cursor = 0u;
    return true;
}

bool W25Q16JV_WritePage(w25q16jv_handle_t *dev, uint32_t addr, const uint8_t *data, uint16_t len)
{
    if (len == 0u || len > W25Q_PAGE_SIZE) return false;
    /* Protección de límite de página: si addr+len cruza el borde de 256B,
     * el chip envuelve la escritura dentro de la misma página (comportamiento
     * documentado), corrompiendo el log. Se rechaza explícitamente. */
    if ((addr % W25Q_PAGE_SIZE) + len > W25Q_PAGE_SIZE) return false;
    if (addr + len > W25Q_TOTAL_SIZE_BYTES) return false;

    if (!write_enable(dev)) return false;

    uint8_t header[4] = {
        W25Q_CMD_PAGE_PROGRAM,
        (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr
    };

    cs_low(dev);
    bool ok = (HAL_SPI_Transmit(dev->hspi, header, 4, SPI_TIMEOUT_MS) == HAL_OK);
    ok = ok && (HAL_SPI_Transmit(dev->hspi, (uint8_t *)data, len, SPI_TIMEOUT_MS) == HAL_OK);
    cs_high(dev);

    if (!ok) return false;

    wait_ready(dev, 10u); /* t_pp típico 0.4-3 ms por página */

    if (addr + len > dev->write_cursor) {
        dev->write_cursor = addr + len;
    }
    return true;
}

bool W25Q16JV_ReadData(w25q16jv_handle_t *dev, uint32_t addr, uint8_t *out, uint32_t len)
{
    if (addr + len > W25Q_TOTAL_SIZE_BYTES) return false;

    uint8_t header[4] = {
        W25Q_CMD_READ_DATA,
        (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr
    };

    cs_low(dev);
    bool ok = (HAL_SPI_Transmit(dev->hspi, header, 4, SPI_TIMEOUT_MS) == HAL_OK);
    ok = ok && (HAL_SPI_Receive(dev->hspi, out, len, SPI_TIMEOUT_MS) == HAL_OK);
    cs_high(dev);

    return ok;
}
