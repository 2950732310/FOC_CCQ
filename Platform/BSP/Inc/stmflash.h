#ifndef __STMFLASH_H
#define __STMFLASH_H

#include "main.h"
#include <stdint.h>

/**
 * @brief       STM32G431 Flash 定义
 */
#define STM32_FLASH_BASE        0x08000000UL
#define STM32_FLASH_SIZE        (128UL * 1024UL)
#define STM32_FLASH_END         (STM32_FLASH_BASE + STM32_FLASH_SIZE)

/* G431 Flash page size: 2KB */
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE         0x800UL
#endif

#define STM32_FLASH_PAGE_SIZE   FLASH_PAGE_SIZE
#define STM32_FLASH_PAGE_NUM    (STM32_FLASH_SIZE / STM32_FLASH_PAGE_SIZE)

/**
 * @brief       用户区域地址
 */
#define STMFLASH_USER_ADDR      (STM32_FLASH_END - STM32_FLASH_PAGE_SIZE)

/**
 * @brief       Flash 状态枚举
 */
typedef enum
{
    STMFLASH_OK = 0,
    STMFLASH_ERROR_ADDR,
    STMFLASH_ERROR_ALIGN,
    STMFLASH_ERROR_ERASE,
    STMFLASH_ERROR_WRITE,
    STMFLASH_ERROR_VERIFY
} stmflash_status_t;

/**
 * @brief       读取单字数据
 */
uint32_t stmflash_read_word(uint32_t faddr);
void stmflash_read(uint32_t raddr, uint32_t *pbuf, uint32_t length);

/**
 * @brief       擦除单页
 */
stmflash_status_t stmflash_erase_page(uint32_t page_addr);
stmflash_status_t stmflash_erase(uint32_t start_addr, uint32_t length_bytes);

/**
 * @brief       写入单字数据
 */
stmflash_status_t stmflash_write(uint32_t waddr, const uint32_t *pbuf, uint32_t length);
stmflash_status_t stmflash_write_bytes(uint32_t waddr, const uint8_t *pbuf, uint32_t length_bytes);

/**
 * @brief       测试写入单字数据
 */
void test_write(uint32_t waddr, uint32_t wdata);

#endif