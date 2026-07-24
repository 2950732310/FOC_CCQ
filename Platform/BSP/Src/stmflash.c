#include "stmflash.h"
#include <string.h>

/**
 * @brief 判断地址是否在 STM32G431 主 Flash 范围内
 */
static uint8_t stmflash_is_valid_addr(uint32_t addr)
{
    if ((addr >= STM32_FLASH_BASE) && (addr < STM32_FLASH_END))
    {
        return 1;
    }

    return 0;
}

/**
 * @brief 根据地址获取 Flash 页号
 */
static uint32_t stmflash_get_page(uint32_t addr)
{
    return (addr - STM32_FLASH_BASE) / STM32_FLASH_PAGE_SIZE;
}

/**
 * @brief 获取地址所在 Bank
 *
 * 对 STM32G431 128KB 单 Bank 工程，一般就是 FLASH_BANK_1。
 * 这里保留函数，后面如果换成双 Bank 型号，也方便扩展。
 */
static uint32_t stmflash_get_bank(uint32_t addr)
{
    (void)addr;
    return FLASH_BANK_1;
}

/**
 * @brief 从指定地址读取一个 32 位字
 */
uint32_t stmflash_read_word(uint32_t faddr)
{
    return *(volatile uint32_t *)faddr;
}

/**
 * @brief 从指定地址读取指定长度的数据
 * @param raddr  读取起始地址
 * @param pbuf   数据缓冲区
 * @param length 读取的 32 位字数量，不是字节数
 */
void stmflash_read(uint32_t raddr, uint32_t *pbuf, uint32_t length)
{
    uint32_t i;

    for (i = 0; i < length; i++)
    {
        pbuf[i] = stmflash_read_word(raddr);
        raddr += 4;
    }
}

/**
 * @brief 擦除指定地址所在的 Flash 页
 * @param page_addr 页内任意地址都可以
 */
stmflash_status_t stmflash_erase_page(uint32_t page_addr)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0;
    HAL_StatusTypeDef status;

    if (!stmflash_is_valid_addr(page_addr))
    {
        return STMFLASH_ERROR_ADDR;
    }

    HAL_FLASH_Unlock();

    /*
     * 清除可能存在的错误标志。
     * STM32G4 新片或者刚下载后，有时会有 OPTVERR 标志。
     */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks     = stmflash_get_bank(page_addr);
    erase_init.Page      = stmflash_get_page(page_addr);
    erase_init.NbPages   = 1;

    status = HAL_FLASHEx_Erase(&erase_init, &page_error);

    HAL_FLASH_Lock();

    if (status != HAL_OK)
    {
        return STMFLASH_ERROR_ERASE;
    }

    return STMFLASH_OK;
}

/**
 * @brief 擦除一个地址范围覆盖到的所有页
 * @param start_addr    起始地址
 * @param length_bytes  需要擦除的数据长度，单位字节
 */
stmflash_status_t stmflash_erase(uint32_t start_addr, uint32_t length_bytes)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0;
    uint32_t start_page;
    uint32_t end_page;
    uint32_t nb_pages;
    uint32_t end_addr;
    HAL_StatusTypeDef status;

    if (length_bytes == 0)
    {
        return STMFLASH_OK;
    }

    if (!stmflash_is_valid_addr(start_addr))
    {
        return STMFLASH_ERROR_ADDR;
    }

    end_addr = start_addr + length_bytes - 1;

    if (!stmflash_is_valid_addr(end_addr))
    {
        return STMFLASH_ERROR_ADDR;
    }

    start_page = stmflash_get_page(start_addr);
    end_page   = stmflash_get_page(end_addr);
    nb_pages   = end_page - start_page + 1;

    HAL_FLASH_Unlock();

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks     = stmflash_get_bank(start_addr);
    erase_init.Page      = start_page;
    erase_init.NbPages   = nb_pages;

    status = HAL_FLASHEx_Erase(&erase_init, &page_error);

    HAL_FLASH_Lock();

    if (status != HAL_OK)
    {
        return STMFLASH_ERROR_ERASE;
    }

    return STMFLASH_OK;
}

/**
 * @brief 按字节写入 Flash
 *
 * STM32G431 / STM32G4 写 Flash 必须按 double-word，即 8 字节写入。
 * 所以这里会把不足 8 字节的数据补成 0xFF。
 *
 * 注意:
 * 1. 写入前目标区域必须已经擦除。
 * 2. waddr 必须 8 字节对齐。
 * 3. 该函数不会自动擦除。
 */
stmflash_status_t stmflash_write_bytes(uint32_t waddr, const uint8_t *pbuf, uint32_t length_bytes)
{
    HAL_StatusTypeDef status;
    uint32_t i;
    uint32_t double_word_count;
    uint32_t addr;
    uint64_t data64;
    uint32_t copy_len;

    if ((pbuf == NULL) || (length_bytes == 0))
    {
        return STMFLASH_OK;
    }

    if (!stmflash_is_valid_addr(waddr))
    {
        return STMFLASH_ERROR_ADDR;
    }

    if (!stmflash_is_valid_addr(waddr + length_bytes - 1))
    {
        return STMFLASH_ERROR_ADDR;
    }

    /*
     * STM32G4 double-word 编程要求 8 字节对齐。
     */
    if ((waddr % 8U) != 0U)
    {
        return STMFLASH_ERROR_ALIGN;
    }

    double_word_count = (length_bytes + 7U) / 8U;
    addr = waddr;

    HAL_FLASH_Unlock();

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    for (i = 0; i < double_word_count; i++)
    {
        data64 = 0xFFFFFFFFFFFFFFFFULL;

        copy_len = length_bytes - i * 8U;
        if (copy_len > 8U)
        {
            copy_len = 8U;
        }

        memcpy(&data64, &pbuf[i * 8U], copy_len);

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   addr,
                                   data64);

        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return STMFLASH_ERROR_WRITE;
        }

        /*
         * 简单校验
         */
        if (*(volatile uint64_t *)addr != data64)
        {
            HAL_FLASH_Lock();
            return STMFLASH_ERROR_VERIFY;
        }

        addr += 8U;
    }

    HAL_FLASH_Lock();

    return STMFLASH_OK;
}

/**
 * @brief 写入 32 位数据数组
 * @param waddr  写入起始地址，必须 8 字节对齐
 * @param pbuf   数据缓冲区
 * @param length 写入的 32 位字数量，不是字节数
 *
 *
 * 注意:
 * 这个函数会自动擦除覆盖范围内的页。
 */
stmflash_status_t stmflash_write(uint32_t waddr, const uint32_t *pbuf, uint32_t length)
{
    stmflash_status_t ret;
    uint32_t length_bytes;

    if ((pbuf == NULL) || (length == 0))
    {
        return STMFLASH_OK;
    }

    length_bytes = length * 4U;

    /*
     * 先擦除覆盖区域。
     * 注意：Flash 擦除是按页擦，整页都会变成 0xFF。
     * 如果同一页里还有别的数据，会被一起擦掉。
     */
    ret = stmflash_erase(waddr, length_bytes);
    if (ret != STMFLASH_OK)
    {
        return ret;
    }

    return stmflash_write_bytes(waddr, (const uint8_t *)pbuf, length_bytes);
}

/**
 * @brief 测试写入一个 32 位数据
 */
void test_write(uint32_t waddr, uint32_t wdata)
{
    (void)stmflash_write(waddr, &wdata, 1);
}