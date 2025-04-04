/**
 * @file flash.c
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief
 * @version 1.0
 * @date 24-07_2023
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "flash.h"



static uint32_t GetPage(uint32_t Address)
{
    uint32_t address = Address - (uint32_t)0x08000000U;
    uint32_t mentissa = (uint32_t)(address / 1024U); // Each Sector is 1 KB

    return mentissa;
}

uint32_t Flash_Write_Data(uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords)
{
    static FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PAGEError;
    int sofar = 0;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    /* Erase the user Flash area*/

    uint32_t StartPage = GetPage(StartPageAddress);
    uint32_t EndPageAdress = StartPageAddress + numberofwords * 4;
    uint32_t EndPage = GetPage(EndPageAdress);

    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = StartPageAddress;
    EraseInitStruct.NbPages = ((EndPage - StartPage) / FLASH_PAGE_SIZE) + 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
    {
        /*Error occurred while page erase.*/
        return HAL_FLASH_GetError();
    }

    /* Program the user Flash area word by word*/

    while (sofar < numberofwords)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartPageAddress, (uint32_t)*Data) == HAL_OK)
        {
            StartPageAddress += 4; // use StartPageAddress += 2 for half word and 8 for double word
            Data++;
            sofar++;
        }
        else
        {
            /* Error occurred while writing data in Flash memory*/
            return HAL_FLASH_GetError();
        }
    }

    /* Lock the Flash to disable the flash control register access (recommended
       to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();

    return 0;
}

void Flash_Read_Data(uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
    while (1)
    {
        *RxBuf = *(__IO uint32_t *)StartPageAddress;
        StartPageAddress += 4;
        RxBuf++;
        if (!(numberofwords--))
            break;
    }
}
