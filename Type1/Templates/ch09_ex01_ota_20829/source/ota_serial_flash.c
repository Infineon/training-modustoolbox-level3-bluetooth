/******************************************************************************
* File Name:   app_ota_serial_flash.c
*
* Description: Definitions and data structures for the flash operations of OTA
*
* Related Document: See Readme.md
*
*******************************************************************************
* Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifdef ENABLE_OTA

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#include <stdio.h>
#include <assert.h>
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_log.h"
#include "ota_serial_flash.h"
#include "cybsp_smif_init.h"

/*******************************************************************************
 *                      MACROS / VARIABLE DEFINITIONS
 ******************************************************************************/
#define MEM_SLOT                                    (0u)
#define SET_FLAG(mask)                              (status_flags |= (mask))
#define CLEAR_FLAG(mask)                            (status_flags &= ~(mask))
#define CY_SMIF_BASE_MEM_OFFSET                     CY_XIP_BASE

static volatile uint32_t     status_flags;
extern cy_stc_smif_context_t cybsp_smif_context;

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

static uint32_t ota_smif_get_memory_size(void)
{
    uint32_t size = 0;

    if (SMIF_HW != NULL)
    {
        size = smifBlockConfig.memConfig[MEM_SLOT]->deviceCfg->memSize;
    }

    return size;
}

cy_rslt_t ota_mem_read( cy_ota_mem_type_t mem_type, uint32_t addr, void *data, size_t len )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if( mem_type == CY_OTA_MEM_TYPE_INTERNAL_FLASH )
    {
        (void)result;
        printf( "%s() READ not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
    }
    else if( mem_type == CY_OTA_MEM_TYPE_EXTERNAL_FLASH )
    {
        cy_en_smif_status_t cy_smif_result = CY_SMIF_SUCCESS;
        if (addr >= CY_SMIF_BASE_MEM_OFFSET)
        {
            addr -= CY_SMIF_BASE_MEM_OFFSET;
        }



            cy_smif_result = (cy_rslt_t)Cy_SMIF_MemRead(SMIF_HW, smifBlockConfig.memConfig[MEM_SLOT],
                    addr, data, len, &cybsp_smif_context);

        if (cy_smif_result != CY_SMIF_SUCCESS)
        {
            cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s() FAILED cy_smif_result: 0x%x\n", __func__, cy_smif_result);
        }

        return (cy_smif_result == CY_SMIF_SUCCESS) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
    }
    else
    {
        printf( "%s() READ not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
    }
}

cy_rslt_t ota_mem_write( cy_ota_mem_type_t mem_type, uint32_t addr, void *data, size_t len )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if( mem_type == CY_OTA_MEM_TYPE_INTERNAL_FLASH )
    {
        (void)result;
        printf( "%s() Write not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
    }
    else if( mem_type == CY_OTA_MEM_TYPE_EXTERNAL_FLASH )
    {
        cy_en_smif_status_t cy_smif_result = CY_SMIF_SUCCESS;

        if (addr >= CY_SMIF_BASE_MEM_OFFSET)
        {
            addr -= CY_SMIF_BASE_MEM_OFFSET;
        }

        //printf( "%s() WRITE length 0x%08lx bytes to offset: 0x%08lx \n", __func__, length, offset);


            cy_smif_result = Cy_SMIF_MemWrite(SMIF_HW, smifBlockConfig.memConfig[MEM_SLOT],
                    addr, data, len, &cybsp_smif_context);

        if (cy_smif_result != CY_SMIF_SUCCESS)
        {
            printf("%s() FAILED cy_en_smif_result: [0x%X]\n", __func__, (unsigned int)cy_smif_result);
        }

        return (cy_smif_result == CY_SMIF_SUCCESS) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
    }
    else
    {
        printf( "%s() Write not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
    }
}

cy_rslt_t ota_mem_erase( cy_ota_mem_type_t mem_type, uint32_t addr, size_t len )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if( mem_type == CY_OTA_MEM_TYPE_INTERNAL_FLASH )
    {
        (void)result;
        printf( "%s() Erase not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
    }
    else if( mem_type == CY_OTA_MEM_TYPE_EXTERNAL_FLASH )
    {
        cy_en_smif_status_t cy_smif_result = CY_SMIF_SUCCESS;

        if (addr >= CY_SMIF_BASE_MEM_OFFSET)
        {
            addr -= CY_SMIF_BASE_MEM_OFFSET;
        }

            // If the erase is for the entire chip, use chip erase command
            if ((addr == 0u) && (len == ota_smif_get_memory_size()))
            {
                cy_smif_result = Cy_SMIF_MemEraseChip(SMIF_HW,
                                                    smifBlockConfig.memConfig[MEM_SLOT],
                                                    &cybsp_smif_context);
            }
            else
            {
                // Cy_SMIF_MemEraseSector() returns error if (addr + length) > total flash size or if
                // addr is not aligned to erase sector size or if (addr + length) is not aligned to
                // erase sector size.
                /* Make sure the base offset is correct */
                uint32_t erase_size;
                uint32_t diff;
                erase_size = ota_mem_get_erase_size(CY_OTA_MEM_TYPE_EXTERNAL_FLASH, addr);
                diff = addr & (erase_size - 1);
                addr -= diff;
                len += diff;
                /* Make sure the length is correct */
                len = (len + (erase_size - 1)) & ~(erase_size - 1);
                cy_smif_result = Cy_SMIF_MemEraseSector(SMIF_HW,
                                                      smifBlockConfig.memConfig[MEM_SLOT],
                                                      addr, len, &cybsp_smif_context);
            }

        if (cy_smif_result != CY_SMIF_SUCCESS)
        {
            printf( "%s() FAILED cy_en_smif_result: [0x%X]\n", __func__, (unsigned int)cy_smif_result);
        }
        return (cy_smif_result == CY_SMIF_SUCCESS) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
    }
    else
    {
        printf( "%s() Erase not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
    }
}

size_t ota_mem_get_prog_size ( cy_ota_mem_type_t mem_type, uint32_t addr )
{
    if( mem_type == CY_OTA_MEM_TYPE_INTERNAL_FLASH )
    {
        return 0;
    }
    else if( mem_type == CY_OTA_MEM_TYPE_EXTERNAL_FLASH )
    {
        uint32_t    program_size = 0;
        (void)addr;

            if (SMIF_HW != NULL)
            {
                program_size = smifBlockConfig.memConfig[MEM_SLOT]->deviceCfg->programSize;
            }

        return program_size;
    }
    else
    {
        return 0;
    }
}

size_t ota_mem_get_erase_size ( cy_ota_mem_type_t mem_type, uint32_t addr )
{
    if( mem_type == CY_OTA_MEM_TYPE_INTERNAL_FLASH )
    {
        return 0;
    }
    else if( mem_type == CY_OTA_MEM_TYPE_EXTERNAL_FLASH )
    {
        uint32_t                            erase_sector_size = 0;
        cy_stc_smif_hybrid_region_info_t*   hybrid_info = NULL;
        cy_en_smif_status_t                 smif_status;

        if (addr >= CY_SMIF_BASE_MEM_OFFSET)
        {
            addr -= CY_SMIF_BASE_MEM_OFFSET;
        }

            /* Cy_SMIF_MemLocateHybridRegion() does not access the external flash, just data tables from RAM  */
            smif_status = Cy_SMIF_MemLocateHybridRegion(smifBlockConfig.memConfig[MEM_SLOT], &hybrid_info, addr);

            if (CY_SMIF_SUCCESS != smif_status)
            {
                erase_sector_size = (size_t)smifBlockConfig.memConfig[MEM_SLOT]->deviceCfg->eraseSize;
            }
            else
            {
                erase_sector_size = (size_t)hybrid_info->eraseSize;
            }

        return erase_sector_size;
    }
    else
    {
        return 0;
    }
}

#endif /* end of ENABLE_OTA */
