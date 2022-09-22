/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the QSPI Flash hal Read and Write 
*              example for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
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


#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <inttypes.h>

/*******************************************************************************
* Macros
********************************************************************************/
#define PACKET_SIZE             (64u)     /* Memory Read/Write size */


/* Used when an array of data is printed on the console */
#define NUM_BYTES_PER_LINE      (16u)
#define LED_TOGGLE_DELAY_MSEC   (1000u)   /* LED blink delay */
#define QSPI_BUS_FREQUENCY_HZ   (50000000lu)
#define FLASH_DATA_AFTER_ERASE  (0xFFu)   /* Flash data after erase */
#define ADDRESS                 (0u)

#define DEVICE_SPECIFIC_RDSR1_COMMAND                  (0x05)
#define DEVICE_SPECIFIC_WREN_COMMAND                   (0x06)
#define DEVICE_SPECIFIC_4SE_COMMAND                    (0xdc)
#define DEVICE_SPECIFIC_4QPP_COMMAND                   (0x34)
#define DEVICE_SPECIFIC_4QIOR_COMMAND                  (0xEC)

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void qspi_wait_mem_free(void);

/*******************************************************************************
* Global Variables
********************************************************************************/
static cyhal_qspi_t qspi_obj;
static uint8_t qspi_mem_sta;

/* read status command structure */
cyhal_qspi_command_t device_specific_rdsr1_command =
{
    .instruction.bus_width      = CYHAL_QSPI_CFG_BUS_SINGLE,    /* Bus width for the instruction */
    .instruction.data_rate      = CYHAL_QSPI_DATARATE_SDR,      /* Data rate for instruction (SDR/DDR) */
    .instruction.two_byte_cmd   = false,                        /* command is 1-byte value */
    .instruction.value          = DEVICE_SPECIFIC_RDSR1_COMMAND,/* Instruction value */
    .instruction.disabled       = false,                        /* Instruction phase skipped if disabled, set to false */                                                               
    
    .address.bus_width          = CYHAL_QSPI_CFG_BUS_SINGLE,    /* Bus width for the address */
    .address.data_rate          = CYHAL_QSPI_DATARATE_SDR,      /* Data rate for address (SDR/DDR) */
    .address.size               = CYHAL_QSPI_CFG_SIZE_8,        /* Address size in bits */
    .address.disabled           = true,                         /* Address phase skipped if disabled, set to true */

    .mode_bits.disabled         = true,                         /* Mode bits phase skipped if disabled, set to true */
    .dummy_cycles.dummy_count   = 0u,                           /* Dummy cycles count */
    
    .data.bus_width             = CYHAL_QSPI_CFG_BUS_SINGLE,    /* Bus width for data */
    .data.data_rate             = CYHAL_QSPI_DATARATE_SDR,      /* Data rate for data (SDR/DDR) */
};

/* Write enable command structure */
cyhal_qspi_command_t device_specific_wren_command =
{
    .instruction.bus_width      = CYHAL_QSPI_CFG_BUS_SINGLE,    /* Bus width for the instruction */
    .instruction.data_rate      = CYHAL_QSPI_DATARATE_SDR,      /* Data rate for instruction (SDR/DDR) */
    .instruction.two_byte_cmd   = false,                        /* command is 1-byte value */
    .instruction.value          = DEVICE_SPECIFIC_WREN_COMMAND, /* Instruction value */
    .instruction.disabled       = false,                        /* Instruction phase skipped if disabled, set to false */
    
    .address.bus_width          = CYHAL_QSPI_CFG_BUS_SINGLE,    /* Bus width for the address */
    .address.data_rate          = CYHAL_QSPI_DATARATE_SDR,      /* Data rate for address (SDR/DDR) */
    .address.size               = CYHAL_QSPI_CFG_SIZE_8,        /* Address size in bits */
    .address.disabled           = true,                         /* Address phase skipped if disabled, set to true */
    
    .mode_bits.disabled         = true,                         /* Mode bits phase skipped if disabled, set to true */
    .dummy_cycles.dummy_count   = 0u,                           /* Dummy cycles count */
    
    .data.bus_width             = CYHAL_QSPI_CFG_BUS_SINGLE,    /* Bus width for data */
    .data.data_rate             = CYHAL_QSPI_DATARATE_SDR,      /* Data rate for data (SDR/DDR) */
};

/* sector erase command structure */
cyhal_qspi_command_t device_specific_4se_command =
{
    .instruction.bus_width      = CYHAL_QSPI_CFG_BUS_SINGLE,    /* Bus width for the instruction */
    .instruction.data_rate      = CYHAL_QSPI_DATARATE_SDR,      /* Data rate for instruction (SDR/DDR) */
    .instruction.two_byte_cmd   = false,                        /* command is 1-byte value */
    .instruction.value          = DEVICE_SPECIFIC_4SE_COMMAND,  /* Instruction value */
    .instruction.disabled       = false,                        /* Instruction phase skipped if disabled, set to true */
   
    .address.bus_width          = CYHAL_QSPI_CFG_BUS_SINGLE,    /* Bus width for the address */
    .address.data_rate          = CYHAL_QSPI_DATARATE_SDR,      /* Data rate for address (SDR/DDR) */
    .address.size               = CYHAL_QSPI_CFG_SIZE_32,       /* Address size in bits */
    .address.disabled           = false,                        /* Address phase skipped if disabled, set to false */
    
    .mode_bits.disabled         = true,                         /* Mode bits phase skipped if disabled, set to true */
    .dummy_cycles.dummy_count   = 0u,                           /* Dummy cycles count */
    
    .data.bus_width             = CYHAL_QSPI_CFG_BUS_SINGLE,    /* Bus width for data */
    .data.data_rate             = CYHAL_QSPI_DATARATE_SDR,      /* Data rate for data (SDR/DDR) */
};

/* Quad Page Program command structure */
cyhal_qspi_command_t device_specific_4qpp_command =
{
    .instruction.bus_width      = CYHAL_QSPI_CFG_BUS_SINGLE,    /* Bus width for the instruction */
    .instruction.data_rate      = CYHAL_QSPI_DATARATE_SDR,      /* Data rate for instruction (SDR/DDR) */
    .instruction.two_byte_cmd   = false,                        /* command is 1-byte value */ 
    .instruction.value          = DEVICE_SPECIFIC_4QPP_COMMAND, /* Instruction value */
    .instruction.disabled       = false,                        /* Instruction phase skipped if disabled, set to false */

    .address.bus_width          = CYHAL_QSPI_CFG_BUS_SINGLE,    /* Bus width for the address */
    .address.data_rate          = CYHAL_QSPI_DATARATE_SDR,      /* Data rate for address (SDR/DDR) */
    .address.size               = CYHAL_QSPI_CFG_SIZE_32,       /* Address size in bits */
    .address.disabled           = false,                        /* Address phase skipped if disabled, set to true */

    .mode_bits.disabled         = true,                         /* Mode bits phase skipped if disabled, set to true */
    .dummy_cycles.dummy_count   = 0u,                           /* Dummy cycles count */
    
    .data.bus_width             = CYHAL_QSPI_CFG_BUS_QUAD,      /* Bus width for data */
    .data.data_rate             = CYHAL_QSPI_DATARATE_SDR,      /* Data rate for data (SDR/DDR) */
};

/* Quad I/O Read command structure */
cyhal_qspi_command_t device_specific_4qior_command =
{
    .instruction.bus_width      = CYHAL_QSPI_CFG_BUS_SINGLE,     /* Bus width for the instruction */
    .instruction.data_rate      = CYHAL_QSPI_DATARATE_SDR,       /* Data rate for instruction (SDR/DDR) */
    .instruction.two_byte_cmd   = false,                         /* command is 1-byte value */
    .instruction.value          = DEVICE_SPECIFIC_4QIOR_COMMAND, /* Instruction value */
    .instruction.disabled       = false,                         /* Instruction phase skipped if disabled, set to false */
    
    .address.bus_width          = CYHAL_QSPI_CFG_BUS_QUAD,       /* Bus width for the address */
    .address.data_rate          = CYHAL_QSPI_DATARATE_SDR,       /* Data rate for address (SDR/DDR) */
    .address.size               = CYHAL_QSPI_CFG_SIZE_32,        /* Address size in bits */
    .address.disabled           = false,                         /* Address phase skipped if disabled, set to false */
   
    .mode_bits.data_rate        = CYHAL_QSPI_DATARATE_SDR,
    .mode_bits.size             = CYHAL_QSPI_CFG_SIZE_8,
    .mode_bits.disabled         = false,                          /* Mode bits phase skipped if disabled, set to false */

    /* The 8-bit mode byte. This value is 0xFFFFFFFF when there is no mode present. */
    .mode_bits.value            = 0xFFFFFFFFU,
    .mode_bits.bus_width        = CYHAL_QSPI_CFG_BUS_QUAD,

    .dummy_cycles.dummy_count   = 4u,                             /* Dummy cycles count */
    .dummy_cycles.bus_width     = CYHAL_QSPI_CFG_BUS_SINGLE,

    .data.bus_width             = CYHAL_QSPI_CFG_BUS_QUAD,        /* Bus width for data */
    .data.data_rate             = CYHAL_QSPI_DATARATE_SDR,        /* Data rate for data (SDR/DDR) */
};


/*******************************************************************************
* Function Name: check_status
****************************************************************************//**
* Summary:
*  Prints the message, indicates the non-zero status by turning the LED on, and
*  asserts the non-zero status.
*
* Parameters:
*  message - message to print if status is non-zero.
*  status - status for evaluation.
*
*******************************************************************************/
void check_status(char *message, uint32_t status)
{
    if (0u != status)
    {
        printf("\r\n================================================================================\r\n");
        printf("\nFAIL: %s\r\n", message);
        printf("Error Code: 0x%08"PRIX32"\n", status);
        printf("\r\n================================================================================\r\n");

        /* On failure, turn the LED ON */
        cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
        CY_ASSERT(false);
    }
}

/*******************************************************************************
* Function Name: print_array
****************************************************************************//**
* Summary:
*  Prints the content of the buffer to the UART console.
*
* Parameters:
*  message - message to print before array output
*  buf - buffer to print on the console.
*  size - size of the buffer.
*
*******************************************************************************/
void print_array(char *message, uint8_t *buf, uint32_t size)
{
    printf("\r\n%s (%"PRIu32" bytes):\r\n", message, size);
    printf("-------------------------\r\n");

    for (uint32_t index = 0; index < size; index++)
    {
        printf("0x%02X ", buf[index]);

        if (0u == ((index + 1) % NUM_BYTES_PER_LINE))
        {
            printf("\r\n");
        }
    }
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  This is the main function for CM7_0 CPU. It does...
*  1. Initializes UART for console output and SMIF for interfacing a QSPI flash.
*  2. Performs erase followed by write and verifies the written data by
*     reading it back.
*
* Parameters:
*  None
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint8_t tx_buf[PACKET_SIZE];
    uint8_t rx_buf[PACKET_SIZE];
    size_t size = PACKET_SIZE;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);
    
    /* Enable global interrupts */
    __enable_irq();
    
    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("*************** HAL: QSPI Flash Read and Write ***************\r\n\n");
    
    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
              CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    check_status("User LED initialization failed", result);

    /* QSPI parameter initialize */
    cyhal_qspi_slave_pin_config_t memory_pin_set =
    {
        .io   = { CYBSP_QSPI_D0,
                CYBSP_QSPI_D1, CYBSP_QSPI_D2, CYBSP_QSPI_D3, NC, NC, NC, NC, },
        .ssel = CYBSP_QSPI_SS
    };
    /* QSPI device initialize */
    result = cyhal_qspi_init(&qspi_obj, CYBSP_QSPI_SCK, &memory_pin_set, QSPI_BUS_FREQUENCY_HZ, 0, NULL);
    check_status("Serial Flash initialization failed", result);

    /* Write enable */
    result = cyhal_qspi_transfer(&qspi_obj, &device_specific_wren_command, 0, NULL, 0, \
                                      NULL, 0);
    check_status("Writing to memory failed", result);

    /* sector erase */
    printf("\r\n1.Erasing \r\n");
    result = cyhal_qspi_transfer(&qspi_obj, &device_specific_4se_command, ADDRESS, NULL, 0, \
                                     NULL, 0);
    /* Wait memory free */
    qspi_wait_mem_free();

    /* sector read */
    printf("\r\n2. Reading after Erase and verifying that each byte is 0xFF\r\n");
    result = cyhal_qspi_read(&qspi_obj, &device_specific_4qior_command, ADDRESS, rx_buf, &size);
    check_status("Writing to memory failed", result);
    print_array("Received Data", rx_buf, PACKET_SIZE);
    memset(tx_buf, FLASH_DATA_AFTER_ERASE, PACKET_SIZE);
    check_status("Flash contains data other than 0xFF after erase",
                 memcmp(tx_buf, rx_buf, PACKET_SIZE));

    /* Prepare the TX buffer */
    for (uint32_t index = 0; index < PACKET_SIZE; index++)
    {
        tx_buf[index] = (uint8_t)index;
    }

    /* Write enable */
    result = cyhal_qspi_transfer(&qspi_obj, &device_specific_wren_command, 0, NULL, 0, \
                                     NULL, 0);
    check_status("Writing to memory failed", result);

    /* Write the content of the TX buffer to the memory */
    printf("\r\n3. Writing data to memory\r\n");
    /* Write page */
    result = cyhal_qspi_write(&qspi_obj, &device_specific_4qpp_command, ADDRESS, tx_buf, &size);
    check_status("Writing to memory failed", result);
    print_array("Written Data", tx_buf, PACKET_SIZE);

    /* Wait memory free */
    qspi_wait_mem_free();

    /* Read back after Write for verification */
    printf("\r\n4. Reading back for verification\r\n");
    /* read page */
    result = cyhal_qspi_read(&qspi_obj, &device_specific_4qior_command, ADDRESS, rx_buf, &size);
    check_status("Writing to memory failed", result);
    print_array("Received Data", rx_buf, PACKET_SIZE);

    check_status("Read data does not match with written data. Read/Write operation failed.",
           memcmp(tx_buf, rx_buf, PACKET_SIZE));

    printf("\r\n================================================================================\r\n");
    printf("\r\nSUCCESS: Read data matches with written data!\r\n");
    printf("\r\n================================================================================\r\n");

    for (;;)
    {
        cyhal_gpio_toggle(CYBSP_USER_LED);
        cyhal_system_delay_ms(LED_TOGGLE_DELAY_MSEC);
    }
}

/*******************************************************************************
* Function Name: qspi_wait_mem_free
****************************************************************************//**
* Summary:
*  waiting for qspi flash free.
*
* Parameters:
*  None
*
* Return:
*  none
*
*******************************************************************************/
static void qspi_wait_mem_free(void)
{
    cy_rslt_t result;
    do 
    {
        result = cyhal_qspi_transfer(&qspi_obj, &device_specific_rdsr1_command, 0, NULL, 0, \
                                     &qspi_mem_sta, 1u);
        check_status("read status failed", result);
    } while(qspi_mem_sta!=0);
}
/* [] END OF FILE */


