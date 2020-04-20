/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017,2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_hid.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (defined(FSL_FEATURE_SOC_USBHSDCD_COUNT) && (FSL_FEATURE_SOC_USBHSDCD_COUNT > 0U))
#include "usb_hsdcd.h"
#endif
#include "mouse.h"
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U))
#include "usb_phydcd.h"
#endif
#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"

#include <stdio.h>
#include <stdlib.h>
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

#if ((defined FSL_FEATURE_SOC_USBPHY_COUNT) && (FSL_FEATURE_SOC_USBPHY_COUNT > 0U))
#include "usb_phy.h"
#endif

#include "pin_mux.h"

#include "hid_keyboard.h"	// Table of characters to send

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_InitHardware(void);
void USB_DeviceClockInit(void);
void USB_DeviceIsrEnable(void);
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTaskFn(void *deviceHandle);
#endif

static usb_status_t USB_DeviceHidMouseAction(void);
static usb_status_t USB_DeviceHidMouseCallback(class_handle_t handle, uint32_t event, void *param);
static usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param);
static void USB_DeviceApplicationInit(void);
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U))
extern void HW_TimerControl(uint8_t enable);
#endif
/*******************************************************************************
 * Variables
 ******************************************************************************/

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_MouseBuffer[USB_HID_KEYBOARD_REPORT_LENGHT];
usb_hid_mouse_struct_t g_UsbDeviceHidMouse;

extern usb_device_class_struct_t g_UsbDeviceHidMouseConfig;

/* Set class configurations */
usb_device_class_config_struct_t g_UsbDeviceHidConfig[1] = {{
    USB_DeviceHidMouseCallback, /* HID mouse class callback pointer */
    (class_handle_t)NULL,       /* The HID class handle, This field is set by USB_DeviceClassInit */
    &g_UsbDeviceHidMouseConfig, /* The HID mouse configuration, including class code, subcode, and protocol, class type,
                           transfer type, endpoint address, max packet size, etc.*/
}};

/* Set class configuration list */
usb_device_class_config_list_struct_t g_UsbDeviceHidConfigList = {
    g_UsbDeviceHidConfig, /* Class configurations */
    USB_DeviceCallback,   /* Device callback pointer */
    1U,                   /* Class count */
};

/*******************************************************************************
 * Code
 ******************************************************************************/
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U))
void USBHS_IRQHandler(void)
{
    USB_DeviceEhciIsrFunction(g_UsbDeviceHidMouse.deviceHandle);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
    exception return operation might vector to incorrect interrupt */
    __DSB();
}
#endif
#if (defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U))
void USB0_IRQHandler(void)
{
    USB_DeviceKhciIsrFunction(g_UsbDeviceHidMouse.deviceHandle);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
    exception return operation might vector to incorrect interrupt */
    __DSB();
}
#endif
void USB_DeviceClockInit(void)
{
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL,
        BOARD_USB_PHY_TXCAL45DP,
        BOARD_USB_PHY_TXCAL45DM,
    };
#endif
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
    CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_UsbPhySrcExt, BOARD_XTAL0_CLK_HZ);
    CLOCK_EnableUsbhs0Clock(kCLOCK_UsbSrcUnused, 0U);
    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);
#endif
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U)
    SystemCoreClockUpdate();
    CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcIrc48M, 48000000U);
/*
 * If the SOC has USB KHCI dedicated RAM, the RAM memory needs to be clear after
 * the KHCI clock is enabled. When the demo uses USB EHCI IP, the USB KHCI dedicated
 * RAM can not be used and the memory can't be accessed.
 */
#if (defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U))
#if (defined(FSL_FEATURE_USB_KHCI_USB_RAM_BASE_ADDRESS) && (FSL_FEATURE_USB_KHCI_USB_RAM_BASE_ADDRESS > 0U))
    for (int i = 0; i < FSL_FEATURE_USB_KHCI_USB_RAM; i++)
    {
        ((uint8_t *)FSL_FEATURE_USB_KHCI_USB_RAM_BASE_ADDRESS)[i] = 0x00U;
    }
#endif /* FSL_FEATURE_USB_KHCI_USB_RAM_BASE_ADDRESS */
#endif /* FSL_FEATURE_USB_KHCI_USB_RAM */
#endif
}
void USB_DeviceIsrEnable(void)
{
    uint8_t irqNumber;
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
    uint8_t usbDeviceEhciIrq[] = USBHS_IRQS;
    irqNumber                  = usbDeviceEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];
#endif
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U)
    uint8_t usbDeviceKhciIrq[] = USB_IRQS;
    irqNumber                  = usbDeviceKhciIrq[CONTROLLER_ID - kUSB_ControllerKhci0];
#endif
    /* Install isr, set priority, and enable IRQ. */
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
    EnableIRQ((IRQn_Type)irqNumber);
}
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTaskFn(void *deviceHandle)
{
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
    USB_DeviceEhciTaskFunction(deviceHandle);
#endif
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U)
    USB_DeviceKhciTaskFunction(deviceHandle);
#endif
}
#endif

/* Update mouse pointer location. Draw a rectangular rotation*/
static usb_status_t USB_DeviceHidMouseAction(void)
{
    static int8_t x = 0U;	// Aux for Delay
    static int8_t x_axis = 0U;	// Aux for decide side of the Square
    static int8_t y_axis = 0U;	// Aux for decide side of the Square


    enum
    {
    	INICIO1,	// Verify state change in correct delay	(Print only 1 key)
		INICIO2,	// Verify state change in correct delay (Print only 1 key)

    	MinimizaAll,// Windows + M	// (keyboard)
    	WINDOWS_1,	// Ctrl + Esc	// (keyboard)
		key_P,						// (keyboard)
		key_A,						// (keyboard)
		key_I,						// (keyboard)
		key_N,						// (keyboard)
		ENTER_1,	// Open Paint	// (keyboard)
		PUNTO_PAINT1,// Start drawing	// (mouse)
		PUNTO_PAINT2,				// (mouse)
        RIGHT,						// (mouse)
        DOWN,						// (mouse)
        LEFT,						// (mouse)
        UP,							// (mouse)
		PUNTO_PAINT3,				// (mouse)
		PUNTO_PAINT4,// Finish Square	// (mouse)
		SAVE_1,		// Ctrl + S		// (keyboard)
		BORRA_1,					// (keyboard)
		NAME_1,						// (keyboard)
		ENTER_2,					// (keyboard)
		WINDOWS_2,	// Ctrl + Esc	// (keyboard)
		key1_N,						// (keyboard)
		key1_O,						// (keyboard)
		key1_T,						// (keyboard)
		key1_E,						// (keyboard)
		ENTER_3,	// Open 1st Notes	// (keyboard)
		I,			// For the expedient number:
		E,			// Ie717807 = Crack!
		firstDigit__7,
		secondDigit_1,
		thirdDigit__7,
		fourthDigit_8,
		fifthDigit__0,
		sixthDigit__7,
		ENTER_4,
		SEL_TEXT,	// Ctrl + A		// (keyboard)
		COPIAR,		// Ctrl + C		// (keyboard)
		PANTALLA_IZQ,	// Windows + left
		WINDOWS_3,	// Ctrl + Esc	// (keyboard)
		key2_N,						// (keyboard)
		key2_O,						// (keyboard)
		key2_T,						// (keyboard)
		key2_E,						// (keyboard)
		ENTER_5,	// Open 2nd Notes	// (keyboard)
		PANTALLA_DER,	// Windows + right
		PEGAR,		// Ctrl + V		// (keyboard)
		F5,			// Hora actual	// (keyboard)
		SAVE_2,		// Ctrl + S		// (keyboard)
		BORRA_2,					// (keyboard)
		NAME_2,						// (keyboard)
		ENTER_6,					// (keyboard)

		FIN1,		// Cycle infinite Minimize all:
		FIN2		// Windows + M	// (keyboard)
    };

    const uint8_t delay_max = 60U;
    const uint8_t delay_min =  2U;

    const uint8_t ID_1 = 0x01U;	// Report ID 1 (mouse)
    const uint8_t ID_2 = 0x02U;	// Report ID 2 (keyboard)

    uint32_t USB_HID_REPORT_LENGTH = 0x00;	// For ID -> (mouse) or (keyboard)

    static uint8_t dir = INICIO1;		// To start!

    switch (dir)
    {
		case INICIO1:
			/* Move right. Increase X value. */
			x++;
			if (x > delay_max)
			{
				dir = INICIO2;
				g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_1_EXCLAMATION_MARK;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
        case INICIO2:
            /* Move left. Discrease X value. */
            x--;
            if (x < delay_min)
            {
                dir = MinimizaAll;
            	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
    			g_UsbDeviceHidMouse.buffer[3] = KEY_2_AT;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
            }
            break;
        case MinimizaAll:
            /* Move left. Discrease X value. */
			x++;
			if (x > delay_max)
            {
                dir = WINDOWS_1;
            	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = MODIFERKEYS_LEFT_GUI;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
            	g_UsbDeviceHidMouse.buffer[3] = KEY_M;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
            }
            break;
		case WINDOWS_1:
            x--;
            if (x < delay_min)
            {
				dir = key_P;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = MODIFERKEYS_LEFT_CTRL;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_ESCAPE;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case key_P:
			x++;
			if (x > delay_max)
			{
				dir = key_A;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_P;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case key_A:
            x--;
            if (x < delay_min)
            {
				dir = ENTER_1;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_A;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;

			// Agregar palabra completa
			//
			//

		case ENTER_1:
			x++;
			if (x > delay_max)
			{
				dir = PUNTO_PAINT1;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_ENTER;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case PUNTO_PAINT1:
            x--;
            if (x < delay_min)
            {
				dir = PUNTO_PAINT2;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_1;	// Report ID (mouse)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x01U;	// Buttons + no press left
	        	g_UsbDeviceHidMouse.buffer[2] = 0U;		// X
	            g_UsbDeviceHidMouse.buffer[3] = 0U;		// Y
	            g_UsbDeviceHidMouse.buffer[4] = 0U; 	// Z
			}
			break;
		case PUNTO_PAINT2:
			x++;
			if (x > delay_max)
			{
				dir = RIGHT;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_1;	// Report ID (mouse)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Buttons + no press left
	        	g_UsbDeviceHidMouse.buffer[2] = 0U;		// X
	            g_UsbDeviceHidMouse.buffer[3] = 0U;		// Y
	            g_UsbDeviceHidMouse.buffer[4] = 0U; 	// Z
			}
			break;
        case RIGHT:
            /* Move right. Increase X value. */
        	g_UsbDeviceHidMouse.buffer[0] = ID_1;	// Report ID (mouse)
        	g_UsbDeviceHidMouse.buffer[1] = 0x01U;	// Buttons + press left
        	g_UsbDeviceHidMouse.buffer[2] = 2U;		// X
            g_UsbDeviceHidMouse.buffer[3] = 0U;		// Y
            g_UsbDeviceHidMouse.buffer[4] = 0U; 	// Z
            x_axis++;
            if (x_axis > 99U)
            {
            	dir = DOWN;
            }
            break;
        case DOWN:
            /* Move down. Increase Y value. */
        	g_UsbDeviceHidMouse.buffer[0] = ID_1;	// Report ID (mouse)
        	g_UsbDeviceHidMouse.buffer[1] = 0x01U;	// Buttons + no press left
        	g_UsbDeviceHidMouse.buffer[2] = 0U;		// X
            g_UsbDeviceHidMouse.buffer[3] = 2U;		// Y
            g_UsbDeviceHidMouse.buffer[4] = 0U; 	// Z
            y_axis++;
            if (y_axis > 99U)
            {
            	dir = LEFT;
            }
            break;
        case LEFT:
            /* Move left. Discrease X value. */
        	g_UsbDeviceHidMouse.buffer[0] = ID_1;	// Report ID (mouse)
        	g_UsbDeviceHidMouse.buffer[1] = 0x01U;	// Buttons + press left
        	g_UsbDeviceHidMouse.buffer[2] = (uint8_t)(-2);// X
            g_UsbDeviceHidMouse.buffer[3] = 0U;		// Y
            g_UsbDeviceHidMouse.buffer[4] = 0U; 	// Z
            x_axis--;
            if (x_axis < 2U)
            {
            	dir = UP;
            }
            break;
        case UP:
            /* Move up. Discrease Y value. */
        	g_UsbDeviceHidMouse.buffer[0] = ID_1;	// Report ID (mouse)
        	g_UsbDeviceHidMouse.buffer[1] = 0x01U;	// Buttons + press left
        	g_UsbDeviceHidMouse.buffer[2] = 0U;		// X
            g_UsbDeviceHidMouse.buffer[3] = (uint8_t)(-2);// Y
            g_UsbDeviceHidMouse.buffer[4] = 0U; 	// Z
            y_axis--;
            if  (y_axis < 59U)	// (y < 2U)
            {
                dir = PUNTO_PAINT3;
                x_axis = 0U;	// Restart figure values
                y_axis = 0U;	// Restart figure values
            }
            break;
		case PUNTO_PAINT3:
            x--;
            if (x < delay_min)
            {
				dir = PUNTO_PAINT4;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_1;	// Report ID (mouse)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Buttons + no press left
	        	g_UsbDeviceHidMouse.buffer[2] = 0U;		// X
	            g_UsbDeviceHidMouse.buffer[3] = 0U;		// Y
	            g_UsbDeviceHidMouse.buffer[4] = 0U; 	// Z
			}
			break;
		case PUNTO_PAINT4:
			x++;
			if (x > delay_max)
			{
				dir = SAVE_1;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_1;	// Report ID (mouse)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Buttons + no press left
	        	g_UsbDeviceHidMouse.buffer[2] = 0U;		// X
	            g_UsbDeviceHidMouse.buffer[3] = 0U;		// Y
	            g_UsbDeviceHidMouse.buffer[4] = 0U; 	// Z
			}
			break;
		case SAVE_1:
            x--;
            if (x < delay_min)
            {
				dir = BORRA_1;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = MODIFERKEYS_LEFT_CTRL;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_S;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case BORRA_1:
			x++;
			if (x > delay_max)
			{
				dir = NAME_1;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_BACKSPACE;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case NAME_1:
            x--;
            if (x < delay_min)
            {
				dir = ENTER_2;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_A;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case ENTER_2:
			x++;
			if (x > delay_max)
			{
// Debug Figure into a infinity cycle
				dir = PUNTO_PAINT1;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_ENTER;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case WINDOWS_2:
            x--;
            if (x < delay_min)
            {
				dir = key1_N;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = MODIFERKEYS_LEFT_CTRL;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_ESCAPE;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case key1_N:
			x++;
			if (x > delay_max)
			{
				dir = key1_O;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_N;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case key1_O:
            x--;
            if (x < delay_min)
            {
				dir = ENTER_3;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_O;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;

			// Agregar palabra completa
			//
			//

		case ENTER_3:
			x++;
			if (x > delay_max)
			{
				dir = I;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_ENTER;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case I:
            x--;
            if (x < delay_min)
            {
				dir = E;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[3] = KEY_I;
			}
			break;
		case E:
			x++;
			if (x > delay_max)
			{
				dir = firstDigit__7;	// Next digit
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[3] = KEY_E;
			}
			break;
		case firstDigit__7:
            x--;
            if (x < delay_min)
            {
				dir = secondDigit_1;	// Next digit
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[3] = KEY_7_AMPERSAND;
			}
			break;
		case secondDigit_1:
			x++;
			if (x > delay_max)
			{
				dir = thirdDigit__7;		// To start of the ENUM !
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[3] = KEY_1_EXCLAMATION_MARK;
			}
			break;
		case thirdDigit__7:
            x--;
            if (x < delay_min)
            {
				dir = fourthDigit_8;	// Next digit
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[3] = KEY_7_AMPERSAND;
			}
			break;
		case fourthDigit_8:
			x++;
			if (x > delay_max)
			{
				dir = fifthDigit__0;	// Next digit
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[3] = KEY_8_ASTERISK;
			}
			break;
		case fifthDigit__0:
            x--;
            if (x < delay_min)
            {
				dir = sixthDigit__7;	// Next digit
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[3] = KEY_0_CPARENTHESIS;
			}
			break;
		case sixthDigit__7:
			x++;
			if (x > delay_max)
			{
				dir = ENTER_4;	// To final command
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[3] = KEY_7_AMPERSAND;
			}
			break;

		case ENTER_4:
            x--;
            if (x < delay_min)
            {
				dir = SEL_TEXT;		// To start of the ENUM !
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[3] = KEY_ENTER;
			}
			break;
		case SEL_TEXT:
			x++;
			if (x > delay_max)
			{
				dir = COPIAR;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = MODIFERKEYS_LEFT_CTRL;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_A;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case COPIAR:
            x--;
            if (x < delay_min)
            {
				dir = PANTALLA_IZQ;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = MODIFERKEYS_LEFT_CTRL;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_C;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case PANTALLA_IZQ:
			x++;
			if (x > delay_max)
			{
				dir = WINDOWS_3;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = MODIFERKEYS_LEFT_GUI;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_LEFTARROW;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case WINDOWS_3:
            x--;
            if (x < delay_min)
			{
				dir = key2_N;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = MODIFERKEYS_LEFT_CTRL;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_ESCAPE;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case key2_N:
			x++;
			if (x > delay_max)
            {
				dir = key2_O;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_N;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case key2_O:
            x--;
            if (x < delay_min)
            {
				dir = ENTER_5;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_O;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;

			// Agregar palabra completa
			//
			//

		case ENTER_5:
			x++;
			if (x > delay_max)
            {
				dir = PANTALLA_DER;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_ENTER;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case PANTALLA_DER:
            x--;
            if (x < delay_min)
			{
				dir = PEGAR;	// Next key
				g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
				g_UsbDeviceHidMouse.buffer[1] = MODIFERKEYS_LEFT_GUI;	// Modifier (Ctrl + ...)
				g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_RIGHTARROW;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case PEGAR:
			x++;
			if (x > delay_max)
			{
				dir = F5;	// Next key
				g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
				g_UsbDeviceHidMouse.buffer[1] = MODIFERKEYS_LEFT_CTRL;	// Modifier (Ctrl + ...)
				g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_V;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case F5:
            x--;
            if (x < delay_min)
			{
				dir = SAVE_2;	// Next key
				g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
				g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
				g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_F5;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case SAVE_2:
			x++;
			if (x > delay_max)
            {
				dir = BORRA_2;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = MODIFERKEYS_LEFT_CTRL;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_S;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case BORRA_2:
            x--;
            if (x < delay_min)
            {
				dir = NAME_2;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_BACKSPACE;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case NAME_2:
			x++;
			if (x > delay_max)
            {
				dir = ENTER_6;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_A;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case ENTER_6:
            x--;
            if (x < delay_min)
			{
				dir = FIN1;	// Next key
				g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
				g_UsbDeviceHidMouse.buffer[1] = 0x00U;	// Modifier (Ctrl + ...)
				g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_ENTER;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key

				x = 0U;			// Restart x value because is the compare of "Final Delay"
			}
			break;
		case FIN1:
			x++;
			if (x > delay_max)
            {
				dir = FIN2;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = MODIFERKEYS_LEFT_GUI;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_M;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;
		case FIN2:
            x--;
            if (x < delay_min)
            {
				dir = FIN1;	// Next key
	        	g_UsbDeviceHidMouse.buffer[0] = ID_2;	// Report ID (keyboard)
	        	g_UsbDeviceHidMouse.buffer[1] = MODIFERKEYS_LEFT_GUI;	// Modifier (Ctrl + ...)
	        	g_UsbDeviceHidMouse.buffer[2] = 0x00U;	// Reserved
				g_UsbDeviceHidMouse.buffer[3] = KEY_M;	// key
				g_UsbDeviceHidMouse.buffer[4] = 0x00U; 	// key
				g_UsbDeviceHidMouse.buffer[5] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[6] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[7] = 0x00U;	// key
				g_UsbDeviceHidMouse.buffer[8] = 0x00U;	// key
			}
			break;

        default:
            break;
    }

    if (g_UsbDeviceHidMouse.buffer[0] == ID_1) {
    	USB_HID_REPORT_LENGTH = USB_HID_MOUSE_REPORT_LENGTH;
	}

    if (g_UsbDeviceHidMouse.buffer[0] == ID_2) {
    	USB_HID_REPORT_LENGTH = USB_HID_KEYBOARD_REPORT_LENGHT;
	}

    /* Send mouse report to the host */
    return USB_DeviceHidSend(g_UsbDeviceHidMouse.hidHandle, USB_HID_MOUSE_ENDPOINT_IN, g_UsbDeviceHidMouse.buffer,
    		USB_HID_REPORT_LENGTH);
}

/* The hid class callback */
static usb_status_t USB_DeviceHidMouseCallback(class_handle_t handle, uint32_t event, void *param)
{
    usb_status_t error                                     = kStatus_USB_Error;
    usb_device_endpoint_callback_message_struct_t *message = (usb_device_endpoint_callback_message_struct_t *)param;

    switch (event)
    {
        case kUSB_DeviceHidEventSendResponse:
            /* Resport sent */
            if (g_UsbDeviceHidMouse.attach)
            {
                if ((NULL != message) && (message->length == USB_UNINITIALIZED_VAL_32))
                {
                    return error;
                }
                error = USB_DeviceHidMouseAction();
            }
            break;
        case kUSB_DeviceHidEventGetReport:
        case kUSB_DeviceHidEventSetReport:
        case kUSB_DeviceHidEventRequestReportBuffer:
            error = kStatus_USB_InvalidRequest;
            break;
        case kUSB_DeviceHidEventGetIdle:
        case kUSB_DeviceHidEventGetProtocol:
        case kUSB_DeviceHidEventSetIdle:
        case kUSB_DeviceHidEventSetProtocol:
            break;
        default:
            break;
    }

    return error;
}

/* The device callback */
static usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Error;
    uint16_t *temp16   = (uint16_t *)param;
    uint8_t *temp8     = (uint8_t *)param;

    switch (event)
    {
        case kUSB_DeviceEventBusReset:
        {
            /* USB bus reset signal detected */
            g_UsbDeviceHidMouse.attach               = 0U;
            g_UsbDeviceHidMouse.currentConfiguration = 0U;
            error                                    = kStatus_USB_Success;
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) || \
    (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
            /* Get USB speed to configure the device, including max packet size and interval of the endpoints. */
            if (kStatus_USB_Success == USB_DeviceClassGetSpeed(CONTROLLER_ID, &g_UsbDeviceHidMouse.speed))
            {
                USB_DeviceSetSpeed(handle, g_UsbDeviceHidMouse.speed);
            }
#endif
        }
        break;
#if (defined(USB_DEVICE_CONFIG_DETACH_ENABLE) && (USB_DEVICE_CONFIG_DETACH_ENABLE > 0U))
        case kUSB_DeviceEventAttach:
        {
            g_UsbDeviceHidMouse.connectStateChanged = 1U;
            g_UsbDeviceHidMouse.connectState        = 1U;
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (((defined(FSL_FEATURE_SOC_USBHSDCD_COUNT) && (FSL_FEATURE_SOC_USBHSDCD_COUNT > 0U))) ||  \
     (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U)))
            g_UsbDeviceHidMouse.dcdDectionStatus = kUSB_DeviceDCDDectionInit;
#else

#if (defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U))
#else
            USB_DeviceRun(g_UsbDeviceHidMouse.deviceHandle);
#endif
#endif
        }
        break;
        case kUSB_DeviceEventDetach:
        {
            g_UsbDeviceHidMouse.connectStateChanged = 1U;
            g_UsbDeviceHidMouse.connectState        = 0U;
            g_UsbDeviceHidMouse.attach              = 0U;
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (((defined(FSL_FEATURE_SOC_USBHSDCD_COUNT) && (FSL_FEATURE_SOC_USBHSDCD_COUNT > 0U))) ||  \
     (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U)))
            g_UsbDeviceHidMouse.dcdDectionStatus = kUSB_DeviceDCDDectionInit;
#else

#if (defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U))
#else
            USB_DeviceStop(g_UsbDeviceHidMouse.deviceHandle);
#endif

#endif
        }
        break;
#endif
        case kUSB_DeviceEventSetConfiguration:
            if (0U == (*temp8))
            {
                g_UsbDeviceHidMouse.attach               = 0;
                g_UsbDeviceHidMouse.currentConfiguration = 0U;
            }
            else if (USB_HID_MOUSE_CONFIGURE_INDEX == (*temp8))
            {
                /* Set device configuration request */
                g_UsbDeviceHidMouse.attach               = 1U;
                g_UsbDeviceHidMouse.currentConfiguration = *temp8;
                error                                    = USB_DeviceHidMouseAction();
            }
            else
            {
                error = kStatus_USB_InvalidRequest;
            }
            break;
        case kUSB_DeviceEventSetInterface:
            if (g_UsbDeviceHidMouse.attach)
            {
                /* Set device interface request */
                uint8_t interface        = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                uint8_t alternateSetting = (uint8_t)(*temp16 & 0x00FFU);
                if (interface < USB_HID_MOUSE_INTERFACE_COUNT)
                {
                    g_UsbDeviceHidMouse.currentInterfaceAlternateSetting[interface] = alternateSetting;
                    if (alternateSetting == 0U)
                    {
                        error = USB_DeviceHidMouseAction();
                    }
                }
            }
            break;
        case kUSB_DeviceEventGetConfiguration:
            if (param)
            {
                /* Get current configuration request */
                *temp8 = g_UsbDeviceHidMouse.currentConfiguration;
                error  = kStatus_USB_Success;
            }
            break;
        case kUSB_DeviceEventGetInterface:
            if (param)
            {
                /* Get current alternate setting of the interface request */
                uint8_t interface = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                if (interface < USB_HID_MOUSE_INTERFACE_COUNT)
                {
                    *temp16 = (*temp16 & 0xFF00U) | g_UsbDeviceHidMouse.currentInterfaceAlternateSetting[interface];
                    error   = kStatus_USB_Success;
                }
                else
                {
                    error = kStatus_USB_InvalidRequest;
                }
            }
            break;
        case kUSB_DeviceEventGetDeviceDescriptor:
            if (param)
            {
                /* Get device descriptor request */
                error = USB_DeviceGetDeviceDescriptor(handle, (usb_device_get_device_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetConfigurationDescriptor:
            if (param)
            {
                /* Get device configuration descriptor request */
                error = USB_DeviceGetConfigurationDescriptor(handle,
                                                             (usb_device_get_configuration_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetStringDescriptor:
            if (param)
            {
                /* Get device string descriptor request */
                error = USB_DeviceGetStringDescriptor(handle, (usb_device_get_string_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetHidDescriptor:
            if (param)
            {
                /* Get hid descriptor request */
                error = USB_DeviceGetHidDescriptor(handle, (usb_device_get_hid_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetHidReportDescriptor:
            if (param)
            {
                /* Get hid report descriptor request */
                error =
                    USB_DeviceGetHidReportDescriptor(handle, (usb_device_get_hid_report_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetHidPhysicalDescriptor:
            if (param)
            {
                /* Get hid physical descriptor request */
                error = USB_DeviceGetHidPhysicalDescriptor(handle,
                                                           (usb_device_get_hid_physical_descriptor_struct_t *)param);
            }
            break;
#if (defined(USB_DEVICE_CONFIG_CV_TEST) && (USB_DEVICE_CONFIG_CV_TEST > 0U))
        case kUSB_DeviceEventGetDeviceQualifierDescriptor:
            if (param)
            {
                /* Get device descriptor request */
                error = USB_DeviceGetDeviceQualifierDescriptor(
                    handle, (usb_device_get_device_qualifier_descriptor_struct_t *)param);
            }
            break;
#endif
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (((defined(FSL_FEATURE_SOC_USBHSDCD_COUNT) && (FSL_FEATURE_SOC_USBHSDCD_COUNT > 0U))) ||  \
     (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U)))
        case kUSB_DeviceEventDcdDetectionfinished:
            /*temp pointer point to detection result*/
            if (param)
            {
                if (kUSB_DcdSDP == *temp8)
                {
                    g_UsbDeviceHidMouse.dcdDectionStatus = kUSB_DeviceDCDDectionSDP;
                }
                else if (kUSB_DcdCDP == *temp8)
                {
                    g_UsbDeviceHidMouse.dcdDectionStatus = kUSB_DeviceDCDDectionCDP;
                }
                else if (kUSB_DcdDCP == *temp8)
                {
                    g_UsbDeviceHidMouse.dcdDectionStatus = kUSB_DeviceDCDDectionDCP;
                }
                else if (kUSB_DcdTimeOut == *temp8)
                {
                    g_UsbDeviceHidMouse.dcdDectionStatus = kUSB_DeviceDCDDectionTimeOut;
                }
                else if (kUSB_DcdError == *temp8)
                {
                    g_UsbDeviceHidMouse.dcdDectionStatus = kUSB_DeviceDCDDectionError;
                }
                else
                {
                }
            }
            break;
#endif
        default:
            break;
    }

    return error;
}

static void USB_DeviceApplicationInit(void)
{
    USB_DeviceClockInit();
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
    SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

    /* Set HID mouse to default state */
    g_UsbDeviceHidMouse.speed        = USB_SPEED_FULL;
    g_UsbDeviceHidMouse.attach       = 0U;
    g_UsbDeviceHidMouse.hidHandle    = (class_handle_t)NULL;
    g_UsbDeviceHidMouse.deviceHandle = NULL;
    g_UsbDeviceHidMouse.buffer       = s_MouseBuffer;

    /* Initialize the usb stack and class drivers */
    if (kStatus_USB_Success !=
        USB_DeviceClassInit(CONTROLLER_ID, &g_UsbDeviceHidConfigList, &g_UsbDeviceHidMouse.deviceHandle))
    {
        usb_echo("USB device mouse failed\r\n");
        return;
    }
    else
    {
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (((defined(FSL_FEATURE_SOC_USBHSDCD_COUNT) && (FSL_FEATURE_SOC_USBHSDCD_COUNT > 0U))) ||  \
     (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U)))
        usb_echo("USB device DCD + HID mouse demo\r\n");
#else
        usb_echo("USB device HID mouse demo -> Lab2 de SIE2\r\n");
#endif
        /* Get the HID mouse class handle */
        g_UsbDeviceHidMouse.hidHandle = g_UsbDeviceHidConfigList.config->classHandle;
    }
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (((defined(FSL_FEATURE_SOC_USBHSDCD_COUNT) && (FSL_FEATURE_SOC_USBHSDCD_COUNT > 0U))) ||  \
     (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U)))
    g_UsbDeviceHidMouse.dcdDectionStatus = kUSB_DeviceDCDDectionInit;
#endif

    USB_DeviceIsrEnable();

#if (defined(USB_DEVICE_CONFIG_DETACH_ENABLE) && (USB_DEVICE_CONFIG_DETACH_ENABLE > 0U))
    /*USB_DeviceRun could not be called here to avoid DP/DM confliction between DCD function and USB function in case
      DCD is enabled. Instead, USB_DeviceRun should be called after the DCD is finished immediately*/
#if (defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U))
    /* Start USB device HID mouse */
    USB_DeviceRun(g_UsbDeviceHidMouse.deviceHandle);
#endif
#else
    /* Start USB device HID mouse */
    USB_DeviceRun(g_UsbDeviceHidMouse.deviceHandle);
#endif
}

#if defined(USB_DEVICE_CONFIG_USE_TASK) && (USB_DEVICE_CONFIG_USE_TASK > 0)
void USB_DeviceTask(void *handle)
{
    while (1U)
    {
        USB_DeviceTaskFn(handle);
    }
}
#endif

void APP_task(void *handle)
{
    USB_DeviceApplicationInit();

#if USB_DEVICE_CONFIG_USE_TASK
    if (g_UsbDeviceHidMouse.deviceHandle)
    {
        if (xTaskCreate(USB_DeviceTask,                       /* pointer to the task */
                        "usb device task",                    /* task name for kernel awareness debugging */
                        5000L / sizeof(portSTACK_TYPE),       /* task stack size */
                        g_UsbDeviceHidMouse.deviceHandle,     /* optional task startup argument */
                        5U,                                   /* initial priority */
                        &g_UsbDeviceHidMouse.deviceTaskHandle /* optional task handle to create */
                        ) != pdPASS)
        {
            usb_echo("usb device task create failed!\r\n");
            return;
        }
    }
#endif

    while (1U)
    {
#if (defined(USB_DEVICE_CONFIG_DETACH_ENABLE) && (USB_DEVICE_CONFIG_DETACH_ENABLE > 0U))

        if (g_UsbDeviceHidMouse.connectStateChanged)
        {
            g_UsbDeviceHidMouse.connectStateChanged = 0;
            if (g_UsbDeviceHidMouse.connectState)
            {
                /*user need call USB_DeviceRun here to usb function run if dcd function is disabled*/
                /*USB_DeviceRun(g_UsbDeviceHidMouse.deviceHandle);*/
                usb_echo("USB device attached.\r\n");
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U))
                HW_TimerControl(1U);
#endif
            }
            else
            {
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U))
                /*USB_DeviceStop should be called here to avoid DP/DM confliction between DCD function and USB function
                 * in case next time DCD dection. */
                USB_DeviceStop(g_UsbDeviceHidMouse.deviceHandle);
#endif
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U))
                HW_TimerControl(0U);
#endif
                usb_echo("USB device detached.\r\n");
            }
        }
#if (defined(USB_DEVICE_CONFIG_CHARGER_DETECT) && (USB_DEVICE_CONFIG_CHARGER_DETECT > 0U)) && \
    (((defined(FSL_FEATURE_SOC_USBHSDCD_COUNT) && (FSL_FEATURE_SOC_USBHSDCD_COUNT > 0U))) ||  \
     (defined(FSL_FEATURE_SOC_USB_ANALOG_COUNT) && (FSL_FEATURE_SOC_USB_ANALOG_COUNT > 0U)))
        if ((kUSB_DeviceDCDDectionInit != g_UsbDeviceHidMouse.dcdDectionStatus) &&
            (kUSB_DeviceDCDDectionFinished != g_UsbDeviceHidMouse.dcdDectionStatus))
        {
            switch (g_UsbDeviceHidMouse.dcdDectionStatus)
            {
                case kUSB_DeviceDCDDectionSDP:
                {
                    usb_echo("SDP(standard downstream port) is detected.\r\n");
                    /* Start USB device HID mouse */
                    USB_DeviceRun(g_UsbDeviceHidMouse.deviceHandle);
                }
                break;
                case kUSB_DeviceDCDDectionDCP:
                {
                    usb_echo("DCP (dedicated charging port) is detected.\r\n");
#if (defined(FSL_FEATURE_USBPHY_HAS_DCD_ANALOG) && (FSL_FEATURE_USBPHY_HAS_DCD_ANALOG > 0U))
                    /*
                     * This is a work-around to fix the DCP device detach event missing issue.
                     * The device (IP3511HS controller) VBUSDEBOUNCED bit is not updated on time before softeware read
                     * this bit, so when DCP is detached from usb port, softeware can't detect DCP disconnection.
                     */
                    USB_DeviceRun(g_UsbDeviceHidMouse.deviceHandle);
#endif
                }
                break;
                case kUSB_DeviceDCDDectionCDP:
                {
                    usb_echo("CDP(charging downstream port) is detected.\r\n");
                    /* Start USB device HID mouse */
                    USB_DeviceRun(g_UsbDeviceHidMouse.deviceHandle);
                }
                break;
                case kUSB_DeviceDCDDectionTimeOut:
                {
                    usb_echo("Timeout error.\r\n");
                }
                break;
                case kUSB_DeviceDCDDectionError:
                {
                    usb_echo("Detect error.\r\n");
                }
                break;
                default:
                    break;
            }
            g_UsbDeviceHidMouse.dcdDectionStatus = kUSB_DeviceDCDDectionFinished;
        }
#endif
#endif
    }
}

#if defined(__CC_ARM) || (defined(__ARMCC_VERSION)) || defined(__GNUC__)
int main(void)
#else
void main(void)
#endif
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    if (xTaskCreate(APP_task,                                  /* pointer to the task */
                    "app task",                                /* task name for kernel awareness debugging */
                    5000L / sizeof(portSTACK_TYPE),            /* task stack size */
                    &g_UsbDeviceHidMouse,                      /* optional task startup argument */
                    4U,                                        /* initial priority */
                    &g_UsbDeviceHidMouse.applicationTaskHandle /* optional task handle to create */
                    ) != pdPASS)
    {
        usb_echo("app task create failed!\r\n");
#if (defined(__CC_ARM) || (defined(__ARMCC_VERSION)) || defined(__GNUC__))
        return 1U;
#else
        return;
#endif
    }

    vTaskStartScheduler();

#if (defined(__CC_ARM) || (defined(__ARMCC_VERSION)) || defined(__GNUC__))
    return 1U;
#endif
}
