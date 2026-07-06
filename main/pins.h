/* IDL0 firmware pin assignments.
 *
 * Single source of truth for every GPIO. Values are derived from the
 * KiCad netlist export at firmware/hardware/netlist.net. To regenerate
 * after a schematic change, re-run P3 Task 1 of the firmware bring-up
 * plan.
 *
 * Hardware notes:
 *   - Status LED uses the XIAO ESP32-C6 **onboard** user LED on GPIO15
 *     (active-low — drive LOW to light). The custom board's own LED net
 *     `/LED_STATUS` is on GPIO9, but that footprint was mirrored in the
 *     PCB revision in hand, so the onboard LED is used for bring-up
 *     until the board is reworked.
 *   - GPIO4 / GPIO5 / GPIO6 carry JTAG aliases (MTMS / MTDO / MTCK).
 *     Used here as plain GPIO; on-chip JTAG is unavailable with this
 *     firmware loaded.
 *   - /BUTTONS is a single net (GPIO5) — likely a resistor-ladder bank
 *     read by ADC rather than two discrete GPIOs. Implementation
 *     deferred; see P4+.
 *   - All high-speed sensors share SPI2_HOST (SCK / MOSI / MISO) with
 *     individual chip-selects.
 */

#pragma once

#include "driver/gpio.h"

/* Status LED — XIAO ESP32-C6 onboard user LED. Active-low. */
#define IDL0_PIN_LED              GPIO_NUM_15

/* Buttons. Single net /BUTTONS — likely ADC resistor ladder. */
#define IDL0_PIN_BUTTONS          GPIO_NUM_5

/* Shared SPI bus (SD + IMUs). */
#define IDL0_PIN_SPI_SCK          GPIO_NUM_19
#define IDL0_PIN_SPI_MOSI         GPIO_NUM_18
#define IDL0_PIN_SPI_MISO         GPIO_NUM_20

/* Chip-selects (one per SPI device). */
#define IDL0_PIN_CS_SD            GPIO_NUM_23
#define IDL0_PIN_CS_IMU0          GPIO_NUM_2    /* /IMU1_CS — sprung mass (onboard) */
#define IDL0_PIN_CS_IMU1          GPIO_NUM_21   /* /IMU_CS_FRONT — front unsprung */
#define IDL0_PIN_CS_IMU2          GPIO_NUM_22   /* /IMU_CS_REAR — rear unsprung */

/* GPS UART (u-blox MAX-M10S). */
#define IDL0_PIN_GPS_TX           GPIO_NUM_16   /* host TX → module RX */
#define IDL0_PIN_GPS_RX           GPIO_NUM_17   /* module TX → host RX */

/* Wheel speed (Hall, edge-triggered GPIO interrupts). */
#define IDL0_PIN_SPEED_FRONT      GPIO_NUM_4
#define IDL0_PIN_SPEED_REAR       GPIO_NUM_6

/* Pressure sensors (ADC1). */
#define IDL0_PIN_PRESSURE_FRONT   GPIO_NUM_0    /* ADC1_CH0 */
#define IDL0_PIN_PRESSURE_REAR    GPIO_NUM_1    /* ADC1_CH1 */

/* TEMP: I²C bus for external IMUs (IMU1 / IMU2). The Adafruit 4692
 * breakouts have a passive MOSFET level shifter on their SDA/SCL pads
 * that blocks push-pull SPI; I²C is what the breakout is designed for
 * and what these pins serve. Reuses the GPIOs freed by removing the
 * per-IMU SPI chip-selects (CS_IMU1=GPIO21, CS_IMU2=GPIO22) — the same
 * conductors at J2 carry the new signals. Remove these defines, the
 * imu_driver_i2c module, and the bus dispatch in imu_task.c once the
 * external boards are reworked or replaced with bare-PCB equivalents. */
#define IDL0_PIN_I2C_SDA          GPIO_NUM_22
#define IDL0_PIN_I2C_SCL          GPIO_NUM_21
