#include "main.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

//put this at the begining of usbd_cdc_if.c:CDC_Receive_FS()
//   pw_cdc_received(Buf, *Len);
//this to usbd_cdc_if.h:
//   void pw_cdc_received(uint8_t *Buf, uint32_t Len);
//use this to send (may return USBD_BUSY)
//   uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
#include "usbd_cdc_if.h"
extern USBD_HandleTypeDef hUsbDeviceFS;

uint8_t usb_send_timeout(char *data);

char txt_buf[64];

/* --- Command reassembly buffer -------------------------------------------- */
/* CDC FS delivers at most 64 bytes per OUT packet, but a single bridge       */
/* command can be up to 4 (header) + 1024 (payload) bytes, so it may span      */
/* several packets. pw_cdc_received() runs in USB IRQ context and simply       */
/* appends every received byte here; the main2() super-loop parses and         */
/* executes complete commands. CMD_BUF_SIZE holds one full command with a bit  */
/* of slack for back-to-back packets.                                          */
#define CMD_MAX_PAYLOAD 1024
#define CMD_HEADER_LEN  4
#define CMD_BUF_SIZE    (CMD_HEADER_LEN + CMD_MAX_PAYLOAD + 64)

volatile uint8_t cmd_buf[CMD_BUF_SIZE];
volatile uint32_t cmd_len = 0;

void pw_cdc_received(uint8_t *Buf, uint32_t Len)
{
    uint32_t i;
    for(i = 0; i < Len; i++)
    {
        if(cmd_len < CMD_BUF_SIZE)
            cmd_buf[cmd_len++] = Buf[i];
        /* else: buffer full, drop — main loop will resync/flush on next parse */
    }
}

uint8_t usb_send_timeout(char *data)
{
    uint8_t result = USBD_OK;
    int v = 100;
    while(v--)
    {
        USBD_CDC_SetTxBuffer(&hUsbDeviceFS, data, strlen(data));
        result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
        if(result == USBD_OK)
            return result;
        HAL_Delay(1);
    }
    return result;
}
uint8_t usb_send_timeout_sz(uint8_t *data, int sz)
{
    uint8_t result = USBD_OK;
    int v = 100;
    while(v--)
    {
        USBD_CDC_SetTxBuffer(&hUsbDeviceFS, data, sz);
        result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
        if(result == USBD_OK)
            return result;
        HAL_Delay(1);
    }
    return result;
}

void print_hex(uint8_t *data, int sz)
{
    int p = 0;
    while(p < sz)
    {
        sprintf(txt_buf, "%02x: %02x %02x %02x %02x %02x %02x %02x %02x\r\n", p, data[p+0], data[p+1], data[p+2], data[p+3], data[p+4], data[p+5], data[p+6], data[p+7]);
        usb_send_timeout(txt_buf);
        p+=8;
    }
}

#define UART_BUFFER_SIZE 1024
volatile uint8_t uart_buffer[UART_BUFFER_SIZE];
volatile int uart_pos = 0;
volatile int usb_pos = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uart_pos = (uart_pos + 1) % UART_BUFFER_SIZE;
    if (uart_pos == usb_pos)
        usb_pos = (usb_pos + 1) % UART_BUFFER_SIZE;
    
    HAL_UART_Receive_IT(&huart1, uart_buffer+uart_pos, 1);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_Receive_IT(&huart1, uart_buffer+uart_pos, 1);
}

/* Current CDC line coding, kept in sync with the UART configuration.       */
/* Defaults match the UART init in main.c (115200 8N1).                     */
volatile uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};

/* Apply a CDC line-coding structure received from the USB host to USART1.  */
void pw_set_line_coding(uint8_t *buf)
{
    int i;
    for(i = 0; i < 7; i++)
        line_coding[i] = buf[i];

    uint32_t baud = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
                    ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    uint8_t format   = buf[4];
    uint8_t parity   = buf[5];
    uint8_t databits = buf[6];

    if(baud == 0)
        return; /* ignore invalid request, keep current setting */

    huart1.Init.BaudRate = baud;
    huart1.Init.StopBits = (format == 2) ? UART_STOPBITS_2 : UART_STOPBITS_1;

    /* On STM32F1 the word length includes the parity bit, so 8 data bits    */
    /* with parity needs a 9-bit word. Mark/Space parity has no hardware     */
    /* equivalent and is treated as no parity.                               */
    if(parity == 1 || parity == 2)
    {
        huart1.Init.Parity = (parity == 1) ? UART_PARITY_ODD : UART_PARITY_EVEN;
        huart1.Init.WordLength = (databits >= 8) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;
    }
    else
    {
        huart1.Init.Parity = UART_PARITY_NONE;
        huart1.Init.WordLength = (databits >= 9) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;
    }

    HAL_UART_DeInit(&huart1);
    HAL_UART_Init(&huart1);

    /* Re-arm the single-byte interrupt reception after reconfiguration. */
    HAL_UART_Receive_IT(&huart1, uart_buffer+uart_pos, 1);
}

/* Report the current line coding back to the USB host (GET_LINE_CODING).   */
void pw_get_line_coding(uint8_t *buf)
{
    int i;
    for(i = 0; i < 7; i++)
        buf[i] = line_coding[i];
}

/*
    Protocol description (binary packets over the USB CDC endpoint).

    Every command has a FIXED 4-byte header; payload (if any) starts at offset 4.

    host -> device:
        offset
        0           command: 'I' = I2C, 'S' = SPI, 'G' = GPIO

        I2C:
        1           target address in 8-bit form: (7-bit addr << 1) | R/W
                    R/W bit (bit 0): 0 = write, 1 = read
        2           data length (bytes), 0..255
        3           reserved
        4..         data to write (write commands only)

        SPI: chip-select (PA4, active low) is asserted around the transfer.
        1           mode: 0 = send, 1 = receive, 2 = send+receive (full duplex)
        2..3        data length (LSB first), 0..1024
        3           (high byte of length)
        4..         data to send (modes 0 and 2)

        GPIO:
        1           bit values for outputs A0..A3 (PA0..PA3); low 4 bits used,
                    A4/PA4 is the SPI CS and is NOT affected
        2..3        reserved

    device -> host:
        offset
        0           error code (0 = success, 1 = error)
        1..n        read data (I2C read, SPI receive / send+receive)
*/

#define BRIDGE_OK       0
#define BRIDGE_ERR      1
#define BRIDGE_TIMEOUT  1000   /* ms, for blocking HAL transfers */

/* Response scratch: byte 0 is the status, bytes 1.. hold any read data. */
static uint8_t resp_buf[1 + CMD_MAX_PAYLOAD];

/* SPI chip-select is on PA4 and is active low. */
static void spi_cs_low(void)  { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); }
static void spi_cs_high(void) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); }

/* Drive user GPIO outputs A0..A3 (PA0..PA3) from the low 4 bits of val. */
static void gpio_write_a0_a3(uint8_t val)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (val & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (val & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, (val & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, (val & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*
 * Work out the total length (header + payload) of the command at the front of
 * the buffer, given that `have` bytes are currently available.
 *   returns 0   -> not enough bytes yet to know the length
 *   returns >0  -> full command length in bytes
 *   *bad = 1    -> leading byte is not a valid command (caller should flush)
 */
static uint32_t command_total_len(const volatile uint8_t *buf, uint32_t have, int *bad)
{
    *bad = 0;
    if(have < 1)
        return 0;

    switch(buf[0])
    {
        case 'G':
            return CMD_HEADER_LEN;                 /* no payload */

        case 'I':
            if(have < CMD_HEADER_LEN)
                return 0;
            if(buf[1] & 0x01)
                return CMD_HEADER_LEN;             /* read: no payload */
            return CMD_HEADER_LEN + buf[2];        /* write: data follows */

        case 'S':
            if(have < CMD_HEADER_LEN)
                return 0;
            {
                uint8_t  mode = buf[1];
                uint32_t len  = buf[2] | ((uint32_t)buf[3] << 8);
                if(mode == 0 || mode == 2)
                    return CMD_HEADER_LEN + len;   /* send / send+receive */
                return CMD_HEADER_LEN;             /* receive only */
            }

        default:
            *bad = 1;
            return 0;
    }
}

/* Execute one complete command and transmit the response packet. */
static void process_command(volatile uint8_t *cmd)
{
    HAL_StatusTypeDef st;
    uint32_t resp_len = 1;

    switch(cmd[0])
    {
        case 'I':  /* ---------------- I2C ---------------- */
        {
            uint8_t  addr8   = cmd[1] & 0xFE;  /* HAL takes the 8-bit address */
            uint8_t  is_read = cmd[1] & 0x01;
            uint16_t len     = cmd[2];
            if(is_read)
            {
                st = HAL_I2C_Master_Receive(&hi2c1, addr8, &resp_buf[1], len, BRIDGE_TIMEOUT);
                if(st == HAL_OK)
                    resp_len = 1 + len;
            }
            else
            {
                st = HAL_I2C_Master_Transmit(&hi2c1, addr8, (uint8_t *)&cmd[CMD_HEADER_LEN], len, BRIDGE_TIMEOUT);
            }
            resp_buf[0] = (st == HAL_OK) ? BRIDGE_OK : BRIDGE_ERR;
            break;
        }

        case 'S':  /* ---------------- SPI ---------------- */
        {
            uint8_t  mode = cmd[1];
            uint16_t len  = cmd[2] | ((uint16_t)cmd[3] << 8);
            if(len > CMD_MAX_PAYLOAD)
            {
                resp_buf[0] = BRIDGE_ERR;
                break;
            }
            spi_cs_low();
            if(mode == 0)
                st = HAL_SPI_Transmit(&hspi1, (uint8_t *)&cmd[CMD_HEADER_LEN], len, BRIDGE_TIMEOUT);
            else if(mode == 1)
                st = HAL_SPI_Receive(&hspi1, &resp_buf[1], len, BRIDGE_TIMEOUT);
            else
                st = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&cmd[CMD_HEADER_LEN], &resp_buf[1], len, BRIDGE_TIMEOUT);
            spi_cs_high();
            resp_buf[0] = (st == HAL_OK) ? BRIDGE_OK : BRIDGE_ERR;
            if(st == HAL_OK && (mode == 1 || mode == 2))
                resp_len = 1 + len;
            break;
        }

        case 'G':  /* ---------------- GPIO --------------- */
            gpio_write_a0_a3(cmd[1]);
            resp_buf[0] = BRIDGE_OK;
            break;

        default:
            resp_buf[0] = BRIDGE_ERR;
            break;
    }

    usb_send_timeout_sz(resp_buf, (int)resp_len);
}

/* Discard the whole accumulation buffer (used on protocol/length errors). */
static void cmd_flush(void)
{
    __disable_irq();
    cmd_len = 0;
    __enable_irq();
}

void main2()
{
    int bad;
    uint32_t total;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  /* LED off (if present) */
    spi_cs_high();                                        /* CS idle high */

    while(1)
    {
        while(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
            HAL_Delay(1);

        while(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
        {
            if(cmd_len == 0)
                continue;

            total = command_total_len(cmd_buf, cmd_len, &bad);

            if(bad || total > CMD_BUF_SIZE)
            {
                /* unknown command or a length that can never fit: report and flush */
                resp_buf[0] = BRIDGE_ERR;
                usb_send_timeout_sz(resp_buf, 1);
                cmd_flush();
                continue;
            }

            if(total == 0 || cmd_len < total)
                continue;   /* header incomplete, or waiting for the payload */

            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); /* LED on */
            process_command(cmd_buf);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

            /* drop the consumed command, preserving any bytes of the next one */
            __disable_irq();
            {
                uint32_t leftover = cmd_len - total;
                if(leftover)
                    memmove((uint8_t *)cmd_buf, (uint8_t *)cmd_buf + total, leftover);
                cmd_len = leftover;
            }
            __enable_irq();
        }
    }
}
