#include "main.h"

extern UART_HandleTypeDef huart1;

//put this at the begining of usbd_cdc_if.c:CDC_Receive_FS()
//   pw_cdc_received(Buf, *Len);
//this to usbd_cdc_if.h:
//   void pw_cdc_received(uint8_t *Buf, uint32_t Len);
//use this to send (may return USBD_BUSY)
//   uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
#include "usbd_cdc_if.h"
extern USBD_HandleTypeDef hUsbDeviceFS;

uint8_t usb_send_timeout(char *data);

volatile uint8_t *usb_rx_buf;
volatile uint32_t usb_rx_len;
volatile received_flag = 0;
char txt_buf[64];

void pw_cdc_received(uint8_t *Buf, uint32_t Len)
{
	usb_rx_buf = Buf;
	usb_rx_len = Len;
	received_flag = 1;
    //USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    //USBD_CDC_ReceivePacket(&hUsbDeviceFS);
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
    Protocol description:
    host to device:
        offset
        0           I - I2C, S - SPI, G - GPIO

        I2C:
        1           address with R/W bit
        2           data length
        4           data (if writing)

        SPI:
        1           0 send, 1 receive, 2 send/receive
        2..3        data length (LSB first, 1024 max)
        4           data (if sending)

        GPIO:
        1           bit values (A0-A3, A4 to be used as SPI CSN)

    device to host:
        offset
        0           error code (0 - success, 1 - error)
        1..n        data (if reading)
*/
void main2()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_UART_Receive_IT(&huart1, uart_buffer+uart_pos, 1);
    while(1)
    {
        while(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
            HAL_Delay(1);
        HAL_Delay(1000);

        //usb_send_timeout("Hello world!\r\n");
        while(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
        {
            if(received_flag)
            {
                int i;
                for(i=0; i<usb_rx_len; i++)
                {
                    HAL_UART_Transmit(&huart1, (uint8_t *)&usb_rx_buf[i], 1, 100);
                }
                USBD_CDC_SetRxBuffer(&hUsbDeviceFS, usb_rx_buf);
                USBD_CDC_ReceivePacket(&hUsbDeviceFS);
                received_flag = 0;
            }
            if(usb_pos != uart_pos)
            {
                int uart_pos_local_copy = uart_pos;
                if(usb_pos <= uart_pos)
                {
                    usb_send_timeout_sz(uart_buffer+usb_pos, uart_pos_local_copy-usb_pos);
                    usb_pos = uart_pos_local_copy;
                } else {
                    usb_send_timeout_sz(uart_buffer+usb_pos, UART_BUFFER_SIZE-usb_pos);
                    usb_pos = 0;
                }
            }
        }
    }
}
