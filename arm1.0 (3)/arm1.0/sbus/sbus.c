#include "sbus.h"
#include "DMmotor.h"

uint8_t sbus_rx_buffer[SBUS_FRAME_SIZE];
sbus_data sbus_rx_data = {0};

void sbus_decode(uint8_t *sbus_rx_buffer, sbus_data *sbus_rx_data)
{
    HAL_UART_Receive_DMA(&huart1, sbus_rx_buffer, 25);

    sbus_rx_data->is_enable = 0;

    // 检查帧头（0x0F）
    if (sbus_rx_buffer[0] == 0x0F)
    {
        // 通道1-8（与前8个通道解码相同）
        sbus_rx_data->channel0 = ((sbus_rx_buffer[1] | sbus_rx_buffer[2] << 8) & 0x07FF);
        sbus_rx_data->channel1 = ((sbus_rx_buffer[2] >> 3 | sbus_rx_buffer[3] << 5) & 0x07FF);
        sbus_rx_data->channel2 = ((sbus_rx_buffer[3] >> 6 | sbus_rx_buffer[4] << 2 | sbus_rx_buffer[5] << 10) & 0x07FF);
        sbus_rx_data->channel3 = ((sbus_rx_buffer[5] >> 1 | sbus_rx_buffer[6] << 7) & 0x07FF);
        sbus_rx_data->sw0 = ((sbus_rx_buffer[6] >> 4 | sbus_rx_buffer[7] << 4) & 0x07FF);
        sbus_rx_data->sw1 = ((sbus_rx_buffer[7] >> 7 | sbus_rx_buffer[8] << 1 | sbus_rx_buffer[9] << 9) & 0x07FF);
        sbus_rx_data->sw2 = ((sbus_rx_buffer[9] >> 2 | sbus_rx_buffer[10] << 6) & 0x07FF);
        sbus_rx_data->sw3 = ((sbus_rx_buffer[10] >> 5 | sbus_rx_buffer[11] << 3) & 0x07FF);

        // 通道9-10
        sbus_rx_data->roll0 = ((sbus_rx_buffer[12] | sbus_rx_buffer[13] << 8) & 0x07FF);
        sbus_rx_data->roll1 = ((sbus_rx_buffer[13] >> 3 | sbus_rx_buffer[14] << 5) & 0x07FF);

        // 接收正常开启使能
        sbus_rx_data->is_enable = 1;
    }
}
