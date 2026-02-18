/*
 * m_SerialComm.c
 *
 *  Created on: Jan 27, 2026
 *      Author: Baris
 */

#include "m_SerialComm.h"
#include "m_SharedMemory.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// *** Değişkenler m_SerialComm.c
uint8_t serial_comm_ready = 0u;
uint8_t rx_data_ready = 0u;
uint8_t tx_data_ready = 0u;
uint8_t rx_buffer[RX_BUFFER_SIZE] = {0};
static uint8_t rx_buffer_temp[RX_BUFFER_SIZE] = {0};
uint8_t tx_buffer[TX_BUFFER_SIZE] = {0};
static volatile uint16_t rx_buffer_temp_index = 0;
static volatile uint16_t rx_data_length = 0;

void ReceiveSerialData_Task(uint8_t rx_byte)
{
	// --- 1. Bitiş Karakteri Kontrolü (\r veya \n) ---
	if (rx_byte == '\r' || rx_byte == '\n')
	{
		// Sadece bufferda veri varsa işlem yap (Boş satırları ve \r\n çakışmasını önler)
		if (rx_buffer_temp_index > 0)
		{
			// 1) RX buffer ı temizler, veri alımına uygun hale getirir.
			memset(rx_buffer, 0, RX_BUFFER_SIZE);

			// 2) Veriyi asıl buffer'a kopyalar.
			memcpy(rx_buffer, rx_buffer_temp, rx_buffer_temp_index);

			// 3) Veri uzunluğu kaydet
			rx_data_length = rx_buffer_temp_index;

			// 4) Verinin hazır olduğunu bildir
			rx_data_ready = 1;

			// 5) RX buffer indexini sıfırla
			rx_buffer_temp_index = 0;
		}
		// Eğer rx_buffer_temp_index 0 ise (yani ardı ardına \r\n geldiyse),
		// \n karakterini sessizce yoksayar.
	}
	// --- 2. Normal Veri Geldi ---
	else
	{
		if (rx_buffer_temp_index < RX_BUFFER_SIZE - 1)
		{
			rx_buffer_temp[rx_buffer_temp_index] = rx_byte;
			rx_buffer_temp_index++;
		}
	}
}

void ParseSerialData(uint8_t *line)
{
    if (line == NULL) return;
    if (line[0] == '\0') return;

    /* 1) Ping algılama: "ping;123" */
    if (line[0] == 'p' && line[1] == 'i' && line[2] == 'n' && line[3] == 'g' && line[4] == ';')
    {
        /* parametreyi al (örn 123) */
        uint8_t ping_value = String2Uint32((uint8_t*)&line[5]);
        if(ping_value == 123)
        {
        	sm.serial_comm_active = 1;
        	snprintf((char*)tx_buffer, TX_BUFFER_SIZE, "pong;321");
        	tx_data_ready = 1;
        }
        else
        {
        	sm.serial_comm_active = 0;
        	snprintf((char*)tx_buffer, TX_BUFFER_SIZE, "pong;0");
        	tx_data_ready = 1;
        }
        return; /* şimdilik başka işlem yok */
    }

    /* Kusursuz format: "X;param" */
    if (line[1] != ';') return;

    char cmd = line[0];
    uint8_t *arg = (uint8_t*)&line[2];

    switch (cmd)
    {
        case 'M': /* Mode -> sm.control_mode (0/1) */
        {
            uint8_t v = String2Uint8(arg);
            if(v <= 2u)
            {
            	sm.control_mode = v;

            	snprintf((char*)tx_buffer, TX_BUFFER_SIZE, "M;1");
            	tx_data_ready = 1;
            }
            else
            {
            	snprintf((char*)tx_buffer, TX_BUFFER_SIZE, "M;0");
            	tx_data_ready = 1;
            }
        } break;

        case 'A': /* Angle -> sm.ref_position (float) */
        {
            float v = String2Float(arg);
            sm.ref_position = v;

            snprintf((char*)tx_buffer, TX_BUFFER_SIZE, "A;1");
            tx_data_ready = 1;
        } break;

        case 'B': /* Başla / Durdur */
        {
            uint8_t v = String2Uint8(arg);
            if (v <= 1u)
            {
            	sm.operation_active = v;
            	if (v == 1)
            	{
            		sm.operation_start = 1u;
            		sm.operation_stop = 0u;
            	}
            	else
            	{
            		sm.operation_start = 0u;
            		sm.operation_stop = 1u;
            	}

            	snprintf((char*)tx_buffer, TX_BUFFER_SIZE, "B;1");
            	tx_data_ready = 1;
            }
            else
            {
            	snprintf((char*)tx_buffer, TX_BUFFER_SIZE, "B;0");
            	tx_data_ready = 1;
            }
        } break;

        case 'H': /* Speed -> sm.ref_velocity (float) */
        {
            float v = String2Float(arg);
            sm.ref_velocity = v;

            snprintf((char*)tx_buffer, TX_BUFFER_SIZE, "H;1");
            tx_data_ready = 1;
        } break;

        case 'T': /* Time -> time_delay (uint16_t) */
        {
            uint16_t v = String2Uint16(arg);
            sm.time_delay = v;
            snprintf((char*)tx_buffer, TX_BUFFER_SIZE, "T;1");
            tx_data_ready = 1;
        } break;

        case 'P': /* Pedal active -> pedal_active (0/1) */
        {
            uint8_t v = String2Uint8(arg);
            if (v <= 1u)
            {
            	sm.pedal_active = v;

            	snprintf((char*)tx_buffer, TX_BUFFER_SIZE, "P;1");
            	tx_data_ready = 1;
            }
            else
            {
            	snprintf((char*)tx_buffer, TX_BUFFER_SIZE, "P;0");
            	tx_data_ready = 1;
            }
        } break;

        case 'S': /* Counter -> pedal_counter (uint32_t) */
        {
            uint32_t v = String2Uint32(arg);
            sm.pedal_counter = v;

            snprintf((char*)tx_buffer, TX_BUFFER_SIZE, "S;1");
            tx_data_ready = 1;
        } break;

        case 'D': /* Emergency -> emergency_stop (0/1) */
        {
            uint8_t v = String2Uint8(arg);
            if (v <= 1u)
            {
            	sm.emergency_stop = v;

            	snprintf((char*)tx_buffer, TX_BUFFER_SIZE, "D;1");
            	tx_data_ready = 1;
            }
            else
            {
            	snprintf((char*)tx_buffer, TX_BUFFER_SIZE, "D;0");
            	tx_data_ready = 1;
            }
        } break;

        case 'I': /* Emergency -> emergency_stop (0/1) */
        {
           	snprintf((char*)tx_buffer, TX_BUFFER_SIZE,
            			"I;%ld;%ld;%u;%u;%u;%u;%lu",
						(int32_t)(sm.act_position_relative * 100.0f),	// %ld	(int32_t)
						(int32_t)(sm.act_velocity),						// %ld	(int32_t)
						(uint16_t)(sm.act_motor_current * 100.0f),		// %u	(uint16_t)
						(uint16_t)(sm.act_motor_voltage * 100.f),		// %u	(uint16_t)
						sm.control_mode,     							// %u	(uint8_t)
						sm.pedal_active,     							// %u	(uint8_t)
						sm.pedal_counter);   							// %lu	(uint32)
           	tx_data_ready = 1;
        } break;

        case 'G': /* Duty Cycle -> dutycycle_command (float) */
        {
            float v = String2Float(arg);
            sm.duty_cycle_command = (v / 10000.0f);
            snprintf((char*)tx_buffer, TX_BUFFER_SIZE, "F;1");
            tx_data_ready = 1;
        } break;
    }
}




uint8_t String2Uint8(uint8_t *buffer)
{
    char *endptr;

    // Buffer boşsa...
    if (buffer == NULL) return 0;

    unsigned long val = strtoul((char*)buffer, &endptr, 10);

    // Buffer içinde hiç sayı yoksa...
    if (endptr == (char*)buffer) return 0;

    // Sayı dışında karakter varsa (örn "12a" veya "123 ")...
    if (*endptr != '\0') return 0;

    // uint8_t sınırı aşmışsa...
    if (val > 255) val = 255;

    return (uint8_t)val;
}

uint16_t String2Uint16(uint8_t *buffer)
{
    char *endptr;

    // Buffer boşsa...
    if (buffer == NULL) return 0;

    unsigned long val = strtoul((char*)buffer, &endptr, 10);

    // Buffer içinde hiç sayı yoksa...
    if (endptr == (char*)buffer) return 0;

    // Sayı dışında karakter varsa (örn "12a" veya "123 ")...
    if (*endptr != '\0') return 0;

    // uint8_t sınırı aşmışsa...
    if (val > 65535) val = 65535;

    return (uint16_t)val;
}

uint32_t String2Uint32(uint8_t *buffer)
{
    char *endptr;

    // Buffer boşsa...
    if (buffer == NULL) return 0;

    unsigned long val = strtoul((char*)buffer, &endptr, 10);

    // Buffer içinde hiç sayı yoksa...
    if (endptr == (char*)buffer) return 0;

    // Sayı dışında karakter varsa (örn "12a" veya "123 ")...
    if (*endptr != '\0') return 0;

    // uint8_t sınırı aşmışsa...
    if (val > 4294967295) val = 4294967295;

    return (uint32_t)val;
}

float String2Float(uint8_t *buffer)
{
    char *endptr;

    // Buffer boşsa...
    if (buffer == NULL) return 0.0f;

    float val = strtof((char*)buffer, &endptr);

    // Buffer içinde hiç sayı yoksa...
    if (endptr == (char*)buffer) return 0.0f;

    // Sayı dışında karakter varsa (örn "12a" veya "123 ")...
    if (*endptr != '\0') return 0.0f;

    return val;
}
