/*
 * m_SerialComm.c
 *
 *  Created on: Jan 27, 2026
 *      Author: Baris
 */

#include "m_SerialComm.h"
#include "m_SharedMemory.h"
#include "stm32f1xx_hal.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "stm32f1xx.h"

#define FIRMWARE_NAME    "ERTIP_CROWN"
#define FIRMWARE_VERSION "1.0.8.4.2"

// Timed-run (non-blocking)
typedef struct {
	uint8_t active;
	uint32_t start_ms;
	uint32_t duration_ms;
	float duty; // 0..1
} timed_run_t;

static timed_run_t g_timed_run = {0};

// Stream telemetry
static uint8_t g_stream_enabled = 0u;
static uint16_t g_stream_period_ms = 50u;
static uint32_t g_stream_last_ms = 0u;

// forward
static void UniCom_HandleLine(uint8_t *line);
void UniCom_TimedTasks(void);
static void UniCom_SendMotorStatus(void);

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

    // 0) UniCom / Arduino protocol (SYS./DEV./PIN.)
    // Format: "GROUP.CMD" or "GROUP.CMD:PARAM"
    if ((line[0] == 'S' && line[1] == 'Y' && line[2] == 'S' && line[3] == '.') ||
        (line[0] == 'D' && line[1] == 'E' && line[2] == 'V' && line[3] == '.') ||
        (line[0] == 'P' && line[1] == 'I' && line[2] == 'N' && line[3] == '.'))
    {
        UniCom_HandleLine(line);
        return;
    }

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

//-------------

static void UniCom_Send(const char *s)
{
	if (!s) return;
	snprintf((char*)tx_buffer, TX_BUFFER_SIZE, "%s", s);
	tx_data_ready = 1u;
}

static void UniCom_SendAck(const char *cmd)
{
	char buf[96];
	snprintf(buf, sizeof(buf), "ACK:%s", cmd);
	UniCom_Send(buf);
}

static void UniCom_SendErr(const char *err)
{
	char buf[96];
	snprintf(buf, sizeof(buf), "ERR:%s", err);
	UniCom_Send(buf);
}

static void UniCom_Split(uint8_t *line, char *cmdOut, uint32_t cmdCap, char *paramsOut, uint32_t paramsCap)
{
	// cmdOut = before ':' ; paramsOut = after ':'
	char *colon = strchr((char*)line, ':');
	if (!colon)
	{
		snprintf(cmdOut, cmdCap, "%s", (char*)line);
		paramsOut[0] = '\0';
		return;
	}
	*colon = '\0';
	snprintf(cmdOut, cmdCap, "%s", (char*)line);
	snprintf(paramsOut, paramsCap, "%s", (colon + 1));
}

static void UniCom_SendMotorStatus(void)
{
	// Encoder + akım + voltaj zaten sm içinde var.
	// Not: I; komutundaki formatı bozmayalım, burada Arduino tarzı DATA döndürüyoruz.
	char buf[200];
	snprintf(buf, sizeof(buf),
			"DATA:MOTOR:%.2f:%.2f:%.3f:%.2f:%.4f:%u:%u",
			(float)sm.act_position_relative,     // deg
			(float)sm.act_velocity,              // rpm
			(float)sm.act_motor_current,         // A
			(float)sm.act_motor_voltage,         // V
			(float)sm.act_pwm_duty,              // 0..1
			(unsigned)sm.hall_state,             // 0/1 (şimdilik 0 kalabilir)
			(unsigned)sm.control_mode            // 0/1/2
	);
	UniCom_Send(buf);
}

static void UniCom_HandleLine(uint8_t *line)
{
	char cmd[64];
	char params[96];
	UniCom_Split(line, cmd, sizeof(cmd), params, sizeof(params));

	// --- SYS ---
	if (strcmp(cmd, "SYS.PING") == 0)
	{
		UniCom_Send("PONG");
		return;
	}
	if (strcmp(cmd, "SYS.INFO") == 0)
	{
		char buf[96];
		snprintf(buf, sizeof(buf), "INFO:%s:%s", FIRMWARE_NAME, FIRMWARE_VERSION);
		UniCom_Send(buf);
		return;
	}
	if (strcmp(cmd, "SYS.RESET") == 0)
	{
		// cevap dönmeden reset
		NVIC_SystemReset();
		return;
	}

	// --- DEV.MOTOR ---
	if (strcmp(cmd, "DEV.MOTOR.SET_PWM") == 0)
	{
		int pwm = atoi(params); // 0..255
		if (pwm < 0) pwm = 0;
		if (pwm > 255) pwm = 255;

		// Manual mode'a geç: STM32 state machine bunu OP_STATE_MAN_MODE ile yönetiyor
		sm.control_mode = 2u;                    // 2 => manual
		sm.duty_cycle_command = (float)pwm / 255.0f;

		// Başlat: idle state, operation_start görünce MAN mode'a geçiyor
		sm.operation_start = 1u;
		sm.operation_stop  = 0u;

		UniCom_SendAck("DEV.MOTOR.SET_PWM");
		return;
	}

	if (strcmp(cmd, "DEV.MOTOR.SET_DIR") == 0)
	{
		int dir = atoi(params); // 0/1
		sm.motor_direction = (dir != 0) ? 1u : 0u;
		UniCom_SendAck("DEV.MOTOR.SET_DIR");
		return;
	}

	if (strcmp(cmd, "DEV.MOTOR.STOP") == 0)
	{
		g_timed_run.active = 0u;
		sm.operation_stop = 1u; // main state machine motoru durduracak
		UniCom_SendAck("DEV.MOTOR.STOP");
		return;
	}

	if (strcmp(cmd, "DEV.MOTOR.EXEC_TIMED_RUN") == 0)
	{
		// params: "PWM|MS"
		char *sep = strchr(params, '|');
		if (!sep) { UniCom_SendErr("INVALID_PARAM"); return; }
		*sep = '\0';

		int pwm = atoi(params);
		int ms  = atoi(sep + 1);

		if (pwm < 0) pwm = 0;
		if (pwm > 255) pwm = 255;
		if (ms < 0) ms = 0;

		g_timed_run.active = 1u;
		g_timed_run.start_ms = HAL_GetTick();
		g_timed_run.duration_ms = (uint32_t)ms;
		g_timed_run.duty = (float)pwm / 255.0f;

		sm.control_mode = 2u;
		sm.duty_cycle_command = g_timed_run.duty;
		sm.operation_start = 1u;
		sm.operation_stop  = 0u;

		UniCom_SendAck("DEV.MOTOR.EXEC_TIMED_RUN");
		return;
	}

	if (strcmp(cmd, "DEV.MOTOR.GET_STATUS") == 0)
	{
		UniCom_SendMotorStatus();
		return;
	}

	if (strcmp(cmd, "DEV.MOTOR.STREAM") == 0)
	{
		// params: "0/1|period_ms"  (period opsiyonel)
		// ör: "1|50"
		char *sep = strchr(params, '|');
		int en = 0;
		int per = (int)g_stream_period_ms;

		if (sep)
		{
			*sep = '\0';
			en = atoi(params);
			per = atoi(sep + 1);
		}
		else
		{
			en = atoi(params);
		}

		g_stream_enabled = (en != 0) ? 1u : 0u;
		if (per >= 5 && per <= 1000) g_stream_period_ms = (uint16_t)per;
		g_stream_last_ms = HAL_GetTick();

		UniCom_SendAck("DEV.MOTOR.STREAM");
		return;
	}

	// --- PIN.* (sen devre dışı bırakmak istiyorsun) ---
	if (strncmp(cmd, "PIN.", 4) == 0)
	{
		UniCom_SendErr("UNSUPPORTED");
		return;
	}

	if (strcmp(cmd, "DEV.MOTOR.SET_MODE") == 0)
	{
	    if (strcmp(params, "CONT") == 0) {
	        sm.control_mode = 0u;
	        UniCom_SendAck("DEV.MOTOR.SET_MODE");
	        return;
	    }
	    if (strcmp(params, "OSC") == 0) {
	        sm.control_mode = 1u;
	        UniCom_SendAck("DEV.MOTOR.SET_MODE");
	        return;
	    }
	    UniCom_SendErr("INVALID_PARAM");
	    return;
	}

	if (strcmp(cmd, "DEV.MOTOR.SET_RPM") == 0)
	{
	    float rpm = (float)atof(params);
	    if (rpm < 0) rpm = 0;
	    sm.ref_velocity = rpm;
	    UniCom_SendAck("DEV.MOTOR.SET_RPM");
	    return;
	}

	if (strcmp(cmd, "DEV.MOTOR.OSC.SET_AMP") == 0)
	{
	    float amp_deg = (float)atof(params);
	    if (amp_deg < 0) amp_deg = -amp_deg;
	    sm.ref_position = amp_deg;      // projede amplitude gibi kullanılıyor
	    UniCom_SendAck("DEV.MOTOR.OSC.SET_AMP");
	    return;
	}

	if (strcmp(cmd, "DEV.MOTOR.OSC.SET_RPM") == 0)
	{
	    float rpm = (float)atof(params);
	    if (rpm < 0) rpm = 0;
	    sm.ref_velocity = rpm;
	    UniCom_SendAck("DEV.MOTOR.OSC.SET_RPM");
	    return;
	}

	if (strcmp(cmd, "DEV.MOTOR.OSC.SET_DWELL") == 0)
	{
	    int ms = atoi(params);
	    if (ms < 0) ms = 0;
	    sm.time_delay = (uint16_t)ms;
	    UniCom_SendAck("DEV.MOTOR.OSC.SET_DWELL");
	    return;
	}

	if (strcmp(cmd, "DEV.MOTOR.START") == 0)
	{
	    sm.operation_start = 1u;
	    sm.operation_stop  = 0u;
	    UniCom_SendAck("DEV.MOTOR.START");
	    return;
	}

	if (strcmp(cmd, "DEV.MOTOR.STOP") == 0)
	{
	    sm.operation_stop = 1u;
	    UniCom_SendAck("DEV.MOTOR.STOP");
	    return;
	}

	UniCom_SendErr("INVALID_CMD");
}

void UniCom_TimedTasks(void)
{
	// Timed-run bitiş kontrolü
	if (g_timed_run.active)
	{
		uint32_t now = HAL_GetTick();
		if ((now - g_timed_run.start_ms) >= g_timed_run.duration_ms)
		{
			g_timed_run.active = 0u;
			sm.operation_stop = 1u;
			UniCom_Send("DONE:DEV.MOTOR.EXEC_TIMED_RUN");
		}
	}

	// Telemetry stream
	if (g_stream_enabled)
	{
		uint32_t now = HAL_GetTick();
		if ((now - g_stream_last_ms) >= g_stream_period_ms)
		{
			g_stream_last_ms = now;
			// tx buffer meşgulse çakışma olmasın: basit koruma
			if (!tx_data_ready)
			{
				UniCom_SendMotorStatus();
			}
		}
	}
}
