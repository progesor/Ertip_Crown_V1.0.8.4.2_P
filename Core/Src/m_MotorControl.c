/*
 * m_MotorControl.c
 *
 *  Created on: Dec 7, 2025
 *      Author: Baris
 */

#include "m_MotorControl.h"
#include "m_SharedMemory.h"
#include "AppConfig.h"

// MotorControl_Task()
volatile float ref_position_deg = 0.0f;
volatile float act_position_offset = 0.0f;   			// Trajectory generation başındaki mutlak pozisyon
volatile uint8_t stall_exists = 0u;		// *** Debug modunda stall a girip girmediğini algılamak için geçici olarak global ekledim.

// ***VelocitySupervisor detection algoritmasını geliştirirken geçici olarak global tanımladım. Normalde local olacak.
volatile float rv = 0.0f;
volatile float rv1 = 0.0f;

// Position kontrol için (P kontrolör)
float sat_limit_max_position = REF_VELOCITY_MAX;	// Pozisyon kontrol döngüsündeki maks. hız saturasyon limiti (rpm)

// Velocity kontrol için (PI kontrolör)
float k_anti_windup = 0.0f;					// Hız kontrol K_Anti_Windup kazancı *** TI çarpılacak
float ui_vel = 0.0f;						// İntegratör çıkış değeri
float uo_vel_sat0 = 0.0f;

// TrajectoryGeneratorVel()
static float ramped_ref_rpm = 0.0f;

// TrajectoryGeneratorPosRel()
//volatile float traj_pos = 0.0f;      						// Anlık Yörünge Konumu (Sanal Lider)
//volatile float traj_vel = 0.0f;      						// Anlık Yörünge Hızı (Feed Forward için)
//volatile uint8_t target_index = 0;   						// 0: Min konuma git, 1: Max konuma git


// StallSupervisor()
StallState_t act_stall_state = STALL_IDLE;
uint16_t stall_suppress_cnt = 0u;
uint16_t stall_wait_cnt = 0u;
uint8_t stall_detected = 0u;	// *** Debug modunda stall a girip girmediğini algılamak için geçici olarak global ekledim.

void MotorControl_Task(void)
{
	switch(sm.act_motor_state)
	{
		case MOT_STATE_IDLE:
		{
			DriveMotor(0.0f, 1u, 1u);
		} break;

		case MOT_STATE_POS_CONTROL_INIT:
		{
			InitControlVel();
			InitControlPos();

			sm.act_motor_state = MOT_STATE_POS_CONTROL;
		} break;

		case MOT_STATE_POS_CONTROL:
		{
			// 1) Her tick’te relative uzaydaki act_position değerini günceller.
			sm.act_position_relative = sm.act_position - act_position_offset;

			// 2) Kaskad Konum-Hız Kontrol
			// a- Relative uzaydaki ref_position değerini konum kontrolcüye uygular.
			rv = PositionControl(ref_position_deg, sm.act_position, sm.ref_velocity);

			// b- Konum kontrolcüde hesaplanan hız referansını yumuşatarak hız kontrolcüye uygular.
			//rv1 = TrajectoryGeneratorVel(rv, TRAGEN_VEL_ACCEL_RPM_S);

			// c- Motor milinde sıkışma (stall) varsa kontrolü durdur
			//if (StallSupervisor(sm.ref_velocity, sm.act_velocity, sm.pwm_duty_cycle, sm.act_position_relative)) break;  // supervisor DriveMotor çağırdı; normal kontrolü atla
//			stall_exists = VelocitySupervisor(sm.ref_velocity, sm.act_velocity, sm.pwm_duty_cycle);
//			stall_exists = PositionSupervisor(sm.ref_position, sm.act_position_relative, sm.pwm_duty_cycle);
//			if(stall_exists) break;
			//if(VelocitySupervisor(sm.ref_velocity, sm.act_velocity, sm.pwm_duty_cycle)) break;

			//stall_exists = StallSupervisor(sm.act_motor_current, sm.act_pwm_duty, rv1, sm.act_velocity);
			stall_exists = StallSupervisor(sm.act_motor_current, sm.act_pwm_duty, sm.ref_position, sm.act_position_relative);
			if(stall_exists) break;


			// d- Konum kontrolcünün ürettiği referans hız ve yörünge hesabından gelen hız ileri besleme
			// sinyallerini hız kontrolcüye uygula
			VelocityControl(rv, sm.act_velocity, 0.0f);
			//VelocityControl(rv1, sm.act_velocity, rv1);

			// 3) Osilasyon yapılabilmesi için bir yöndeki hareketin hedefe ulaşmadaki hata değerini heasplar.
			float act_pos_error_rel = ref_position_deg - sm.act_position;

			// 4) Osilasyon yapılabilmesi için hedef konum hatasının ölü bölge içine girip girmediğini denetler.
			// Konum hatası ölü bölgeye girmişse yön değiştirir.
			if(fabsf(act_pos_error_rel) < TRAGEN_POS_ERROR_DEADBAND_DEG)
			{
				InitControlVel();

				static uint8_t toggle_direction = 0u;
				if(toggle_direction)
				{
					ref_position_deg =  sm.ref_position + act_position_offset;	// Hedef konuma relative uzaydaki ref_position konumunu atar.
					toggle_direction = 0u;
				}
				else
				{
					ref_position_deg = act_position_offset;						// Hedef konuma osilasyon başlangıç konumunu atar.
					toggle_direction = 1u;
				}
			}

		} break;

		case MOT_STATE_POS_CONTROL_END:
		{
			VelocityControl(0.0f, sm.act_velocity, 0.0f);

			if(fabsf(sm.act_velocity) < VEL_ERROR_DEADBAND)
			{
				DriveMotor(0.0f, 1u, 0u);
				sm.act_motor_state = MOT_STATE_IDLE;
			}

		} break;

		case MOT_STATE_VEL_CONTROL_INIT:
		{
			InitControlVel();
			sm.act_motor_state = MOT_STATE_VEL_CONTROL;
		} break;

		case MOT_STATE_VEL_CONTROL:
		{
			float rv = TrajectoryGeneratorVel(sm.ref_velocity, TRAGEN_VEL_ACCEL_RPM_S);
			VelocityControl(rv, sm.act_velocity, 0.0f);
		} break;

		case MOT_STATE_VEL_CONTROL_END:
		{
			float rv = TrajectoryGeneratorVel(0.0f, TRAGEN_VEL_ACCEL_RPM_S);
			VelocityControl(rv, sm.act_velocity, 0.0f);
			//VelocityControl(0.0f, sm.act_velocity, 0.0f);
			if(fabsf(sm.act_velocity) < VEL_ERROR_DEADBAND)
			{
				DriveMotor(0.0f, 1u, 0u);
				sm.act_motor_state = MOT_STATE_IDLE;
			}
		} break;

		case MOT_STATE_MANUAL_CONTROL:
		{
			//float dc = sm.duty_cycle_command;
			////float dc = sm.act_pot_value * K_POT2PWM_VALUE;
			//DriveMotor(dc, 1u, 1u);
			float dc = sm.duty_cycle_command;
			DriveMotor(dc, sm.motor_direction, 1u);   // brake_status=1 => brake OFF
		} break;

	}
}

void InitControlVel(void)
{
	k_anti_windup = 1.0f * (sm.ki_velocity / sm.kp_velocity);	// VelocityControl() içindeki Anti-Windup katsayısını günceller.
	ui_vel = 0.0f;				// VelocityControl() içindeki integral kontrolcü çıkışını resetler.
	uo_vel_sat0 = 0.0f;			// VelocityControl() içindeki kontrol sinyali çıkışında yer alan LPF eski çıkış değerini resetler.
	ramped_ref_rpm = 0.0f;		// TrajectoryGeneratorVel() içindeki rampa referans değerini sıfırlar.
}

void InitControlPos(void)
{
	act_position_offset = sm.act_position;	// Başlangıç mutlak konumu al, offset olarak ayarlar. (relative referansın orijini)
	ref_position_deg = sm.ref_position + act_position_offset;	// Realtive uzayda tanımlı ref_position ı mutlak uzaya taşır.
	sm.act_position_relative = 0.0f;		// Relative uzaydaki relative konum hesabında kullanılır.
	stall_suppress_cnt = 0u;
	stall_wait_cnt = 0u;
	stall_detected = 0u;
}

float PositionControl(float ref_pos_deg, float act_pos_deg, float uo_vel_limit)
{
	// 1) Hata: referans derece - ölçülen derece hesaplanır.
	float error = ref_pos_deg - act_pos_deg;

	// 2) Oransal kontrolcü çıkışı hesaplanır ve konum ileri besleme eklenir.
	float up_pos = sm.kp_position * error;

	// 3) P kontrolcü çıkışı HIZ doyum sınırlarına göre değerlendirilir.
//	if(up_pos > POS_UO_SAT_LIMIT_MAX) up_pos = POS_UO_SAT_LIMIT_MAX;
//	else if(up_pos < POS_UO_SAT_LIMIT_MIN) up_pos = POS_UO_SAT_LIMIT_MIN;

	if(up_pos > uo_vel_limit) up_pos = uo_vel_limit;
	else if(up_pos < -uo_vel_limit) up_pos = -uo_vel_limit;

	return up_pos;
}

void VelocityControl(float ref_vel_rpm, float act_vel_rpm, float ff_vel_rpm)
{
	uint8_t dir = 1;

	// 1) Gelen velocity reference < 0 mı bakılır.
	if(ref_vel_rpm < 0)
	{
		dir = 0;
		ref_vel_rpm *= -1.0f;
		act_vel_rpm *= -1.0f;
		ff_vel_rpm *= -1.0f;
	}

	float ref_vel_norm = ref_vel_rpm * K_INV_VELOCITY;
	float act_vel_norm = act_vel_rpm * K_INV_VELOCITY;
	float ff_vel_norm = ff_vel_rpm * K_INV_VELOCITY;

	// 2) Hata: referans rpm - ölçülen rpm hesaplanır.
	float error = ref_vel_norm - act_vel_norm;

	// 3) Oransal kontrolcü çıkışı hesaplanır.
	float up_vel = sm.kp_velocity * error;
//	if(stall_detected) up_vel *= 20.0f;		// Eğer motor milinde sıkışma varsa gücü yarıya düşür.

	// 4) İntegral kontrolcü çıkışı hesaplanır.
	if(!stall_detected)
	{
		ui_vel += sm.ki_velocity * error;

		// Integrator kırpma (clamp) uygulanır.
		if(ui_vel >  VEL_UI_CLAMP_LIMIT_MAX) ui_vel = VEL_UI_CLAMP_LIMIT_MAX;
		if(ui_vel < VEL_UI_CLAMP_LIMIT_MIN) ui_vel = VEL_UI_CLAMP_LIMIT_MIN;
	}

	// 5) Kuru sürtünmeyi yenmek için Hız İleri Besleme ayarı yapılır.
	float friction_compansator = 0.0f;
	if(ref_vel_rpm > VEL_FRICTION_FF_LIMIT_RPM) friction_compansator = 0.0f; // friction_compansator = VEL_FRICTION_FF_DUTY;

	// 6) PI kontrolcü çıkışı hesaplanır, hız ileri besleme ve friction compensator eklenir.
	float uo_vel = up_vel + ui_vel + ff_vel_norm + friction_compansator;

	// 7) PI kontrolcü çıkışı PWM Periyodu doyum sınırlarına göre değerlendirilir.
	float uo_vel_sat = uo_vel;
	if(uo_vel_sat > VEL_UO_SAT_LIMIT_MAX) uo_vel_sat = VEL_UO_SAT_LIMIT_MAX;
	else if(uo_vel_sat < VEL_UO_SAT_LIMIT_MIN) uo_vel_sat = VEL_UO_SAT_LIMIT_MIN;

	// 8) Back-calculation anti-windup değeri hesaplanır.
    // (u_sat - u) farkı, integratörün ne kadar sarmaya çalıştığını gösterir.
	// İntegratör durumunu güncelleyip anti-windup terimini ekle.
	// Bu, integrator_new'un aşırıya kaçmasını engeller.
	if(!stall_detected)
	{
		ui_vel += k_anti_windup * (uo_vel_sat - uo_vel);

		// Integrator kırpma (clamp) uygulanır.
		if(ui_vel >  VEL_UI_CLAMP_LIMIT_MAX) ui_vel = VEL_UI_CLAMP_LIMIT_MAX;
		if(ui_vel < VEL_UI_CLAMP_LIMIT_MIN) ui_vel = VEL_UI_CLAMP_LIMIT_MIN;
	}

//	uo_vel_sat = LowPassFilter(uo_vel_sat0, uo_vel_sat, K_LPF_UO_VEL);
//	uo_vel_sat0 = uo_vel_sat;

	// 9) Sonucu 0.0 ile 1.0 arasında PWM duty cycle olarak geri döndür.
	uint8_t brake = 1;	// 0: Fren aktif; 1: Fren yok
	//brake = (uo_vel_sat == 0) ? 0 : 1;
	DriveMotor(uo_vel_sat, dir, brake);
}

float TrajectoryGeneratorVel(float target_rpm, float accel_limit_rpm_s)
{
	if (!isfinite(target_rpm))  target_rpm = 0.0f;
    if (!isfinite(accel_limit_rpm_s)) accel_limit_rpm_s = 0.0f;
	accel_limit_rpm_s = fabsf(accel_limit_rpm_s);

	// 1. Bir adımda değişebilecek maksimum hız (RPM)
    float max_delta = accel_limit_rpm_s * CONTROL_LOOP_PERIOD;

    // 2. Hedef ile o anki rampa değeri arasındaki fark
    float err = target_rpm - ramped_ref_rpm;

    // 3. Rampa değerini güncelle (Slew Rate Limiter Mantığı)
    if (err > max_delta)
    {
        ramped_ref_rpm += max_delta; // Pozitif yönde hızlan
    }
    else if (err < -max_delta)
    {
        ramped_ref_rpm -= max_delta; // Negatif yönde hızlan (veya yavaşla)
    }
    else
    {
        ramped_ref_rpm = target_rpm; // Hedefe çok yakınız, eşitle.
    }
    return ramped_ref_rpm;
}

uint8_t StallSupervisor(float current, float pwm_duty, float ref_pos, float act_pos)
{
	//float pos_error = fabsf(ref_pos - act_pos);
	float pos_error_limit = ref_pos * 0.1f;

	// 1. Algılama Mantığı:
	// Akım çok yüksekse VEYA (PWM doyumda VE Hız hatası büyükse)
	//if( (current > STALL_CURRENT_LIMIT) || (pwm_duty > STALL_PWM_LIMIT) || (pos_error < -STALL_POS_ERROR_LIMIT) )
	if( (act_pos > ref_pos + pos_error_limit) || (act_pos < 0 - pos_error_limit) )
	{
		stall_detected = 1u;
	}

	switch(act_stall_state)
	{
		case STALL_IDLE:
		{
			if(stall_detected)
			{
				act_stall_state = STALL_SUPPRESS; // Anında baskılamaya geç
				stall_suppress_cnt = 0u;
			}
		} break;

		case STALL_SUPPRESS:	// Bu modda VelocityControl, Integratörü sıfırlayacak ve Kp'yi artıracak.
		{
			if (stall_detected)
			{
				stall_suppress_cnt++;
				ui_vel = 0.0f;				// integratör birikimini sıfırla

				// Sıkışma çok uzun sürerse motoru korumaya al
				if(stall_suppress_cnt > STALL_SUPPRESS_TIME_TICKS)
				{
					stall_suppress_cnt = 0u;
					stall_wait_cnt = 0u;
					act_stall_state = STALL_PROTECT;
				}
			}
			else
			{
				stall_detected = 0u;
				act_stall_state = STALL_IDLE;
				// Yük kalktıysa normale dön (Histeresis eklenebilir)
			}
		} break;

		case STALL_PROTECT:		// Bu moddaa motor tamamen durdurulur (PWM = 0)
		{
			stall_wait_cnt++;
			ui_vel = 0.0f;				// integratör birikimini sıfırla
			DriveMotor(0.0f, 1u, 0u);	// Motoru durdur (yumuşak: brake off, duty=0)

			if(stall_wait_cnt > STALL_WAIT_TIME_TICKS)
			{
				stall_wait_cnt = 0u;
				stall_detected = 0u;
				act_stall_state = STALL_IDLE;
				// Çıkışta yumuşak başlangıç için hedef konum güncellenebilir
			}
			return 1;	// VelocityControl'ü atla (Motoru durdur)
		} break;
	}

	return 0;	// Normal kontrol çalışsın (SUPPRESS modunda da çalışır)
}


