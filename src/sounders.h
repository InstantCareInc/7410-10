#ifndef SOUNDERS_H
#define SOUNDERS_H

#include <stdint.h>

/**
 * @brief Initialize buzzer/sounder PWM and GPIOs
 * 
 * Initializes:
 * - Buzzer0 (SN1): SMT_0940_T-3V-R with PWM at 4kHz
 * - Buzzer1 (SN2): TE034003-1 with PWM at 4kHz
 * - Buzzer2 (SN3): Unkown sounder (set to 4khz for now)
 * 
 * @return 0 on success, negative errno on failure
 */
int sounders_init(void);

/**
 * @brief Set buzzer 0 state (SMT_0940_T-3V-R with PWM at 4kHz)
 * @param enable 1 to enable (play 4kHz tone), 0 to disable (silence)
 * @return 0 on success, negative errno on failure
 */
int buzzer0_set(int enable);

/**
 * @brief Set buzzer 1 state (TE034003-1 with PWM at 4kHz)
 * @param enable 1 to enable (play 4kHz tone), 0 to disable (silence)
 * @return 0 on success, negative errno on failure
 */
int buzzer1_set(int enable);

/**
 * @brief Set buzzer 2 state (TE034003-1 with PWM at 4kHz)
 * @param enable 1 to enable (play 4kHz tone), 0 to disable (silence)
 * @return 0 on success, negative errno on failure
 */
int buzzer2_set(int value);

/**
 * @brief Set buzzer 0 PWM duty cycle (for volume control)
 * @param duty_cycle_percent Duty cycle percentage (0-100)
 * @return 0 on success, negative errno on failure
 */
int buzzer0_set_volume(uint8_t duty_cycle_percent);

/**
 * @brief Set buzzer 1 PWM duty cycle (for volume control)
 * @param duty_cycle_percent Duty cycle percentage (0-100)
 * @return 0 on success, negative errno on failure
 */
int buzzer1_set_volume(uint8_t duty_cycle_percent);

/**
 * @brief Set buzzer 0 PWM duty cycle (for volume control)
 * @param duty_cycle_percent Duty cycle percentage (0-100)
 * @return 0 on success, negative errno on failure
 */
int buzzer2_set_volume(uint8_t duty_cycle_percent);

#endif /* SOUNDERS_H */