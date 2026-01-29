#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include "sounders.h"

LOG_MODULE_REGISTER(sounders, LOG_LEVEL_INF);

/* Buzzer 0 is SMT_0940_T-3V-R - requires PWM at 4kHz */
#define BUZZER0_NODE DT_ALIAS(buzzer0)
/* Buzzer 1 is TE034003-1 - requires PWM at 4kHz */
#define BUZZER1_NODE DT_ALIAS(buzzer1)
/* Buzzer 2 is undefined - Undefined at 4kHz */
#define BUZZER2_NODE DT_ALIAS(buzzer2)

/* Both piezo transducers have 4kHz resonant frequency */
#define PIEZO_FREQUENCY_HZ 4000  /* 4kHz resonant frequency */
#define PIEZO_PERIOD_NS (1000000000UL / PIEZO_FREQUENCY_HZ)  /* ~250000 ns */
#define PIEZO_DEFAULT_DUTY_CYCLE 50  /* 50% duty cycle */

static const struct pwm_dt_spec buzzer0_pwm = PWM_DT_SPEC_GET(BUZZER0_NODE);
static const struct pwm_dt_spec buzzer1_pwm = PWM_DT_SPEC_GET(BUZZER1_NODE);
static const struct pwm_dt_spec buzzer2_pwm = PWM_DT_SPEC_GET(BUZZER2_NODE);

static uint32_t buzzer0_duty_ns = (PIEZO_PERIOD_NS * PIEZO_DEFAULT_DUTY_CYCLE) / 100;
static uint32_t buzzer1_duty_ns = (PIEZO_PERIOD_NS * PIEZO_DEFAULT_DUTY_CYCLE) / 100;
static uint32_t buzzer2_duty_ns = (PIEZO_PERIOD_NS * PIEZO_DEFAULT_DUTY_CYCLE) / 100;


int sounders_init(void)
{
	int ret;

	/* Initialize PWM for buzzer0 (SMT_0940_T-3V-R) */
	if (!pwm_is_ready_dt(&buzzer0_pwm)) {
		LOG_ERR("Buzzer0 PWM device not ready");
		return -ENODEV;
	}
	
	/* Set PWM to 0% duty cycle (off) initially */
	ret = pwm_set_dt(&buzzer0_pwm, PIEZO_PERIOD_NS, 0);
	if (ret < 0) {
		LOG_ERR("Failed to initialize buzzer0 PWM: %d", ret);
		return ret;
	}

	/* Initialize PWM for buzzer1 (TE034003-1) */
	if (!pwm_is_ready_dt(&buzzer1_pwm)) {
		LOG_ERR("Buzzer1 PWM device not ready");
		return -ENODEV;
	}
	
	/* Set PWM to 0% duty cycle (off) initially */
	ret = pwm_set_dt(&buzzer1_pwm, PIEZO_PERIOD_NS, 0);
	if (ret < 0) {
		LOG_ERR("Failed to initialize buzzer1 PWM: %d", ret);
		return ret;
	}

	/* Initialize PWM for buzzer2 (Unknown) */
	if (!pwm_is_ready_dt(&buzzer2_pwm)) {
		LOG_ERR("Buzzer0 PWM device not ready");
		return -ENODEV;
	}
	
	/* Set PWM to 0% duty cycle (off) initially */
	ret = pwm_set_dt(&buzzer2_pwm, PIEZO_PERIOD_NS, 0);
	if (ret < 0) {
		LOG_ERR("Failed to initialize buzzer0 PWM: %d", ret);
		return ret;
	}

	return 0;
}

int buzzer0_set(int enable)
{
	if (enable) {
		/* Start PWM at 4kHz with configured duty cycle */
		return pwm_set_dt(&buzzer0_pwm, PIEZO_PERIOD_NS, buzzer0_duty_ns);
	} else {
		/* Stop PWM (0% duty cycle = silence) */
		return pwm_set_dt(&buzzer0_pwm, PIEZO_PERIOD_NS, 0);
	}
}

int buzzer1_set(int enable)
{
	if (enable) {
		/* Start PWM at 4kHz with configured duty cycle */
		return pwm_set_dt(&buzzer1_pwm, PIEZO_PERIOD_NS, buzzer1_duty_ns);
	} else {
		/* Stop PWM (0% duty cycle = silence) */
		return pwm_set_dt(&buzzer1_pwm, PIEZO_PERIOD_NS, 0);
	}
}

int buzzer2_set(int enable)
{
	if (enable) {
		/* Start PWM at 4kHz with configured duty cycle */
		return pwm_set_dt(&buzzer2_pwm, PIEZO_PERIOD_NS, buzzer2_duty_ns);
	} else {
		/* Stop PWM (0% duty cycle = silence) */
		return pwm_set_dt(&buzzer2_pwm, PIEZO_PERIOD_NS, 0);
	}
}

int buzzer0_set_volume(uint8_t duty_cycle_percent)
{
	if (duty_cycle_percent > 100) {
		duty_cycle_percent = 100;
	}
	
	buzzer0_duty_ns = (PIEZO_PERIOD_NS * duty_cycle_percent) / 100;
	LOG_DBG("Buzzer0 volume set to %d%%", duty_cycle_percent);
	
	/* If buzzer is currently on, update immediately */
	/* Note: This assumes we're tracking state or always updates */
	return 0;
}

int buzzer1_set_volume(uint8_t duty_cycle_percent)
{
	if (duty_cycle_percent > 100) {
		duty_cycle_percent = 100;
	}
	
	buzzer1_duty_ns = (PIEZO_PERIOD_NS * duty_cycle_percent) / 100;
	LOG_DBG("Buzzer1 volume set to %d%%", duty_cycle_percent);
	
	/* If buzzer is currently on, update immediately */
	return 0;
}

int buzzer2_set_volume(uint8_t duty_cycle_percent)
{
	if (duty_cycle_percent > 100) {
		duty_cycle_percent = 100;
	}
	
	buzzer2_duty_ns = (PIEZO_PERIOD_NS * duty_cycle_percent) / 100;
	LOG_DBG("Buzzer2 volume set to %d%%", duty_cycle_percent);
	
	/* If buzzer is currently on, update immediately */
	return 0;
}

static int cmd_set_sounder(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);

    int err = 0;
    int val = shell_strtobool(argv[2], 10, &err) ? 1 : 0;
    if (err) {
		shell_error(sh, "Cannot parse %s. Use 'on' or 'off' to set SN1 state.", argv[2]);
		return -EINVAL;
	}

    if (strcmp(argv[1], "SN1") == 0) {
        err = buzzer0_set(val);
    } else if (strcmp(argv[1], "SN2") == 0) {
        err = buzzer1_set(val);
    } else if (strcmp(argv[1], "SN3") == 0) {
        err = buzzer2_set(val);
    } else {
        shell_error(sh, "%s is not a valid sounder. Available options are: SN1, SN2, SN3", argv[1]);
        return -EINVAL;
    }

    if (err < 0) {
        shell_error(sh, "Issue setting %s with error: %d", argv[1], err);
		return err;
    }

    shell_print(sh, "%s turned %s", argv[1], val ? "ON" : "OFF");
    return 0;
}

static int cmd_volume_sounder(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);

    int err;
    int val = atoi(argv[2]);
    if(val < 0 || val > 100) {
        shell_error(sh, "%d is an invalid volume %%. Please use values 0-100 only.", val);
        return -EINVAL;
    }

    if (strcmp(argv[1], "SN1") == 0) {
        err = buzzer0_set_volume((uint8_t)val);
    } else if (strcmp(argv[1], "SN2") == 0) {
        err = buzzer1_set_volume((uint8_t)val);
    } else if (strcmp(argv[1], "SN3") == 0) {
        err = buzzer2_set_volume((uint8_t)val);
    } else {
        shell_error(sh, "%s is not a valid sounder. Available options are: SN1, SN2, SN3", argv[1]);
        return -EINVAL;
    }

    if (err < 0) {
        shell_error(sh, "Issue setting volume for %s with error: %d", argv[1], err);
		return err;
    }

    shell_print(sh, "%s volume set to %d", argv[1], val);
    return 0;
}

/* Register shell commands */
SHELL_STATIC_SUBCMD_SET_CREATE(sounder_subcmds,
								SHELL_CMD_ARG(set, NULL, "Set Sounder ON/OFF(Usage: sounder set <SN1|SN2|SN3> <on|off>)", cmd_set_sounder, 3, 0),
								SHELL_CMD_ARG(volume, NULL, "Set LED ON/OFF (Usage: sounder volume <SN1|SN2|SN3> <0-100>)", cmd_volume_sounder, 3, 0),
								SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(sounder, &sounder_subcmds, "Sounder control commands", NULL);