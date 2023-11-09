#include <core_pins.h>

void config_SPEED_HOIST_PIN();
void config_LOWERING_RELAY_PIN();
void config_RAISING_RELAY_PIN();
void config_EN_RELAY_PIN();
void config_RAISE_HOIST_PIN();
void config_LOWER_HOIST_PIN();
void set_pwm_speed(uint8_t duty_cycle);
void set_raise_mode();
void set_lower_mode();
void stop_all();
void stop();
void off();
void config_all_pin();
void setup_hoistController();