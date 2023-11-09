#include "HoistController.h"
//Controlling the Hoist
//TEST MACRO:
#define RAISE_MODE_TEST 1
#define LOWER_MODE_TEST 2
#define SPEED_WRITE_TEST 3
#define STOP_MODE_TEST 4
#define TEST_CASE RAISE_MODE_TEST
////////////////////////////////////////////////
#define EN_RELAY_PIN 15
#define SPEED_HOIST_PIN 19
#define LOWERING_RELAY_PIN 16
#define RAISING_RELAY_PIN 14
#define RAISE_HOIST_PIN 37
#define LOWER_HOIST_PIN 36
#define LOWER_IND_LED 6
#define RAISE_IND_LED 7

void config_SPEED_HOIST_PIN()
{
  pinMode(SPEED_HOIST_PIN, OUTPUT);
  analogWriteFrequency(SPEED_HOIST_PIN, 1500);
}

void config_LOWERING_RELAY_PIN()
{
  pinMode(LOWERING_RELAY_PIN, OUTPUT);
  digitalWrite(LOWERING_RELAY_PIN, HIGH);
}

void config_RAISING_RELAY_PIN()
{
  pinMode(RAISING_RELAY_PIN, OUTPUT);
  digitalWrite(RAISING_RELAY_PIN, HIGH);
}

void config_EN_RELAY_PIN()
{
  pinMode(EN_RELAY_PIN, OUTPUT);
  digitalWrite(EN_RELAY_PIN, HIGH);
  
}

void config_RAISE_HOIST_PIN()
{
  pinMode(RAISE_HOIST_PIN, OUTPUT);
  pinMode(RAISE_IND_LED, OUTPUT);
}

void config_LOWER_HOIST_PIN()
{
  pinMode(LOWER_HOIST_PIN, OUTPUT);
  pinMode(LOWER_IND_LED, OUTPUT);
}
void set_pwm_speed(uint8_t duty_cycle)
{
  //digitalWrite(EN_RELAY_PIN, HIGH);
  analogWrite(SPEED_HOIST_PIN, (float) duty_cycle / 100 * 255);
}

void set_raise_mode()
{
  digitalWrite(LOWERING_RELAY_PIN, HIGH);
  digitalWrite(RAISING_RELAY_PIN, HIGH);
  digitalWrite(LOWER_HOIST_PIN, LOW);
  digitalWrite(RAISE_HOIST_PIN, HIGH);
  digitalWrite(LOWER_IND_LED, LOW);
  digitalWrite(RAISE_IND_LED, HIGH);
}

void set_lower_mode()
{
  
  digitalWrite(RAISING_RELAY_PIN, HIGH);
  digitalWrite(LOWERING_RELAY_PIN, HIGH);
  digitalWrite(RAISE_HOIST_PIN, LOW);
  digitalWrite(LOWER_HOIST_PIN, HIGH);
  digitalWrite(RAISE_IND_LED, LOW);
  digitalWrite(LOWER_IND_LED, HIGH);
}
void off()
{
  digitalWrite(RAISING_RELAY_PIN, LOW);
  digitalWrite(LOWERING_RELAY_PIN, LOW);
}

void stop_all()
{
  digitalWrite(EN_RELAY_PIN, LOW);
  digitalWrite(LOWER_HOIST_PIN, LOW);
  digitalWrite(RAISE_HOIST_PIN, LOW);
  digitalWrite(LOWERING_RELAY_PIN, LOW);
  digitalWrite(RAISING_RELAY_PIN, LOW);
  analogWrite(SPEED_HOIST_PIN, 0);
}

void stop()
{
  digitalWrite(LOWER_HOIST_PIN, LOW);
  digitalWrite(RAISE_HOIST_PIN, LOW);
  analogWrite(SPEED_HOIST_PIN, 0);
  digitalWrite(LOWER_IND_LED, LOW);
  digitalWrite(RAISE_IND_LED, LOW);
}
void config_all_pin()
{
  config_SPEED_HOIST_PIN();
  config_LOWERING_RELAY_PIN();
  config_RAISING_RELAY_PIN();
  config_EN_RELAY_PIN();
  config_RAISE_HOIST_PIN();
  config_LOWER_HOIST_PIN();
}

void setup_hoistController()
{
  config_SPEED_HOIST_PIN();
  config_LOWERING_RELAY_PIN();
  config_RAISING_RELAY_PIN();
  config_EN_RELAY_PIN();
  config_RAISE_HOIST_PIN();
  config_LOWER_HOIST_PIN();
}
