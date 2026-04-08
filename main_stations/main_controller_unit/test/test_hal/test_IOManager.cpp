#include <Arduino.h>
#include <unity.h>
#include "ElevatorHal.h"
#include "Config.h" 

IOManager *io;

void setUp(void)
{
    io = new IOManager(
        PIN_UP, PIN_DOWN, PIN_BRAKE, 
        PIN_SS_FLOOR_1, PIN_SS_FLOOR_2, 
        PIN_EMO, PIN_NO_POWER, PIN_SPEED, PIN_SLING
    );
    io->init_pins();
}

void tearDown(void)
{
    delete io;
}

void test_get_active_floor_logic(void) 
{
    io->status_floor_1 = true;
    io->status_floor_2 = false;
    TEST_ASSERT_EQUAL(1, io->get_active_floor());
    TEST_ASSERT_TRUE(io->is_at_floor(1));

    io->status_floor_1 = false;
    io->status_floor_2 = true;
    TEST_ASSERT_EQUAL(2, io->get_active_floor());
    TEST_ASSERT_TRUE(io->is_at_floor(2));

    io->status_floor_1 = false;
    io->status_floor_2 = false;
    TEST_ASSERT_EQUAL(0, io->get_active_floor());
    TEST_ASSERT_FALSE(io->is_at_floor(1));
    
}


void test_motor_and_brake_outputs(void)
{
    io->motor_rotate(elevator_direction_t::UP);
    TEST_ASSERT_EQUAL(HIGH, digitalRead(PIN_UP));
    TEST_ASSERT_EQUAL(LOW, digitalRead(PIN_DOWN));

    io->motor_rotate(elevator_direction_t::DOWN);
    TEST_ASSERT_EQUAL(LOW, digitalRead(PIN_UP));
    TEST_ASSERT_EQUAL(HIGH, digitalRead(PIN_DOWN));

    io->motor_stop();
    TEST_ASSERT_EQUAL(LOW, digitalRead(PIN_UP));
    TEST_ASSERT_EQUAL(LOW, digitalRead(PIN_DOWN));

    io->engage_brake(true);
    TEST_ASSERT_EQUAL(HIGH, digitalRead(PIN_BRAKE));

    io->engage_brake(false);
    TEST_ASSERT_EQUAL(LOW, digitalRead(PIN_BRAKE));
}


void test_update_sensor_consistency(void)
{
    
    for(int i = 0; i < 20; i++) {
        io->update_sensor();
        delay(1);
    }

    bool is_emo = io->status_emo;
    uint8_t count_emo = io->emo_counter;
    
    if (is_emo) {
        TEST_ASSERT_GREATER_OR_EQUAL(io->STABLE_THRESHOLD, count_emo);
    } else {
        TEST_ASSERT_LESS_THAN(io->STABLE_THRESHOLD, count_emo);
    }
}

// ---------------------------------------------------------
// Main Test Runner
// ---------------------------------------------------------
void setup()
{
    delay(2000); 

    Serial.begin(115200); 
    delay(500);

    UNITY_BEGIN();

    RUN_TEST(test_get_active_floor_logic);
    RUN_TEST(test_motor_and_brake_outputs);
    RUN_TEST(test_update_sensor_consistency);

    UNITY_END();
}

void loop() 
{
}

