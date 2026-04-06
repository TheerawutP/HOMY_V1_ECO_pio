#include <Arduino.h>
#include <unity.h>
#include "ElevatorLogic.h"
#include "ElevatorHal.h"
#include "ElevatorTypes.h"


class MockHal : public ElevatorHal {
public:
    uint8_t simulated_floor = 1; 
    bool is_motor_running = false;
    elevator_direction_t current_dir = elevator_direction_t::NONE;
    // bool is_brake_engaged = true;

    void init_pins() override {}
    
    void motor_rotate(elevator_direction_t dir) override {
        is_motor_running = true;
        current_dir = dir;
    }
    
    void motor_stop() override {
        is_motor_running = false;
        current_dir = elevator_direction_t::NONE;
    }
    
    // void engage_brake(bool engage) override {
    //     is_brake_engaged = engage;
    // }
    
    void update_sensor() override {} 
    
    bool is_at_floor(uint8_t floor) override {
        return simulated_floor == floor;
    }
    
    uint8_t get_active_floor() override {
        return simulated_floor;
    }
};

MockHal* mockHal;
Orchestrator* logic;

void setUp(void) {
    mockHal = new MockHal();
    logic = new Orchestrator(mockHal);
}

void tearDown(void) {
    delete logic;
    delete mockHal;
}

void test_normal_transit_and_stop_at_target(void) {
    mockHal->simulated_floor = 1;
    logic->update(); 
    
    user_command cmd = {2, command_type_t::TRANSIT};
    logic->user_command_handle(cmd);
    
    logic->update();
    
    TEST_ASSERT_EQUAL(elevator_state_t::RUNNING, logic->data.current_state);
    TEST_ASSERT_TRUE(mockHal->is_motor_running);
    TEST_ASSERT_EQUAL(elevator_direction_t::UP, mockHal->current_dir);
    // TEST_ASSERT_FALSE(mockHal->is_brake_engaged);

    mockHal->simulated_floor = 2;
    logic->update(); 
    
    TEST_ASSERT_EQUAL(elevator_state_t::IDLE, logic->data.current_state);
    TEST_ASSERT_FALSE(mockHal->is_motor_running);
    // TEST_ASSERT_TRUE(mockHal->is_brake_engaged);
}


void test_soft_stop_midway(void) {
    mockHal->simulated_floor = 1;
    logic->update();
    
    user_command cmd1 = {2, command_type_t::TRANSIT};
    logic->user_command_handle(cmd1);
    mockHal->simulated_floor = 0; 
    logic->update();
    
    TEST_ASSERT_EQUAL(elevator_state_t::RUNNING, logic->data.current_state);
    TEST_ASSERT_TRUE(mockHal->is_motor_running);

    user_command cmd2 = {0, command_type_t::STOP};
    logic->user_command_handle(cmd2);
    logic->update(); 
    
    TEST_ASSERT_EQUAL(elevator_state_t::IDLE, logic->data.current_state);
    TEST_ASSERT_FALSE(mockHal->is_motor_running);
    // TEST_ASSERT_TRUE(mockHal->is_brake_engaged);
    TEST_ASSERT_EQUAL(0, logic->data.target); 
}



void setup() {
    delay(2000); 
    UNITY_BEGIN();

    RUN_TEST(test_normal_transit_and_stop_at_target);
    RUN_TEST(test_soft_stop_midway);

    UNITY_END();
}

void loop() {}