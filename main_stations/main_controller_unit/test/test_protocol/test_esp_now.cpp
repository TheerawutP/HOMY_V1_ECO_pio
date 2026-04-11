#include <Arduino.h>
#include <unity.h>
#include "EspNowManager.h"

EspNow* espNowBroker;

void setUp(void) {
    espNowBroker = new EspNow();
    espNowBroker->init();
}

void tearDown(void) {
    delete espNowBroker;
}

// --- Test 1: ระบบลงทะเบียนและการลบ ---
void test_registration_and_deletion(void) {
    uint8_t dummy_mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    station_info_t cabin_station = {station_role_t::CABIN, {0}};
    memcpy(cabin_station.mac, dummy_mac, 6);

    // ลงทะเบียนควรจะผ่าน
    TEST_ASSERT_TRUE(espNowBroker->regist_station(cabin_station));

    // ส่งข้อมูลไป CABIN ควรพยายามส่ง (ถึงจะส่งไม่สำเร็จจริงๆ ก็จะคืนค่า fail ไม่ใช่ error จากการไม่เจอ MAC)
    // ตรงนี้อาจจะไม่เทสต์ send_command ถ้าไม่ได้ต่อ WiFi ไว้
    
    // ลบออก
    espNowBroker->del_station(station_role_t::CABIN);

    // ลองดึงข้อมูล Mailbox ต้องไม่ได้ เพราะลบไปแล้ว
    espnow_msg_t empty_msg;
    TEST_ASSERT_FALSE(espNowBroker->get_latest_data(station_role_t::CABIN, &empty_msg));
}

// --- Test 2: จำลองการรับข้อมูล (Auto Pairing & Mailbox) ---
void test_simulated_receive_and_auto_pairing(void) {
    uint8_t dummy_mac[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
    
    // สร้างแพ็กเกจข้อมูลปลอม สมมติว่ามาจาก VSG (Role 6)
    espnow_msg_t fake_incoming;
    fake_incoming.fromID = (uint8_t)station_role_t::VSG;
    fake_incoming.commandFrame = 0;
    fake_incoming.responseFrame = 0xABCD; // สมมติว่าส่ง Status 0xABCD มา
    fake_incoming.shouldResponse = false;

    // 💡 ทริค: แกล้งเรียก Static Callback ของ C++ ด้วยตัวเอง (เสมือนมีคลื่นวิทยุวิ่งเข้ามาจริง)
    // เราต้อง cast คลาสนี้ไปเรียกฟังก์ชัน static แบบดื้อๆ
    EspNow::OnDataRecvStatic(dummy_mac, (const uint8_t*)&fake_incoming, sizeof(fake_incoming));

    // ดึงค่ามาเช็ค
    espnow_msg_t mailbox_data;
    bool has_data = espNowBroker->get_latest_data(station_role_t::VSG, &mailbox_data);

    // ตรวจสอบ
    TEST_ASSERT_TRUE(has_data);
    TEST_ASSERT_EQUAL(0xABCD, mailbox_data.responseFrame); // ข้อมูลต้องตรงกัน
}


void test_queue_fifo_processing(void) {
    uint8_t dummy_mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    
    // สร้างพัสดุ 3 กล่องที่มีข้อมูลต่างกัน
    espnow_msg_t msg1 = { (uint8_t)station_role_t::CABIN, 0, 100, false };
    espnow_msg_t msg2 = { (uint8_t)station_role_t::INVERTER, 0, 200, false };
    espnow_msg_t msg3 = { (uint8_t)station_role_t::VSG, 0, 300, false };

    // แกล้งจำลองว่ามีข้อมูลวิ่งเข้ามา 3 ครั้งติดกัน (สมมติสถานการณ์ชุลมุน)
    EspNow::OnDataRecvStatic(dummy_mac, (const uint8_t*)&msg1, sizeof(msg1));
    EspNow::OnDataRecvStatic(dummy_mac, (const uint8_t*)&msg2, sizeof(msg2));
    EspNow::OnDataRecvStatic(dummy_mac, (const uint8_t*)&msg3, sizeof(msg3));

    espnow_msg_t out_msg;
    
    // ดึงครั้งที่ 1: ต้องได้ข้อมูลของ CABIN (100)
    TEST_ASSERT_TRUE(espNowBroker->receive_message(&out_msg));
    TEST_ASSERT_EQUAL(100, out_msg.responseFrame);

    // ดึงครั้งที่ 2: ต้องได้ข้อมูลของ INVERTER (200)
    TEST_ASSERT_TRUE(espNowBroker->receive_message(&out_msg));
    TEST_ASSERT_EQUAL(200, out_msg.responseFrame);

    // ดึงครั้งที่ 3: ต้องได้ข้อมูลของ VSG (300)
    TEST_ASSERT_TRUE(espNowBroker->receive_message(&out_msg));
    TEST_ASSERT_EQUAL(300, out_msg.responseFrame);

    // ดึงครั้งที่ 4: คิวต้องว่างแล้ว ต้องคืนค่า false
    TEST_ASSERT_FALSE(espNowBroker->receive_message(&out_msg));
}

// --- Test 4: ทดสอบการป้องกัน Error เวลาสั่งส่งข้อมูลผิดพลาด ---
void test_send_command_error_handling(void) {
    // 1. ลองส่งไปหาบอร์ดที่เรารู้จัก (HALL_1) แต่ "ยังไม่ได้ลงทะเบียน"
    // ระบบต้องคืนค่า false ทันที โดยไม่ไปกวน ESP-NOW Hardware
    bool send_unregistered = espNowBroker->send_command(station_role_t::HALL_1, 0xFFFF);
    TEST_ASSERT_FALSE(send_unregistered);

    // 2. ลองส่งไปหา Role มั่วๆ ที่อยู่นอกเหนือจาก 0-9 (Array Out of Bound)
    // ระบบต้องไม่ Crash และคืนค่า false ออกมาอย่างปลอดภัย
    station_role_t invalid_role = (station_role_t)99;
    bool send_invalid = espNowBroker->send_command(invalid_role, 0xFFFF);
    TEST_ASSERT_FALSE(send_invalid);
}

void setup() {
    delay(2000);
    Serial.begin(115200);
    delay(5000); // รอ PIO Monitor (ตามที่เราเคยแก้บั๊กกัน)

    UNITY_BEGIN();
    RUN_TEST(test_registration_and_deletion);
    RUN_TEST(test_simulated_receive_and_auto_pairing);
    RUN_TEST(test_queue_fifo_processing);     
    RUN_TEST(test_send_command_error_handling); 
    UNITY_END();
}

void loop() {}