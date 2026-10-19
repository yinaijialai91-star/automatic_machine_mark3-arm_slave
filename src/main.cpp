#include <Wire.h>
#include <SCServo.h>

#define I2C_DEV_ADDR 0x67

#define TX_PIN TX
#define RX_PIN RX

#define ID1_FIRST 3000
#define ID1_SECOND 2100
#define ID1_THIRD 1100
#define ID1_KYUIN 950

#define ID2_FIRST 3000
#define ID2_SECOND 2011
#define ID2_THIRD 2100

#define ID3_FIRST 3100
#define ID3_SECOND 2000
#define ID3_THIRD 1100
#define ID3_KYUIN 2100

#define ID4_FIRST 3100
#define ID4_SECOND 2000
#define ID4_THIRD 1100
#define ID4_KYUIN 2100

#define ARM_ID_1 5
#define ARM_ID_2 6
#define ARM_ID_3 7
#define ARM_ID_4 8

#define last_1 1440
#define last_2 2140 // 2300
#define last_3 1950
#define last_4 1950

int DATA[8] = {0};

SMS_STS st;

void servo_setup();
void servo_setup_sync();
void arm_move_first();
void arm_move_second();
void arm_move_third();
void arm_move_fourth();
void arm_move_last();
void arm_first_set();
void arm_move_first_sync();
void arm_move_second_sync();
void arm_move_last_sync();

void onReceive(int len)
{
    Serial.print("Received len: ");
    Serial.println(len);
    for (int i = 0; i < len && i < 8; i++)
    {
        DATA[i] = Wire.read();
        Serial.print("DATA[");
        Serial.print(i);
        Serial.print("]: ");
        Serial.println(DATA[i], HEX);
    }

    switch (DATA[0])
    {
    case 0x01:
        arm_move_first_sync();
        break;

    case 0x02:
        arm_move_second_sync();
        break;

    case 0x03:
        arm_move_last_sync();
        break;

    default:
        break;
    }
}

void servo_setup()
{

    Serial1.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN);
    st.pSerial = &Serial1;
    vTaskDelay(pdMS_TO_TICKS(1000));

    for (int i = 1; i <= 8; i++)
    {

        // st.unLockEprom(i); // EEPROMのロック解除
        // st.writeByte(i, SMS_STS_MODE, 0);
        // st.LockEprom(i); // EEPROMを再ロック
        // st.EnableTorque(i, 1);

        delay(1);
    }

    st.RegWritePosEx(ARM_ID_1, ID1_FIRST, 3400);
    st.RegWritePosEx(ARM_ID_2, ID2_FIRST, 3400);
    st.RegWritePosEx(ARM_ID_3, ID3_FIRST, 3400);
    st.RegWritePosEx(ARM_ID_4, ID4_FIRST, 3400);

    st.RegWriteAction();
}

void servo_setup_sync()
{
    Serial1.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN);
    st.pSerial = &Serial1;

    u8 ID[4] = {5, 6, 7, 8};
    s16 POSITION[4] = {ID1_FIRST, ID2_FIRST, ID3_FIRST, ID4_FIRST};
    u16 speed[4] = {3400, 3400, 3400, 3400};
    u8 acc[4] = {50, 50, 50, 50};

    st.SyncWritePosEx(ID, sizeof(ID), POSITION, speed, acc);
}

void arm_move_first()
{
    st.RegWritePosEx(ARM_ID_3, ID3_SECOND, 3400);
    st.RegWritePosEx(ARM_ID_4, ID4_SECOND, 3400);

    st.RegWriteAction();

    delay(1500);

    st.RegWritePosEx(ARM_ID_1, ID1_SECOND, 3400);
    st.RegWritePosEx(ARM_ID_2, ID2_SECOND, 3400);

    st.RegWriteAction();

    return;
}

void arm_move_second()
{

    st.WritePosEx(ARM_ID_1, ID1_THIRD, 3400);

    delay(3000);

    st.WritePosEx(ARM_ID_1, ID1_KYUIN, 3400);

    delay(1000);

    st.RegWritePosEx(ARM_ID_3, ID3_KYUIN, 3400);
    st.RegWritePosEx(ARM_ID_4, ID4_KYUIN, 3400);

    st.RegWriteAction();

    delay(1000);

    st.WritePosEx(ARM_ID_1, ID1_SECOND, 2500);

    delay(1500);

    st.RegWritePosEx(ARM_ID_3, ID3_THIRD, 3400);
    st.RegWritePosEx(ARM_ID_4, ID4_THIRD, 3400);

    st.RegWriteAction();
}

void arm_move_third()
{
    st.RegWritePosEx(ARM_ID_3, ID3_SECOND, 3400);
    st.RegWritePosEx(ARM_ID_4, ID4_SECOND, 3400);

    st.RegWriteAction();

    delay(1500);

    st.WritePosEx(ARM_ID_1, ID1_THIRD, 3400);
}

void arm_move_fourth()
{
    st.WritePosEx(ARM_ID_1, ID1_SECOND, 3400);
}

void arm_move_last()
{
    st.RegWritePosEx(ARM_ID_1, last_1, 3400);
    st.RegWritePosEx(ARM_ID_2, last_2, 3400);
    st.RegWritePosEx(ARM_ID_3, last_3, 3400);
    st.RegWritePosEx(ARM_ID_4, last_4, 3400);

    st.RegWriteAction();
}

void arm_first_set()
{
    st.RegWritePosEx(ARM_ID_1, ID1_FIRST, 3400);
    st.RegWritePosEx(ARM_ID_2, ID2_FIRST, 3400);
    st.RegWritePosEx(ARM_ID_3, ID3_FIRST, 3400);
    st.RegWritePosEx(ARM_ID_4, ID4_FIRST, 3400);

    st.RegWriteAction();
}

void arm_move_first_sync()
{
    u8 ID[] = {7, 8};
    s16 POSITION[] = {ID3_SECOND, ID4_SECOND};
    u16 speed[4] = {3400, 3400, 3400, 3400};
    u8 acc[4] = {50, 50, 50, 50};

    st.SyncWritePosEx(ID, sizeof(ID), POSITION, speed, acc);

    delay(1500);

    u8 ID_[] = {5, 6};
    s16 POSITION_[] = {ID1_SECOND, ID2_SECOND};

    st.SyncWritePosEx(ID_, sizeof(ID_), POSITION_, speed, acc);

    return;
}

void arm_move_second_sync()
{

    st.WritePosEx(ARM_ID_1, ID1_THIRD, 3400);

    delay(3000);

    st.WritePosEx(ARM_ID_1, ID1_KYUIN, 3400);

    delay(1000);

    u8 ID[] = {7, 8};
    s16 POSITION[] = {ID3_KYUIN, ID4_KYUIN};
    u16 speed[4] = {3400, 3400, 3400, 3400};
    u8 acc[4] = {50, 50, 50, 50};

    st.SyncWritePosEx(ID, sizeof(ID), POSITION, speed, acc);

    delay(1000);

    st.WritePosEx(ARM_ID_1, ID1_SECOND, 2500);

    delay(1500);

    u8 ID_[] = {7, 8};
    s16 POSITION_[] = {ID3_THIRD, ID4_THIRD};

    st.SyncWritePosEx(ID_, sizeof(ID_), POSITION_, speed, acc);
}

void arm_move_last_sync()
{

    u8 ID[4] = {5, 6, 7, 8};
    s16 POSITION[4] = {last_1, last_2, last_3, last_4};
    u16 speed[4] = {3400, 3400, 3400, 3400};
    u8 acc[4] = {50, 50, 50, 50};

    st.SyncWritePosEx(ID, sizeof(ID), POSITION, speed, acc);
}

void setup()
{

    Serial.begin(115200);
    delay(1000);

    // XIAO ESP32-C6のデフォルトピン(D4=SDA, D5=SCL)とアドレスを指定
    Wire.begin(I2C_DEV_ADDR, D4, D5, 100000);
    Wire.onReceive(onReceive);
    Serial.println("Slave Ready on D4/D5");

    servo_setup_sync();
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1));
}
