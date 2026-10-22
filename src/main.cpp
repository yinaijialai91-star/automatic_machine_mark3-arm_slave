#include <Arduino.h>
#include <Wire.h>
#include <SCServo.h>

#define I2C_DEV_ADDR 0x67

#define TX_PIN TX
#define RX_PIN RX

// ---------------- 状態 ----------------
#define STATE_IDLE 0x00
#define STATE_BUSY 0x01

#define RESULT_OK 0x00
#define RESULT_ERROR 0x01

// ---------------- アーム目標位置 ----------------
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
#define last_2 2140
#define last_3 1950
#define last_4 1950

#define POS_TOLERANCE 20
#define MOVE_TIMEOUT_MS 5000 // 1ステップの完了待ち上限

SMS_STS st;

// ---------------- マスターとの共有変数 ----------------
volatile uint8_t g_state = STATE_BUSY; // 起動中はBUSY
volatile uint8_t g_done_seq = 0;
volatile uint8_t g_result = RESULT_OK;

volatile uint8_t g_pending_cmd = 0;
volatile uint8_t g_pending_seq = 0;
volatile bool g_pending = false;

// ---------------- I2C コールバック ----------------
// 受信フォーマット: DATA[0]=コマンド, DATA[1]=seq
void onReceive(int len)
{
    uint8_t buf[8] = {0};
    int n = 0;
    while (Wire.available())
    {
        uint8_t b = Wire.read();
        if (n < 8)
            buf[n++] = b;
    }
    if (n < 2)
        return;

    if (g_state == STATE_BUSY || g_pending)
        return;

    g_pending_cmd = buf[0];
    g_pending_seq = buf[1];
    g_state = STATE_BUSY;
    g_pending = true;
}

void onRequest()
{
    Wire.write((uint8_t)g_state);
    Wire.write((uint8_t)g_done_seq);
    Wire.write((uint8_t)g_result);
}

// ---------------- サーボ ----------------
// 全サーボが目標位置の許容誤差内に入るまで待つ
bool waitForServo(const u8 *ids, const s16 *targets, int n,
                  uint32_t timeout = MOVE_TIMEOUT_MS, int tol = POS_TOLERANCE)
{
    uint32_t start = millis();
    delay(30);

    while (millis() - start < timeout)
    {
        bool all_completed = true;
        for (int i = 0; i < n; i++)
        {
            int pos = st.ReadPos(ids[i]);
            if (pos == -1 || abs(pos - (int)targets[i]) > tol)
            {
                all_completed = false;
                break;
            }
        }
        if (all_completed)
            return true;

        vTaskDelay(pdMS_TO_TICKS(15));
    }

    Serial.println("Warning: Servo Motion Timeout (arm)");
    return false;
}

// 指令 → 到達待ち → 最低 min_ms は経過させる
// (元コードの固定 delay を「最低待ち時間」として残し、到達も保証する)
bool moveAndWait(const u8 *ids, const s16 *targets, int n, uint16_t speedVal = 3400,
                 uint32_t min_ms = 0, uint32_t timeout = MOVE_TIMEOUT_MS)
{
    u16 speed[4] = {speedVal, speedVal, speedVal, speedVal};
    u8 acc[4] = {50, 50, 50, 50};

    uint32_t start = millis();
    st.SyncWritePosEx((u8 *)ids, n, (s16 *)targets, speed, acc);

    bool ok = waitForServo(ids, targets, n, timeout);

    uint32_t elapsed = millis() - start;
    if (elapsed < min_ms)
        delay(min_ms - elapsed);

    return ok;
}

void servo_setup_sync()
{
    Serial1.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN);
    st.pSerial = &Serial1;
}

// ---------------- アーム動作 ----------------
bool arm_move_first_sync()
{
    bool ok = true;

    const u8 ID_A[] = {ARM_ID_3, ARM_ID_4};
    const s16 POS_A[] = {ID3_SECOND, ID4_SECOND};
    ok &= moveAndWait(ID_A, POS_A, 2, 3400, 1500);

    const u8 ID_B[] = {ARM_ID_1, ARM_ID_2};
    const s16 POS_B[] = {ID1_SECOND, ID2_SECOND};
    ok &= moveAndWait(ID_B, POS_B, 2, 3400, 0);

    return ok;
}

bool arm_move_second_sync()
{
    bool ok = true;

    // ID1 を THIRD へ(元: delay 3000)
    const u8 ID_1[] = {ARM_ID_1};
    const s16 POS_THIRD_1[] = {ID1_THIRD};
    ok &= moveAndWait(ID_1, POS_THIRD_1, 1, 3400, 3000, 3000);

    // ID1 を KYUIN へ(元: delay 1000)
    // 吸着対象に当たって目標まで届かない可能性があるので、失敗扱いにはしない
    const s16 POS_KYUIN_1[] = {ID1_KYUIN};
    moveAndWait(ID_1, POS_KYUIN_1, 1, 3400, 1000, 2000);

    // ID3,4 を KYUIN へ(元: delay 1000)
    const u8 ID_34[] = {ARM_ID_3, ARM_ID_4};
    const s16 POS_KYUIN_34[] = {ID3_KYUIN, ID4_KYUIN};
    ok &= moveAndWait(ID_34, POS_KYUIN_34, 2, 3400, 1000);

    // ID1 を SECOND へ 速度2500(元: delay 1500)
    const s16 POS_SECOND_1[] = {ID1_SECOND};
    ok &= moveAndWait(ID_1, POS_SECOND_1, 1, 2500, 1500);

    // ID3,4 を THIRD へ
    const s16 POS_THIRD_34[] = {ID3_THIRD, ID4_THIRD};
    ok &= moveAndWait(ID_34, POS_THIRD_34, 2, 3400, 0);

    return ok;
}

bool arm_move_last_sync()
{
    const u8 ID[4] = {ARM_ID_1, ARM_ID_2, ARM_ID_3, ARM_ID_4};
    const s16 POS[4] = {last_1, last_2, last_3, last_4};
    return moveAndWait(ID, POS, 4, 3400, 0);
}

bool arm_initial_pose()
{
    const u8 ID[4] = {ARM_ID_1, ARM_ID_2, ARM_ID_3, ARM_ID_4};
    const s16 POS[4] = {ID1_FIRST, ID2_FIRST, ID3_FIRST, ID4_FIRST};
    return moveAndWait(ID, POS, 4, 3400, 0);
}

// ---------------- setup / loop ----------------
void setup()
{
    Serial.begin(115200);
    delay(1000);

    // XIAO ESP32 のデフォルトピン(D4=SDA, D5=SCL)とアドレスを指定
    Wire.begin(I2C_DEV_ADDR, D4, D5, 100000);
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);
    Serial.println("Slave(ARM) Ready on D4/D5");

    servo_setup_sync();

    // 初期姿勢へ(完了するまでBUSYのまま)
    arm_initial_pose();

    g_result = RESULT_OK;
    g_state = STATE_IDLE;
}

void loop()
{
    if (g_pending)
    {
        uint8_t cmd = g_pending_cmd;
        uint8_t seq = g_pending_seq;
        g_pending = false;

        Serial.printf("cmd=0x%02X seq=%d\n", cmd, seq);

        bool ok = false;
        switch (cmd)
        {
        case 0x01:
            ok = arm_move_first_sync();
            break;
        case 0x02:
            ok = arm_move_second_sync();
            break;
        case 0x03:
            ok = arm_move_last_sync();
            break;
        default:
            Serial.printf("error! unknown cmd: 0x%02X\n", cmd);
            break;
        }

        g_result = ok ? RESULT_OK : RESULT_ERROR;
        g_done_seq = seq;
        g_state = STATE_IDLE;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
}
