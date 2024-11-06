/*************************   添加头文件  ****************************/
#include <ESP32Servo.h>
#include <uart.h>         //先把uart.h复制工程include目录下，//#include "C:\Users\Hugh\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.11\tools\sdk\esp32s3\include\driver\include\driver\uart.h"
#include <ry_ble.h>       //ble
#include <oled_ssd1306.h> //OLED
#include <ry_code.h>      //decode
#include <ry_json.h>      //arduinoJson
/*************************  电机变量声明  ****************************/
#if CONFIG_IDF_TARGET_ESP32S3
#define LEFT_FRONT_SERVO_PIN 39
#define RIGHT_FRONT_SERVO_PIN 40
#define LEFT_REAR_SERVO_PIN 41
#define RIGHT_REAR_SERVO_PIN 42
#elif ESP32
#define LEFT_FRONT_SERVO_PIN 33
#define RIGHT_FRONT_SERVO_PIN 26
#define LEFT_REAR_SERVO_PIN 14
#define RIGHT_REAR_SERVO_PIN 12
#endif

#define MAX_WIDTH 2500
#define MIN_WIDTH 500

Servo leftFrontServo; // 定义 servo 对象
Servo rightFrontServo;
Servo leftRearServo;
Servo rightRearServo;
/************************* 全局变量声明 ****************************/
code_CMD CMD_NOW = CODE_START;
code_CMD CMD_LAST = CODE_START;
/************************* 串口变量声明 ****************************/
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
#define EX_UART_NUM UART_NUM_1 // set uart1
#define PATTERN_CHR_NUM (3)

#define UART_PIN_TX GPIO_NUM_9  // 串口发送引脚GPIO_18
#define UART_PIN_RX GPIO_NUM_10 // 串口接收引脚GPIO_19

static QueueHandle_t uart1_queue; // 串口接收队列,当串口完成数据接收后，发送消息给该队列，只需在线程等待该消息队列完成数据处理

/************************* 函数/方法声明 ****************************/
void uart_init(void);                             // 串口初始化
void motor_init(void);                            // 电机初始化
void Uart_data_process(void);                     // 串口数据处理函数
void Get_CMD_NOW(uint8_t *dtmp, uint16_t length); // 解析当前指令
void move_motor(uint8_t *angle);                  // 电机动作函数
void test(uint8_t *dtmp);                         // 电机测试

static void uart_event_task(void *pvParameters); // 接收串口处理函数
void task_move(void *p);
void task2(void *p);
// ble tasks
void task1_ble(void *arg);
void task2_ble(void *arg);
// BLE数据处理队列---------------------------------------------------------------------------------------------------------------------------------
// extern QueueHandle_t BLE_DATA_Queue;
// extern uint8_t my_BLE_data_buffer[200];
// task-------------------------------------------------------------------------------------------------------------------------------------------

TaskHandle_t myTask1 = NULL;
TaskHandle_t myTask2 = NULL;

/*************************  setup/loop  ****************************/
void setup()
{
    //esp_log_level_set(RYMCU_TAG, ESP_LOG_NONE);
    Serial.begin(115200); // 初始化调试串口
    uart_init();          // 初始化接收串口
    motor_init();         // 电机初始化
    initBLE();            // 蓝牙初始化
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ry_json_test(); // 测试arduinojson
                    // init_my_oled();// 初始化OLED
    // ble task
    xTaskCreate(task1_ble, "task1_ble", 4096, NULL, 10, &myTask1);
    xTaskCreate(task2_ble, "task2_ble", 4096, NULL, 10, &myTask2); // 处理蓝牙消息

    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL); // 处理串口消息

    xTaskCreate(task_move, "task_move", 4096, NULL, 2, NULL); // 运动
    xTaskCreate(task2, "task2", 4096, NULL, 1, NULL);
}

void loop()
{
    Serial.println("hello loop.");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
}

/*************************  串口初始化  ****************************/
void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart1_queue, 0); // 安装串口驱动，并关联队列uart1_queue
    uart_param_config(EX_UART_NUM, &uart_config);

    uart_set_pin(EX_UART_NUM, UART_PIN_TX, UART_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // 设置串口引脚（TX:18,RX:19）
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);               // Set uart pattern detect function.
    uart_pattern_queue_reset(EX_UART_NUM, 20);                                                   // Reset the pattern queue length to record at most 20 pattern positions.
}

/*************************      电机初始化       ****************************/
void motor_init(void)
{
    // 分配硬件定时器
    ESP32PWM::allocateTimer(0);
#if CONFIG_IDF_TARGET_ESP32S3
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
#endif
    // 设置频率
    leftFrontServo.setPeriodHertz(50);
    rightFrontServo.setPeriodHertz(50);
    leftRearServo.setPeriodHertz(50);
    rightRearServo.setPeriodHertz(50);
    // 关联 servo 对象与 GPIO 引脚，设置脉宽范围
    leftFrontServo.attach(LEFT_FRONT_SERVO_PIN, MIN_WIDTH, MAX_WIDTH);
    rightFrontServo.attach(RIGHT_FRONT_SERVO_PIN, MIN_WIDTH, MAX_WIDTH);
    leftRearServo.attach(LEFT_REAR_SERVO_PIN, MIN_WIDTH, MAX_WIDTH);
    rightRearServo.attach(RIGHT_REAR_SERVO_PIN, MIN_WIDTH, MAX_WIDTH);
}

/*************************  串口中断事件处理线程  ****************************/
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t *dtmp = (uint8_t *)malloc(RD_BUF_SIZE);
    while (1)
    {
        if (xQueueReceive(uart1_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            bzero(dtmp, RD_BUF_SIZE);
            switch (event.type)
            {

            case UART_DATA:
                uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY); // 读出接收到的数据
                uart_write_bytes(EX_UART_NUM, (const char *)dtmp, event.size); // 打印接收到的数据
                Serial.printf("[UART DATA]: %d", event.size);
                Serial.printf("[UART DATA]: %d,%d,%d", dtmp[0], dtmp[1], dtmp[2]);
                // if (event.size == 4)
                //  test(dtmp);
                // Uart_data_process(dtmp, event.size); // 处理接收到的数据
                Get_CMD_NOW(dtmp, event.size);
                break;
            default:
                Serial.printf("uart event type: %d", event.type);
                break;
            }
        }
        taskYIELD();
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

/*************************      task_move     ****************************/
void task_move(void *p)
{
    while (1)
    {

        Uart_data_process();
        // Serial.println("task1.");
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

/*************************      task2     ****************************/
void task2(void *p)
{
    while (1)
    {
        Serial.println("task2.");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
/************************* 动作指令比较 *******************************/

enum code_CMD data_memcmp(uint8_t *dtmp)
{
    // enum code_CMD cmd;
    for (int cmd = CODE_START; cmd < CODE_END; cmd++)
    {
        if (0 == memcmp(dtmp, &(my_code_db[cmd]), CODE_LENGTH)) // 字符串相等
            return (enum code_CMD)cmd;
    }
    return CODE_END;
}
/********************* 获取四个电机的当前角度 *******************************/
uint8_t angle_current[4]; // 当前角度变量，//[前左，前右，后左，后右]
uint8_t angle_next[4];

void Set_Angle(uint8_t leftFront, uint8_t rightFront, uint8_t leftRear, uint8_t rightRear)
{
    angle_next[0] = leftFront;
    angle_next[1] = rightFront;
    angle_next[2] = leftRear;
    angle_next[3] = rightRear;
}
void Get_Angle(void)
{
    angle_current[0] = leftFrontServo.read();
    angle_current[1] = rightFrontServo.read();
    angle_current[2] = leftRearServo.read();
    angle_current[3] = rightRearServo.read();

    for (size_t i = 0; i < 4; i++)
    {
        if (angle_current[i] > 180)
            angle_current[i] = 0;
    }
    angle_current[1] = 180 - angle_current[1];
    angle_current[3] = 180 - angle_current[3];

    Serial.printf("current_angel:%d %d %d %d\r\n", angle_current[0], angle_current[1], angle_current[2], angle_current[3]);
}
/********************* 获取四个电机的当前角度 *******************************/
void move_time(uint8_t *angle_now, uint8_t *angle_next, uint8_t step, uint8_t ms_delay_per_step) // 不同位置电机，在相同时间到达各自指定位置
{
    // 1.求出需要运行最长时间的电机
    typedef struct
    {
        uint8_t flag;       // 0：角度变大，1：角度变小
        uint8_t angle_deta; // 前后chazhi

    } angle_dir_t;
    angle_dir_t angle_dir[4];
    uint8_t angle_deta_max = 0; // 最大偏差

    for (size_t i = 0; i < 4; i++)
    {
        if (angle_next[i] > angle_now[i])
        {
            angle_dir[i].angle_deta = angle_next[i] - angle_now[i];
            angle_dir[i].flag = 0;
        }
        else
        {
            angle_dir[i].angle_deta = angle_now[i] - angle_next[i];
            angle_dir[i].flag = 1;
        }
        if (angle_dir[i].angle_deta > angle_deta_max)
            angle_deta_max = angle_dir[i].angle_deta;
    }
    // 2.每一步运动一次
    for (size_t j = 0; j < angle_deta_max / step; j++)
    {
        for (size_t i = 0; i < 4; i++)
        {
            if (0 == angle_dir[i].flag)
                angle_now[i] = angle_now[i] + step * angle_dir[i].angle_deta / angle_deta_max;
            else
                angle_now[i] = angle_now[i] - step * angle_dir[i].angle_deta / angle_deta_max;
        }
        move_motor(angle_now);
        vTaskDelay(ms_delay_per_step / portTICK_PERIOD_MS);
    }
}
/************************* 电机时间运行函数 *******************************/
// now:当前角度
// next:目标角度
// step:单次步进度数
// ms_delay_per_step：每次步进延迟时间ms
uint8_t step_angle = 5;
uint8_t ms_delay_per_step = 30;

void move_function(uint8_t now, uint8_t next, uint8_t step, uint8_t ms_delay_per_step)
{
    uint8_t my_angle[4];        //[前左，前右，后左，后右]
    uint8_t flag;               // 0：角度变大，1：角度变小
    uint8_t detal = next - now; // 90-0
    if (next > now)
    {
        detal = next - now;
        flag = 0;
    }
    else
    {
        detal = now - next;
        flag = 1;
    }

    // 1.先调到90°站立状态
    my_angle[0] = now;
    my_angle[1] = now;
    my_angle[2] = now;
    my_angle[3] = now;
    move_motor(my_angle);

    // 2.500ms均匀转到150°
    for (size_t j = 0; j < detal / step; j++)
    {
        for (size_t i = 0; i < 4; i++)
        {
            if (0 == flag)
                my_angle[i] = my_angle[i] + step;
            else
                my_angle[i] = my_angle[i] - step;
        }

        move_motor(my_angle);
        vTaskDelay(ms_delay_per_step / portTICK_PERIOD_MS);
    }
}
/*************************   解析当前指令  ****************************/
void Get_CMD_NOW(uint8_t *dtmp, uint16_t length)
{
    if (length != CODE_LENGTH)
    {
        Serial.println("not cmd.\r\n");
        return; // 数据长度不符合返回
    }
    CMD_NOW = data_memcmp(dtmp); // 获取指令
}
/*************************   串口数据处理  ****************************/
void Uart_data_process(void)
{
    switch (CMD_NOW) // 比较字符串
    {
    case CODE_START: //
    case CRAZY_DOG:
    case BIG_SMART:
        Serial.println("BARK_DOG\r\n");
        CMD_NOW = CMD_LAST;
        break;
    case SHAKE_HAND: //
        Serial.println("SHAKE_HAND\r\n");
        while (CMD_NOW == SHAKE_HAND)
        {
            CMD_LAST = SHAKE_HAND;
            Get_Angle();               // 获取当前
            Set_Angle(60, 70, 90, 20); // 目的位置
            move_time(angle_current, angle_next, step_angle, ms_delay_per_step);

            Get_Angle();               // 获取当前
            Set_Angle(20, 70, 90, 20); // 目的位置
            move_time(angle_current, angle_next, step_angle, ms_delay_per_step);
        }
        break;
    case SIT_DOWN: //
        Serial.println("SIT_DOWN\r\n");
        while (CMD_NOW == SIT_DOWN)
        {
            CMD_LAST = SIT_DOWN;
            Get_Angle();             // 获取当前
            Set_Angle(90, 90, 0, 0); // 目的位置
            move_time(angle_current, angle_next, step_angle, ms_delay_per_step);
        }
        break;
    case STAND_UP: //
        Serial.println("STAND_UP\r\n");
        while (CMD_NOW == STAND_UP)
        {
            CMD_LAST = STAND_UP;
            Get_Angle();               // 获取当前
            Set_Angle(90, 90, 90, 90); // 目的位置
            move_time(angle_current, angle_next, step_angle, ms_delay_per_step);
        }
        break;
    case GET_DOWN: //
        Serial.println("GET_DOWN\r\n");
        while (CMD_NOW == GET_DOWN)
        {
            CMD_LAST = GET_DOWN;
            Get_Angle();
            Set_Angle(10, 10, 180, 180); // 目的位置
            move_time(angle_current, angle_next, step_angle, ms_delay_per_step);
        }
        break;
    case MOVE_FORWARD:
        // step_angle = step_angle + 5;
        //  ms_delay_per_step = ms_delay_per_step -5;
        /*********************************************************
         **90 90***90  60***120  60***120  90****90  90***********
         **90 90***60  90***60  120***90   120***90  90***********/
        Serial.println("MOVE_FORWARD\r\n");
        while (CMD_NOW == MOVE_FORWARD)
        {
            CMD_LAST = MOVE_FORWARD;
            Get_Angle();               // 获取当前
            Set_Angle(90, 90, 90, 90); // 目的位置
            move_time(angle_current, angle_next, step_angle, ms_delay_per_step);

            Get_Angle();               // 获取当前
            Set_Angle(90, 60, 60, 90); // 目的位置
            move_time(angle_current, angle_next, step_angle, ms_delay_per_step);

            Get_Angle();                 // 获取当前
            Set_Angle(120, 60, 60, 120); // 目的位置
            move_time(angle_current, angle_next, step_angle, ms_delay_per_step);

            Get_Angle();                 // 获取当前
            Set_Angle(120, 90, 90, 120); // 目的位置
            move_time(angle_current, angle_next, step_angle, ms_delay_per_step);
            Get_Angle(); // 获取当前
        }
        break;
    case MOVE_BACK:
        Serial.println("MOVE_BACK\r\n");
        while (CMD_NOW == MOVE_BACK)
        {
            CMD_LAST = MOVE_BACK;

            Get_Angle();                 // 获取当前
            Set_Angle(120, 90, 90, 120); // 目的位置
            move_time(angle_current, angle_next, step_angle, ms_delay_per_step);
            Get_Angle(); // 获取当前

            Get_Angle();                 // 获取当前
            Set_Angle(120, 60, 60, 120); // 目的位置
            move_time(angle_current, angle_next, step_angle, ms_delay_per_step);

            Get_Angle();               // 获取当前
            Set_Angle(90, 60, 60, 90); // 目的位置
            move_time(angle_current, angle_next, step_angle, ms_delay_per_step);

            Get_Angle();               // 获取当前
            Set_Angle(90, 90, 90, 90); // 目的位置
            move_time(angle_current, angle_next, step_angle, ms_delay_per_step);
        }
        break;
    case TURN_LEFT:
        CMD_NOW = CMD_LAST;
        break;
    case TURN_LIGHT:
        CMD_NOW = CMD_LAST; 
        break;
    case SPEED_ADD:
    case SPEED_MORE:
    case SPEED_FAST:
        CMD_NOW = CMD_LAST;
        // step_angle = step_angle + 5;
        ms_delay_per_step = ms_delay_per_step - 5;
        if (ms_delay_per_step <= 5)
            ms_delay_per_step = 5;
        Serial.printf("CMD_LAST=%d,step_angle=%d,ms_delay_per_step=%d\r\n", (uint8_t)CMD_LAST, step_angle, ms_delay_per_step);
        break;
    case SPEED_SUB:
    case SPEED_LESS:
    case SPEED_SLOW:
        CMD_NOW = CMD_LAST;
        // step_angle = step_angle - 5;
        ms_delay_per_step = ms_delay_per_step + 5;
        if (ms_delay_per_step >= 255)
            ms_delay_per_step = 255;
        Serial.printf("CMD_LAST=%d,step_angle=%d,ms_delay_per_step=%d\r\n", (uint8_t)CMD_LAST, step_angle, ms_delay_per_step);
        break;
    default:
        // Serial.println("unknow CMD\r\n");
        break;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // do nothing
}
/*************************   电机动作处理  ****************************/
void move_motor(uint8_t *angle)
{
    // Serial.println("to be move motor.");
    leftFrontServo.write(angle[0]);
    rightFrontServo.write(180 - angle[1]); // 180-
    leftRearServo.write(angle[2]);
    rightRearServo.write(180 - angle[3]); // 180-
}
/*************************   电机测试代码  ****************************/
void test(uint8_t *dtmp)
{
    // Serial.println("to be move motor.");
    Get_Angle();
}
/*************************   BLE task  ****************************/
void task1_ble(void *arg)
{
    while (1)
    {
        Serial.println("task1_ble.");
        // test(0);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
void task2_ble(void *arg)
{
    uint8_t my_buff[200];
    while (1)
    { /// 读取队列的消息,当消息队列为空的时候，挂起线程
        if (xQueueReceive(BLE_DATA_Queue, my_buff, portMAX_DELAY))
        {
            Serial.printf("message:%s\r\n", my_buff);
            // uint8_t my_buff[4] = {0};
            // test(my_buff);
            // Uart_data_process(my_buff, CODE_LENGTH);
            Get_CMD_NOW(my_buff, CODE_LENGTH);
        }
        // vTaskDelay(pdMS_TO_TICKS(1000));
        taskYIELD();
    }
}
/*************************  end of program  ***************************/