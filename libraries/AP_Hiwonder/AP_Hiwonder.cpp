#include "AP_Hiwonder/AP_Hiwonder.h"
#include <AP_HAL/AP_HAL.h> //// 硬件抽象层头文件
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h> // 串口管理头文件
#include <SRV_Channel/SRV_Channel.h>           // 伺服通道头文件

extern const AP_HAL::HAL& hal; // 引用全局 HAL 对象

// singleton instance   // 声明单例实例指针
AP_Hiwonder_RF* AP_Hiwonder_RF::_singleton; // 右前(RF)舵机控制单例
AP_Hiwonder_RB* AP_Hiwonder_RB::_singleton;
AP_Hiwonder_LB* AP_Hiwonder_LB::_singleton;
AP_Hiwonder_LF* AP_Hiwonder_LF::_singleton;

#define AP_SERIALMANAGER_Hiwonder_BAUD       115200
#define AP_SERIALMANAGER_Hiwonder_BUFSIZE_RX 0
#define AP_SERIALMANAGER_Hiwonder_BUFSIZE_TX 0

// 计算校验和的函数  确保数据传输的完整性，防止因通信错误导致舵机执行错误的指令
// 求和校验（Checksum）：简单累加，适用于低复杂度场景（如舵机控制）。
uint8_t AP_Hiwonder::serial_servo_checksum(const uint8_t buf[])
{
    uint16_t temp = 0;
    for (int i = 2; i < buf[3] + 2; ++i) {
        temp += buf[i];
    }
    return (uint8_t)(~temp);
}

// 设置舵机位置的函数
void AP_Hiwonder::set_position(uint32_t servo_id, int position, uint32_t duration)
{
    if (_port == NULL) // 检查串口是否初始化
        return;

    Hiwonder_MOVE_WRITE_TypeDef frame;
    frame.header_1 = SERIAL_SERVO_FRAME_HEADER;    // 帧头1
    frame.header_2 = SERIAL_SERVO_FRAME_HEADER;    // 帧头2
    frame.servo_id = servo_id;                     // 舵机ID
    frame.command  = SERIAL_SERVO_MOVE_TIME_WRITE; // 移动指令

    position = constrain_int32(position, 100, 900);

    // 将位置和持续时间参数分解为高低字节
    frame.args[0] = LOWBYTE(position);
    frame.args[1] = HIGHBYTE(position);
    frame.args[2] = LOWBYTE(duration);
    frame.args[3] = HIGHBYTE(duration);

    frame.length = sizeof(frame.args) + 3;                  // 计算帧长度
    frame.crc    = serial_servo_checksum((uint8_t*)&frame); // 计算校验和

    // 通过串口发送帧数据
    _port->write((uint8_t*)&frame, sizeof(frame));
}

void AP_Hiwonder::adjust_offset(uint32_t servo_id, int8_t adjust)
{
    if (_port == NULL)
        return;

    // 定义偏移调整指令帧结构
    Hiwonder_OFFSET_ADJUST_TypeDef frame;
    frame.header_1 = SERIAL_SERVO_FRAME_HEADER;
    frame.header_2 = SERIAL_SERVO_FRAME_HEADER;
    frame.servo_id = servo_id;
    frame.command  = SERVO_ANGLE_OFFSET_ADJUST;

    // 限制调整范围在-125到125之间
    adjust     = constrain_int32(adjust, -125, 125);
    frame.args = (uint8_t)adjust;

    frame.length = sizeof(frame.args) + 3;
    frame.crc    = serial_servo_checksum((uint8_t*)&frame);

    _port->write((uint8_t*)&frame, sizeof(frame));
}

// 写入舵机偏移量的函数
void AP_Hiwonder::write_offset(uint32_t servo_id)
{
    if (_port == NULL)
        return;

    Hiwonder_WRITE_ADJUST_TypeDef frame;
    frame.header_1 = SERIAL_SERVO_FRAME_HEADER;
    frame.header_2 = SERIAL_SERVO_FRAME_HEADER;
    frame.servo_id = servo_id;
    frame.command  = SERVO_ANGLE_OFFSET_WRITE;

    frame.length = 3;
    frame.crc    = serial_servo_checksum((uint8_t*)&frame);

    _port->write((uint8_t*)&frame, sizeof(frame));
}

// 右前(RF)舵机初始化函数
void AP_Hiwonder_RF::init(void)
{
    AP_SerialManager& serial_manager = AP::serialmanager();
    // 尝试从串口管理器中找到配置的串口
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Hiwonder_RF, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AP_SERIALMANAGER_Hiwonder_BAUD,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_RX,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_TX);
    } else {
        // 如果没找到，使用默认串口1
        _port = hal.serial(1);
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AP_SERIALMANAGER_Hiwonder_BAUD,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_RX,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_TX);
    }
}

void AP_Hiwonder_RB::init(void)
{
    AP_SerialManager& serial_manager = AP::serialmanager();
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Hiwonder_RB, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AP_SERIALMANAGER_Hiwonder_BAUD,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_RX,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_TX);
    } else {
        _port = hal.serial(2);
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AP_SERIALMANAGER_Hiwonder_BAUD,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_RX,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_TX);
    }
}

void AP_Hiwonder_LB::init(void)
{
    AP_SerialManager& serial_manager = AP::serialmanager();
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Hiwonder_LB, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AP_SERIALMANAGER_Hiwonder_BAUD,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_RX,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_TX);
    } else {
        _port = hal.serial(3);
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AP_SERIALMANAGER_Hiwonder_BAUD,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_RX,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_TX);
    }
}

void AP_Hiwonder_LF::init(void)
{
    AP_SerialManager& serial_manager = AP::serialmanager();
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Hiwonder_LF, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AP_SERIALMANAGER_Hiwonder_BAUD,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_RX,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_TX);
    } else {
        _port = hal.serial(4);
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AP_SERIALMANAGER_Hiwonder_BAUD,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_RX,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_TX);
    }
}

namespace AP {

AP_Hiwonder_LF& hiwonder_LF()
{
    return *AP_Hiwonder_LF::get_singleton();    // 获取左前(LF)舵机控制单例的引用
}

AP_Hiwonder_RF& hiwonder_RF()
{
    return *AP_Hiwonder_RF::get_singleton();
}

AP_Hiwonder_RB& hiwonder_RB()
{
    return *AP_Hiwonder_RB::get_singleton();
}

AP_Hiwonder_LB& hiwonder_LB()
{
    return *AP_Hiwonder_LB::get_singleton();
}

}