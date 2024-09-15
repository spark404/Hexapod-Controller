#ifndef DYNAMIXEL_PROTOCOL_H
#define DYNAMIXEL_PROTOCOL_H

#define PING                 0x01
#define READ                 0x02
#define WRITE                0x03
#define REG_WRITE            0x04
#define ACTION               0x05
#define FACTORY_RESET        0x06
#define REBOOT               0x08
#define CLEAR                0x10
#define CONTROL_TABLE_BACKUP 0x20
#define STATUS               0x55
#define SYNC_READ            0x82
#define SYNC_WRITE           0x83
#define FAST_SYNC_READ       0x8A
#define BULK_READ            0x92
#define BULK_WRITE           0x93
#define FAST_BULK_READ       0x9A

#define XL430_CT_EEP_OPERATING_MODE    11
#define XL430_CT_RAM_TORQUE_ENABLE     64
#define XL430_CT_RAM_LED               65
#define XL430_CT_RAM_GOAL_POSITION    116
#define XL430_CT_RAM_PRESENT_POSITION 132

#endif /* DYNAMIXEL_PROTOCOL_H */
