#ifndef MAIN_SSD1306_H_
#define MAIN_SSD1306_H_

// Following definitions are bollowed from 
// http://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html

// SLA (0x3C) + WRITE_MODE (0x00) =  0x78 (0b01111000)
#define OLED_I2C_ADDRESS   0x3C

//---- CONTROL BYTES UNDERSTANDING ---- 
/*
Co : bit 8 : Continuation Bit
* 1 = no-continuation (only one byte to follow)
* 0 = the controller should expect a stream of bytes.
D/C# : bit 7 : Data/Command Select bit
* 1 = the next byte or byte stream will be Data.
* 0 = a Command byte or byte stream will be coming up next.
Bits 6-0 will be all zeros.
*/
#include "driver/i2c.h"
// Control byte
#define OLED_CONTROL_BYTE_CMD_SINGLE    0x80 //Control byte dùng để thông báo các byte I2C OLED nhận tiếp theo nhận duy nhất 1 byte lệnh
#define OLED_CONTROL_BYTE_CMD_STREAM    0x00 //Control byte dùng để thông báo các byte I2C OLED nhận tiếp theo là 1 luồng lệnh nhiều bytes
#define OLED_CONTROL_BYTE_DATA_STREAM   0x40 //Control byte dùng để thông báo các byte I2C OLED nhập tiếp theo là 1 luồng stream toàn các byte dữ liệu (DATA STREAM)

// Fundamental commands (pg.28)
#define OLED_CMD_SET_CONTRAST           0x81    // follow with 0x7F //điều chỉnh độ sáng màn hình, 0x7F là dùng để reset độ tương phản
#define OLED_CMD_DISPLAY_RAM            0xA4    // Resume nội dung hiển thị RAM (RESET)
#define OLED_CMD_DISPLAY_ALLON          0xA5    // Display ON toàn bộ, ignore RAM content
#define OLED_CMD_DISPLAY_NORMAL         0xA6    // Bật màn hình display lên ở normal mode (Không đảo, ngược)
#define OLED_CMD_DISPLAY_INVERTED       0xA7    // Hiển thị ngược (Inverse display)
#define OLED_CMD_DISPLAY_OFF            0xAE    // OFF Display OLED
#define OLED_CMD_DISPLAY_ON             0xAF    // ON Display OLED

// Addressing Command Table (pg.30) 
#define OLED_CMD_SET_MEMORY_ADDR_MODE   0x20    // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
// dùng command này để thiết lập Mem Addressing Mode
/* Các chế độ addressing mode:
    1. Hortizontal addressing mode A[1:0] = 00b
    2. Vertical Addressing mode A[1:0] = 01b
    3. Page addressing mode (RESET) A[1:0] = 10b
    4. Invalid A[1:0] = 11b
*/

#define OLED_CMD_SET_COLUMN_RANGE       0x21    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127 
//dùng command này có thể điều chỉnh để hiển thị pixel tại cột mong muốn
#define OLED_CMD_SET_PAGE_RANGE         0x22    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7


// Hardware Config (pg.31) (Page resolution and layout related)
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40 
// Set display RAM start line register from 0 - 63 sử dụng X5X4X3X2X1X0
// Màn hình dòng bắt đầu được reset với giá trị 000000b (RESET)
#define OLED_CMD_SET_SEGMENT_REMAP      0xA1    // địa chỉ cột 127 được map tới SEG0
#define OLED_CMD_SET_MUX_RATIO          0xA8    // follow with 0x3F = 64 MUX 
#define OLED_CMD_SET_COM_SCAN_MODE      0xC8    // chế độ remmaped. Scan từ COM[n-1] đến COM0
#define OLED_CMD_SET_DISPLAY_OFFSET     0xD3    // follow with 0x00.  
// Thiết lập độ dời display
// set vertical shift by COM  from 0d-63d
// giá trị được reset là 00h sau khi reset, tức là độ dời sẽ bị xóa đi sau khi RESET
#define OLED_CMD_SET_COM_PIN_MAP        0xDA    // follow with 0x12
//Command dùng để;
/*
    Disable COM Left/ Right
    (RESET) Alternative COM Pin config
*/
#define OLED_CMD_NOP                    0xE3    // NOP
//CMD NOP for no operation

// Timing and Driving Scheme (pg.32)
#define OLED_CMD_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80
#define OLED_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP        0x8D    // follow with 0x14
//Enable charge pump during display ON

void ssd1306_init(i2c_port_t i2c_num);
void task_ssd1306_display_text(const void *arg_text, i2c_port_t i2c_num);
void task_ssd1306_display_clear(i2c_port_t i2c_num);

#endif /* MAIN_SSD1306_H_ */