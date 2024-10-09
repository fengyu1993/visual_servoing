#ifndef KEY_H
#define KEY_H
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

const int KEYCODE_W = 0x77;
const int KEYCODE_S = 0x73;
const int KEYCODE_A = 0x61;
const int KEYCODE_D = 0x64;
const int KEYCODE_R = 0x72;
const int KEYCODE_F = 0x66;
const int KEYCODE_Q = 0x71;
 
const int KEYCODE_A_CAP = 0x41;
const int KEYCODE_D_CAP = 0x44;
const int KEYCODE_S_CAP = 0x53;
const int KEYCODE_W_CAP = 0x57;
const int KEYCODE_R_CAP = 0x52;
const int KEYCODE_F_CAP = 0x46;
const int KEYCODE_Q_CAP = 0x51;

class Keyboard_ctrl {
  struct termios initial_settings, new_settings;
  int peek_character = -1;
 
 public:
  Keyboard_ctrl() { init_keyboard(); };
  ~Keyboard_ctrl() { close_keyboard(); };
 
  int get_keyboard_press_key() {
    kbhit();
    return readch();
    // printf("%02x \n", readch());
  };
 
 private:
  void init_keyboard() {
    tcgetattr(0, &initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    new_settings.c_cc[VEOL] = 1;
    new_settings.c_cc[VEOF] = 2;
    tcsetattr(0, TCSANOW, &new_settings);
  }
 
  void close_keyboard() { tcsetattr(0, TCSANOW, &initial_settings); }
 
  int kbhit() {
    unsigned char ch;
    int nread;
 
    if (peek_character != -1) return 1;
    new_settings.c_cc[VMIN] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0, &ch, 1);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);
    if (nread == 1) {
      peek_character = ch;
      return 1;
    }
    return 0;
  }
 
  int readch() {
    char ch;
    if (peek_character != -1) {
      ch = peek_character;
      peek_character = -1;
      return ch;
    }
    // read(0, &ch, 1);
    ch = get_char();
    return ch;
  }

  int get_char()
{
    fd_set rfds;
    struct timeval tv;
    int ch = 0;
 
    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 5; //设置等待超时时间
 
    //检测键盘是否有输入
    if (select(1, &rfds, NULL, NULL, &tv) > 0){
        ch = getchar(); 
    }
    return ch;
}
};
 
#endif  // KEY_H
