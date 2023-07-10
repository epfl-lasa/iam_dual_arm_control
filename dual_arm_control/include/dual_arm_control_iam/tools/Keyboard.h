//|
//|    Copyright (C) 2021 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Patrick Sgrò (maintainer)
//|    email:    patrick.sgro@hotmail.com
//|    website:  lasa.epfl.ch
//|

#pragma once

#include <iostream>
#include <stdio.h>
#include <termios.h>

namespace keyboard {

int khBit() {
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &fds);
}

// ///////////////////////////////////////////
//
// ///////////////////////////////////////////
void nonBlock(int state) {
  struct termios ttystate;
  tcgetattr(STDIN_FILENO, &ttystate);

  if (state == 1) {
    ttystate.c_lflag &= (~ICANON & ~ECHO);//Not display character
    ttystate.c_cc[VMIN] = 1;
  } else if (state == 0) {
    ttystate.c_lflag |= ICANON;
    ttystate.c_lflag |= ECHO;
  }
  tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

// ///////////////////////////////////////////
// return true if the key character is pressed
// ///////////////////////////////////////////
bool keyState(char key) {
  bool pressed = false;
  int i = khBit();//Alow to read from terminal
  if (i != 0) {
    char c = fgetc(stdin);
    if (c == key) {
      pressed = true;
    } else {
      pressed = false;
    }
  }

  return pressed;
}

// ///////////////////////////////////////////
// It returns the pressed character
// ///////////////////////////////////////////
char getChar() {
  char c;
  int i = khBit();//Alow to read from terminal
  if (i != 0) {
    c = fgetc(stdin);
    fflush(stdin);
  }
  return c;
}

}// namespace keyboard