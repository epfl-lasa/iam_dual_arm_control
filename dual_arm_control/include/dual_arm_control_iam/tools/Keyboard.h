//|
//|    Copyright (C) 2021-2023 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors: Michael Bombile (maintainer)
//|
//|    email:   michael.bombile@epfl.ch/micbombile@gmail.com
//|
//|    Other contributors:
//|             Elise Jeandupeux (elise.jeandupeux@epfl.ch)
//|
//|    website: lasa.epfl.ch
//|
//|    This file is part of iam_dual_arm_control.
//|    This work was supported by the European Community's Horizon 2020 Research and Innovation
//|    programme (call: H2020-ICT-09-2019-2020, RIA), grant agreement 871899 Impact-Aware Manipulation.
//|
//|    iam_dual_arm_control is free software: you can redistribute it and/or modify  it under the terms
//|    of the GNU General Public License as published by  the Free Software Foundation,
//|    either version 3 of the License, or  (at your option) any later version.
//|
//|    iam_dual_arm_control is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
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