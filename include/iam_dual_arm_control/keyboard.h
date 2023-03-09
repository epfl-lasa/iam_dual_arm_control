//|
//|    Copyright (C) 2021 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Patrick Sgrò (maintainer)
//|    email:    patrick.sgro@hotmail.com
//|    website:  lasa.epfl.ch
//|
#ifndef KEYBOARD_CONTROLLER_H
#define KEYBOARD_CONTROLLER_H

#include <ros/ros.h>

// std headers
#include <iostream>
#include <stdio.h>
#include <termios.h>

namespace keyboard {
class Keyboard {
 public:
  static void nonblock_2(int state);
  static bool keyState_2(char key);
  static char get_char();

 // protected:
  static int khbit_2();
  

};  // class Keyboard

int Keyboard::khbit_2() {
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

// ///////////////////////////////////////////
// 
// ///////////////////////////////////////////
void Keyboard::nonblock_2(int state) {
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);

    if ( state == 1)
    {
        ttystate.c_lflag &= (~ICANON & ~ECHO); //Not display character
        ttystate.c_cc[VMIN] = 1;
    }
    else if (state == 0)
    {
        ttystate.c_lflag |= ICANON;
        ttystate.c_lflag |= ECHO;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

// ///////////////////////////////////////////
// return true if the key character is pressed
// ///////////////////////////////////////////
bool Keyboard::keyState_2(char key) {
    bool pressed = false;
    int i = Keyboard::khbit_2(); //Alow to read from terminal
    if (i != 0)
    {
        char c = fgetc(stdin);
        if (c == key)
        {
            pressed = true;
        }
        else
        {
            pressed = false;
        }
    }

    return pressed;
}

// ///////////////////////////////////////////
// It returns the pressed character
// ///////////////////////////////////////////
char Keyboard::get_char() {
    char c;
    int i = Keyboard::khbit_2(); //Alow to read from terminal
    if (i != 0) {
        c = fgetc(stdin);
        fflush(stdin);
    }
    return c;
}

}  // namespace keyboard

#endif  // KEYBOARD_CONTROLLER_H