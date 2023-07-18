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




namespace keyboardinteraction {

enum TaskType {
    REACH = 0,
    PICK_AND_LIFT = 1,
    TOSSING = 2,
    PICK_AND_TOSS = 3,
    PICK_AND_PLACE = 4,
    PLACE_TOSSING = 5,
    THROWING = 6,
    HANDINGOVER = 7,
    PAUSE_MOTION = 8
};

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

/*
* Return true if the key character is pressed
*/
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

/*
* Returns the pressed character
*/
char getChar() {
  char c;
  int i = khBit();//Alow to read from terminal
  if (i != 0) {
    c = fgetc(stdin);
    fflush(stdin);
  }
  return c;
}


StateMachine getKeyboard(StateMachine stateMachine) {

  nonBlock(1);

  if (khBit() != 0) {
    char keyboardCommand = fgetc(stdin);
    fflush(stdin);

    switch (keyboardCommand) {
      case 'q': {
        stateMachine.goHome = !stateMachine.goHome;
        if (stateMachine.goHome) {
          stateMachine.goToAttractors = true;
          //   stateMachine.startlogging = false;
        } else if (!stateMachine.goHome) {
          //   stateMachine.startlogging = true;
        }
      } break;
      case 'g': {
        stateMachine.goToAttractors = !stateMachine.goToAttractors;
        if (stateMachine.goToAttractors) {
          stateMachine.goHome = false;
          stateMachine.releaseAndretract = false;
        }
      } break;

      // Release or throwing
      case 'r': {
        stateMachine.releaseAndretract = !stateMachine.releaseAndretract;
      } break;
      case 'l': {
        stateMachine.dualTaskSelector = PICK_AND_LIFT;
      } break;
      case 't': {
        stateMachine.isThrowing = !stateMachine.isThrowing;
        if (stateMachine.isThrowing) {
          stateMachine.dualTaskSelector = PICK_AND_TOSS;
        } else if (!stateMachine.isThrowing) {
          stateMachine.dualTaskSelector = PICK_AND_LIFT;
        }
      } break;
      case 'p': {
        stateMachine.isPlacing = !stateMachine.isPlacing;
        if (stateMachine.isPlacing) {
          stateMachine.dualTaskSelector = PICK_AND_PLACE;
        } else if (!stateMachine.isPlacing) {
          stateMachine.dualTaskSelector = PICK_AND_LIFT;
        }
      } break;
      case 'o': {
        stateMachine.isPlaceTossing = !stateMachine.isPlaceTossing;
        if (stateMachine.isPlaceTossing) {
          stateMachine.dualTaskSelector = PLACE_TOSSING;
        } else if (!stateMachine.isPlaceTossing) {
          stateMachine.dualTaskSelector = PICK_AND_LIFT;
        }
      } break;

      // Impact and tossing velocity
      case 'v': {
        stateMachine.desVtoss -= 0.05f;
        if (stateMachine.desVtoss < 0.2f) { stateMachine.desVtoss = 0.2f; }
      } break;
      case 'b': {
        stateMachine.desVtoss += 0.05f;
        if (stateMachine.desVtoss > 2.0f) { stateMachine.desVtoss = 2.0f; }
      } break;
      case 'y': {
        stateMachine.desiredVelImp -= 0.05f;
        if (stateMachine.desiredVelImp < 0.05f) { stateMachine.desiredVelImp = 0.05f; }
      } break;
      case 'u': {
        stateMachine.desiredVelImp += 0.05f;
        if (stateMachine.desiredVelImp > 0.6f) { stateMachine.desiredVelImp = 0.6f; }
      } break;

      // Reset the data logging
      case 'c': {
        stateMachine.startlogging = false;// TODO WHEN LOGGING
        // dataLog_.reset(ros::package::getPath(std::string("dual_arm_control")) + "/Data");
      } break;

      // Disturb the target speed
      case 'e': {
        stateMachine.isDisturbTarget = !stateMachine.isDisturbTarget;
      } break;

      // Placing hight
      case 'x': {
        if (stateMachine.dualTaskSelector == PICK_AND_TOSS) {
            stateMachine.releasePosY -= 0.01;
        } else {
          stateMachine.placingPosHeight -= 0.01;
        }
      } break;
      case 'n': {
        if (stateMachine.dualTaskSelector == PICK_AND_TOSS) {
            stateMachine.releasePosY += 0.01;
        } else {
          stateMachine.placingPosHeight += 0.01;
        }
      } break;

        // // Conveyor belt control
        // case 'a': {
        //   if (stateMachine.ctrlModeConveyorBelt) {
        //     stateMachine.modeConveyorBelt = 2;
        //     publishConveyorBeltCmds();
        //     stateMachine.startlogging = true;
        //   } else if (incrementReleasePos_) {
        //     deltaRelPos_(0) -= 0.025f;//[m]
        //   } else {
        //     deltaPos_(0) -= 0.01f;
        //   }
        // } break;
        // case 's': {
        //   if (stateMachine.ctrlModeConveyorBelt) {
        //     stateMachine.modeConveyorBelt = 0;
        //     publishConveyorBeltCmds();
        //   } else if (incrementReleasePos_) {
        //     deltaRelPos_(0) += 0.025f;//[m]
        //   } else {
        //     deltaPos_(0) += 0.01f;
        //   }
        // } break;
        // case 'd': {
        //   if (stateMachine.ctrlModeConveyorBelt) {
        //     stateMachine.modeConveyorBelt = 1;
        //     publishConveyorBeltCmds();
        //   } else if (incrementReleasePos_) {
        //     deltaRelPos_(1) -= 5.0f;//[deg]
        //   } else {
        //     deltaPos_(1) -= 0.01f;
        //   }
        // } break;
        // case 'f': {
        //   if (incrementReleasePos_) {
        //     deltaRelPos_(1) += 5.0f;//[deg]
        //   } else {
        //     deltaPos_(1) += 0.01f;
        //   }
        // } break;
        // case 'z': {
        //   if (stateMachine.ctrlModeConveyorBelt) {
        //     trackingFactor_ -= 0.01f;
        //   } else if (incrementReleasePos_) {
        //     deltaRelPos_(2) -= 5.0f;//[deg]
        //   } else {
        //     deltaPos_(2) -= 0.01f;
        //   }
        // } break;
        // case 'w': {
        //   if (stateMachine.ctrlModeConveyorBelt) {
        //     trackingFactor_ += 0.01f;
        //   } else if (incrementReleasePos_) {
        //     deltaRelPos_(2) += 5.0f;//[deg]
        //   } else {
        //     deltaPos_(2) += 0.01f;
        //   }
        // } break;
        // case 'h': {
        //   if (stateMachine.ctrlModeConveyorBelt) {
        //     nominalSpeedConveyorBelt_ -= 50;
        //   } else {
        //     deltaAng_(0) -= 0.05f;
        //   }
        // } break;
        // case 'j': {
        //   if (stateMachine.ctrlModeConveyorBelt) {
        //     nominalSpeedConveyorBelt_ += 50;
        //   } else {
        //     deltaAng_(0) += 0.05f;
        //   }
        // } break;
        // case 'k': {
        //   if (stateMachine.ctrlModeConveyorBelt) {
        //     adaptationActive_ = !adaptationActive_;
        //   } else {
        //     deltaAng_(1) -= 0.05f;
        //   }
        // } break;
        // case 'm': {
        //   if (stateMachine.ctrlModeConveyorBelt) {
        //     magniturePertConveyorBelt_ -= 50;
        //   } else {
        //     deltaAng_(2) -= 0.05f;
        //   }
        // } break;
        // case 'i': {
        //   if (stateMachine.ctrlModeConveyorBelt) {
        //     magniturePertConveyorBelt_ += 50;
        //   } else {
        //     deltaAng_(2) += 0.05f;
        //   }
        // } break;
    }
  }
  nonBlock(0);

  return stateMachine;
}


}// namespace keyboardinteraction