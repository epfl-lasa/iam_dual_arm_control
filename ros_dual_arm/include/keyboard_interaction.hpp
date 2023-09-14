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

struct InteractionVar {
  StateMachine stateMachine;
  ConveyorBeltState conveyorBeltState;
  bool startLogging;
  bool resetLogging;
};

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

InteractionVar getKeyboard(InteractionVar interactionVar) {

  nonBlock(1);

  if (khBit() != 0) {
    char keyboardCommand = fgetc(stdin);
    fflush(stdin);

    switch (keyboardCommand) {
      case 'q': {
        interactionVar.stateMachine.goHome = !interactionVar.stateMachine.goHome;
        if (interactionVar.stateMachine.goHome) {
          interactionVar.stateMachine.goToAttractors = true;
          interactionVar.startLogging = false;
        } else if (!interactionVar.stateMachine.goHome) {
          interactionVar.startLogging = true;
        }
      } break;
      case 'g': {
        interactionVar.stateMachine.goToAttractors = !interactionVar.stateMachine.goToAttractors;
        if (interactionVar.stateMachine.goToAttractors) {
          interactionVar.stateMachine.goHome = false;
          interactionVar.stateMachine.releaseAndretract = false;
        }
      } break;

      // Release or throwing
      case 'r': {
        interactionVar.stateMachine.releaseAndretract = !interactionVar.stateMachine.releaseAndretract;
      } break;
      case 'l': {
        interactionVar.stateMachine.dualTaskSelector = PICK_AND_LIFT;
      } break;
      case 't': {
        interactionVar.stateMachine.isThrowing = !interactionVar.stateMachine.isThrowing;
        if (interactionVar.stateMachine.isThrowing) {
          interactionVar.stateMachine.dualTaskSelector = PICK_AND_TOSS;
        } else if (!interactionVar.stateMachine.isThrowing) {
          interactionVar.stateMachine.dualTaskSelector = PICK_AND_LIFT;
        }
      } break;
      case 'p': {
        interactionVar.stateMachine.isPlacing = !interactionVar.stateMachine.isPlacing;
        if (interactionVar.stateMachine.isPlacing) {
          interactionVar.stateMachine.dualTaskSelector = PICK_AND_PLACE;
        } else if (!interactionVar.stateMachine.isPlacing) {
          interactionVar.stateMachine.dualTaskSelector = PICK_AND_LIFT;
        }
      } break;
      case 'o': {
        interactionVar.stateMachine.isPlaceTossing = !interactionVar.stateMachine.isPlaceTossing;
        if (interactionVar.stateMachine.isPlaceTossing) {
          interactionVar.stateMachine.dualTaskSelector = PLACE_TOSSING;
        } else if (!interactionVar.stateMachine.isPlaceTossing) {
          interactionVar.stateMachine.dualTaskSelector = PICK_AND_LIFT;
        }
      } break;

      // Impact and tossing velocity
      case 'v': {
        interactionVar.stateMachine.desVtoss -= 0.05f;
        if (interactionVar.stateMachine.desVtoss < 0.2f) { interactionVar.stateMachine.desVtoss = 0.2f; }
      } break;
      case 'b': {
        interactionVar.stateMachine.desVtoss += 0.05f;
        if (interactionVar.stateMachine.desVtoss > 2.0f) { interactionVar.stateMachine.desVtoss = 2.0f; }
      } break;
      case 'y': {
        interactionVar.stateMachine.desiredVelImp -= 0.05f;
        if (interactionVar.stateMachine.desiredVelImp < 0.05f) { interactionVar.stateMachine.desiredVelImp = 0.05f; }
      } break;
      case 'u': {
        interactionVar.stateMachine.desiredVelImp += 0.05f;
        if (interactionVar.stateMachine.desiredVelImp > 0.6f) { interactionVar.stateMachine.desiredVelImp = 0.6f; }
      } break;

      // Reset the data logging
      case 'c': {
        interactionVar.startLogging = false;
        interactionVar.resetLogging = true;
      } break;

      // Disturb the target speed
      case 'e': {
        interactionVar.conveyorBeltState.isDisturbTarget = !interactionVar.conveyorBeltState.isDisturbTarget;
      } break;

      // Placing height
      case 'x': {
        if (interactionVar.stateMachine.dualTaskSelector == PICK_AND_TOSS) {
          interactionVar.stateMachine.releasePosY -= 0.01;
        } else {
          interactionVar.stateMachine.placingPosHeight -= 0.01;
        }
      } break;
      case 'n': {
        if (interactionVar.stateMachine.dualTaskSelector == PICK_AND_TOSS) {
          interactionVar.stateMachine.releasePosY += 0.01;
        } else {
          interactionVar.stateMachine.placingPosHeight += 0.01;
        }
      } break;

      // Conveyor belt control
      case 'a': {
        if (interactionVar.conveyorBeltState.ctrlModeConveyorBelt) {
          interactionVar.conveyorBeltState.modeConveyorBelt = 2;
          interactionVar.startLogging = true;
        } else if (interactionVar.stateMachine.incrementReleasePos) {
          interactionVar.stateMachine.deltaRelPos(0) -= 0.025f;//[m]
        } else {
          interactionVar.stateMachine.deltaPos(0) -= 0.01f;
        }
      } break;
      case 's': {
        if (interactionVar.conveyorBeltState.ctrlModeConveyorBelt) {
          interactionVar.conveyorBeltState.modeConveyorBelt = 0;
        } else if (interactionVar.stateMachine.incrementReleasePos) {
          interactionVar.stateMachine.deltaRelPos(0) += 0.025f;//[m]
        } else {
          interactionVar.stateMachine.deltaPos(0) += 0.01f;
        }
      } break;
      case 'd': {
        if (interactionVar.conveyorBeltState.ctrlModeConveyorBelt) {
          interactionVar.conveyorBeltState.modeConveyorBelt = 1;
        } else if (interactionVar.stateMachine.incrementReleasePos) {
          interactionVar.stateMachine.deltaRelPos(1) -= 5.0f;//[deg]
        } else {
          interactionVar.stateMachine.deltaPos(1) -= 0.01f;
        }
      } break;
      case 'f': {
        if (interactionVar.stateMachine.incrementReleasePos) {
          interactionVar.stateMachine.deltaRelPos(1) += 5.0f;//[deg]
        } else {
          interactionVar.stateMachine.deltaPos(1) += 0.01f;
        }
      } break;
      case 'z': {
        if (interactionVar.conveyorBeltState.ctrlModeConveyorBelt) {
          interactionVar.stateMachine.trackingFactor -= 0.01f;
        } else if (interactionVar.stateMachine.incrementReleasePos) {
          interactionVar.stateMachine.deltaRelPos(2) -= 5.0f;//[deg]
        } else {
          interactionVar.stateMachine.deltaPos(2) -= 0.01f;
        }
      } break;
      case 'w': {
        if (interactionVar.conveyorBeltState.ctrlModeConveyorBelt) {
          interactionVar.stateMachine.trackingFactor += 0.01f;
        } else if (interactionVar.stateMachine.incrementReleasePos) {
          interactionVar.stateMachine.deltaRelPos(2) += 5.0f;//[deg]
        } else {
          interactionVar.stateMachine.deltaPos(2) += 0.01f;
        }
      } break;
      case 'h': {
        if (interactionVar.conveyorBeltState.ctrlModeConveyorBelt) {
          interactionVar.conveyorBeltState.nominalSpeedConveyorBelt -= 50;
        }
      } break;
      case 'j': {
        if (interactionVar.conveyorBeltState.ctrlModeConveyorBelt) {
          interactionVar.conveyorBeltState.nominalSpeedConveyorBelt += 50;
        }
      } break;
      case 'k': {
        if (interactionVar.conveyorBeltState.ctrlModeConveyorBelt) {
          interactionVar.stateMachine.adaptationActive = !interactionVar.stateMachine.adaptationActive;
        }
      } break;
      case 'm': {
        if (interactionVar.conveyorBeltState.ctrlModeConveyorBelt) {
          interactionVar.conveyorBeltState.magniturePertConveyorBelt -= 50;
        }
      } break;
      case 'i': {
        if (interactionVar.conveyorBeltState.ctrlModeConveyorBelt) {
          interactionVar.conveyorBeltState.magniturePertConveyorBelt += 50;
        }
      } break;
    }
  }
  nonBlock(0);

  return interactionVar;
}

}// namespace keyboardinteraction