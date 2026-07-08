#pragma once
#include <Arduino.h>
#include <U8g2lib.h>
#include "state.h"

void ui_init();
void ui_update();
void ui_nextPage();
void ui_markDirty();
