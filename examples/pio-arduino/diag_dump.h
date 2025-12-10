#pragma once
#include <Arduino.h>
#include <esp_camera.h>

#ifdef __cplusplus
extern "C" {
#endif

void initDiagnostics();     // call once in setup
void runDiagnostics();      // dumps everything BEFORE server start

#ifdef __cplusplus
}
#endif
