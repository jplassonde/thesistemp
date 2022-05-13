#pragma once

#include <cstdint>
#include "stm32f7xx_hal_sai.h"
#include "stm32f7xx_hal_dma.h"

class Wm8994 {
public:
    enum freq : uint8_t {f44khz, f48khz};
    enum input : uint8_t {passthrough, sai};
    static Wm8994* getInstance();
    static void init();
    virtual ~Wm8994();

    void setFs(freq frequency);
    void setSource(input in);
    void setVolume(uint8_t vol);
    void mute();
    void unmute();

private:
    Wm8994();
    void setCodecReg(uint16_t addr, uint16_t data);
    static bool isInstanciated;
    static Wm8994 *wm;

};
