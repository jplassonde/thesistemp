#include "project.h"
#include "Wm8994.h"
#include "DRCConf.h"

bool Wm8994::isInstanciated = false;
Wm8994* Wm8994::wm = nullptr;
DMA_HandleTypeDef saiTxDMA;
DMA_HandleTypeDef saiRxDMA;
SAI_HandleTypeDef saiHandle;
SAI_HandleTypeDef saiHandleRx;
#define WMADDR (uint16_t)0x34

volatile uint16_t saiBuffer[882] __attribute__((aligned (32))) __attribute__ ((section(".dtcmram")));

// Test A440
const uint16_t dmaBuffer[] = {
		0x8000,0x8405,0x8809,0x8c0b,0x900a,0x9405,0x97fc,0x9bec,0x9fd4,0xa3b5,
		0xa78d,0xab5b,0xaf1e,0xb2d5,0xb67f,0xba1c,0xbda9,0xc128,0xc495,0xc7f2,
		0xcb3c,0xce73,0xd196,0xd4a5,0xd79e,0xda82,0xdd4e,0xe003,0xe29f,0xe523,
		0xe78d,0xe9dd,0xec12,0xee2c,0xf02a,0xf20c,0xf3d0,0xf578,0xf702,0xf86e,
		0xf9bb,0xfaea,0xfbfa,0xfcea,0xfdbb,0xfe6c,0xfefd,0xff6e,0xffbe,0xffef,
		0xffff,0xffef,0xffbe,0xff6e,0xfefd,0xfe6c,0xfdbb,0xfcea,0xfbfa,0xfaea,
		0xf9bb,0xf86e,0xf702,0xf578,0xf3d0,0xf20c,0xf02a,0xee2c,0xec12,0xe9dd,
		0xe78d,0xe523,0xe29f,0xe003,0xdd4e,0xda82,0xd79e,0xd4a5,0xd196,0xce73,
		0xcb3c,0xc7f2,0xc495,0xc128,0xbda9,0xba1c,0xb67f,0xb2d5,0xaf1e,0xab5b,
		0xa78d,0xa3b5,0x9fd4,0x9bec,0x97fc,0x9405,0x900a,0x8c0b,0x8809,0x8405,
		0x8000,0x7bfa,0x77f6,0x73f4,0x6ff5,0x6bfa,0x6803,0x6413,0x602b,0x5c4a,
		0x5872,0x54a4,0x50e1,0x4d2a,0x4980,0x45e3,0x4256,0x3ed7,0x3b6a,0x380d,
		0x34c3,0x318c,0x2e69,0x2b5a,0x2861,0x257d,0x22b1,0x1ffc,0x1d60,0x1adc,
		0x1872,0x1622,0x13ed,0x11d3,0xfd5,0xdf3,0xc2f,0xa87,0x8fd,0x791,
		0x644,0x515,0x405,0x315,0x244,0x193,0x102,0x91,0x41,0x10,
		0x0,0x10,0x41,0x91,0x102,0x193,0x244,0x315,0x405,0x515,
		0x644,0x791,0x8fd,0xa87,0xc2f,0xdf3,0xfd5,0x11d3,0x13ed,0x1622,
		0x1872,0x1adc,0x1d60,0x1ffc,0x22b1,0x257d,0x2861,0x2b5a,0x2e69,0x318c,
		0x34c3,0x380d,0x3b6a,0x3ed7,0x4256,0x45e3,0x4980,0x4d2a,0x50e1,0x54a4,
		0x5872,0x5c4a,0x602b,0x6413,0x6803,0x6bfa,0x6ff5,0x73f4,0x77f6,0x7bfa};

Wm8994::Wm8994() {
    // Init SAI1 GPIO
    GPIO_InitTypeDef GPIO_Init;
    GPIO_Init.Pin = GPIO_PIN_7;  // MCLKA
    GPIO_Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_Init.Mode = GPIO_MODE_AF_PP;
    GPIO_Init.Alternate = GPIO_AF6_SAI1;
    GPIO_Init.Pull      = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOG, &GPIO_Init);

    GPIO_Init.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6; // SDB, FSA, SCKA, SDA
    HAL_GPIO_Init(GPIOE, &GPIO_Init);
/*
    GPIO_Init.Pin = GPIO_PIN_12;
    GPIO_Init.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOJ, &GPIO_Init);

    NVIC_SetPriority(EXTI15_10_IRQn, 10);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
*/
    // Configure TX DMA

/*
    saiTxDMA.Instance = DMA2_Stream1;
    saiTxDMA.Init.Channel = DMA_CHANNEL_0;
    saiTxDMA.Init.Direction = DMA_MEMORY_TO_PERIPH;
    saiTxDMA.Init.PeriphInc = DMA_PINC_DISABLE;
    saiTxDMA.Init.MemInc = DMA_MINC_ENABLE;
    saiTxDMA.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    saiTxDMA.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    saiTxDMA.Init.Mode = DMA_CIRCULAR;
    saiTxDMA.Init.Priority = DMA_PRIORITY_HIGH;
    saiTxDMA.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    saiTxDMA.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    saiTxDMA.Parent = &saiHandle;
    HAL_DMA_Init(&saiTxDMA);

    NVIC_SetPriority(DMA2_Stream1_IRQn, SAI_DMA_PRIO);
    NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    */

	__HAL_RCC_DMA2_CLK_ENABLE();

    saiRxDMA.Instance = DMA2_Stream4;
	saiRxDMA.Init.Channel = DMA_CHANNEL_1;
	saiRxDMA.Init.Direction = DMA_PERIPH_TO_MEMORY;
	saiRxDMA.Init.PeriphInc = DMA_PINC_DISABLE;
	saiRxDMA.Init.MemInc = DMA_MINC_ENABLE;
	saiRxDMA.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	saiRxDMA.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	saiRxDMA.Init.Mode = DMA_CIRCULAR;
	saiRxDMA.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	saiRxDMA.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	saiRxDMA.Parent = &saiHandleRx;
    NVIC_SetPriority(DMA2_Stream4_IRQn, 8);
    NVIC_EnableIRQ(DMA2_Stream4_IRQn);
	HAL_DMA_Init(&saiRxDMA);

    // Configure SAI block & start MCLK
    __HAL_SAI_RESET_HANDLE_STATE(&saiHandle);
    saiHandle.Instance = SAI1_Block_A;
    __HAL_SAI_DISABLE(&saiHandle);

	saiHandle.Init.MonoStereoMode = SAI_STEREOMODE;
    saiHandle.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
    saiHandle.Init.AudioMode = SAI_MODEMASTER_TX;
    saiHandle.Init.NoDivider = SAI_MASTERDIVIDER_ENABLED;
    saiHandle.Init.Protocol = SAI_FREE_PROTOCOL;
    saiHandle.Init.DataSize = SAI_DATASIZE_16;
    saiHandle.Init.FirstBit = SAI_FIRSTBIT_MSB;
    saiHandle.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
    saiHandle.Init.Synchro = SAI_ASYNCHRONOUS;
    saiHandle.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLED;
    saiHandle.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_HF;
    saiHandle.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
    saiHandle.Init.Mckdiv = 0;
    saiHandle.FrameInit.FrameLength = 32;
    saiHandle.FrameInit.ActiveFrameLength = 16;
    saiHandle.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
    saiHandle.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
    saiHandle.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
    saiHandle.SlotInit.FirstBitOffset = 0;
    saiHandle.SlotInit.SlotSize = SAI_SLOTSIZE_16B;
    saiHandle.SlotInit.SlotNumber = 2;
    saiHandle.SlotInit.SlotActive = SAI_SLOTACTIVE_ALL;

    saiHandleRx = saiHandle;

    __HAL_RCC_SAI1_CLK_ENABLE();
	HAL_SAI_Init(&saiHandle);
    __HAL_SAI_ENABLE(&saiHandle);


    saiHandleRx.hdmarx = &saiRxDMA;
    saiHandleRx.Instance = SAI1_Block_B;
    saiHandleRx.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
    saiHandleRx.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLED;
    saiHandleRx.Init.Synchro = SAI_SYNCHRONOUS;
    saiHandleRx.Init.AudioMode = SAI_MODESLAVE_RX;

    HAL_SAI_Init(&saiHandleRx);
    __HAL_SAI_ENABLE(&saiHandleRx);

    NVIC_SetPriority(SAI1_IRQn, SAI_DMA_PRIO); // Overrun detection, not really used for anything yet though
    NVIC_EnableIRQ(SAI1_IRQn);

    // Reset Codec
    setCodecReg(0, 0);
    HAL_Delay(200);

    setCodecReg(0x102, 0x0003);
    setCodecReg(0x817, 0x0000);
    setCodecReg(0x102, 0x0000);

    // Enable headphone's DACs & Input path
    setCodecReg(0x05, 0x303);

    // Digital Output mixer control (p65-69). Timeslot 0 to DAC 1
    setCodecReg(0x601, 1); // Left
    setCodecReg(0x602, 1); // Right

    // AIF1 Setup
    setCodecReg(0x210, 0x73); // Set AIF1 sampling to 44.1khz
    //setCodecReg(0x300, 0x0050); // Word length set to 24 bit, Data format = I2S
    setCodecReg(0x300, 0x4010); // Word length set to 16 bit, DF Left Justified / MSB on first clock edge

    //setCodecReg(0x300, 0x0010); // FORCE MONO!!!1


    setCodecReg(0x208, 0x0A); // AIF1 Processing Clock & Digital Mixing Clock enable
    setCodecReg(0x200, 0x01); // set AIF1 clock source to MCLK & enable it

    // Analog Path
    setCodecReg(0x2D, 0x100); // Enable path from DAC1L to HPOUT1L
    setCodecReg(0x2E, 0x100); // Enable path from DAC1L to HPOUT1R
    // Default startup sequence -> Headphone cold start-up (datasheet p218)
    setCodecReg(0x110, 0x8100);
    HAL_Delay(350); // Pause for startup

    setCodecReg(0x51, 0x05);; // Dynamic Charge pump
    setCodecReg(0x610, 0xC0); // Soft unmute DAC
    setCodecReg(0x611, 0xC0);
    setCodecReg(0x1C, 0x3F/2 | 0x140); // Set PGA vol to half
    setCodecReg(0x1D, 0x3F/2 | 0x140);
    setCodecReg(0x420, 0); // Soft unmute DAC input path

    // Setup the Line-In path
    // VMID, VMID Divider 2 x 40kOhm & bias current generator are already enabled through the headphone cold startup sequence
    //setCodecReg(0x02, 0x350); // IN1L & IN1R PGA Enabled, R/L in mixers Enabled

    // DRC
    setCodecReg(0x440,
    		SIG_DET_PK_VAL << SIG_DET_PK_POS |
			SIG_DET_MODE << SIG_DET_MODE_POS |
			SIG_DET_ENA << SIG_DET_ENA_POS |
			NG_ENA << NG_ENA_POS |
			KNEE_2_ENA << KNEE_2_ENA_POS |
			QR_ENA << QR_ENA_POS |
			AC_ENA << AC_ENA_POS |
			DRC_OUT);

    setCodecReg(0x441,
    		GAIN_AR << GAIN_AR_POS |
			GAIN_DCAY << GAIN_DCAY_POS |
			MIN_ATT_GAIN << MIN_ATT_GAIN_POS |
			MAX_BOOST_GAIN << MAX_BOOST_GAIN_POS );

    setCodecReg(0x442,
    		NG_MIN_GAIN << NG_MIN_GAIN_POS |
			NG_SLOPE << NG_SLOPE_POS |
			QR_THRES << QR_THRES_POS |
			QR_DCAY << QR_DCAY_POS |
			UPPER_SLOPE << UPPER_SLOPE_POS |
			LOWER_SLOPE << LOWER_SLOPE_POS );

    setCodecReg(0x443,
    		KNEE_IN_LVL << KNEE_IN_LVL_POS |
			KNEE_OUT_LVL << KNEE_OUT_LVL_POS );

    setCodecReg(0x444,
    		NG_KNEE_IN_LVL << NG_KNEE_IN_LVL_POS |
    		NG_KNEE_OUT_LVL << NG_KNEE_OUT_LVL_POS);

    setCodecReg(0x700, 0x0D); // GPIO. DRC Signal detect
    setCodecReg(0x02, 0x6350); // Power Management - thermal sensor, etc
    setCodecReg(0x01, 0x3303); // Power Management - HP out enabled / VMID SEL
    setCodecReg(0x620, 0x02); // ADC & DAC oversampling set to high performance
    setCodecReg(0x208, 0x0A); // AIF1 Processing & Digital Mixing Clock enable
    setCodecReg(0x4c, 0x9f25);
    HAL_Delay(15);
    setCodecReg(0x410, 0x1800); // HPF on ADC L/R
    setCodecReg(0x421, 0x0300); // 3D Stereo on line out

    setCodecReg(0x28, 0x11); // Single ended mode. Non-inverting input connected to VMID, Inverting input connected to IN1LN/IN1RN
    setCodecReg(0x18, 0x0B); // Unmute Left PGA 0db (D= + 3dB)
    setCodecReg(0x1A, 0x0B); // Unmute Right PGA 0db (D= + 3dB)
    setCodecReg(0x29, 0x25); // Unmute Left PGA in input mixer / vol 0dB
    setCodecReg(0x2A, 0x25); // Unmute Right PGA in input mixer / vol 0dB
    setCodecReg(0x04, 0x303); // Left and right ADC Enabled // AIF1ADC1 output path to AIF1 TS0 Enabled
    setCodecReg(0x606, 0x02); // Enable ADCL to AIF1/Timeslot 0, Left
    setCodecReg(0x607, 0x02); // Enable ADCR to AIF1/Timeslot 0, Right

    // Mixer input
    setCodecReg(0x402, 0x100 | 0x7F);
    setCodecReg(0x403, 0x100 | 0x7F);

    //setCodecReg(0x301, 0x4001); // AIF1 Digital Loopback (Line in to AIF1)


    for (int i = 0; i < 882; i++)
    	saiBuffer[i] = 0;
    HAL_SAI_Receive_DMA(&saiHandleRx, (uint8_t*)saiBuffer, 882);
    //HAL_SAI_Receive_IT(&saiHandle, (uint8_t*)saiBuffer, 882);
    HAL_SAI_Transmit_IT(&saiHandle, (uint8_t*)saiBuffer, 882);
    isInstanciated = true;
    wm = this;
}

Wm8994::~Wm8994() {
    isInstanciated = false;
    wm = nullptr;
}

Wm8994* Wm8994::getInstance() {
    if (!isInstanciated) {
        wm = new Wm8994();
        isInstanciated = true;
    }
    return wm;
}

void Wm8994::init() {
	if (wm == nullptr)
		wm = new Wm8994();
}

void Wm8994::setFs(freq frequency) {
	// Only 44100 Hz is supported at the moment
	if (frequency == f44khz) {
		// Set I2S block
		// Set Codec
	} else if (frequency == f48khz) {
		// Set I2S block
		// Set codec
	}
}

// Was meant for MP3 playback, but it has been removed from the project, keeping the stub.
void Wm8994::setSource(input in) {
	if (in == passthrough) {

	} else if (in == sai) {

	}
}

void Wm8994::setVolume(uint8_t vol) {
	uint8_t val = 0x3F * vol / 100;
	if (vol > 2) {
	    setCodecReg(0x1C, 0x140 | val);
	    setCodecReg(0x1D, 0x140 | val);
	} else {
		 setCodecReg(0x1C, 0x100);
		 setCodecReg(0x1D, 0x100);
	}


// This must have been used for testing. Not sure why or why it is still there.
	/*
    setCodecReg(0x1C, 0x140 | val); // Update both L/R Volume
    setCodecReg(0x1D, 0x140 | val); // Update both L/R Volume

	uint8_t val = vol * 0xC0 / 100;
	setCodecReg(0x402, 0x100 | val);
	setCodecReg(0x403, 0x100 | val);
	}
	*/
}

// Could be useful at some point but since the noise has been resolved there is no immediate need for a mute button.
void Wm8994::mute() {

}

void Wm8994::unmute() {

}

void Wm8994::setCodecReg(uint16_t addr, uint16_t data) {
	uint16_t txData = ((data << 8) & 0xFF00) |  ((data >> 8) & 0xFF);

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xSemaphoreTake(xI2C4Mutex, portMAX_DELAY);
    }
    HAL_I2C_Mem_Write(&i2c4, WMADDR, addr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&txData, 2, 1000);

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xSemaphoreGive(xI2C4Mutex);
    }
}

// IRQ Handlers
extern "C" {
void SAI1_IRQHandler() {
    HAL_SAI_IRQHandler(&saiHandle);
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
  // HAL_SAI_Receive_IT(&saiHandle, (uint8_t*)saiBuffer, 882);

}
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {
	 HAL_SAI_Transmit_IT(&saiHandle, (uint8_t*)saiBuffer, 882);
}

void DMA2_Stream1_IRQHandler() {
    HAL_DMA_IRQHandler(&saiRxDMA);
}

void DMA2_Stream4_IRQHandler() {
	 HAL_DMA_IRQHandler(&saiRxDMA);
}

void DMA1_Stream3_IRQHandler() {
    HAL_DMA_IRQHandler(&saiRxDMA);
}

}
