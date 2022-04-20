#include "Haptics.h"

Haptics::Haptics(int count)
{
    Wire.begin();
    for (int i = 0; i < count; i++)
    {
        vibeselect(i);

        // drv setup
        drv->begin();

        // I2C trigger by sending 'go' command
        drv->setMode(DRV2605_MODE_INTTRIG); // default, internal trigger when sending GO command

        drv->selectLibrary(1);
        // 5ms for each wave segment
        drv->setWaveform(0, 14); // ramp up medium 1, see datasheet part 11.2
        // drv->setWaveform(1, 47); // strong click 100%, see datasheet part 11.2
        drv->setWaveform(1, 0); // end of waveforms
    }
}

void Haptics::vibeselect(uint8_t i)
{
    if (i > 7)
        return;

    switch (i)
    {
    case 0:
        drv = &drv0;
        break;
    case 1:
        drv = &drv1;
        break;
    case 2:
        drv = &drv2;
        break;
    case 3:
        drv = &drv3;
        break;
    case 4:
        drv = &drv4;
        break;
    }

    Wire.beginTransmission(MUXADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}
