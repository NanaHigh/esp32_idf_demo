#ifndef BH1750_H
#define BH1750_H

#define BH1750_LOW_ADDR             0x23
#define BH1750_POWER_ON_CMD         0x01
#define BH1750_RESET_CMD            0x07
#define BH1750_CONTINUOUS_HIGH_RES_MODE_CMD 0x10

void bh1750_init(void);
float bh1750_read_light(void);

#endif
