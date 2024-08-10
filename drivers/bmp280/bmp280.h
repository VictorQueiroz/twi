#ifndef BMP280_H
#define BMP280_H

#include <twi.h>

struct bmp280_t
{
  struct twi_t* twi;
};

enum twi_result_t bmp280_init(struct bmp280_t*);

#endif //BMP280_H
