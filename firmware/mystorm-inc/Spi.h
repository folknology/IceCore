#pragma once

#include <stdint.h>
#include "mystorm.h"

class Spi  {
    public:
        Spi(){};
        void Boot_Enable(void);
        void Enable(void);
        void Disable(void);
};
