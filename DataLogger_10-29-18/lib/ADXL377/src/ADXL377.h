#include "Arduino.h"

class ADXL377{
  private:
    uint8_t xpin;
    uint8_t ypin;
    uint8_t zpin;

    float mapf(float,float, float, float, float);
  public:
    ADXL377();

    void begin(uint8_t,uint8_t,uint8_t);

    float readX();
    float readY();
    float readZ();

    void set_x(uint8_t);
    void set_y(uint8_t);
    void set_z(uint8_t);
};
