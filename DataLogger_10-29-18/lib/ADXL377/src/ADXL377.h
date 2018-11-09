#ifndef __ADXL377_H__
#define __ADXL377_H__
class ADXL377{
  private:
    int xpin;
    int ypin;
    int zpin;

    float mapf(float,float, float, float, float);
  public:
    ADXL377();
//    ADXL337(int, int, int);

    float readX();
    float readY();
    float readZ();

    void set_x(int);
    void set_y(int);
    void set_z(int);
};
#endif
