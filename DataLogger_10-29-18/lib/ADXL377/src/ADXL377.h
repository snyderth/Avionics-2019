
class ADXL377{
  private:
    int xpin;
    int ypin;
    int zpin;

    float mapf(float,float, float, float, float);
  public:
    ADXL377();

    float readX();
    float readY();
    float readZ();

    void set_x(int);
    void set_y(int);
    void set_z(int);
};
