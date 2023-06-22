//Warning: define baud rate before define the object.
class Encoder {
  private:
    signed int initValue;
    signed int AnalogPort;

  public:
    Encoder(int port) {
      AnalogPort = port;
    }

    void initializeEncoder(float angleOffSet) {
      for (int i = 0; i < 10; i++) analogRead(AnalogPort);
      initValue = analogRead(AnalogPort)+int(angleOffSet/360*1023);
    }

    void initializeEncoder(float angleOffSet, int userInitValue) {
      initValue = userInitValue+int(angleOffSet/360*1023);
    }
    
    float readADC(){
      return analogRead(AnalogPort);
    }
    
    float readRawAngle() {
      float angle = float(analogRead(AnalogPort) - initValue) / 1023 * 360;
      return angle < 0 ? angle + 360 : angle;
    }

    float getAngleDeplacedBy(float angleDeplaced){
      return readRawAngle() + angleDeplaced;
    }

    int getInitValue() {
      return initValue;
    }

    float getInitAngle() {
      return float(initValue) / 1023 * 360;
    }

};
