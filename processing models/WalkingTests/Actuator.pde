public class Actuator {
  float length;
  float targetLength;

  float minLength;
  float maxLength;
  float midLength;
  float maxSpeed;  // in mm/sec

  private float lastUpdate;  // in milliseconds
  private float kP;

  private float drift;
  private float noiseFactor;
  
  Leg parent;
  
  Actuator(Leg parent, float minLength, float maxLength, float maxSpeed) {
    this.parent = parent;
    this.minLength = minLength;
    this.maxLength = maxLength;
    this.midLength = minLength + (maxLength-minLength)/2;
    this.maxSpeed = maxSpeed;
    
    this.length = midLength;
    
    this.kP = 0.5f;
    this.drift = 0; //random(-maxSpeed/200, maxSpeed/200);
    this.noiseFactor = 0; //random(0, maxSpeed/200.);
    
    lastUpdate = millis();
  }
  
  void update() {
    float dT = millis() - lastUpdate;
    lastUpdate = millis();
    float dP = (targetLength-length) * this.kP;
    float speed = abs(dP/(dT/1000.));
    if(speed > maxSpeed)
      dP *= maxSpeed/speed;
      
    float newLength = length + dP;
    newLength += this.drift;
    newLength += random(-noiseFactor, noiseFactor);
    if(isPossible(newLength))
      this.length = newLength;
  }
  
  void draw() {
    
  }
  
  boolean isPossible(float L) {
    return (L <= this.maxLength && L >= this.minLength);
  }
  
  boolean setTarget(float L, boolean force) {
    if(force || isPossible(L)) {
      this.targetLength = L;
      return true;
    }
    else return false;
  }
}
