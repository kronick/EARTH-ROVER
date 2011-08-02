class MoveVector {
  PVector translation;
  float heading;
  float speed;
  float rotation;
  float pitch;
  float yaw;
  float roll;
  
  MoveVector() {
    this.translation = new PVector(0,0,0);
    this.heading = 0;
    this.speed = 0;
    this.rotation = 0;
  }
  MoveVector(PVector t, float r) {
    this.translation = t.get();
    this.rotation = r;
    this.yaw = r;
  }
  MoveVector(PVector t, float yaw, float pitch, float roll) {
    this.translation = t.get();
    this.rotation = yaw;
    this.yaw = yaw;
    this.pitch = pitch;
    this.roll = roll;
  }
  
  MoveVector get() {
    return new MoveVector(this.translation, this.yaw, this.pitch, this.roll);
  }
}
