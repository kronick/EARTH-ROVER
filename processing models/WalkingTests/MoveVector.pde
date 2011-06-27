class MoveVector {
  PVector translation;
  float heading;
  float speed;
  float rotation;
  
  MoveVector() {
    this.translation = new PVector(0,0,0);
    this.heading = 0;
    this.speed = 0;
    this.rotation = 0;
  }
  MoveVector(PVector t, float r) {
    this.translation = t.get();
    this.rotation = r;
  }
  
  MoveVector get() {
    return new MoveVector(this.translation, this.rotation);
  }
}
