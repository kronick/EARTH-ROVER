class ActuatorSet {
  public float a, b, c;
  float min = 352;
  float max = 499;
  
  ActuatorSet(float a, float b, float c) {
    this.a = a;
    this.b = b;
    this.c = c;
  }
  
  boolean isValid(ActuatorSet S) {
    return (S.a < max && S.a > min && S.b < max && S.b > min && S.c < max && S.c > min);
  }
}
