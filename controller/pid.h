// PID Control
int pidControl(
    int target, 
    int current, 
    int summed_error_limit, 
    int control_limit_min, 
    int control_limit_max,
    float kp,
    float ki,
    float kd
) {

  int u;
  static int oldsensor = 0;
  int velocity = current - oldsensor;
  
  static int summederror = 0;
  int error = target - current;
  summederror += error;

  // Anti-windup
  int LARGE = 1000;
  if (error < 0) summederror = (int)(summederror * 0.5);
  if (summederror > summed_error_limit) summederror = summed_error_limit;

  u = (int)(kp * error + ki * summederror + kd * velocity);

  // Max and min control - we will provide this control input as PWM to the motor
  const int MAX = control_limit_max;
  const int MIN = control_limit_min;
  if (u > MAX) u = MAX;
  if (u < MIN) u = MIN;
  
  oldsensor = current;
  return u;
}