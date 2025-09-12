namespace whiskehSuperHeavy.Autopilot;

public class PIDController
{
    public float Kp { get; set; }  // Proportional gain
    public float Ki { get; set; }  // Integral gain
    public float Kd { get; set; }  // Derivative gain

    private float integral = 0f;
    private float previousError = 0f;

    public PIDController(float kp, float ki, float kd)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    public float Update(float error, float deltaTime)
    {
        integral += error * deltaTime;  // Accumulate integral
        float derivative = (error - previousError) / deltaTime;
        previousError = error;

        return (Kp * error) + (Ki * integral) + (Kd * derivative);
    }

    public void Reset()
    {
        integral = 0f;
        previousError = 0f;
    }
}