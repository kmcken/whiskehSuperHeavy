namespace whiskehSuperHeavy.Avionics;

public class AngleOfAttack
{
    
    /// <summary>
    /// Computes angle of attack from quaternion and intertial velocity.
    /// </summary>
    public static double AOAangle(double[] quaternion, double[] velocityInertial)
    {
        if (quaternion.Length != 4 || velocityInertial.Length != 3)
            throw new ArgumentException("Invalid input sizes.");

        double[] velocityBody = TransformVelocity(quaternion, velocityInertial);
        return Math.Atan2(velocityBody[2], velocityBody[0]) * (180 / Math.PI); // To degrees
    }
        
    /// <summary>
    /// Computes sideslip angle from body velocity [u, v, w].
    /// </summary>
    public static double SideSlipAngle(double[] quaternion, double[] velocityInertial)
    {
        if (quaternion.Length != 4 || velocityInertial.Length != 3)
            throw new ArgumentException("Invalid input sizes.");

        double[] velocityBody = TransformVelocity(quaternion, velocityInertial);
        double u = velocityBody[0], v = velocityBody[1], w = velocityBody[2];
        double V = Math.Sqrt(u * u + v * v + w * w);
        if (V == 0) return 0; // Avoid division by zero
        return Math.Asin(v / V) * (180 / Math.PI); // To degrees
    }
    
    /// <summary>
    /// Transforms inertial velocity to body frame using quaternion.
    /// </summary>
    /// <param name="quaternion">Quaternion [w, x, y, z]</param>
    /// <param name="vInertial">Inertial velocity [vx, vy, vz]</param>
    /// <returns>Body velocity [u, v, w]</returns>
    public static double[] TransformVelocity(double[] quaternion, double[] vInertial)
    {
        if (quaternion.Length != 4 || vInertial.Length != 3)
            throw new ArgumentException("Invalid input sizes.");

        // Rotation matrix (inertial to body)
        double[,] R = AvionicsMath.RotationMatrix(quaternion);

        // Matrix-vector multiplication: v_body = R * v_inertial
        double[] vBody = new double[3];
        for (int i = 0; i < 3; i++)
        {
            vBody[i] = R[i, 0] * vInertial[0] + R[i, 1] * vInertial[1] + R[i, 2] * vInertial[2];
        }

        return vBody;
    }


}