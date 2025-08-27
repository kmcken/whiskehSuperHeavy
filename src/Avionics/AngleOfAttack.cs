namespace whiskehSuperHeavy.Avionics;

public static class AngleOfAttack
{
    
    /// <summary>
    /// Computes angle of attack from quaternion and intertial velocity.
    /// </summary>
    /// <param name="quaternion">Quaternion [w, x, y, z]</param>
    /// <param name="vInertial">Inertial velocity [vx, vy, vz]</param>
    public static double AOAangle(double[] quaternion, double[] vInertial)
    {
        if (quaternion.Length != 4 || vInertial.Length != 3)
            throw new ArgumentException("Invalid input sizes.");

        double[] velocityBody = TransformVelocity(quaternion, vInertial);
        return Math.Atan2(velocityBody[2], velocityBody[0]) * (180 / Math.PI); // To degrees
    }
        
    /// <summary>
    /// Computes sideslip angle from body velocity [u, v, w].
    /// </summary>
    /// <param name="quaternion">Quaternion [w, x, y, z]</param>
    /// <param name="vInertial">Inertial velocity [vx, vy, vz]</param>
    public static double SideSlipAngle(double[] quaternion, double[] vInertial)
    {
        if (quaternion.Length != 4 || vInertial.Length != 3)
            throw new ArgumentException("Invalid input sizes.");

        double[] velocityBody = TransformVelocity(quaternion, vInertial);
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

        // Matrix-vector multiplication: v_body = R * v_inertial
        double[,] R = AvionicsMath.RotationMatrixInertialToBody(quaternion);
        double[] vBody = AvionicsMath.FrameTransform(R, vInertial);

        return vBody;
    }


}