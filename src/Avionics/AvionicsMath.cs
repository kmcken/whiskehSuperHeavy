namespace whiskehSuperHeavy.Avionics;

public class AvionicsMath
{
    
    /// <summary>
    /// Rotation Matrix
    /// Transforms a vector from inertial to body frame using quaternion.
    /// </summary>
    public static double[,] RotationMatrix(double[] quaternion)
    {
        if (quaternion.Length != 4)
            throw new ArgumentException("Invalid input size."); 
        
        double w = quaternion[0], x = quaternion[1], y = quaternion[2], z = quaternion[3];

        // Rotation matrix (inertial to body)
        double[,] R = new double[3, 3];
        R[0, 0] = 1 - 2 * (y * y + z * z);
        R[0, 1] = 2 * (x * y - z * w);
        R[0, 2] = 2 * (x * z + y * w);

        R[1, 0] = 2 * (x * y + z * w);
        R[1, 1] = 1 - 2 * (x * x + z * z);
        R[1, 2] = 2 * (y * z - x * w);

        R[2, 0] = 2 * (x * z - y * w);
        R[2, 1] = 2 * (y * z + x * w);
        R[2, 2] = 1 - 2 * (x * x + y * y);

        return R;
    }
    
    /// <summary>
    /// Transforms a vector from inertial to body frame using quaternion.
    /// </summary>
    public static double[] FrameTransform(double[] q, double[] vecInertial)
    {
        double w = q[0], x = q[1], y = q[2], z = q[3];

        // Rotation matrix (inertial to body)
        double[,] R = RotationMatrix(q);

        // Matrix-vector multiplication
        double[] vecBody = new double[3];
        for (int i = 0; i < 3; i++)
        {
            vecBody[i] = R[i, 0] * vecInertial[0] + R[i, 1] * vecInertial[1] + R[i, 2] * vecInertial[2];
        }

        return vecBody;
    }
}