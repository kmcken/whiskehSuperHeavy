namespace whiskehSuperHeavy.Avionics;

public class GLoading
{
    private static double[] prevVelocityInertial = null;
    private const double g = 9.80665; // m/s²
    private const double dt = 0.02; // 50 Hz
    
    
    /// <summary>
    /// Computes G-loadings [n_x, n_y, n_z, total_G] from quaternion and current inertial velocity.
    /// Returns null if no previous velocity.
    /// </summary>
    public static double[] ComputeGLoadings(double[] q, double[] vInertialCurrent)
    {
        if (prevVelocityInertial == null || q.Length != 4 || vInertialCurrent.Length != 3)
            return null;

        // Step 1: Inertial acceleration
        double[] aInertial = new double[3];
        for (int i = 0; i < 3; i++)
        {
            aInertial[i] = (vInertialCurrent[i] - prevVelocityInertial[i]) / dt;
        }

        // Step 2: Specific force in inertial (NED z down)
        double[] gInertial = { 0.0, 0.0, g };
        double[] fInertial = new double[3];
        for (int i = 0; i < 3; i++)
        {
            fInertial[i] = aInertial[i] - gInertial[i];
        }

        // Step 3: Transform to body frame
        double[] fBody = AvionicsMath.FrameTransform(q, fInertial);

        // Step 4: Load factors
        double n_x = fBody[0] / g;
        double n_y = fBody[1] / g;
        double n_z = -fBody[2] / g; // Convention for positive in maneuvers
        double totalG = Math.Sqrt(n_x * n_x + n_y * n_y + n_z * n_z);

        return new double[] { n_x, n_y, n_z, totalG };
    }
}