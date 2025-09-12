using UnityGERunner;

namespace whiskehSuperHeavy.Avionics;

public static class GLoading
{
    private static Vector3 _prevVelocityInertial = Vector3.zero;
    private static Vector4 _gLoads = Vector4.zero;
    private const double _g = 9.80665; // m/s²
    private const double _dt = 0.0111111; 
    
    
    /// <summary>
    /// Computes G-loadings [n_x, n_y, n_z, total_G] from quaternion and current inertial velocity.
    /// Returns null if no previous velocity.
    /// </summary>
    /// <param name="quaternion">Quaternion [w, x, y, z]</param>
    /// <param name="vInertialCurrent">Current inertial velocity [vx, vy, vz]</param>
    public static Vector4 ComputeGLoadings(Quaternion quaternion, Vector3 vInertialCurrent)
    {
        // Step 1: Inertial acceleration
        double[] aInertial = new double[3];
        aInertial[0] = (vInertialCurrent.x - _prevVelocityInertial.x) / _dt;
        aInertial[1] = (vInertialCurrent.y - _prevVelocityInertial.y) / _dt;
        aInertial[2] = (vInertialCurrent.z - _prevVelocityInertial.z) / _dt;

        // Step 2: Specific force in inertial (NED z down)
        double[] gInertial = { 0.0, 0.0, _g };
        double[]  fInertial = new double[3];
        for (int i = 0; i < 3; i++)
        {
            fInertial[i] = aInertial[i] - gInertial[i];
        }
        
        Vector3 totalInertial = Vector3.zero;
        totalInertial.Set(Convert.ToSingle(fInertial[0]), Convert.ToSingle(fInertial[1]), 
            Convert.ToSingle(fInertial[2]));

        // Step 3: Transform to body frame
        double[,] R = AvionicsMath.RotationMatrixInertialToBody(quaternion);
        Vector3 fBody = AvionicsMath.MatrixMultiplication(R, totalInertial);

        // Step 4: Load factors
        double n_x = fBody.x / _g;
        double n_y = fBody.y / _g;
        double n_z = -fBody.z / _g; // Convention for positive in maneuvers
        double totalG = Math.Sqrt(n_x * n_x + n_y * n_y + n_z * n_z);

        _prevVelocityInertial = vInertialCurrent;
        _gLoads.Set(Convert.ToSingle(totalG), Convert.ToSingle(n_x), Convert.ToSingle(n_y), Convert.ToSingle(n_z));
        return _gLoads;
    }
}