using System.Text.RegularExpressions;
using UnityGERunner;

namespace whiskehSuperHeavy.Avionics;

public class FlightData
{
    public Vector3 InertialPosition;
    public Quaternion Quaternions;
    public Vector3 InertialVelocity;

    public FlightData( Vector3 inertialPosition, Quaternion quaternion, Vector3 inertialVelocity)
    {
        InertialPosition = inertialPosition;
        Quaternions = quaternion;
        InertialVelocity = inertialVelocity;
    }

    private double[,] RotationMatrix => RotationMatrixInertialToBody(Quaternions);
    private double[,] RotationTranspose => RotationMatrixBodyToInertial(Quaternions);
    public Vector3 BodyVelocity => MatrixMultiplication(RotationMatrix, InertialVelocity);
    public float Airspeed => Convert.ToSingle(InertialVelocity.magnitude * 1.94384);
    public float Alpha => Convert.ToSingle(-1 * Math.Atan2(BodyVelocity.y, BodyVelocity.z) * (180 / Math.PI) + 1);
    public float Beta => BodyVelocity.magnitude == 0 ? 0 : 
        Convert.ToSingle(Math.Asin(BodyVelocity.x / BodyVelocity.magnitude) * (180 / Math.PI));
    public float NoseVecPitch => Convert.ToSingle(Math.Asin(RotationMatrix[2,1]) * 180 / Math.PI);
    public float NoseVecRoll => Convert.ToSingle(Math.Atan2(RotationMatrix[0,1], RotationMatrix[1,1]) * 180 / Math.PI);
    public float NoseVecAzi => Math.Atan2(RotationMatrix[2, 0], RotationMatrix[0,0]) * 180 / Math.PI < 0 ? 
        Convert.ToSingle(Math.Atan2(RotationMatrix[2, 0], RotationMatrix[0,0]) * 180 / Math.PI + 360) : 
        Convert.ToSingle(Math.Atan2(RotationMatrix[2, 0], RotationMatrix[0,0]) * 180 / Math.PI);
    public float VelVecPitch => Convert.ToSingle(Math.Atan2(InertialVelocity.y, 
        Math.Sqrt(InertialVelocity.x * InertialVelocity.x + InertialVelocity.z * InertialVelocity.z)) * 180 / Math.PI);
    public float VelVecAzi => Math.Atan2(InertialVelocity.x, InertialVelocity.z) * 180 / Math.PI < 0 ? 
        Convert.ToSingle(Math.Atan2(InertialVelocity.x, InertialVelocity.z) * 180 / Math.PI + 360) : 
        Convert.ToSingle(Math.Atan2(InertialVelocity.x, InertialVelocity.z) * 180 / Math.PI);

    
    /// <summary>
    /// Rotation Matrix from the Inertial to the Body Frame
    /// Transforms a vector from inertial to body frame using quaternion.
    /// </summary>
    /// <param name="quaternion">Quaternion [w, x, y, z]</param>
    private static double[,] RotationMatrixInertialToBody(Quaternion quaternion)
    {
        // Rotation matrix (inertial to body)
        double[,] rotation = new double[3, 3];
        // Row 1
        rotation[0, 0] = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
        rotation[0, 1] = 2 * (quaternion.x * quaternion.y - quaternion.w * quaternion.z);
        rotation[0, 2] = 2 * (quaternion.x * quaternion.z + quaternion.w * quaternion.y);

        // Row 2
        rotation[1, 0] = 2 * (quaternion.x * quaternion.y + quaternion.w * quaternion.z);
        rotation[1, 1] = 1 - 2 * (quaternion.x * quaternion.x + quaternion.z * quaternion.z);
        rotation[1, 2] = 2 * (quaternion.y * quaternion.z - quaternion.w * quaternion.x);

        // Row 3
        rotation[2, 0] = 2 * (quaternion.x * quaternion.z - quaternion.w * quaternion.y);
        rotation[2, 1] = 2 * (quaternion.y * quaternion.z + quaternion.w * quaternion.x); 
        rotation[2, 2] = 1 - 2 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y);

        return rotation;
    }
    
    /// <summary>
    /// Rotation Matrix from the Body to the Inertial Frame
    /// Transforms a vector from body to inertial frame using quaternion.
    /// </summary>
    /// <param name="quaternion">Quaternion [w, x, y, z]</param>
    private static double[,] RotationMatrixBodyToInertial(Quaternion quaternion)
    {
        // Rotation matrix (inertial to body)
        double[,] rotation = new double[3, 3];
        // Column 1
        rotation[0, 0] = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
        rotation[1, 0] = 2 * (quaternion.x * quaternion.y - quaternion.w * quaternion.z);
        rotation[2, 0] = 2 * (quaternion.x * quaternion.z + quaternion.w * quaternion.y);

        // Column 2
        rotation[0, 1] = 2 * (quaternion.x * quaternion.y + quaternion.w * quaternion.z);
        rotation[1, 1] = 1 - 2 * (quaternion.x * quaternion.x + quaternion.z * quaternion.z);
        rotation[2, 1] = 2 * (quaternion.y * quaternion.z - quaternion.w * quaternion.x);

        // Column 3
        rotation[0, 2] = 2 * (quaternion.x * quaternion.z - quaternion.w * quaternion.y);
        rotation[1, 2] = 2 * (quaternion.y * quaternion.z + quaternion.w * quaternion.x); 
        rotation[2, 2] = 1 - 2 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y);

        return rotation;
    }
    
    /// <summary>
    /// Transforms a vector from one frame to another.
    /// </summary>
    /// <param name="rotation">Rotation matrix</param>
    /// <param name="vectorIn">Rotation matrix</param>
    private static Vector3 MatrixMultiplication(double[,] rotation, Vector3 vectorIn)
    {
        // Matrix-vector multiplication
        Vector3 vectorOut = Vector3.zero;
        vectorOut.x = Convert.ToSingle(rotation[0, 0] * vectorIn.x + rotation[0, 1] * vectorIn.y + rotation[0, 2] * vectorIn.z);
        vectorOut.y = Convert.ToSingle(rotation[1, 0] * vectorIn.x + rotation[1, 1] * vectorIn.y + rotation[1, 2] * vectorIn.z);
        vectorOut.z = Convert.ToSingle(rotation[2, 0] * vectorIn.x + rotation[2, 1] * vectorIn.y + rotation[2, 2] * vectorIn.z);
        
        return vectorOut;
    }
}