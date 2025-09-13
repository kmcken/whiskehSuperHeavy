using System.Text.RegularExpressions;
using UnityGERunner;
using UnityGERunner.UnityApplication;

namespace whiskehSuperHeavy.Avionics;

public class FlightData
{
    public FlightData() { }
    
    public OutboundState State;
    private Vector3 InertialPosition => State.kinematics.position;
    private Quaternion Quaternions => Quaternion.Inverse(State.kinematics.rotation);
    private Vector3 InertialVelocity => State.kinematics.velocity;

    private double[,] RotationMatrix()
    {
        return RotationMatrixInertialToBody(Quaternions);
    }

    private double[,] RotationTranspose()
    {
        return RotationMatrixBodyToInertial(Quaternions);
    }

    public Vector3 BodyVelocity()
    {
        return MatrixMultiplication(RotationMatrix(), InertialVelocity);
    }
    
    public float Airspeed()
    {
        return Convert.ToSingle(InertialVelocity.magnitude * 1.94384);
    }
    
    public float Alpha()
    {
        var bodyVel = BodyVelocity();
        return Convert.ToSingle(-1 * Math.Atan2(bodyVel.y, bodyVel.z) * (180 / Math.PI) + 1);
    }

    public float Beta()
    {
        var bodyVel = BodyVelocity();
        return bodyVel.magnitude == 0 ? 0 : Convert.ToSingle(Math.Asin(bodyVel.x / bodyVel.magnitude) * (180 / Math.PI));
    }

    public float AltitudeMeanSeaLevel()
    {
        return Convert.ToSingle(InertialPosition.y * 3.28084);
    }
    
    public float NoseVecPitch()
    {
        return Convert.ToSingle(Math.Asin(RotationMatrix()[2, 1]) * 180 / Math.PI);
    }
    
    public float NoseVecRoll()
    {
        var rotation = RotationMatrixInertialToBody(Quaternions);
        return Convert.ToSingle(Math.Atan2(rotation[0, 1], rotation[1, 1]) * 180 / Math.PI);
    }

    public float NoseVecAzi()
    {
        var rotation = RotationMatrixInertialToBody(Quaternions);
        return Math.Atan2(rotation[2, 0], rotation[0,0]) * 180 / Math.PI < 0 ? 
            Convert.ToSingle(Math.Atan2(rotation[2, 0], rotation[0,0]) * 180 / Math.PI + 360) : 
            Convert.ToSingle(Math.Atan2(rotation[2, 0], rotation[0,0]) * 180 / Math.PI);
    }

    public float VelVecPitch()
    {
        return Convert.ToSingle(Math.Atan2(InertialVelocity.y, 
            Math.Sqrt(InertialVelocity.x * InertialVelocity.x + InertialVelocity.z * InertialVelocity.z)) * 180 / Math.PI);
    }
    public float VelVecAzi()
    {
        return Math.Atan2(InertialVelocity.x, InertialVelocity.z) * 180 / Math.PI < 0
            ? Convert.ToSingle(Math.Atan2(InertialVelocity.x, InertialVelocity.z) * 180 / Math.PI + 360)
            : Convert.ToSingle(Math.Atan2(InertialVelocity.x, InertialVelocity.z) * 180 / Math.PI);
    }
    public Vector3 VelVecNorm()
    {
        return InertialVelocity.normalized;
    }
    public Vector3 NoseVecNorm()
    {
        var pitch = NoseVecPitch();
        var azi = NoseVecAzi();
        return new Vector3(Convert.ToSingle(Math.Cos(pitch) * Math.PI / 180 * Math.Cos(azi * Math.PI / 180)), 
            Convert.ToSingle(Math.Sin(pitch * Math.PI / 180)),
            Convert.ToSingle(Math.Sin(pitch * Math.PI / 180) * Math.Cos(azi * Math.PI / 180)));
    }
    
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