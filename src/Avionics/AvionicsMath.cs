using System.Text.RegularExpressions;
using UnityGERunner;

namespace whiskehSuperHeavy.Avionics;

public static class AvionicsMath
{
    /// <summary>
    /// Calculates the velocity vector in the body frame via frame rotation using the quaternions.
    /// </summary>
    /// <param name="quaternion">Quaternion [w, x, y, z]</param>
    /// <param name="vInertial">Vector3 [vx, vy, vz]</param>
    /// <returns>airspeed in knots or m/s</returns>
    public static Vector3 BodyVelocity(Quaternion quaternion, Vector3 vInertial)
    {
        var rotation = RotationMatrixInertialToBody(quaternion);
        return MatrixMultiplication(rotation, vInertial);
    }

    /// <summary>
    /// Calculates the airspeed
    /// </summary>
    /// <param name="vBody">Vector3 [vx, vy, vz]</param>
    /// <param name="knots">bool ? knots : m/s </param>
    /// <returns>airspeed in knots or m/s</returns>
    public static float Airspeed(Vector3 vBody, bool knots = true)
    {
        return knots ? Convert.ToSingle(vBody.magnitude * 1.94384) :  Convert.ToSingle(vBody.magnitude);
    }
    
    /// <summary>
    /// Computes angle of attack (Alpha) and sideslip angle (Beta) from quaternion and intertial velocity.
    /// </summary>
    /// <param name="quaternion">Quaternion [w, x, y, z]</param>
    /// <param name="vInertial">Inertial velocity [vx, vy, vz]</param>
    /// <returns>Alpha and Beta in degrees; Vector2 [alpha, beta]</returns>
    public static Vector2 AlphaBeta(Quaternion quaternion, Vector3 vInertial)
    {
        Vector3 velocityBody = BodyVelocity(quaternion, vInertial);
        var alpha = -1 * Math.Atan2(velocityBody.y, velocityBody.z) * (180 / Math.PI) + 1;
        
        double beta;
        if (velocityBody.magnitude == 0 || velocityBody.y == 0) {beta = 0;} 
        else {beta = Math.Asin(velocityBody.x / velocityBody.magnitude) * (180 / Math.PI);}
        
        return new Vector2(Convert.ToSingle(alpha), Convert.ToSingle(beta));
    }
    
    /// <summary>
    /// Calculates Roll, Pitch, and Azimuth of the aircraft nose vector in degrees for a given quaternion.
    /// </summary>
    /// <param name="quaternion">Quaternion [w, x, y, z]</param>
    /// <returns>Pitch and Azimuth; Vector2 [theta, phi]</returns>
    public static Vector3 NoseVectorPitchAziRoll(Quaternion quaternion)
    {
        var rotation = RotationMatrixInertialToBody(quaternion);
        
        var pitch = Math.Asin(rotation[2,1]) * 180 / Math.PI;
        var roll = Math.Atan2(rotation[0,1], rotation[1,1]) * 180 / Math.PI;
        var azimuth = Math.Atan2(rotation[2, 0], rotation[0,0]) * 180 / Math.PI;

        if (azimuth < 0)
        {
            azimuth = 360 + azimuth;
        }

        return new Vector3(Convert.ToSingle(pitch), Convert.ToSingle(azimuth), Convert.ToSingle(roll));
    }
    
    /// <summary>
    /// Pitch Angle (degrees) and Azimuth (degrees) from the inertial velocity vector.
    /// </summary>
    /// <param name="inertialVel">Vector3 [vel.x, vel.y, vel.z]</param>
    /// <returns>Pitch and Azimuth; Vector2 [theta, phi]</returns>
    public static Vector2 VelocityVectorPitchAzi(Vector3 inertialVel)
    {
        var pitch = Math.Atan2(inertialVel.y, 
            Math.Sqrt(inertialVel.x * inertialVel.x + inertialVel.z * inertialVel.z)) * 180 / Math.PI;
        var azimuth = Math.Atan2(inertialVel.x, inertialVel.z) * 180 / Math.PI;

        if (azimuth < 0)
        {
            azimuth = 360 + azimuth;
        }

        return new Vector2(Convert.ToSingle(pitch), Convert.ToSingle(azimuth));
    }

    /// <summary>
    /// Calculates the velocity from the body frame by transforming the inertial frame velocity with the quaternion
    /// rotational matrix.
    /// </summary>
    /// <param name="quaternion">Quaternion [w, x, y, z]</param>
    /// <param name="vInertial">Inertial velocity [vx, vy, vz]</param>
    /// <param name="inverse">inverse transpose if true</param>
    /// <returns>Body velocity [u, v, w]</returns>
    public static Vector3 FrameTransform(Quaternion quaternion, Vector3 vInertial, bool inverse = false)
    {
        var rotation = inverse ? RotationMatrixBodyToInertial(quaternion) : RotationMatrixInertialToBody(quaternion);
        
        Vector3 vBody = MatrixMultiplication(rotation, vInertial);
        return vBody;
    }
    
    /// <summary>
    /// Rotation Matrix from the Inertial to the Body Frame
    /// Transforms a vector from inertial to body frame using quaternion.
    /// </summary>
    /// <param name="quaternion">Quaternion [w, x, y, z]</param>
    public static double[,] RotationMatrixInertialToBody(Quaternion quaternion)
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
    
    public static double[,] RotationMatrixBodyToInertial(Quaternion quaternion)
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
    public static Vector3 MatrixMultiplication(double[,] rotation, Vector3 vectorIn)
    {
        // Matrix-vector multiplication
        Vector3 vectorOut = Vector3.zero;
        vectorOut.x = Convert.ToSingle(rotation[0, 0] * vectorIn.x + rotation[0, 1] * vectorIn.y + rotation[0, 2] * vectorIn.z);
        vectorOut.y = Convert.ToSingle(rotation[1, 0] * vectorIn.x + rotation[1, 1] * vectorIn.y + rotation[1, 2] * vectorIn.z);
        vectorOut.z = Convert.ToSingle(rotation[2, 0] * vectorIn.x + rotation[2, 1] * vectorIn.y + rotation[2, 2] * vectorIn.z);
        
        return vectorOut;
    }
}