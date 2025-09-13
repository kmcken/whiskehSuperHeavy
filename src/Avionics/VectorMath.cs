using System.Text.RegularExpressions;
using UnityGERunner;
using UnityGERunner.UnityApplication;

namespace whiskehSuperHeavy.Avionics;

public class VectorMath
{
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
    
    /// <summary>
    /// Rotation Matrix from the Body to the Inertial Frame
    /// Transforms a vector from body to inertial frame using quaternion.
    /// </summary>
    /// <param name="quaternion">Quaternion [w, x, y, z]</param>
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
    
    public static Vector3 VectorProjection(Vector3 position, Vector3 unitVector, float distance)
    {
        var finalPosition = Vector3.zero;
        finalPosition.x = position.x + distance * unitVector.x;
        finalPosition.y = position.y + distance * unitVector.y;
        finalPosition.z = position.z + distance * unitVector.z;
        return finalPosition;
    }
}