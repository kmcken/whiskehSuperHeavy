using UnityGERunner;

namespace whiskehSuperHeavy.Avionics;

public static class AngleOfAttack
{
    /// <summary>
    /// Computes angle of attack from quaternion and intertial velocity.
    /// </summary>
    /// <param name="quaternion">Quaternion [w, x, y, z]</param>
    /// <param name="vInertial">Inertial velocity [vx, vy, vz]</param>
    public static double AOAangle(Quaternion quaternion, Vector3 vInertial)
    {
        Vector3 velocityBody = AvionicsMath.BodyVelocity(quaternion, vInertial);
        return -1 * Math.Atan2(velocityBody.y, velocityBody.z) * (180 / Math.PI) + 1; // To degrees
    }
        
    /// <summary>
    /// Computes sideslip angle from body velocity [u, v, w].
    /// </summary>
    /// <param name="quaternion">Quaternion [w, x, y, z]</param>
    /// <param name="vInertial">Inertial velocity [vx, vy, vz]</param>
    public static double SideSlipAngle(Quaternion quaternion, Vector3 vInertial)
    {
        Vector3 velocityBody = AvionicsMath.BodyVelocity(quaternion, vInertial);
        if (velocityBody.magnitude == 0 || velocityBody.y == 0) return 0; // Avoid division by zero
        return Math.Asin(velocityBody.x / velocityBody.magnitude) * (180 / Math.PI); // To degrees
    }
}