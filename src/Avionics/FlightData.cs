using System.Text.RegularExpressions;
using UnityGERunner;
using UnityGERunner.UnityApplication;

namespace whiskehSuperHeavy.Avionics;

public class FlightData
{
    public OutboundState State;
    public Vector3 InertialPosition => State.kinematics.position;
    public Quaternion Quaternions => Quaternion.Inverse(State.kinematics.rotation);
    public Vector3 InertialVelocity => State.kinematics.velocity;

    private double[,] RotationMatrix()
    {
        return VectorMath.RotationMatrixInertialToBody(Quaternions);
    }

    private double[,] RotationTranspose()
    {
        return VectorMath.RotationMatrixBodyToInertial(Quaternions);
    }

    public Vector3 BodyVelocity()
    {
        return VectorMath.MatrixMultiplication(RotationMatrix(), InertialVelocity);
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
        return -1 * Quaternion.Inverse(Quaternions).eulerAngles.x;
    }
    
    public float NoseVecRoll()
    {
        return Quaternion.Inverse(Quaternions).eulerAngles.z;
    }

    public float NoseVecAzi()
    {
        var azi = Quaternion.Inverse(Quaternions).eulerAngles.y;
        if (azi < 0)
        {
            return azi + 360;
        }
        return azi;
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
        var noseVec = new Vector3(Convert.ToSingle(Math.Cos(pitch * Math.PI / 180) * Math.Sin(azi * Math.PI / 180)),
            Convert.ToSingle(Math.Sin(pitch * Math.PI / 180)),
            Convert.ToSingle(Math.Cos(pitch * Math.PI / 180) * Math.Cos(azi * Math.PI / 180)));
        return noseVec.normalized;
    }
}