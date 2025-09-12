using System.Runtime.CompilerServices;
using UnityGERunner;  // Or AIPSim namespace
using UnityGERunner.UnityApplication;
using whiskehSuperHeavy.Avionics;

public class AIPProvider : IAIPProvider
{
    private FlightData _flightData = new FlightData(Vector3.zero, Quaternion.identity, Vector3.zero);
    
    public override SetupActions Start(SetupInfo info)
    {
        return new SetupActions
        {
            name = "whiskehXv0", 
            fuel = 2500,
            hardpoints = new string[]
                {"HPEquips/AFighter/fa26_gun"}
        };
    }
    
    public override InboundState Update(OutboundState state)
    {
        _flightData.Quaternions.Set(-1 * state.kinematics.rotation.x, -1 * state.kinematics.rotation.y, 
            -1 * state.kinematics.rotation.z, state.kinematics.rotation.w);
        
        _flightData.InertialVelocity.Set(state.kinematics.velocity.x, state.kinematics.velocity.y, 
            state.kinematics.velocity.z);
        
        _flightData.InertialPosition.Set(state.kinematics.position.x, state.kinematics.position.y, 
            state.kinematics.position.z);

        Graph("airspeed", _flightData.Airspeed);
        Graph("alpha", _flightData.Alpha);
        Graph("beta", _flightData.Beta);
        Graph("nosePitch", _flightData.NoseVecPitch);
        Graph("velPitch", _flightData.VelVecPitch);
        Graph("noseAzi", _flightData.NoseVecAzi);
        Graph("velAzi", _flightData.VelVecAzi);
        Graph("noseRoll", _flightData.NoseVecRoll);
        
        return new InboundState
        {
            pyr = new NetVector { x = -.5f, y = 0f, z = -0.25f },
            throttle = 1f,
            irLookDir = new NetVector { x = 0f, y = 0f, z = 1f }
        };
    }
}