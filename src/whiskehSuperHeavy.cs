using System.Runtime.CompilerServices;
using UnityGERunner;  // Or AIPSim namespace
using UnityGERunner.UnityApplication;
using whiskehSuperHeavy.Avionics;

public class AIPProvider : IAIPProvider
{
    private FlightData _flightData = new FlightData();
    
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
        _flightData.State = state;

        // Graph("airspeed", _flightData.Airspeed());
        // Graph("alpha", _flightData.Alpha());
        // Graph("beta", _flightData.Beta());
        // Graph("nosePitch", _flightData.NoseVecPitch());
        // Graph("velPitch", _flightData.VelVecPitch());
        // Graph("noseAzi", _flightData.NoseVecAzi());
        // Graph("velAzi", _flightData.VelVecAzi());
        // Graph("noseRoll", _flightData.NoseVecRoll());
        Graph("noseVec", _flightData.NoseVecNorm());
        Graph("velVec", _flightData.VelVecNorm());
        
        return new InboundState
        {
            pyr = new NetVector { x = -.5f, y = 0f, z = -0.25f },
            throttle = 1f,
            irLookDir = new NetVector { x = 0f, y = 0f, z = 1f }
        };
    }
}