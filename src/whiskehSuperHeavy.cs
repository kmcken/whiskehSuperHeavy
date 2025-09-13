using System.Runtime.CompilerServices;
using UnityGERunner;  // Or AIPSim namespace
using UnityGERunner.UnityApplication;
using whiskehSuperHeavy.Avionics;
using whiskehSuperHeavy.HCDebug;

public class AIPProvider : IAIPProvider
{
    private FlightData _flightData = new FlightData();
    private AircraftVector _aircraftVector = new AircraftVector();
    private bool _debug = true;
    private int[] _prevID = new int[2];
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
        if (_debug)
        {
            RemoveDebugShape(_prevID[0]);
            RemoveDebugShape(_prevID[1]);
            var lines = _aircraftVector.PointingVectors(state);
            DebugShape(lines[0]);
            DebugShape(lines[1]);
            _prevID[0] = lines[0].id;
            _prevID[1] = lines[1].id;
        }

        var pitch = _flightData.NoseVecPitch();
        var roll = _flightData.NoseVecRoll();
        var azi = _flightData.NoseVecAzi();
        var angles = Quaternion.Inverse(_flightData.Quaternions).eulerAngles;
        
        Graph("pitch", pitch);
        Graph("roll", roll);
        Graph("azi", azi);
        Graph("angles", angles);
        
        return new InboundState
        {
            pyr = new NetVector { x = -.5f, y = 0f, z = -0f },
            throttle = 1f,
            irLookDir = new NetVector { x = 0f, y = 0f, z = 1f }
        };
    }
}