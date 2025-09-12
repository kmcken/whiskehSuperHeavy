using System.Runtime.CompilerServices;
using UnityGERunner;  // Or AIPSim namespace
using UnityGERunner.UnityApplication;
using whiskehSuperHeavy.Avionics;

public class AIPProvider : IAIPProvider
{
    private static Quaternion _refQuaternion = Quaternion.identity;
    private Quaternion _quaternion = Quaternion.identity;
    private Vector3 _inertialVelocity = Vector3.one;
    private Vector3 _bodyVelocity = Vector3.zero;
    private Vector3 _velocityVectorPitchAzi = Vector2.zero;
    private Vector3 _noseVectorPitchAziRoll = Vector3.zero;
    private Vector3 _alphaBeta = Vector2.zero;
    
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
        _quaternion.Set(state.kinematics.rotation.x, state.kinematics.rotation.y, 
            state.kinematics.rotation.z, state.kinematics.rotation.w);
        _quaternion = Quaternion.Inverse(_quaternion);
        
        _inertialVelocity.Set(state.kinematics.velocity.x, state.kinematics.velocity.y, 
            state.kinematics.velocity.z);
        
        _bodyVelocity = AvionicsMath.BodyVelocity(_quaternion, _inertialVelocity);
        
        _noseVectorPitchAziRoll = AvionicsMath.NoseVectorPitchAziRoll(_quaternion);
        
        _velocityVectorPitchAzi = AvionicsMath.VelocityVectorPitchAzi(_inertialVelocity);

        _alphaBeta = AvionicsMath.AlphaBeta(_quaternion, _inertialVelocity);
        
        // Graph("airspeed", Convert.ToSingle(airspeed));
        Graph("alphabeta", _alphaBeta);
        Graph("bodyVelocity", _bodyVelocity);
        Graph("noseVec PitchAziRoll", _noseVectorPitchAziRoll);
        Graph("velVec PitchAzi", _velocityVectorPitchAzi);
        
        return new InboundState
        {
            pyr = new NetVector { x = -.5f, y = 0f, z = -0.25f },
            throttle = 1f,
            irLookDir = new NetVector { x = 0f, y = 0f, z = 1f }
        };
    }
}