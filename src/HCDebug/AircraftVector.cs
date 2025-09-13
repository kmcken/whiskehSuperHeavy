using Recorder;
using UnityGERunner;
using UnityGERunner.UnityApplication;
using whiskehSuperHeavy.Avionics;
using Random = System.Random;

namespace whiskehSuperHeavy.HCDebug;

public class AircraftVector
{
    private Random _random = new Random();
    
    public DebugLine[] PointingVectors(OutboundState state, float distance = 1000F)
    {
        // TODO: autogenerate unique id
        int id1 = _random.Next();
        int id2 = _random.Next();
        var velLine = new DebugLine(id1);
        var noseLine = new DebugLine(id2);

        
        var data = new FlightData { State = state};
        var finalVel = VectorMath.VectorProjection(data.InertialPosition, data.VelVecNorm(), distance);
        var finalNose = VectorMath.VectorProjection(data.InertialPosition, data.NoseVecNorm(), distance);
        var colorVel = new NetColor { r = 0f, g = 0f, b = 0.586f, a = 1f};
        var colorNose = new NetColor { r = 0.289f, g = 1f, b = 0.223f, a = 1f};
        
        velLine.start = data.InertialPosition;
        velLine.end = finalVel;
        velLine.color = colorVel;
        
        noseLine.start = data.InertialPosition;
        noseLine.end = finalNose;
        noseLine.color = colorNose;
        
        return new[] {velLine, noseLine};
        }
}