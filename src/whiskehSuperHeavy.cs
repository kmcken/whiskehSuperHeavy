using UnityGERunner;  // Or AIPSim namespace
using UnityGERunner.UnityApplication;

public class AIPProvider : IAIPProvider {
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
        return new InboundState
        {
            pyr = new NetVector { x = 0, y = 0, z = 0 },
            throttle = 100,
            irLookDir = new NetVector { x = 0f, y = 0f, z = 1f }
        };
    }
}