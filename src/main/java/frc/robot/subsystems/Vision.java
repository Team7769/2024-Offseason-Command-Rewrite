package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    //TODO: CHANGE TY VALUES TO ACTUAL MEASURED VALUES
    private static final double tyGoal = -18; 
    //when the ty value is near the bottom of the screen, note is close to pick up 
    //LL2 range: -24.85 to 24.85 degrees

    //TODO: is this neccessary?
    private static final double txGoal = 0;
    //for rotation, get the note as close to the middle as possible to pick up

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public Vision()
    {

    }

    public double getTargetAngle()
    {
        var tx = table.getEntry("tx").getDouble(0);
        SmartDashboard.putNumber("NoteFollowerGetAngle", tx); 

        return tx;
    }

    public void moveToNote()
    //BE PREPARED TO STOP THIS WHEN RUNNING
    {

    }

    @Override
    public void periodic()
    {
        getTargetAngle();
    }
}
