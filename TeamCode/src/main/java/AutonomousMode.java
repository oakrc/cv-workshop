import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "CV Workshop Auto", group = "Auto")
public class AutonomousMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ZoneChooser chooser = new ZoneChooser(hardwareMap, telemetry);
        waitForStart();
        Target target = chooser.getTarget();
        chooser.stop();
        switch(target) {
            case A:
                break;
            case B:
                break;
            case C:
                break;
        }
    }
}
