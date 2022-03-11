package frc.robot.subsystems.DMS;

public class DMSStats {
    private static final double VEL_THRESHOLD = 0.85;
    private static final double AMP_THRESHOLD = 0.8;

    public DriveInfo<Double> current = new DriveInfo<>(0.0);
    public DriveInfo<Double> velocity = new DriveInfo<>(0.0);


    public void addDriveCurrent(DriveInfo<Double> driveOutputCurrent) {
        current.FL = (current.FL + Math.abs(driveOutputCurrent.FL)) / 2.0;
        current.FR = (current.FR + Math.abs(driveOutputCurrent.FR)) / 2.0;
        current.RL = (current.RL + Math.abs(driveOutputCurrent.RL)) / 2.0;
        current.RR = (current.RR + Math.abs(driveOutputCurrent.RR)) / 2.0;
    }

    public void addDriveVelocity(DriveInfo<Double> driveVelocity) {
        velocity.FL = (velocity.FL + Math.abs(driveVelocity.FL)) / 2.0;
        velocity.FR = (velocity.FR + Math.abs(driveVelocity.FR)) / 2.0;
        velocity.RL = (velocity.RL + Math.abs(driveVelocity.RL)) / 2.0;
        velocity.RR = (velocity.RR + Math.abs(driveVelocity.RR)) / 2.0;
    }

    public static void print(String label, DriveInfo<?> info) {
        System.out.println(label + 
            " FR: " + info.FR +
            " FL: " + info.FL + 
            " RL: " + info.RL + 
            " RR: " + info.RR);
    }

    public static double average(DriveInfo<Double> info) {
        return (info.FL + info.FR + info.RL + info.RR) / 4.0;
    }

    public DriveInfo<Integer> calculateStatus() {
        DriveInfo<Integer> status = new DriveInfo<>(0);

        double velAvg = DMSStats.average(velocity);
        double ampAvg = DMSStats.average(current);
        System.out.println("Vel Avg: " + velAvg + " | Amp Avg: " + ampAvg);
        status.FL = calc(velocity.FL, velAvg, current.FL, ampAvg);
        status.FR = calc(velocity.FR, velAvg, current.FR, ampAvg);
        status.RL = calc(velocity.RL, velAvg, current.RL, ampAvg);
        status.RR = calc(velocity.RR, velAvg, current.RR, ampAvg);
        return status;
    }

    private int calc(double vel, double velAvg, double amp, double ampAvg) {
        if (amp == 0) {
            return 4;
        }
        if (vel == 0) {
            return 5;
        }
        boolean velOutside = (vel / velAvg) < VEL_THRESHOLD;
        boolean ampOutside = (amp / ampAvg) < AMP_THRESHOLD;

        if (!velOutside && !ampOutside) {
            return 1; // good
        } else if (!velOutside && ampOutside) {
            return 2; // amp outside
        } else if (velOutside && !ampOutside) {
            return 3;
        } else {
            return 2;   // both out of range
        }
    }

}
