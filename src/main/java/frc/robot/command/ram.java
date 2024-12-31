package frc.robot.command;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVision;

public class ram extends Command {
    DoubleSupplier y;
    DoubleSupplier x;
    DoubleSupplier rotate;
    PhotonVision camera; 
    Transform3d d;
    SwerveRequest.ApplyChassisSpeeds cheese = new SwerveRequest.ApplyChassisSpeeds(); 
    
    double desiredAngle;
    double desiredDistance; 

    CommandSwerveDrivetrain ddd;
    

    public ram(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation, PhotonVision c, CommandSwerveDrivetrain d) {
        this.x = x;
        this.y = y;
        this.rotate = rotation;
        camera = c; 

        this.desiredAngle = 0;
        this.desiredAngle = 0.5;
    }

    @Override
    public void execute() {
        var strafe = rotate.getAsDouble();
        d = camera.getPosition();
        ddd.applyRequest(() -> cheese.withSpeeds(new ChassisSpeeds(camera.getDistance(), y.getAsDouble(), desiredAngle - camera.getYaw()) )
);
    }
}
