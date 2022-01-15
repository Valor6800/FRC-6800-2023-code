[1mdiff --git a/Competition/src/main/cpp/subsystems/ValorSwerve.cpp b/Competition/src/main/cpp/subsystems/ValorSwerve.cpp[m
[1mindex 9e57f11..5194f4c 100644[m
[1m--- a/Competition/src/main/cpp/subsystems/ValorSwerve.cpp[m
[1m+++ b/Competition/src/main/cpp/subsystems/ValorSwerve.cpp[m
[36m@@ -142,8 +142,12 @@[m [mfrc::Rotation2d ValorSwerve::getAzimuthRotation2d()[m
 // The angle coming in is an optimized angle. No further calcs should be done on 'angle'[m
 void ValorSwerve::setAzimuthRotation2d(frc::Rotation2d angle)[m
 {[m
[32m+[m[32m    frc::Rotation2d currentAngle = getAzimuthRotation2d();[m[41m[m
[32m+[m[32m    frc::Rotation2d deltaAngle = angle - currentAngle;[m[41m[m
[32m+[m[41m[m
     double countsBefore = getAzimuthEncoderCount();[m
[31m-    double revolutionsWheel = angle.Radians().to<double>() / (2.0 * M_PI);[m
[32m+[m[32m    // @TODO if abs (revolutions) > .5, add or subtract 1 so we don't ever turn more than 180 degrees[m[41m[m
[32m+[m[32m    double revolutionsWheel = deltaAngle.Radians().to<double>() / (2.0 * M_PI);[m[41m[m
     double revolutionsFalon = revolutionsWheel / SwerveConstants::AZIMUTH_GEAR_RATIO;[m
     double countsFromAngle = revolutionsFalon * SwerveConstants::AZIMUTH_COUNTS_PER_REV;[m
     //double countsDelta = fmod(countsFromAngle - countsBefore, SwerveConstants::AZIMUTH_COUNTS_PER_REV / SwerveConstants::AZIMUTH_GEAR_RATIO);[m
