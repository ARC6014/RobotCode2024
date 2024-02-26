package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class UsbCam extends SubsystemBase {

    private UsbCamera usbCamera;

    public UsbCam() {
        CameraServer.startAutomaticCapture();
        usbCamera = CameraServer.startAutomaticCapture(0);
        SmartDashboard.putBoolean("İs Enabled", usbCamera.isConnected());
        // setFPS(60);
        // setResolution(640, 252);
        // usbCameraServer.setExposureAuto();
        // usbCameraServer.setWhiteBalanceAuto();
        // usbCameraServer.setBrightness(10);
    }

    public void setResolution(int width, int height) {
        usbCamera.setResolution(width, height);
    }

    public void setFPS(int fps) {
        usbCamera.setFPS(fps);
    } // //
      // "QUARTER_CW"

    public UsbCamera getCameraFeed() {
        return usbCamera;
    }

    @Override
    public void periodic() {
    }

}