package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UsbCam extends SubsystemBase {

    private UsbCamera usbCamera;

    public UsbCam() {
        usbCamera = CameraServer.startAutomaticCapture(1);
    }

    public void setResolution(int width, int height) {
        usbCamera.setResolution(width, height);
    }

    public void setFPS(int fps) {
        usbCamera.setFPS(fps);
    } 
    public UsbCamera getCameraFeed() {
        return usbCamera;
    }

    @Override
    public void periodic() {
    }

}