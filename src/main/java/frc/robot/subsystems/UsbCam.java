package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class UsbCam extends SubsystemBase implements Loggable{
    private static UsbCamera usbCameraServer;

    public UsbCam(){
        usbCameraServer = CameraServer.startAutomaticCapture();
        usbCameraServer.setExposureAuto();
        usbCameraServer.setWhiteBalanceAuto();
         
    }

    public void setResolution(int width,int height){
        usbCameraServer.setResolution(width, height);
    }

    public void setFPS(int fps){
        usbCameraServer.setFPS(fps);
    }

    @Log.CameraStream(name="USB Camera", tabName="Camera", showControls = true, showCrosshairs = true, rotation = "QUARTER_CW" )
    public UsbCamera getCameraFeed(){
        return usbCameraServer;
    }

    @Override
    public void periodic() {        
    }
    
    
}