
// package frc.diagnostics;

// import edu.wpi.first.cscore.UsbCamera;


// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


// public class Camera extends ShuffleManager{


//     //private ComplexWidget cam;
    
//     //private SendableCameraWrapper camera;
//     UsbCamera camera;
//     //private GenericEntry entry;


//     /**
//      *
//      * @param title
//      * @param defaultValue
//      * @param min
//      * @param max
//      */
//     public Camera(String title, UsbCamera cam){
//         camera = cam;
//         camera.setResolution(640,480);
//         camera.setFPS(60);
       
//         Shuffleboard.getTab("Camera_Live")
//         .add(camera)
//         .withPosition(pos.x, pos.y)
//         .withSize(10, 10);
        
//         //entry = widget.getEntry();
//         pos.translate(1, 0);
//         if(pos.x >= 7) pos.setLocation(1, pos.y + 2);
//     }


// }


