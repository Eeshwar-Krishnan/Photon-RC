package com.outoftheboxrobotics.photoncore;

import com.qualcomm.robotcore.util.RobotLog;

public class PhotonUnsupportedException extends RuntimeException {
    public PhotonUnsupportedException(String message){
        super(message);
    }
}
