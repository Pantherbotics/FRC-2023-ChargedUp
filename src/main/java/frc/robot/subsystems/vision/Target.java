package frc.robot.subsystems.vision;

public class Target {
    private double pitch, yaw, area;
    
    public Target() { 
        pitch = 0;
        yaw = 0;
        area = 0;
    }

    public double getPitch() { 
        return pitch;
    }

    public void setPitch(double pitch) { 
        this.pitch = pitch;
    }

    public double getYaw() { 
        return yaw;
    }

    public void setYaw(double yaw) { 
        this.yaw = yaw;
    }

    public double getArea() { 
        return area;
    }

    public void setArea(double area) { 
        this.area = area;
    }
}
