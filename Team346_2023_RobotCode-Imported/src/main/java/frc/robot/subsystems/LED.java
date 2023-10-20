package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.LED.Breathing;
import frc.robot.commands.LED.Fade;
import frc.robot.commands.LED.Flash;
import frc.robot.commands.LED.Rainbow;

public class LED extends SubsystemBase{
    private static Spark LED1;
    private static AddressableLED LED2;
    private static AddressableLEDBuffer LED2_Buffer;
    private Fade m_fade;
    private Flash m_flash;

    public LED() {
        LED1 = new Spark(Constants.LED_1_PWM_PORT);
        LED2 = new AddressableLED(Constants.LED_2_PWM_PORT);
        LED2_Buffer = new AddressableLEDBuffer(Constants.LED_2_LENGTH);

        
      m_flash = new Flash(this, Color.kAliceBlue, 25);

        m_fade = new Fade(this,
        new Color[] { Constants.ROBOHAWKS_BLUE, Constants.CUBE_PURPLE});
        m_fade.initialize();

    }

    // Changes LED to Green for a Double Substation

    public void red() {
        //LED1.set(0.61); //red
        setSolidColor(new Color(255, 0, 0));
    }   

    public void green() {
        //LED1.set(0.77); //Green
        setSolidColor(new Color(0, 255, 0));
    }

    public void light() {
        if (RobotContainer.grabber.getLaserBreakValue() || RobotContainer.intake.getLaserBreak()) {
            green();
        }
        else {
            red();
        }
    }

    public void setSolidColor(Color color) {
        for (int i = 0; i < LED2_Buffer.getLength(); i++) {
            LED2_Buffer.setLED(i, color);
        }
        //LED2.setData(LED2_Buffer);
    }

    public Command setSolidColorCommand(Color color) {
        return Commands.runOnce(() -> {
          setSolidColor(color);
        });
    }

    public Command allianceColorCommand() {
        Color color = DriverStation.getAlliance() == Alliance.Blue
            ? Color.kMediumBlue
            : Color.kRed;
    
        return setSolidColorCommand(color);
      }

      public void start() {
        LED2.start();
      }

      public void stop() {
        LED2.stop();
      }

      public Command rainbowCommand() {
        return new Rainbow(this);
      }
    
      public Command breathingCommand(Color color) {
        return new Breathing(this, color);
      }

      public void setColors(Color... colors) {
        int length = Math.min(LED2_Buffer.getLength(), colors.length);
    
        for (int i = 0; i < length; i++) {
          if (colors[i] != null) {
            LED2_Buffer.setLED(i, colors[i]);
          }
        }
        LED2.setData(LED2_Buffer);
      }

  //     @Override
  // public void periodic() {
  //   m_fade.execute();
  // }

  public Command setSolidColorNumberCommand(Color color, Color secondColor, int length) {
    return Commands.runOnce(() -> {
      int lengthLeds = Math.min(length, LED2_Buffer.getLength());
      for (int i = 0; i < lengthLeds; i++) {
        LED2_Buffer.setLED(i, color);
      }
      if (lengthLeds < LED2_Buffer.getLength()) {
        for (int i = lengthLeds; i < LED2_Buffer.getLength(); i++) {
            LED2_Buffer.setLED(i, secondColor);
        }
      }
      LED2.setData(LED2_Buffer);
    });
  }

  public void initFlash() {
    m_flash.schedule();
  }

  public Command startFlash() {
    return runOnce(() -> m_flash.start());
  }

  public void resetFlash() {
    m_flash.reset();
  }
}
