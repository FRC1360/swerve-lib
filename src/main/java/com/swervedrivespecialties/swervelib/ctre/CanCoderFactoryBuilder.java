package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoderFactory;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

import edu.wpi.first.wpilibj.DriverStation;

public class CanCoderFactoryBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;
    private int periodMilliseconds = 10;

    public CanCoderFactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }

    public CanCoderFactoryBuilder withDirection(Direction direction) {
        this.direction = direction;
        return this;
    }

    public AbsoluteEncoderFactory<CanCoderAbsoluteConfiguration> build() {
        return configuration -> {
            //CANCoderConfiguration config = new CANCoderConfiguration();
            /*config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            config.magnetOffsetDegrees = Math.toDegrees(configuration.getOffset());
            config.sensorDirection = direction == Direction.CLOCKWISE;

            CANCoder encoder = new CANCoder(configuration.getId());
            CtreUtils.checkCtreError(encoder.configAllSettings(config, 250), "Failed to configure CANCoder");

            CtreUtils.checkCtreError(encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, periodMilliseconds, 250), "Failed to configure CANCoder update rate");

            return new EncoderImplementation(encoder);*/
            return new OrbitEncoderImplementation(configuration.getId());
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final CANCoder encoder;

        private EncoderImplementation(CANCoder encoder) {
            this.encoder = encoder;
        }

        @Override
        public double getAbsoluteAngle() {
            double angle = Math.toRadians(encoder.getAbsolutePosition());
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }
    }

    // This is for Encoder over PWM
    private static class OrbitEncoderImplementation implements AbsoluteEncoder {

        private final DigitalInput dio;
        private final DutyCycle encoder;

        private OrbitEncoderImplementation(int port) {
            String message = "Hello from OrbitEncoderImplementation!";
            DriverStation.reportWarning(String.format("%s", message), false);
            this.dio = new DigitalInput(port);
            this.encoder = new DutyCycle(this.dio);
        }

        private int getAbsolutePosition() {
            int position = (int) Math.round(encoder.getOutput());

            if(position < 0) {
                position = 0;
            } else if (position > 4095) {
                position = 4095;
            }

            position -= 2048;

            if(position > 2047) {
                position -=4096;
            } else if(position < -2048) {
                position += 4096;
            }

            return position;
        }

        @Override
        public double getAbsoluteAngle() {
            //int position = getAbsolutePosition();
            //double angle = (360.0 / 4096.0) * position + 180.0;
            //return Math.toRadians(angle);

            double angle = Math.toRadians(360.0 * encoder.getOutput());

            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }
        
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}
