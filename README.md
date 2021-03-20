This is an Arduino code for sensors such as MPU 6050, HC-SR04 (Ultrasonic Sensor),ESCs,motors,receiver.
"Servo.h" library is used for receiver,motors and ESCs.
Included libraries I2Cdev.h and MPU6050_6Axis_MotionApps20.h for MPU 6050.
For MPU 6050,the Digital Motion Processor is first initialised and then First In First Out Count is taken.The values of yaw,pitch and roll in Euler angle are obtained using the function ‘mpu.dmpGetYawPitchRoll’.
Obstacle avoidance algorithm is used for the Ultrasonic Sensor.In order to avoid collisions the motors are made to rotate clockwise or counterclockwise depending on the distance of the glass from the ultrasonic sensor.
