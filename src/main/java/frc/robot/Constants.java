package frc.robot;

public class Constants {
    public class oi {
        public static int kDriverId = 0;
        public static int kOperId = 1;
        public static double kDeadband = 0.1; //10% deadband for joysticks
    }
    public class drivetrain {
        public static double kMaxSpeed = 3.0; // 3 meters per second
        public static double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
        public class frontright {
            public static int kMotorId = 4;
            public static boolean kInverted = true;
        }
        public class backright {
            public static int kMotorId = 3;
            public static boolean kInverted = true;
        }
        public class frontleft {
            public static int kMotorId = 1;
            public static boolean kInverted = false;
        }
        public class backleft {
            public static int kMotorId = 2;
            public static boolean kInverted = false;
        }
    }
    public class climber {
        public static int kMotorId = 5;
        public static boolean kInverted = false;
        public static double kSpeed = 0.25;
    }
    public class algae {
        public class flipper {
            public static double kSpeed = 0.5; //motor power for moving flipper
            public static double kForwardPosition = 0.25;
            public static double kBackwardPosition = 0.0;
            public class left {
                public static int kMotorId = 6;
                public static boolean kInverted = true;
            }
            public class right {
                public static int kMotorId = 10;
                public static boolean kInverted = false;
            }
        }
        public class intake {
            public static double kInSpeed = 0.5; //motor power for running intake rollers
            public static double kOutSpeed = 0.5;
            public class bottom {
                public static int kMotorId = 7;
                public static boolean kInverted = false;
            }
            public class top {
                public static int kMotorId = 8;
                public static boolean kInverted = false;
            }
        }
    }
    public class coral {

    }
}
