class Odom{
    public:
        static double x;
        static double y;
        // rotation in degrees
        // 0<x<360
        static double rotation;
        static void setPosition(double x, double y);
        static void setRotation(double deg);
        static void addPosition(double x, double y);
        static void addRotation(double deg);
};