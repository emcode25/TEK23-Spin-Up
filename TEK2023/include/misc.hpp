#ifndef MISC_HPP
#define MISC_HPP

struct Coordinate
{
    double x;
    double y;
};

struct PIDConstants
{
    double P;
    double I;
    double D;
};

enum TeamColor
{
    Red,
    Blue
};

class PIDController
{
    public:
        PIDController();
        void setPID(PIDConstants pidc);
        void setTimeout(int ms);
        void setTarget(double inches);
        void goToTarget(void(*action)(int mV), double(*getter)());

    private:
        double target;
        int timeout;
        PIDConstants pid;
};

#endif