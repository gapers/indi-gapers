#ifndef GAPERSSCOPE_H
#define GAPERSSCOPE_H
/*
   INDI Developers Manual
   Tutorial #2
   "Simple Telescope Driver"
   We develop a simple telescope simulator.
   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.
*/
#include <inditelescope.h>
class GapersScope : public INDI::Telescope
{
public:
    GapersScope();
protected:
    // General device functions
    bool Connect();
    bool Disconnect();
    const char *getDefaultName();
    bool initProperties();
    // Telescoe specific functions
    bool ReadScopeStatus();
    bool Goto(double,double);
    bool Abort();
private:
    double currentRA;
    double currentDEC;
    double targetRA;
    double targetDEC;
    unsigned int DBG_SCOPE;

    long CorrectRA( long, double &);
    double rangeDistance( double );

};
#endif // GAPERSSCOPE_H
