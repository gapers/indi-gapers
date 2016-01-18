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
    bool Sync(double,double);
    bool Abort();
private:
    double currentRA;
    double currentDEC;
    double targetRA;
    double targetDEC;
    unsigned int DBG_SCOPE;

    // Properties representing encoder steps and stepper axle rounds used in
    // axis movement. RA and DEC share the same structure for tidyness
    typedef struct AxisMovementParameters {
      double angle;
      long steps;
      long startQuote;
      long endQuote;
      long rounds;
    };
    AxisMovementParameters raMovement, decMovement;

    long CorrectRA( long, double &);
    double rangeDistance( double );
    long ComputeStepsRA( double );
    bool ComputeLongMove( );
    bool _roundCalc(long steps, long &m_sq, long &m_eq, long &m_giri);
};
#endif // GAPERSSCOPE_H
