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
    bool initProperties();

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
    struct AxisMovementParameters {
      double angle;
      long steps;       // motor steps
      long startQuote;  // encoder starting quote
      long endQuote;    // encoder ending quote
      long rotations;   // motor spindle spins for movements > 20 degrees
      double time;      // time necessary for axis movement
    };
    AxisMovementParameters raMovement, decMovement;
    time_t movementStart;

    double rangeDistance( double );
    bool _setMoveDataRA( double );
    bool _setMoveDataDEC( double );
    bool _rotationsCalc(long steps, long &m_sq, long &m_eq, long &m_giri);
};
#endif // GAPERSSCOPE_H
