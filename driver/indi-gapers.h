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
#include <queue>
#include <string>
class GapersScope : public INDI::Telescope
{
public:
  GapersScope();
  void ISGetProperties(const char*);
  virtual bool ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n);

protected:

  // All telescopes should produce equatorial co-ordinates
  INumberVectorProperty Eq2kNP;
  INumber Eq2kN[2];

  // Alt Az coordinates
  INumberVectorProperty AaNP;
  INumber AaN[2];

  ISwitchVectorProperty domesyncSP;
  ISwitch domesyncS[2];

  // General device functions
  bool Connect();
  bool Connect(const char *port, uint16_t baud);
  bool Disconnect();
  const char *getDefaultName();
  void NewRaDec(double ra,double dec);
  virtual bool initProperties();
  virtual bool updateProperties();
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
  bool raIsMoving;
  bool decIsMoving;
  unsigned int DBG_SCOPE;
  IPState lastEq2kState;

  // Serial handling methods and properties
  int tty_connect(const char *device, int bit_rate, int word_size, int parity, int stop_bits, int *fd);
  void commHandler();
  void ParsePLCMessage(const std::string msg);
  void SendMove(int _system, long steps, long m_sq, long m_eq, long m_giri);
  void FinalizeMove();
  void SendCommand( char syst, short int cmd, long val );

  std::queue<std::string> _writequeue;
  std::string _readbuffer = "";
  enum e_cstate {
    STARTWAITING    = 1,
    READINGCOMMAND  = 2
  };
  e_cstate c_state = STARTWAITING;

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

// ascii definitions
#define ASCII_STX               0x02            // Start of text
#define ASCII_ETX               0x03            // End of text

#define ASCII_BEL               0x07
#define ASCII_BS                0x08
#define ASCII_LF                0x0A
#define ASCII_CR                0x0D
#define ASCII_XON               0x11
#define ASCII_XOFF              0x13

#endif // GAPERSSCOPE_H
