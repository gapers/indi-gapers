/*
GAPers Telescope driver

Copyright (C) 2024 Massimiliano Masserelli
Copyright (C) 2024 Gruppo Astrofili Persicetani
Copyright (C) 2014 Maurizio Serrazanetti
*/

#ifndef GAPERSSCOPE_H
#define GAPERSSCOPE_H

#pragma once

#include <string>
#include "indicom.h"

#include <stdint.h>


#include <inditelescope.h>
#include <queue>
#include <string>

class GapersScope : public INDI::Telescope
{
public:
  GapersScope();
  virtual ~GapersScope() = default;

  virtual const char *getDefaultName() override;

  void ISGetProperties(const char*);
  virtual bool ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n);
  virtual bool ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n);

  enum DomeStatus { DOME_IDLE, DOME_SLEWING };

protected:
  // All telescopes should produce equatorial co-ordinates
  INumberVectorProperty Eq2kNP;
  INumber Eq2kN[2];

  // Alt Az coordinates
  INumberVectorProperty AaNP;
  INumber AaN[2];

  // Dome properties

  ISwitchVectorProperty domesyncSP;
  ISwitch domesyncS[2];

  INumberVectorProperty domeAzNP;
  INumber domeAzN[1];

  ISwitchVectorProperty domeCoordSP;
  ISwitch domeCoordS[2];

  INumberVectorProperty domeSpeedNP;
  INumber domeSpeedN[1];

  INumberVectorProperty domeAzThresholdNP;
  INumber domeAzThresholdN[1];

  // General device functions
  void NewRaDec(double ra,double dec);
  void NewAltAz(double alt, double az);
  virtual bool initProperties() override;
  virtual bool updateProperties();
  virtual bool saveConfigItems(FILE *fp);
  // Telescope specific functions
  bool ReadScopeStatus();
  bool Goto(double,double);
  bool DomeGoto(double);
  void DomeManualEnable(bool);
  bool Sync(double,double);
  bool DomeSync(double);
  bool Abort();
  void TimerHit();

private:
  double currentRA;
  double currentDEC;
  double targetRA;
  double targetDEC;
  bool raIsMoving;
  bool decIsMoving;
  unsigned int DBG_SCOPE;
  IPState lastEq2kState;
  double domeCurrentAZ;
  double domeTargetAZ;
  DomeStatus DomeTrackState;
  double domeSpeed;
  double domeAzThreshold;
  time_t domeMovementStart;
  time_t domeMovementEnd;

  // Serial handling methods and properties
  int tty_connect(const char *device, int bit_rate, int word_size, int parity, int stop_bits, int *fd);
  void commHandler();
  void ParsePLCMessage(const std::string msg);
  void SendMove(int _system, long steps, long m_sq, long m_eq, long m_giri);
  void FinalizeMove();
  void SendCommand( char syst, short int cmd, long val );
  bool Handshake();
  int PortFD{-1};

  Connection::Serial *serialConnection{nullptr};


  std::queue<std::string> _writequeue;
  std::string _readbuffer = "";
  enum e_cstate {
    STARTWAITING    = 1,
    READINGCOMMAND  = 2
  };
  e_cstate c_state = STARTWAITING;
  time_t cmdEchoTimeout = 0;

  // Properties representing encoder steps and stepper axle revolutions used in
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

  struct DomeMovementParameters {
    double angle;
    long steps;
    double time;
  };

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
