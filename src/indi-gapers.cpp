/*
GAPers Telescope driver

Copyright (C) 2026 Massimiliano Masserelli
Copyright (C) 2026 Gruppo Astrofili Persicetani
Copyright (C) 2014 Maurizio Serrazanetti
*/

#include "indicom.h"
#include "indilogger.h"
#include "indi-gapers.h"
#include "gapers_math.h"
#include "libindi/connectionplugins/connectionserial.h"

#include <libnova/libnova.h>

#include <map>
#include <cstring>
#include <cmath>
#include <termios.h>
#include <unistd.h>

#include <sys/time.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "config.h"

const char *DOME_TAB = "Cupola";

namespace
{
constexpr double GABETTI_LATITUDE_DEG = 44.63571;
constexpr double GABETTI_LONGITUDE_DEG_EAST = 11.18273;
constexpr double GABETTI_ELEVATION_M = 24.0;
constexpr double OTA_APERTURE_MM = 380.0;
constexpr double OTA_FOCAL_LENGTH_MM = 2000.0;
constexpr double GUIDER_APERTURE_MM = 0.0;
constexpr double GUIDER_FOCAL_LENGTH_MM = 0.0;
}

static std::unique_ptr<GapersScope> gapersScope(new GapersScope());

/**************************************************************************************
** Initilize GapersScope object
***************************************************************************************/
void ISInit()
{
  // gapersScope è inizializzato staticamente; nulla da fare.
}
/**************************************************************************************
** Return properties of device.
***************************************************************************************/
void ISGetProperties (const char *dev)
{
  ISInit();
  gapersScope->ISGetProperties(dev);
}
/**************************************************************************************
** Process new switch from client
***************************************************************************************/
void ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n)
{
  ISInit();
  gapersScope->ISNewSwitch(dev, name, states, names, n);
}
/**************************************************************************************
** Process new text from client
***************************************************************************************/
void ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
{
  ISInit();
  gapersScope->ISNewText(dev, name, texts, names, n);
}
/**************************************************************************************
** Process new number from client
***************************************************************************************/
void ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
  ISInit();
  gapersScope->ISNewNumber(dev, name, values, names, n);
}
/**************************************************************************************
** Process new blob from client
***************************************************************************************/
void ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n)
{
  ISInit();
  gapersScope->ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}
/**************************************************************************************
** Process snooped property from another driver
***************************************************************************************/
void ISSnoopDevice (XMLEle *root)
{
  INDI_UNUSED(root);
}

GapersScope::GapersScope()
{
  setVersion(CDRIVER_VERSION_MAJOR, CDRIVER_VERSION_MINOR);
  LOGF_INFO("Driver version: %s", CDRIVER_VERSION_STR);
  currentRA  = 0;
  currentDEC = 90;
  initialSyncCompleted = false;

  // This mount is controlled only via serial PLC link.
  setTelescopeConnection(CONNECTION_SERIAL);

  // Mount does not support parking facilities.
  SetParkDataType(PARK_NONE);

  // Default polling period (used when no saved config exists).
  setDefaultPollingPeriod(250);

  // Set telescope capabilities
  SetTelescopeCapability(TELESCOPE_CAN_SYNC | TELESCOPE_HAS_TIME | TELESCOPE_HAS_LOCATION | TELESCOPE_CAN_GOTO, 0);

}
/**************************************************************************************
** We init our properties here. The only thing we want to init are the Debug controls
***************************************************************************************/
bool GapersScope::initProperties()
{
  // ALWAYS call initProperties() of parent first
  INDI::Telescope::initProperties();

  // Default mount type: Equatorial German Mount.
  if (!MountTypeSP.load()) {
    MountTypeSP.reset();
    MountTypeSP[MOUNT_EQ_GEM].setState(ISS_ON);
  }

  // Site defaults for Osservatorio G.Abetti (used only if no saved config exists).
  if (!LocationNP.load()) {
    LocationNP[LOCATION_LATITUDE].setValue(GABETTI_LATITUDE_DEG);
    LocationNP[LOCATION_LONGITUDE].setValue(GABETTI_LONGITUDE_DEG_EAST);
    LocationNP[LOCATION_ELEVATION].setValue(GABETTI_ELEVATION_M);
    updateObserverLocation(GABETTI_LATITUDE_DEG, GABETTI_LONGITUDE_DEG_EAST, GABETTI_ELEVATION_M);
  }

  // Default ON_COORD_SET to SYNC if not already configured.
  if (!CoordSP.load()) {
    CoordSP.reset();
    auto *syncSw = CoordSP.findWidgetByName("SYNC");
    if (syncSw != nullptr)
      syncSw->setState(ISS_ON);
  }

  // Add J2K Coordinates handler
  IUFillNumber(&Eq2kN[0], "RA", "RA (hh:mm:ss)", "%010.6m", 0, 24, 0, 0);
  IUFillNumber(&Eq2kN[1], "DEC", "DEC (dd:mm:ss)", "%010.6m", -90, 90, 0, 90);
  IUFillNumberVector(&Eq2kNP, Eq2kN, 2, getDefaultName(), "EQUATORIAL_COORD", "Eq. Coordinates J2000", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

  // Dome auto-sync property
  IUFillSwitch(&domesyncS[0], "AUTO", "Auto", ISS_ON);
  IUFillSwitch(&domesyncS[1], "MANUAL", "Manual", ISS_OFF);
  IUFillSwitchVector(&domesyncSP, domesyncS, 2, getDefaultName(), "DOME_MOVEMENT", "Dome Movement", DOME_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

  // Add Alt Az coordinates
  IUFillNumber(&AaN[0], "ALT", "Alt (dd:mm:ss)", "%010.6m", -90, 90, 0, 0);
  IUFillNumber(&AaN[1], "AZ", "Az (dd:mm:ss)", "%010.6m", 0, 360, 0, 0);
  IUFillNumberVector(&AaNP, AaN, 2, getDefaultName(), "ALTAZ_COORD", "AltAzimuthal Coordinates", MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

  // Optical defaults: OTA 380/2000mm, no guider.
  IUFillNumber(&telescopeInfoN[0], "TELESCOPE_APERTURE", "Telescope aperture (mm)", "%7.2f", 0, 2000, 0, OTA_APERTURE_MM);
  IUFillNumber(&telescopeInfoN[1], "TELESCOPE_FOCAL_LENGTH", "Telescope focal length (mm)", "%7.2f", 0, 10000, 0, OTA_FOCAL_LENGTH_MM);
  IUFillNumber(&telescopeInfoN[2], "GUIDER_APERTURE", "Guider aperture (mm)", "%7.2f", 0, 2000, 0, GUIDER_APERTURE_MM);
  IUFillNumber(&telescopeInfoN[3], "GUIDER_FOCAL_LENGTH", "Guider focal length (mm)", "%7.2f", 0, 10000, 0, GUIDER_FOCAL_LENGTH_MM);
  IUFillNumberVector(&telescopeInfoNP, telescopeInfoN, 4, getDefaultName(), "TELESCOPE_INFO", "Telescope Info", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

  // Dome azimuth and slew mode
  IUFillNumber(&domeAzN[0], "AZ", "Az (dd:mm:ss)", "%010.6m", 0, 360, 0, 0);
  IUFillNumberVector(&domeAzNP, domeAzN, 1, getDefaultName(), "DOME_AZIMUTH", "Dome Azimuth", DOME_TAB, IP_RW, 60, IPS_IDLE);

  IUFillSwitch(&domeCoordS[0], "SLEW", "Slew", ISS_ON);
  IUFillSwitch(&domeCoordS[1], "SYNC", "Sync", ISS_OFF);
  IUFillSwitchVector(&domeCoordSP, domeCoordS, 2, getDefaultName(), "DOME_ON_COORD_SET", "On Set", DOME_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

  // Dome speed and azimuth threshold
  IUFillNumber(&domeSpeedN[0], "PERIOD", "Seconds for a full spin", "%10.4f", 0, 150, 0.01, 94.33);
  IUFillNumberVector(&domeSpeedNP, domeSpeedN, 1, getDefaultName(), "DOME_SPEED", "Dome rotation speed ", DOME_TAB, IP_RW, 60, IPS_IDLE);

  IUFillNumber(&domeAzThresholdN[0], "THRESHOLD", "Azimuth threshold", "%5.2f", 0, 10, 0.1, 2.0);
  IUFillNumberVector(&domeAzThresholdNP, domeAzThresholdN, 1, getDefaultName(), "DOME_THRESHOLD", "Dome azimuth threshold ", DOME_TAB, IP_RW, 60, IPS_IDLE);

  // Add debug/simulation/etc controls to the driver.
  addAuxControls();

  serialConnection = new Connection::Serial(this);
  serialConnection->registerHandshake([&]() { return Handshake(); });
  serialConnection->setDefaultBaudRate(Connection::Serial::B_57600);
  serialConnection->setDefaultPort("/dev/ttyACM0");
  registerConnection(serialConnection);

  addSimulationControl();
  addDebugControl();

  // Require an initial sync before motion: start ON_COORD_SET in SYNC mode.
  IUResetSwitch(&domeCoordSP);
  domeCoordS[0].s = ISS_OFF;  // SLEW - off
  domeCoordS[1].s = ISS_ON;   // SYNC - on

  return true;
}

bool GapersScope::Handshake() {
  if (isSimulation()) {
    LOGF_INFO("Connected successfully to simulated %s", getDeviceName());
    return true;
  }

  // Keep serial I/O non-blocking since commHandler() is polled by TimerHit().
  int flags = fcntl(PortFD, F_GETFL, 0);
  if (flags == -1 || fcntl(PortFD, F_SETFL, flags | O_NONBLOCK) == -1) {
    LOGF_ERROR("Failed to configure non-blocking I/O on %s: %s", serialConnection->port(), strerror(errno));
    return false;
  }

  int newFlags = fcntl(PortFD, F_GETFL, 0);
  if (newFlags == -1)
    LOGF_WARN("Cannot read serial flags after non-blocking setup on %s: %s", serialConnection->port(), strerror(errno));
  else
    LOGF_DEBUG("Serial fd %d flags after handshake: 0x%X (O_NONBLOCK=%s)", PortFD, newFlags, (newFlags & O_NONBLOCK) ? "ON" : "OFF");

  // TODO: Any initial communication needed with our device; we have an active
  // connection with a valid file descriptor called PortFD. This file descriptor
  // can be used with the tty_* functions in indicom.h

  DEBUG(INDI::Logger::DBG_SESSION, "GAPers Scope connected successfully!");

  // Init serial communication handler buffers and state
  _writequeue = std::queue<std::string>();
  _readbuffer = "";
  c_state = STARTWAITING;
  cmdEchoTimeout = 0;

  // initialize telescope and dome status
  // A serial reconnect implies a physical reset of the mount: require a new sync.
  initialSyncCompleted = false;
  TrackState = SCOPE_TRACKING;
  DomeTrackState = DOME_IDLE;

  // Let's set a timer that checks telescopes status every POLLMS milliseconds.
  SetTimer(getCurrentPollingPeriod());

  return true;
}

/*****
 * INDI Timer method
 */
void GapersScope::TimerHit() {
  if (!isConnected())
    return;

  ReadScopeStatus();

  // Let's set a timer that checks telescopes status every POLLMS milliseconds.
  SetTimer(getCurrentPollingPeriod());

}

/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char * GapersScope::getDefaultName()
{
  return "GAPers Telescope";
}
/**************************************************************************************
** Client is asking us to slew to a new position
***************************************************************************************/
bool GapersScope::Goto(double ra, double dec)
{
  // Check for telescope status and abort if slewing
  if (TrackState == SCOPE_SLEWING) {
    DEBUG(INDI::Logger::DBG_SESSION, "Cannot move while telescope is slewing.");
    return false;
  }
  targetRA=ra;
  targetDEC=dec;

  // Calculate target azimuth and move dome accordingly if in auto state
  ln_equ_posn eqc;
  ln_lnlat_posn eqa;
  ln_hrz_posn psn;

  eqc.ra = targetRA * 15.0;
  eqc.dec = targetDEC;
  eqa.lng = m_Location.longitude;
  if (eqa.lng > 180.) eqa.lng -= 360.;
  eqa.lat = m_Location.latitude;
  ln_get_hrz_from_equ(&eqc, &eqa, ln_get_julian_from_sys(), &psn);
  psn.az = normalizeAz(psn.az + 180.);

  char RAStr[64], DecStr[64];
  // Parse the RA/DEC into strings
  fs_sexa(RAStr, targetRA, 2, 3600);
  fs_sexa(DecStr, targetDEC, 2, 3600);

  double raDist, decDist;

  // Zeroes movement data
  raMovement = AxisMovementParameters();
  decMovement = AxisMovementParameters();
  raIsMoving = decIsMoving = false;

  // Find angular distance between current and target position
  // Distance is then expressed in range -180/180 degrees (short path)
  raDist = rangeDistance((currentRA - targetRA) * 15.0);
  // Update movement data for RA (also accounting for sidereal motion )
  if (! _setMoveDataRA(raDist)) {
    DEBUG(INDI::Logger::DBG_SESSION, "Error in setting RA axis movement.");
    return false;
  }
  if (raMovement.steps != 0) {
    // only move if steps are != 0
    SendMove('1', raMovement.steps, raMovement.startQuote, raMovement.endQuote, raMovement.rotations);
    raIsMoving = true;
  } else {
    raIsMoving = false;
  }

  decDist = rangeDistance(currentDEC - targetDEC);
  if (! _setMoveDataDEC(decDist)) {
    DEBUG(INDI::Logger::DBG_SESSION, "Error in setting DEC axis movement.");
    return false;
  }
  if (decMovement.steps != 0) {
    SendMove('2', decMovement.steps, decMovement.startQuote, decMovement.endQuote, decMovement.rotations);
    decIsMoving = true;
  } else {
    decIsMoving = false;
  }

  if (raIsMoving || decIsMoving) {
    FinalizeMove();

    // Actually move dome only if telescope is moving
    auto domeAutoSw = IUFindSwitch(&domesyncSP, "AUTO");
    if ((domeAutoSw != nullptr) && (domeAutoSw->s == ISS_ON)) {
      DomeGoto(psn.az);
    }


    // Get movement start time (plus 5 seconds, since start is delayed of that amount by PLC)
    movementStart = time(NULL) + 5;

    // Mark state as slewing
    TrackState = SCOPE_SLEWING;
    // Inform client we are slewing to a new position
    DEBUGF(INDI::Logger::DBG_SESSION, "Slewing to RA: %s - DEC: %s", RAStr, DecStr);
  } else {
    DEBUG(INDI::Logger::DBG_SESSION, "Null movement: coordinates already at target position.");
    // Close movement as if it was already executed
    currentRA = targetRA;
    currentDEC = targetDEC;
    TrackState = SCOPE_TRACKING;
    NewRaDec(currentRA, currentDEC);
    
    // If Dome in auto mode, sync dome to telescope azimuth
    auto domeAutoSw = IUFindSwitch(&domesyncSP, "AUTO");
    if ((domeAutoSw != nullptr) && (domeAutoSw->s == ISS_ON)) {
      DomeSync(psn.az);
    }
    return true;
  }

  char raDistStr[64];
  fs_sexa(raDistStr, raDist, 2, 3600);
  DEBUGF(INDI::Logger::DBG_SESSION, "RA dist: %s RA steps (corrected): %ld", raDistStr, raMovement.steps);
  char decDistStr[64];
  fs_sexa(decDistStr, decDist, 2, 3600);
  DEBUGF(INDI::Logger::DBG_SESSION, "DEC dist: %s DEC steps (uncorrected): %ld", decDistStr, decMovement.steps);
  // Success!
  return true;
}
/**************************************************************************************
** Client is asking us to move dome
***************************************************************************************/
bool GapersScope::DomeGoto(double az) {
  // Check for dome status and abort if slewing
  if (DomeTrackState == DOME_SLEWING) {
    DEBUG(INDI::Logger::DBG_SESSION, "Cannot move while dome is slewing.");
    return false;
  }
  if (az == domeCurrentAZ) {
    DEBUG(INDI::Logger::DBG_SESSION, "Dome null movement, ignoring.");
    return true;
  }
  domeTargetAZ=az;
  double azDist = rangeDistance(domeTargetAZ - domeCurrentAZ);

  char azDistStr[64];
  fs_sexa(azDistStr, azDist, 2, 3600);
  DEBUGF(INDI::Logger::DBG_SESSION, "Moving dome %s degrees.", azDistStr);

  long movTime = static_cast<long> ((( domeSpeedN[0].value / 360.0 ) * azDist * 1000.0) + 0.5);
  // Disable dome manual commands
  DomeManualEnable(false);
  // Tell dome to move
  SendCommand( '2', 10, movTime);
  SendCommand( '2', 9, 2);
  SendCommand( '2', 5, 1);

  DomeTrackState = DOME_SLEWING;
  domeAzNP.s = IPS_BUSY;
  IDSetNumber(&domeAzNP, NULL);


  // Set values for dome simulation
  domeMovementStart=time(NULL);
  domeMovementEnd = domeMovementStart+(fabs(movTime)/1000);
  return true;
}

void GapersScope::DomeManualEnable(bool enabled) {
  SendCommand( '2', 10, (enabled ? 0 : 1));
  SendCommand( '2', 9, 1);
  SendCommand( '2', 5, 1);
}

/**************************************************************************************
** Client is asking us to sync dome
***************************************************************************************/
bool GapersScope::DomeSync(double az) {
  // Check for dome status and abort if slewing
  if (DomeTrackState == DOME_SLEWING) {
    DEBUG(INDI::Logger::DBG_SESSION, "Cannot sync while dome is slewing.");
    return false;
  }
  char azDistStr[64];
  fs_sexa(azDistStr, az, 2, 3600);
  DEBUGF(INDI::Logger::DBG_SESSION, "Syncing dome to %s.", azDistStr);

  domeAzN[0].value = domeCurrentAZ = az;
  domeAzNP.s = IPS_OK;
  IDSetNumber(&domeAzNP, NULL);
  return true;
}
/**************************************************************************************
** Client is asking us to abort our motion
***************************************************************************************/
bool GapersScope::Abort()
{
  TrackState = SCOPE_IDLE;
  DEBUG(INDI::Logger::DBG_SESSION, "Simple Scope stopped.");
  return true;
}
/**************************************************************************************
** Client is asking us to report telescope status
***************************************************************************************/
bool GapersScope::ReadScopeStatus()
{
  /* If slewing, we simulate telescope movement. */
  switch (TrackState)
  {
    case SCOPE_SLEWING:
      time_t currentTime;
      time(&currentTime);
      double offset;
      double elapsed;
      elapsed = difftime(currentTime, movementStart);
      if (elapsed > 0) {
        // interpolate RA position
        if (elapsed < raMovement.time) {
          offset = ( raMovement.angle * elapsed ) / raMovement.time;
          currentRA = targetRA + ((raMovement.angle - offset)/15.0);
        }
        if (elapsed < decMovement.time) {
          offset = ( decMovement.angle * elapsed ) / decMovement.time;
          currentDEC = targetDEC + ( decMovement.angle - offset );
        }
      }
      if (isSimulation() && (elapsed >= raMovement.time) && (elapsed >= decMovement.time)) {
        currentRA = targetRA;
        currentDEC = targetDEC;
        // Let's set state to TRACKING
        TrackState = SCOPE_TRACKING;
        DEBUG(INDI::Logger::DBG_SESSION, "Telescope slew is complete. Tracking...");
      }
      NewRaDec(currentRA, currentDEC);
      break;
    default:
      break;
  }
  if (DomeTrackState == DOME_SLEWING) {
    domeAzN[0].value = domeTargetAZ - (rangeDistance(domeTargetAZ - domeCurrentAZ) > 0 ? 1 : -1) * ((domeMovementEnd - time(NULL)) / (domeSpeedN[0].value / 360.0));
    while(domeAzN[0].value >= 360.) domeAzN[0].value -= 360.;
    while(domeAzN[0].value < 0.) domeAzN[0].value += 360.;
    if (isSimulation() && (time(NULL) > domeMovementEnd)) {
      DomeTrackState = DOME_IDLE;
      domeCurrentAZ = domeTargetAZ;
      domeAzN[0].value = domeTargetAZ;
      domeAzNP.s = IPS_OK;
      DEBUG(INDI::Logger::DBG_SESSION, "Dome movement end (simulation mode).");
    }
    IDSetNumber(&domeAzNP, NULL);
  }
  // Update AltAzimuthal Coordinates
  ln_equ_posn eqc;
  ln_lnlat_posn eqa;
  ln_hrz_posn psn;

  eqc.ra = currentRA * 15.0;
  eqc.dec = currentDEC;
  eqa.lng = m_Location.longitude;
  if (eqa.lng > 180.) eqa.lng -= 360.;
  eqa.lat = m_Location.latitude;
  ln_get_hrz_from_equ(&eqc, &eqa, ln_get_julian_from_sys(), &psn);
  psn.az = normalizeAz(psn.az + 180.);
  NewAltAz(psn.alt, psn.az);

  // If telescope is not moving and aim azimuth is more distant than threshold from
  // dome azimuth, and dome control is in auto, then move dome accordingly
  auto domeAutoSw = IUFindSwitch(&domesyncSP, "AUTO");
  const bool domeAutoOn = (domeAutoSw != nullptr) && (domeAutoSw->s == ISS_ON);
  if (domeAutoOn) {
    if ((TrackState != SCOPE_SLEWING) && (DomeTrackState == DOME_IDLE) && (psn.alt <= 87.0) && (fabs(rangeDistance(psn.az - domeCurrentAZ)) > domeAzThresholdN[0].value)) {
      char azStr[64];
      fs_sexa(azStr, psn.az, 2, 3600);
      DEBUGF(INDI::Logger::DBG_SESSION, "Auto-moving dome to %s, thresh %f", azStr, domeAzThresholdN[0].value);
      DomeGoto(psn.az);
    }
  }

  // Process serial communication with PLC
  commHandler();
  return true;
}

double GapersScope::_calcMoveTime(double steps, double vp, double rs) const {
  return GapersMath::calcMoveTime(steps, vp, rs);
}

double GapersScope::normalizeAz(double az) {
  return GapersMath::normalizeAz(az);
}

bool GapersScope::_setMoveDataRA( double distance ) {
  GapersMath::AxisMovementData d;
  bool ok = GapersMath::setMoveDataRA(distance, d);
  raMovement.angle      = d.angle;
  raMovement.steps      = d.steps;
  raMovement.startQuote = d.startQuote;
  raMovement.endQuote   = d.endQuote;
  raMovement.rotations  = d.rotations;
  raMovement.time       = d.time;
  return ok;
}

bool GapersScope::_setMoveDataDEC( double distance ) {
  GapersMath::AxisMovementData d;
  bool ok = GapersMath::setMoveDataDEC(distance, d);
  decMovement.angle      = d.angle;
  decMovement.steps      = d.steps;
  decMovement.startQuote = d.startQuote;
  decMovement.endQuote   = d.endQuote;
  decMovement.rotations  = d.rotations;
  decMovement.time       = d.time;
  return ok;
}


double GapersScope::rangeDistance( double angle) {
  return GapersMath::rangeDistance(angle);
}
/**************************************************************************************
** Client is asking us to sync to a new position
***************************************************************************************/
bool GapersScope::Sync(double ra, double dec)
{
  // Check for telescope status and abort if slewing
  if (TrackState == SCOPE_SLEWING) {
    DEBUG(INDI::Logger::DBG_SESSION, "Cannot move while telescope is slewing.");
    return false;
  }
  char RAStr[64], DecStr[64];
  // Parse the RA/DEC into strings
  fs_sexa(RAStr, ra, 2, 3600);
  fs_sexa(DecStr, dec, 2, 3600);

  // Inform client we are slewing to a new position
  DEBUGF(INDI::Logger::DBG_SESSION, "Syncing to RA: %s - DEC: %s", RAStr, DecStr);

  currentRA = ra;
  currentDEC = dec;
  initialSyncCompleted = true;
  NewRaDec(ra,dec);
  // Mark state as slewing
  TrackState = SCOPE_TRACKING;

  // If Dome in auto mode, sync dome to telescope azimuth
  ln_equ_posn eqc;
  ln_lnlat_posn eqa;
  ln_hrz_posn psn;

  eqc.ra = currentRA * 15.0;
  eqc.dec = currentDEC;
  eqa.lng = m_Location.longitude;
  if (eqa.lng > 180.) eqa.lng -= 360.;
  eqa.lat = m_Location.latitude;
  ln_get_hrz_from_equ(&eqc, &eqa, ln_get_julian_from_sys(), &psn);
  psn.az = normalizeAz(psn.az + 180.);
  NewAltAz(psn.alt, psn.az);

  auto domeAutoSw = IUFindSwitch(&domesyncSP, "AUTO");
  const bool domeAutoOn = (domeAutoSw != nullptr) && (domeAutoSw->s == ISS_ON);
  if (domeAutoOn) {
    bool rc = DomeSync(psn.az);
    if (rc)
      domeAzNP.s = IPS_OK;
    else
      domeAzNP.s = IPS_ALERT;
    IDSetNumber(&domeAzNP, NULL);
  }
  return true;
}

bool GapersScope::_rotationsCalc(long steps, long &m_sq, long &m_eq, long &m_giri) {
  bool ok = GapersMath::rotationsCalc(steps, m_sq, m_eq, m_giri);
  if (!ok)
    DEBUG(INDI::Logger::DBG_SESSION, "Requested a movement too small for spin based driving. This procedure should be used only for > 1^23 steps.");
  return ok;
}

void GapersScope::ISGetProperties (const char *dev) {
  //  First we let our parent populate
  INDI::Telescope::ISGetProperties (dev);

  if(isConnected()) {
    // Add eq coord J2000 number
    defineProperty(&Eq2kNP);
    // Add AltAzimuthal coord
    defineProperty(&AaNP);
    // Add optical information
    defineProperty(&telescopeInfoNP);
    // Add dome properties
    defineProperty(&domesyncSP);
    defineProperty(&domeAzNP);
    defineProperty(&domeCoordSP);
    defineProperty(&domeSpeedNP);
    defineProperty(&domeAzThresholdNP);
  }
}

bool GapersScope::updateProperties()
{
  bool rc = true;
  rc = INDI::Telescope::updateProperties();

  // Keep unsupported manual motion controls out of client UI.
  deleteProperty(MovementNSSP);
  deleteProperty(MovementWESP);
  deleteProperty(ReverseMovementSP);
  deleteProperty(MotionControlModeTP);
  deleteProperty(LockAxisSP);

  if(isConnected())
  {
    defineProperty(&Eq2kNP);
    defineProperty(&AaNP);
    defineProperty(&telescopeInfoNP);
    defineProperty(&domesyncSP);
    defineProperty(&domeAzNP);
    defineProperty(&domeCoordSP);
    defineProperty(&domeSpeedNP);
    defineProperty(&domeAzThresholdNP);
  }
  else
  {
    deleteProperty(Eq2kNP.name);
    deleteProperty(AaNP.name);
    deleteProperty(telescopeInfoNP.name);
    deleteProperty(domesyncSP.name);
    deleteProperty(domeAzNP.name);
    deleteProperty(domeCoordSP.name);
    deleteProperty(domeSpeedNP.name);
    deleteProperty(domeAzThresholdNP.name);
  }

  return rc;
}

bool GapersScope::saveConfigItems(FILE *fp) {
  IUSaveConfigNumber(fp, &telescopeInfoNP);
  IUSaveConfigSwitch(fp, &domesyncSP);
  IUSaveConfigSwitch(fp, &domeCoordSP);
  IUSaveConfigNumber(fp, &domeSpeedNP);
  IUSaveConfigNumber(fp, &domeAzThresholdNP);

  return INDI::Telescope::saveConfigItems(fp);
}

void GapersScope::NewAltAz(double alt, double az) {
  AaN[0].value = alt;
  AaN[1].value = az;
  AaNP.s = IPS_IDLE;
  IDSetNumber(&AaNP, NULL);
}

void GapersScope::NewRaDec(double ra,double dec) {
  char RAStr[64], DecStr[64];
  // Parse the RA/DEC into strings
  fs_sexa(RAStr, ra, 2, 3600);
  fs_sexa(DecStr, dec, 2, 3600);
  LOGF_DEBUG("Current RA: %s Current DEC: %s", RAStr, DecStr);

  switch(TrackState)
  {
    case SCOPE_PARKED:
    case SCOPE_IDLE:
      Eq2kNP.s=IPS_IDLE;
      break;

    case SCOPE_SLEWING:
      Eq2kNP.s=IPS_BUSY;
      break;

    case SCOPE_TRACKING:
      Eq2kNP.s=IPS_OK;
      break;

    default:
      break;
  }

  ln_equ_posn jnow, j2k;

  jnow.ra = ra * 15.0;
  jnow.dec = dec;
  ln_get_equ_prec2(&jnow, ln_get_julian_from_sys(), JD2000, &j2k);
  j2k.ra /= 15.0;

  if (Eq2kN[0].value != j2k.ra || Eq2kN[1].value != j2k.dec || Eq2kNP.s != lastEq2kState)
  {
    Eq2kN[0].value = j2k.ra;
    Eq2kN[1].value = j2k.dec;
    lastEq2kState = Eq2kNP.s;
    IDSetNumber(&Eq2kNP, NULL);
  }
  INDI::Telescope::NewRaDec(ra, dec);
}

bool GapersScope::ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n) {
  //  first check if it's for our device
  if(strcmp(dev,getDefaultName())==0) {
    if(strcmp(name, "TELESCOPE_INFO") == 0) {
      IUUpdateNumber(&telescopeInfoNP, values, names, n);
      telescopeInfoNP.s = IPS_OK;
      IDSetNumber(&telescopeInfoNP, NULL);
      saveConfig(true, telescopeInfoNP.name);
      return true;
    }

    bool rc=false;
    double az=-1;
    if(strcmp(name,"DOME_THRESHOLD")==0) {
      for (int x=0; x<n; x++) {
        if (!strcmp(names[x], "THRESHOLD")) {
          domeAzThresholdN[0].value = values[x];
        }
      }
      domeAzThresholdNP.s = IPS_OK;
      IDSetNumber(&domeAzThresholdNP, NULL);
    } else if(strcmp(name,"DOME_SPEED")==0) {
      for (int x=0; x<n; x++) {
        if (!strcmp(names[x], "PERIOD")) {
          domeSpeedN[0].value = values[x];
        }
      }
      domeSpeedNP.s = IPS_OK;
      IDSetNumber(&domeSpeedNP, NULL);
    } else if(strcmp(name,"DOME_AZIMUTH")==0) {
      auto domeAutoSw = IUFindSwitch(&domesyncSP, "AUTO");
      if (domeAutoSw != nullptr && domeAutoSw->s == ISS_ON) {
        DEBUG(INDI::Logger::DBG_WARNING, "Cannot set azimuth while in auto mode.");
        domeAzNP.s = IPS_OK;
        IDSetNumber(&domeAzNP, NULL);
        return true;
      }
      for (int x=0; x<n; x++) {
        if (!strcmp(names[x], "AZ")) {
          az = values[x];
        }
      }
      if ((az >= 0) && (az <= 360)) {
        auto domeSyncSw = IUFindSwitch(&domeCoordSP, "SYNC");
        const bool domeSyncMode = (domeSyncSw != nullptr) && (domeSyncSw->s == ISS_ON);
        if (domeSyncMode) {
          rc = DomeSync(az);
          if (rc)
            domeAzNP.s = IPS_OK;
          else
            domeAzNP.s = IPS_ALERT;
          IDSetNumber(&domeAzNP, NULL);
          return rc;
        }
        domeTargetAZ = az;
        rc = DomeGoto(az);
        if (rc)
          domeAzNP.s = IPS_BUSY;
        else
          domeAzNP.s = IPS_ALERT;
        IDSetNumber(&domeAzNP, NULL);
        return rc;
      }
      domeAzNP.s = IPS_OK;
      IDSetNumber(&domeAzNP, NULL);
    } else if(strcmp(name,"EQUATORIAL_COORD")==0) {
      //  this is for us, and it is a goto
      bool rc=false;
      double ra=-1;
      double dec=-100;

      for (int x=0; x<n; x++)
      {
        if (!strcmp(names[x], "RA")) {
          ra = values[x];
        } else if (!strcmp(names[x], "DEC")) {
          dec = values[x];
        }
      }
      if ((ra>=0)&&(ra<=24)&&(dec>=-90)&&(dec<=90)) {
        // Convert coordinates to JNOW
        ln_equ_posn jnow,j2k;
        j2k.ra = ra*15.0;
        j2k.dec = dec;
        ln_get_equ_prec2(&j2k, JD2000, ln_get_julian_from_sys(), &jnow);
        ra = jnow.ra/15.0;
        dec = jnow.dec;
        // Check if it is already parked.
        if (CanPark()) {
          if (isParked()) {
            DEBUG(INDI::Logger::DBG_WARNING, "Please unpark the mount before issuing any motion/sync commands.");
            Eq2kNP.s = IPS_IDLE;
            lastEq2kState = IPS_IDLE;
            IDSetNumber(&Eq2kNP, NULL);
            return false;
          }
        }
        // Check if it can sync
        auto syncSw  = CoordSP.findWidgetByName("SYNC");
        auto slewSw  = CoordSP.findWidgetByName("SLEW");
        auto trackSw = CoordSP.findWidgetByName("TRACK");

        const bool syncMode = (syncSw != nullptr) && (syncSw->getState() == ISS_ON);
        if (!initialSyncCompleted && !syncMode)
        {
          Eq2kNP.s = IPS_ALERT;
          lastEq2kState = IPS_ALERT;
          IDSetNumber(&Eq2kNP, "Initial sync required before movement. Set ON_COORD_SET to SYNC and send coordinates once.");
          return false;
        }

        // Keep this check for clients that support TRACK mode on ON_COORD_SET.
        if (!initialSyncCompleted && (trackSw != nullptr) && (trackSw->getState() == ISS_ON))
        {
          Eq2kNP.s = IPS_ALERT;
          lastEq2kState = IPS_ALERT;
          IDSetNumber(&Eq2kNP, "Initial sync required before movement. ON_COORD_SET=TRACK is disabled until first sync.");
          return false;
        }

        if (CanSync()) {
          auto syncOnSetSw = CoordSP.findWidgetByName("SYNC");
          const bool syncOnSetMode = (syncOnSetSw != nullptr) && (syncOnSetSw->getState() == ISS_ON);
          if (syncOnSetMode) {
            rc = Sync(ra,dec);
            if (rc)
              Eq2kNP.s = IPS_OK;
            else
              Eq2kNP.s = IPS_ALERT;
            lastEq2kState = Eq2kNP.s;
            IDSetNumber(&Eq2kNP, NULL);
            return rc;
          }
        }
        if (!initialSyncCompleted && slewSw != nullptr && slewSw->getState() == ISS_ON)
        {
          Eq2kNP.s = IPS_ALERT;
          lastEq2kState = IPS_ALERT;
          IDSetNumber(&Eq2kNP, "Initial sync required before slew. Use ON_COORD_SET=SYNC for first alignment.");
          return false;
        }
        // Issue GOTO
        rc=Goto(ra,dec);
        if (rc)
          Eq2kNP.s = (TrackState == SCOPE_SLEWING) ? IPS_BUSY : IPS_OK;
        else
          Eq2kNP.s = IPS_ALERT;
        lastEq2kState = Eq2kNP.s;
        IDSetNumber(&Eq2kNP, NULL);
      }
      return rc;
    }
  }
  return INDI::Telescope::ISNewNumber(dev,name,values,names,n);
}

bool GapersScope::ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n) {
  if(strcmp(dev,getDefaultName())==0) {
    if (!strcmp(name, CoordSP.getName())) {
      bool wantsSlewOrTrack = false;
      for (int i = 0; i < n; i++) {
        if (states[i] != ISS_ON)
          continue;
        if (!strcmp(names[i], "SLEW") || !strcmp(names[i], "TRACK")) {
          wantsSlewOrTrack = true;
          break;
        }
      }

      if (!initialSyncCompleted && wantsSlewOrTrack) {
        CoordSP.setState(IPS_ALERT);
        CoordSP.apply("Initial sync required: ON_COORD_SET SLEW/TRACK are disabled until first sync.");
        return true;
      }

      CoordSP.update(states, names, n);
      CoordSP.setState(IPS_OK);
      CoordSP.apply();
      return true;
    }

    //  This one is for us
    if(!strcmp(domeCoordSP.name, name)) {
      //  client is telling us what to do with co-ordinate requests
      IUUpdateSwitch(&domeCoordSP, states, names, n);
      domeCoordSP.s = IPS_OK;
      IDSetSwitch(&domeCoordSP, NULL);
      return true;
    }
    // Dome position in sync with telescope
    if (!strcmp(domesyncSP.name, name)) {
      IUUpdateSwitch(&domesyncSP, states, names, n);
      domesyncSP.s = IPS_OK;
      IDSetSwitch(&domesyncSP, NULL);
      return true;
    }
  }
  //  Nobody has claimed this, so, ignore it
  return INDI::Telescope::ISNewSwitch(dev,name,states,names,n);
}

void GapersScope::commHandler() {
  unsigned char inbuf[80]; // small buffer for reception, should hold most commands
  std::string rs = ""; // local buffer for holding a complete command
  static bool nonBlockingStateLogged = false;

  if (isSimulation()) // No interaction with RS232 in simulation mode
  return;

  if (! isConnected()) // If telescope hardware is not connected, bail out
  return;

  if (!nonBlockingStateLogged) {
    int flags = fcntl(PortFD, F_GETFL, 0);
    if (flags == -1)
      LOGF_WARN("comm-handler: cannot read serial flags on fd %d: %s", PortFD, strerror(errno));
    else
      LOGF_DEBUG("comm-handler: serial fd %d flags: 0x%X (O_NONBLOCK=%s)", PortFD, flags, (flags & O_NONBLOCK) ? "ON" : "OFF");
    nonBlockingStateLogged = true;
  }

  do {
    int rlen=0; // number of chars read by read below
    rlen = read(PortFD, inbuf, 80);
    if (rlen == -1) {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
        rlen = 0;
      else {
        DEBUGF(INDI::Logger::DBG_SESSION, "comm-handler: serial error reading %s: %s\n", serialConnection->port(), strerror(errno));
        Disconnect();
        return;
      }
    }
    if (rlen > 0) {
      for (int bufp=0; bufp < rlen; ++bufp) {
        unsigned char cbuf=inbuf[bufp];
        switch (c_state) {
          case STARTWAITING:
          if (cbuf == ASCII_STX) {
            _readbuffer.clear();
            c_state = READINGCOMMAND;
          }
          break;
          case READINGCOMMAND:
          if (cbuf == ASCII_STX) {
            // if a new Start char is found before End char,
            // reset queue, since we've likely got a transmission
            // error anyway.
            _readbuffer.clear();
          } else if (cbuf == ASCII_ETX) {
            rs = _readbuffer;
            _readbuffer.clear();

            c_state = STARTWAITING;
          } else {
            _readbuffer.push_back(cbuf);
          }
          break;
        }
        if (!rs.empty()) {
          ParsePLCMessage(rs);
          rs.clear();
        }
      }
    }
    // check for output queue and eventually send its contents, one at a time.
    if ((cmdEchoTimeout == 0) && (_writequeue.size() > 0)) {
      DEBUGF(INDI::Logger::DBG_SESSION, "comm-handler: Sending Xpres command <%s>...\n", _writequeue.front().c_str());
      int rv = write(PortFD, (unsigned char*) _writequeue.front().c_str(), _writequeue.front().size());
      if (rv == -1) {
        // error occurred
        DEBUGF(INDI::Logger::DBG_SESSION, "comm-handler: serial error %s during write\n", strerror(errno));
        // empty queue and abort processing
        Disconnect();
        return;
      }
      _writequeue.pop();
      cmdEchoTimeout = time(NULL);
    }

    // Check for cmd echo timeout. If no echo is received in a reasonable timeout
    // then something awful is happening and user action is required. We abort
    // processing and disconnect. Timeout exception is set at three seconds. should
    // never happen during normal processing (all commands are normally echoed back).
    if ((cmdEchoTimeout > 0) && ((time(NULL) - cmdEchoTimeout) > 3)) {
      DEBUG(INDI::Logger::DBG_SESSION, "comm-handler: no echo received after sending command. Disconnecting.");
      Disconnect();
      return;
    }
  } while (_writequeue.size() > 0);
}

void GapersScope::ParsePLCMessage(const std::string msg) {
  // Structure of a command:
  // <stx><id>[MESSAGE]<sp><chs><etx>
  //
  // As in previous implementation of this procedure from which I borrowed
  // most of the code, CRC check is silently ignored and can happily be
  // filled with imaginary powers of 42.
  char    syst;
  char    cmd[ 8];

  if (msg.empty()) return;

  // Convert 2 fields!
  if( sscanf( msg.c_str(), "%c%7s ", &syst, cmd) != 2) {
    DEBUGF(INDI::Logger::DBG_SESSION, "comm-handler: Xpres syntax error: '%s'\n", msg.c_str());
    return;
  }

  // Find out the read command
  // Received ERROR command
  if( strncasecmp( cmd, "mi", 2) == 0)
  {
    int val = 0;
    char detail[64] = {0};
    int fields = sscanf(msg.substr(4).c_str(), "%d %63s ", &val, detail);

    if (fields >= 2)
      LOGF_DEBUG("comm-handler: Xpres ERROR %c %d (%s)", syst, val, detail);
    else
      LOGF_DEBUG("comm-handler: Xpres ERROR %c %d", syst, val);

    // Dizionario dei codici mi noti (range documentato: 400-582).
    // Fonte: documentazione protocollo Xpress + analisi debugsession.log (2002-2007).
    struct MiCode { int code; uint level; const char *desc; };
    static const MiCode MI_CODES[] = {
      // Startup / ready
      { 500, INDI::Logger::DBG_DEBUG,           "READY - subsystem online" },
      // Warnings / recoverable
      { 547, INDI::Logger::DBG_WARNING,        "PLC warning / soft error" },
      { 510, INDI::Logger::DBG_WARNING,        "Limit switch warning" },
      { 511, INDI::Logger::DBG_WARNING,        "End-of-travel warning" },
      { 520, INDI::Logger::DBG_WARNING,        "Motor overcurrent warning" },
      { 530, INDI::Logger::DBG_WARNING,        "Encoder fault" },
      // Errors
      { 400, INDI::Logger::DBG_ERROR,          "Generic PLC error" },
      { 401, INDI::Logger::DBG_ERROR,          "Comm timeout" },
      { 402, INDI::Logger::DBG_ERROR,          "Checksum error" },
      { 403, INDI::Logger::DBG_ERROR,          "Unknown command" },
      { 450, INDI::Logger::DBG_ERROR,          "Motor driver fault" },
      { 451, INDI::Logger::DBG_ERROR,          "Overcurrent fault" },
      { 452, INDI::Logger::DBG_ERROR,          "Overtemperature fault" },
      { 582, INDI::Logger::DBG_ERROR,          "Emergency stop active" },
    };
    static const int MI_CODES_COUNT = static_cast<int>(sizeof(MI_CODES) / sizeof(MI_CODES[0]));

    const char *miDesc = nullptr;
    uint miLevel = INDI::Logger::DBG_WARNING;
    for (int i = 0; i < MI_CODES_COUNT; ++i)
    {
      if (MI_CODES[i].code == val)
      {
        miDesc  = MI_CODES[i].desc;
        miLevel = MI_CODES[i].level;
        break;
      }
    }
    if (miDesc)
      DEBUGF(miLevel, "comm-handler: subsystem %c mi %d: %s\n", syst, val, miDesc);
    else
      DEBUGF(INDI::Logger::DBG_WARNING, "comm-handler: subsystem %c mi %d (unknown code)\n", syst, val);
  }
  else
  // Received STATUS/INFO string command
  if( strncasecmp( cmd, "vf", 2) == 0)
  {
    int var = 0;
    int whr = 0;
    char text[128] = {0};

    // Typical payload: "002 001 \"A.R. pronta\" ..."
    if (sscanf(msg.substr(4).c_str(), "%d %d \"%127[^\"]\"", &var, &whr, text) == 3)
      LOGF_DEBUG("comm-handler: Xpres STATUS %c %d %d \"%s\"", syst, var, whr, text);
    else
      LOGF_DEBUG("comm-handler: Xpres STATUS %c raw: %s", syst, msg.c_str());

    return;
  }
  else
  // Received VAR update command
  if( strncasecmp( cmd, "vn", 2) == 0)
  {
    int val, var, whr;
    sscanf( msg.substr(4).c_str(), "%d %d %d ", &val, &var, &whr);
    LOGF_DEBUG("comm-handler: Xpres EVENT %c %d %d %d", syst, var, val, whr);
    // m_signal_event.emit(syst, var, val, whr);
    switch (syst) {
      case '1': // Var update in RA subsystem
      switch (var) {
        case 4:          // Stepper quote on PuntaGiri
        case 8:					 // Stepper quote
        if (whr == 1) { // 1 means end of slewing
          currentRA = targetRA;
          raIsMoving = false;
          if (! decIsMoving) {
            TrackState = SCOPE_TRACKING;
            DEBUG(INDI::Logger::DBG_SESSION, "Telescope slew is complete. Tracking...");
          }
          NewRaDec(currentRA, currentDEC);
        }
        break;
      }
      break;
      case '2': // Var update in DEC subsystem
      switch (var) {
        case 4:          // Stepper quote on PuntaGiri
        case 8:					 // Stepper quote
        if (whr == 1) { // 1 means end of slewing
          currentDEC = targetDEC;
          decIsMoving = false;
          if (! raIsMoving) {
            TrackState = SCOPE_TRACKING;
            DEBUG(INDI::Logger::DBG_SESSION, "Telescope slew is complete. Tracking...");
          }
          NewRaDec(currentRA, currentDEC);
        }
        break;
        case 9:         // Dome subsystem notifications
        if (whr == 2) { // 2 means end of slewing
          DomeTrackState = DOME_IDLE;
          DEBUG(INDI::Logger::DBG_SESSION, "Dome rotation is complete. Stopped.");
          DomeManualEnable(true);
          domeAzN[0].value = domeCurrentAZ = domeTargetAZ;
          domeAzNP.s = IPS_OK;
          IDSetNumber(&domeAzNP, NULL);
        }
        break;
      }
      break;
    }
  }
  else
  // Received ECHO of sent command
  if( strncasecmp( cmd, "tx", 2) == 0)
  {
    LOGF_DEBUG("comm-handler: Xpres echo received: %s", msg.c_str());
    cmdEchoTimeout = 0;
    return;
  }
  else // Received ECHO or unhandled command
  {
    LOGF_DEBUG("comm-handler: Xpres unhandled command: %s", msg.c_str());
    return;
  }

}

void GapersScope::SendMove(char _system, long steps, long m_sq, long m_eq, long m_giri) {
  switch (_system) {
    case '1': raIsMoving = true; break;
    case '2': decIsMoving = true; break;
    default:
    DEBUGF(INDI::Logger::DBG_SESSION, "XpresIF: requested movement on non-existent system %c.\n", _system);
    return;
    break;
  }
  if( std::abs( m_giri ) > 0)	{
    DEBUGF(INDI::Logger::DBG_SESSION, "XpresIF: Movement > 2^23 steps on %c axis: %d %d %d %d.\n", _system, steps, m_sq, m_eq, m_giri);
    SendCommand(_system, 10, m_sq);
    SendCommand(_system, 9, 5);
    SendCommand(_system, 5, 1);
    SendCommand(_system, 10, m_eq);
    SendCommand(_system, 9, 6);
    SendCommand(_system, 5, 1);
    SendCommand(_system, 10, m_giri);
    SendCommand(_system, 9, 7);
    SendCommand(_system, 5, 1);
  } else {
    SendCommand(_system, 15, steps);
  }
  return;
}

void GapersScope::SendCommand( char syst, short int cmd, long val )
{
  if (isSimulation()) {
    DEBUGF(INDI::Logger::DBG_SESSION, "XpresIF: simulation mode, not sending %ctx %hd %ld ...\n", syst, cmd, val);
    return;
  }

  if( !isConnected())
  return;

  int len;
  char msg[ 64];
  char chk[  8];
  unsigned char msgCRC = 0;

  // Build up the msg
  len = sprintf( msg, "\x02%ctx %hd %ld ", syst, cmd, val);

  // Calculate CRC
  for( int k = 0; k < len; k++)
  msgCRC ^= msg[ k];

  // Append control info to end of msg
  sprintf( chk, "%02X%02X\x03", len, msgCRC);
  strcat( msg, chk);

  // Queue XPRES msg
  _writequeue.push(msg);
  return;
}

void GapersScope::FinalizeMove() {
  if (raIsMoving && decIsMoving) {
    if ((raMovement.rotations != 0) && (decMovement.rotations != 0)) {
      SendCommand('0', 14 , 1); // move both systems in the same manner
      return;
    } else if ((raMovement.rotations == 0) && (decMovement.rotations == 0)) {
      SendCommand('0', 8 , 1); // move both systems in the same manner
      return;
    }
  }
  if (raIsMoving) {
    SendCommand('1', (raMovement.rotations != 0) ? 14 : 8, 1);
  }
  if (decIsMoving) {
    SendCommand('2', (decMovement.rotations != 0) ? 14 : 8, 1);
  }
}
