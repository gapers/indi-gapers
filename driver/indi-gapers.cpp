/*
GAPers Telescope driver

Copyright (C) 2024 Massimiliano Masserelli
Copyright (C) 2024 Gruppo Astrofili Persicetani
Copyright (C) 2014 Maurizio Serrazanetti
*/

#include "indicom.h"
#include "indilogger.h"
#include "indi-gapers.h"
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
#include <math.h>
#include "config.h"

#define PARITY_NONE 0
#define PARITY_EVEN 1
#define PARITY_ODD  2

const float SIDRATE = 0.004178;                /* sidereal rate, degrees/s */
const int   SLEW_RATE = 15;                    /* slew rate, degrees/s */
//const int   POLLMS = 250;                      /* poll period, ms */

const char *DOME_TAB = "Cupola";

// std::auto_ptr<GapersScope> gapersScope(0);
static std::unique_ptr<GapersScope> gapersScope(new GapersScope());

/**************************************************************************************
** Initilize GapersScope object
***************************************************************************************/
void ISInit()
{
  static int isInit=0;
  if (isInit)
  return;
  if (gapersScope.get() == 0)
  {
    isInit = 1;
    gapersScope.reset(new GapersScope());
  }
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
  currentRA  = 0;
  currentDEC = 90;
  // We add an additional debug level so we can log verbose scope status
  DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");
  
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

  // Add J2K Coordinates handler
  IUFillNumber(&Eq2kN[AXIS_RA],"RA","RA (hh:mm:ss)","%010.6m",0,24,0,0);
  IUFillNumber(&Eq2kN[AXIS_DE],"DEC","DEC (dd:mm:ss)","%010.6m",-90,90,0,0);
  IUFillNumberVector(&Eq2kNP,Eq2kN,2,getDefaultName(),"EQUATORIAL_COORD","Eq. Coordinates J2000",MAIN_CONTROL_TAB,IP_RW,60,IPS_IDLE);

  // Dome properties
  IUFillSwitch(&domesyncS[0], "AUTO", "Auto", ISS_ON);
  IUFillSwitch(&domesyncS[1], "MANUAL", "Manual", ISS_OFF);
  IUFillSwitchVector(&domesyncSP, domesyncS, 2, getDefaultName(), "DOME_MOVEMENT", "Dome Movement", DOME_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

  // Add Alt Az coordinates
  IUFillNumber(&AaN[AXIS_ALT], "ALT", "Alt (dd:mm:ss)","%010.6m",-90,90,0,0);
  IUFillNumber(&AaN[AXIS_AZ], "AZ", "Az (dd:mm:ss)","%010.6m",0,360,0,0);
  IUFillNumberVector(&AaNP,AaN,2,getDefaultName(),"ALTAZ_COORD","AltAzimuthal Coordinates",MAIN_CONTROL_TAB,IP_RO,60,IPS_IDLE);

  IUFillNumber(&domeAzN[0], "AZ", "Az (dd:mm:ss)", "%010.6m",0,360,0,0);
  IUFillNumberVector(&domeAzNP, domeAzN, 1, getDefaultName(), "DOME_AZIMUTH", "Dome Azimuth", DOME_TAB, IP_RW, 60, IPS_IDLE);

  IUFillSwitch(&domeCoordS[0],"SLEW","Slew",ISS_ON);
  IUFillSwitch(&domeCoordS[1],"SYNC","Sync",ISS_OFF);
  IUFillSwitchVector(&domeCoordSP,domeCoordS,2,getDefaultName(),"DOME_ON_COORD_SET","On Set",DOME_TAB,IP_RW,ISR_1OFMANY,60,IPS_IDLE);

  IUFillNumber(&domeSpeedN[0], "PERIOD", "Seconds for a full spin", "%10.4f",0,150,0.01,94.33);
  IUFillNumberVector(&domeSpeedNP, domeSpeedN, 1, getDefaultName(), "DOME_SPEED", "Dome rotation speed ", DOME_TAB, IP_RW, 60, IPS_IDLE);

  IUFillNumber(&domeAzThresholdN[0], "THRESHOLD", "Azimuth threshold", "%5.2f",0,10,0.1,2.0);
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
  return true;
}

bool GapersScope::Handshake() {
  if (isSimulation()) {
    LOGF_INFO("Connected successfully to simulated %s", getDeviceName());
    return true;
  }

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
  TrackState = SCOPE_TRACKING;
  DomeTrackState = DOME_IDLE;

  // Let's set a timer that checks telescopes status every POLLMS milliseconds.
  SetTimer(POLLMS);

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
  SetTimer(POLLMS);

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
  eqa.lng = LocationN[LOCATION_LONGITUDE].value;
  if (eqa.lng > 180.) eqa.lng -= 360.;
  eqa.lat = LocationN[LOCATION_LATITUDE].value;
  ln_get_hrz_from_equ(&eqc, &eqa, ln_get_julian_from_sys(), &psn);
  psn.az += 180.;
  while (psn.az >= 360.) psn.az -= 360.;
  while (psn.az < 0.) psn.az += 360.;

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
    if (domesyncS[0].s == ISS_ON) {
      DomeGoto(psn.az);
    }


    // Get movement start time (plus 5 seconds, since start is delayed of that amount by PLC)
    movementStart = time(NULL) + 5;

    // Mark state as slewing
    TrackState = SCOPE_SLEWING;
    // Inform client we are slewing to a new position
    DEBUGF(INDI::Logger::DBG_SESSION, "Slewing to RA: %s - DEC: %s", RAStr, DecStr);
  } else {
    DEBUG(INDI::Logger::DBG_SESSION, "BAZINGA! Nothing is actually moving.");
    return false;
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
  eqa.lng = LocationN[LOCATION_LONGITUDE].value;
  if (eqa.lng > 180.) eqa.lng -= 360.;
  eqa.lat = LocationN[LOCATION_LATITUDE].value;
  ln_get_hrz_from_equ(&eqc, &eqa, ln_get_julian_from_sys(), &psn);
  // DEBUGF(INDI::Logger::DBG_SESSION, "bubu: %f %f %f %f %f %f %f", eqc.ra, eqc.dec, eqa.lat, eqa.lng, ln_get_julian_from_sys(), psn.az, psn.alt);
  psn.az += 180.;
  while (psn.az >= 360.) psn.az -= 360.;
  while (psn.az < 0.) psn.az += 360.;
  NewAltAz(psn.alt, psn.az);

  // If telescope is not moving and aim azimuth is more distant than threshold from
  // dome azimuth, and dome control is in auto, then move dome accordingly
  ISwitch *sw;
  sw=IUFindSwitch(&domesyncSP,"AUTO");
  if((sw != NULL)&&( sw->s==ISS_ON )) {
    domeAzThreshold = domeAzThresholdN[0].value;
    if ((TrackState != SCOPE_SLEWING) && (DomeTrackState == DOME_IDLE) && (psn.alt <= 87.0) && (fabs(rangeDistance(psn.az - domeCurrentAZ)) > domeAzThreshold)) {
      char azStr[64];
      fs_sexa(azStr, psn.az, 2, 3600);
      DEBUGF(INDI::Logger::DBG_SESSION, "Auto-moving dome to %s, thresh %f", azStr, domeAzThreshold);
      DomeGoto(psn.az);
    }
  }

  // Process serial communication with PLC
  commHandler();
  return true;
}

bool GapersScope::_setMoveDataRA( double distance ) {
  // Durante lo spostamento in ascensione retta occorre compensare il moto
  // siderale che si manifesta nel tempo necessario al movimento dell'asse.

  // Costanti usate nel calcolo
  const double vs = 919.456; // Velocità moto siderale in passi per secondo
  const double vp = 220000.0;  // Velocità movimento asse in passi per secondo
  const double spd = 220088.2; // Passi motore per grado di spostamento asse
  const double rs = 500000.0; // Passi utilizzati per le rampe di salita e discesa
  // Il tempo di rampa viene calcolato utilizzando la velocità media in passi
  // al secondo tra la velocità di partenza e quella di arrivo. Essendo una
  // rampa lineare il valore dovrebbe essere accurato.
  const double tr = rs / ((vp-200.0)/2.0); // Tempo in secondi necessario a completare rampa salita e discesa

  // Variabili d'appoggio
  double tm = 0; // Tempo necessario allo spostamento dell'asse
  double correction = 0; // Passi necessari a compensare il moto siderale occorso durante lo spostamento

  // La componente del moto siderale è sempre positiva (da est ad ovest),
  // pertanto consideriamo il valore assoluto del numero di passi necessari
  // per lo spostamento, memorizzando la direzione per potere alla fine
  // effettuare la correzione nella giusta direzione.
  int direction = ( distance > 0 ? 1 : -1);
  double steps = fabs( distance ) * spd;

  if ( steps > rs ) {
    // Il movimento richiesto è superiore ai passi necessari per completare
    // le rampe di accelerazione e decellerazione dei motori. Viene calcolata
    // la correzione da applicare per il moto siderale considerando il tempo
    // necessario a completare le rampe sommato a quello necessario per
    // compiere i rimanenti passi a velocità di regime
    tm = ((steps - rs) / vp) + tr;
  } else {
    // Calcolo usando proporzione tempo totale (tm) : tempo rampa = steps : rs (passi per compiere entrambe le rampe)
    tm = (steps * tr) / rs;
  }
  // La correzione applicata è pari al tempo totale di spostamento moltiplicato
  // per la velocità siderale in passi al secondo. NB: l'algoritmo non è preciso
  // perché non viene considerato l'effetto della correzione sul tempo totale
  // necessario al movimento, ma l'errore così introdotto dovrebbe essere
  // abbastanza piccolo da essere trascurabile.
  correction = tm * vs;
  // Impostazione variabili necessarie per il movimento (movimento semplice o per giri)
  raMovement.angle = distance;
  // Imposta il numero di passi corretto arrotondato all'intero più vicino
  raMovement.steps = static_cast<long>((steps+0.5) * direction + correction );
  raMovement.startQuote = 0;
  raMovement.endQuote = 0;
  raMovement.rotations = 0;
  raMovement.time = tm;
  if (abs(raMovement.steps) > 80*12800) {
    return _rotationsCalc(raMovement.steps, raMovement.startQuote, raMovement.endQuote, raMovement.rotations);
  }
  return true;
}

bool GapersScope::_setMoveDataDEC( double distance ) {
  // Costanti usate nel calcolo
  // const double vs = 919.456; // Velocità moto siderale in passi per secondo
  const double vp = 220000.0;  // Velocità movimento asse in passi per secondo
  const double spd = 192000.0; // Passi motore per grado di spostamento asse
  const double rs = 500000.0; // Passi utilizzati per le rampe di salita e discesa
  // Il tempo di rampa viene calcolato utilizzando la velocità media in passi
  // al secondo tra la velocità di partenza e quella di arrivo. Essendo una
  // rampa lineare il valore dovrebbe essere accurato.
  const double tr = rs / ((vp-200.0)/2.0); // Tempo in secondi necessario a completare rampa salita e discesa

  // Variabili d'appoggio
  double tm = 0; // Tempo necessario allo spostamento dell'asse

  // La componente del moto siderale è sempre positiva (da est ad ovest),
  // pertanto consideriamo il valore assoluto del numero di passi necessari
  // per lo spostamento, memorizzando la direzione per potere alla fine
  // effettuare la correzione nella giusta direzione.
  int direction = ( distance > 0 ? 1 : -1);
  double steps = fabs( distance ) * spd;

  if ( steps > rs ) {
    // Il movimento richiesto è superiore ai passi necessari per completare
    // le rampe di accelerazione e decellerazione dei motori. Viene calcolata
    // la correzione da applicare per il moto siderale considerando il tempo
    // necessario a completare le rampe sommato a quello necessario per
    // compiere i rimanenti passi a velocità di regime
    tm = ((steps - rs) / vp) + tr;
  } else {
    // Calcolo usando proporzione tempo totale (tm) : tempo rampa = steps : rs (passi per compiere entrambe le rampe)
    tm = (steps * tr) / rs;
  }
  // return static_cast<long>((steps+0.5) * direction + correction ); // Ritorna il numero di passi corretto arrotondato all'intero più vicino
  decMovement.angle = distance;
  decMovement.steps = static_cast<long>((steps+0.5) * direction );
  decMovement.startQuote = 0;
  decMovement.endQuote = 0;
  decMovement.rotations = 0;
  decMovement.time = tm;
  if (abs(decMovement.steps) > 80*12800) {
    return _rotationsCalc(decMovement.steps, decMovement.startQuote, decMovement.endQuote, decMovement.rotations);
  }
  return true;
}


double GapersScope::rangeDistance( double angle) {
  /*
  * Riporta il range di un angolo all'interno di +/-180 gradi
  * da utilizzare nel calcolo delle differenze angolari per
  * gli spostamenti nelle due direzioni in modo da utilizzare
  * sempre il percorso angolare più breve.
  */
  double r = angle;
  while (r < -180.0) r += 360.0;
  while (r > 180.0) r -= 360.0;
  return r;
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
  NewRaDec(ra,dec);
  // Mark state as slewing
  TrackState = SCOPE_TRACKING;

  // If Dome in auto mode, sync dome to telescope azimuth
  ln_equ_posn eqc;
  ln_lnlat_posn eqa;
  ln_hrz_posn psn;

  eqc.ra = currentRA * 15.0;
  eqc.dec = currentDEC;
  eqa.lng = LocationN[LOCATION_LONGITUDE].value;
  if (eqa.lng > 180.) eqa.lng -= 360.;
  eqa.lat = LocationN[LOCATION_LATITUDE].value;
  ln_get_hrz_from_equ(&eqc, &eqa, ln_get_julian_from_sys(), &psn);
  // DEBUGF(INDI::Logger::DBG_SESSION, "bubu: %f %f %f %f %f %f %f", eqc.ra, eqc.dec, eqa.lat, eqa.lng, ln_get_julian_from_sys(), psn.az, psn.alt);
  psn.az += 180.;
  while (psn.az >= 360.) psn.az -= 360.;
  while (psn.az < 0.) psn.az += 360.;
  NewAltAz(psn.alt, psn.az);

  ISwitch *sw;
  sw=IUFindSwitch(&domesyncSP,"AUTO");
  if((sw != NULL)&&( sw->s==ISS_ON )) {
    bool rc=DomeSync(psn.az);
    if (rc)
    domeAzNP.s = IPS_OK;
    else
    domeAzNP.s = IPS_ALERT;
    IDSetNumber(&domeAzNP, NULL);
  }
  return true;
}

bool GapersScope::_rotationsCalc(long steps, long &m_sq, long &m_eq, long &m_giri) {
  // La quota rappresentabile dagli encoder dello stepper dei motori è limitato al
  // range -8388608 <--> +8388607. Questo limita il movimento basato sulla
  // differenza di quota a 2^23 passi, pari a circa 38 gradi.
  // Qualora sia necessario effettuare uno spostamento maggiore, occorre
  // utilizzare una procedura alternativa: si calcola il movimento in giri
  // completi del motore, che viene eseguito superando l'overflow della quota
  // dell'encoder, che ricomincia a contare ripartendo dal valore più basso
  // rappresentabile. Al termine del movimento "per giri", ci si posiziona
  // alla quota necessaria per ottenere lo spostamento preciso richiesto.
  // Siccome il movimento "per giri" è meno preciso di quello per passi regolato
  // dall'encoder, è necessario fermarlo prima di aver compiuto il massimo
  // movimento possibile, altrimenti si rischia di trovarsi oltre la quota di
  // encoder desiderata, costringendo il motore ad invertire il senso di marcia.
  // I due movimenti sono distinti e questo non sarebbe eccessivamente
  // probklematico ma nel caso del movimento in ascensione retta ciò
  // potrebbe rendere meno affidabile la correzione da applicare per compensare
  // il moto siderale apparente. Viene pertanto usato nel calcolo un arbitrario
  // "valore di sicurezza" pari ad 80 giri completi del motore, ovvero circa
  // 1.024.000 passi (4 gradi circa di movimento). Questo valore viene sottratto
  // al numero di passi richiesti per lo spostamento in modo da fermarsi per
  // tempo prima di passare al movimento per passi.
  // Nella procedura di movimento per giri gestita dal PLC vanno comunicati la
  // quota iniziale da impostare sull'encoder, la quota finale da raggiungere
  // ed il numero di giri da compiere. La quota finale viene calcolata
  // considerando il numero totale di passi da compiere e ricominciando a
  // contare dal limite inferiore qualora si oltrepassi il limite superiore
  // rappresentabile dall'encoder (gestione dell'overflow). Nel caso in cui il
  // calcolo porti ad una quota finale di valore pari a 0, quota iniziale e
  // finale vengono aumentati di un valore arbitrario (100). Il valore 0 non
  // è infatti ammesso nei parametri da passare al PLC nella richiesta per
  // avviare questa procedura.

  const long qrange = 8388608+8388608; // range of stepper quota values (from -8388608 to +8388607)
  const long qsafe = 80*12800; // safe quota equivalent of 80 revolutions

  m_sq=0;
  m_eq=0;

  if (abs(steps) < qsafe) {
    // Sanity check: questa procedura dovrebbe essere utilizzata soltanto per
    // spostamenti superiori a 38 gradi, 2^23 passi. Utilizzarla per movimenti
    // più ridotti non è comunque un problema fino a che si sta sopra alla
    // quota di sicurezza utilizzata per il calcolo dei giri, 1 milione di
    // passi ovvero circa 4 gradi.
    //    error("lo spostamento lungo deve essere usato solo per movimenti > 1<<23 passi.");
    DEBUG(INDI::Logger::DBG_SESSION, "Requested a movement too small for spin based driving. This procedure should be used only for > 1^23 steps.");
    return false;
  }

  if (steps > 0) {
    // steps are positive, clockwise movement)
    m_sq = -8388608;
    m_eq = (steps % qrange)+m_sq;
    m_giri = ((steps - qsafe) / 12800) + 1;
    if ( m_eq < (m_sq + qsafe) ) { // Evitiamo di trovarci a cavallo dell'overflow al termine del movimento per giri
      m_sq += qsafe;
      m_eq += qsafe;
    }
    // check for a nasty race condition in plc program
    if (m_eq == 0) {
      m_sq += 100;
      m_eq = 100;
    }
  } else { // steps are negative (counterclockwise movement)
    m_sq = 8388607;
    m_eq = (steps % qrange)+m_sq;
    m_giri = ((steps + qsafe) / 12800) -1;
    if ( m_eq > (m_sq - qsafe) ) { // Evitiamo di trovarci a cavallo dell'overflow al termine del movimento per giri
      m_sq -= qsafe;
      m_eq -= qsafe;
    }
    // check for a nasty race condition in plc program
    if (m_eq == 0) {
      m_sq -= 100;
      m_eq = -100;
    }
  }
return true;
}

void GapersScope::ISGetProperties (const char *dev) {
  //  First we let our parent populate
  INDI::Telescope::ISGetProperties (dev);

  if(isConnected()) {
    // Add eq coord J2000 number
    defineNumber(&Eq2kNP);
    // Add AltAzimuthal coord
    defineNumber(&AaNP);
    // Add dome properties
    defineSwitch(&domesyncSP);
    defineNumber(&domeAzNP);
    defineSwitch(&domeCoordSP);
    defineNumber(&domeSpeedNP);
    defineNumber(&domeAzThresholdNP);
  }
}

bool GapersScope::updateProperties()
{
  bool rc = true;
  rc = INDI::Telescope::updateProperties();

  if(isConnected())
  {
    defineNumber(&Eq2kNP);
    defineNumber(&AaNP);
    defineSwitch(&domesyncSP);
    defineNumber(&domeAzNP);
    defineSwitch(&domeCoordSP);
    defineNumber(&domeSpeedNP);
    defineNumber(&domeAzThresholdNP);

    loadDefaultConfig();
  }
  else
  {
    deleteProperty(Eq2kNP.name);
    deleteProperty(AaNP.name);
    deleteProperty(domesyncSP.name);
    deleteProperty(domeAzNP.name);
    deleteProperty(domeCoordSP.name);
    deleteProperty(domeSpeedNP.name);
    deleteProperty(domeAzThresholdNP.name);
  }

  return rc;
}

bool GapersScope::saveConfigItems(FILE *fp) {
  IUSaveConfigSwitch(fp, &domesyncSP);
  IUSaveConfigSwitch(fp, &domeCoordSP);
  IUSaveConfigNumber(fp, &domeSpeedNP);
  IUSaveConfigNumber(fp, &domeAzThresholdNP);
  IUSaveConfigNumber(fp, &ScopeParametersNP);

  return INDI::Telescope::saveConfigItems(fp);
}

void GapersScope::NewAltAz(double alt, double az) {
  AaN[AXIS_ALT].value = alt;
  AaN[AXIS_AZ].value = az;
  AaNP.s = IPS_IDLE;
  IDSetNumber(&AaNP, NULL);
}

void GapersScope::NewRaDec(double ra,double dec) {
  char RAStr[64], DecStr[64];
  // Parse the RA/DEC into strings
  fs_sexa(RAStr, ra, 2, 3600);
  fs_sexa(DecStr, dec, 2, 3600);
  DEBUGF(DBG_SCOPE, "Current RA: %s Current DEC: %s", RAStr, DecStr );

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

  if (Eq2kN[AXIS_RA].value != j2k.ra || Eq2kN[AXIS_DE].value != j2k.dec || Eq2kNP.s != lastEq2kState)
  {
    Eq2kN[AXIS_RA].value=j2k.ra;
    Eq2kN[AXIS_DE].value=j2k.dec;
    lastEq2kState = Eq2kNP.s;
    IDSetNumber(&Eq2kNP, NULL);
  }
  INDI::Telescope::NewRaDec(ra, dec);
}

bool GapersScope::ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n) {
  //  first check if it's for our device
  if(strcmp(dev,getDefaultName())==0) {
    bool rc=false;
    double az=-1;
    if(strcmp(name,"DOME_THRESHOLD")==0) {
      for (int x=0; x<n; x++) {
        INumber *th = IUFindNumber(&domeAzThresholdNP, names[x]);
        if (th == &domeAzThresholdN[0]) {
          domeAzThreshold = values[x];
          domeAzThresholdN[0].value = domeAzThreshold;
        }
      }
      domeAzThresholdNP.s = IPS_OK;
      IDSetNumber(&domeAzThresholdNP, NULL);
    } else if(strcmp(name,"DOME_SPEED")==0) {
      for (int x=0; x<n; x++) {
        INumber *sp = IUFindNumber(&domeSpeedNP, names[x]);
        if (sp == &domeSpeedN[0]) {
          domeSpeed = values[x];
          domeSpeedN[0].value = domeSpeed;
        }
      }
      domeSpeedNP.s=IPS_OK;
      IDSetNumber(&domeSpeedNP, NULL);
    } else if(strcmp(name,"DOME_AZIMUTH")==0) {
      if (domesyncS[0].s == ISS_ON) {
        DEBUG(INDI::Logger::DBG_WARNING, "Cannot set azimuth while in auto mode.");
        domeAzNP.s=IPS_OK;
        IDSetNumber(&domeAzNP, NULL);
        return true;
      }
      for (int x=0; x<n; x++) {
        INumber *azp = IUFindNumber(&domeAzNP, names[x]);
        if (azp == &domeAzN[0]) {
          az = values[x];
        }
      }
      if ((az >= 0) && (az <= 360)) {
        ISwitch *sw;
        sw=IUFindSwitch(&domeCoordSP,"SYNC");
        if((sw != NULL)&&( sw->s==ISS_ON )) {
          rc=DomeSync(az);
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
        // domeAzN[0].value = az;
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
        INumber *eqp = IUFindNumber (&Eq2kNP, names[x]);
        if (eqp == &Eq2kN[AXIS_RA]) {
          ra = values[x];
        } else if (eqp == &Eq2kN[AXIS_DE]) {
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
            Eq2kNP.s = lastEq2kState = IPS_IDLE;
            IDSetNumber(&Eq2kNP, NULL);
            return false;
          }
        }
        // Check if it can sync
        if (CanSync()) {
          ISwitch *sw;
          sw=IUFindSwitch(&CoordSP,"SYNC");
          if((sw != NULL)&&( sw->s==ISS_ON )) {
            rc=Sync(ra,dec);
            if (rc)
            Eq2kNP.s = lastEq2kState = IPS_OK;
            else
            Eq2kNP.s = lastEq2kState = IPS_ALERT;
            IDSetNumber(&Eq2kNP, NULL);
            return rc;
          }
        }
        // Issue GOTO
        rc=Goto(ra,dec);
        if (rc)
        Eq2kNP.s = lastEq2kState = IPS_BUSY;
        else
        Eq2kNP.s = lastEq2kState = IPS_ALERT;
        IDSetNumber(&Eq2kNP, NULL);
      }
      return rc;
    }
  }
  return INDI::Telescope::ISNewNumber(dev,name,values,names,n);
}

bool GapersScope::ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n) {
  if(strcmp(dev,getDefaultName())==0) {
    //  This one is for us
    if(!strcmp(name,domeCoordSP.name)) {
      //  client is telling us what to do with co-ordinate requests
      domeCoordSP.s=IPS_OK;
      IUUpdateSwitch(&domeCoordSP,states,names,n);
      //  Update client display
      IDSetSwitch(&domeCoordSP, NULL);
      return true;
    }
    // Dome position in sync with telescope
    if (!strcmp(name, domesyncSP.name)) {
      domesyncSP.s=IPS_OK;
      IUUpdateSwitch(&domesyncSP, states, names, n);
      IDSetSwitch(&domesyncSP, NULL);
      return true;
    }
  }
  //  Nobody has claimed this, so, ignore it
  return INDI::Telescope::ISNewSwitch(dev,name,states,names,n);
}

/**
* Reimplemented from indicom.h/indicom.c in order to open and access
* serial port in nonblocking mode. This is mandatory because commands to
* and responses from PLC must be processed asynchronously.
*/
int GapersScope::tty_connect(const char *device, int bit_rate, int word_size, int parity, int stop_bits, int *fd) {
  int t_fd=-1;
  char msg[80];
  int bps;
  struct termios tty_setting;

  if ( (t_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1)
  {
    *fd = -1;
    return TTY_PORT_FAILURE;
  }

  /* Control Modes
  Set bps rate */
  switch (bit_rate) {
    case 0:
    bps = B0;
    break;
    case 50:
    bps = B50;
    break;
    case 75:
    bps = B75;
    break;
    case 110:
    bps = B110;
    break;
    case 134:
    bps = B134;
    break;
    case 150:
    bps = B150;
    break;
    case 200:
    bps = B200;
    break;
    case 300:
    bps = B300;
    break;
    case 600:
    bps = B600;
    break;
    case 1200:
    bps = B1200;
    break;
    case 1800:
    bps = B1800;
    break;
    case 2400:
    bps = B2400;
    break;
    case 4800:
    bps = B4800;
    break;
    case 9600:
    bps = B9600;
    break;
    case 19200:
    bps = B19200;
    break;
    case 38400:
    bps = B38400;
    break;
    case 57600:
    bps = B57600;
    break;
    case 115200:
    bps = B115200;
    break;
    case 230400:
    bps = B230400;
    break;
    default:
    if (snprintf(msg, sizeof(msg), "tty_connect: %d is not a valid bit rate.", bit_rate) < 0)
    perror(NULL);
    else
    perror(msg);
    return TTY_PARAM_ERROR;
  }
  if ((cfsetispeed(&tty_setting, bps) < 0) ||
  (cfsetospeed(&tty_setting, bps) < 0))
  {
    perror("tty_connect: failed setting bit rate.");
    return TTY_PORT_FAILURE;
  }

  /* Control Modes
  set no flow control word size, parity and stop bits.
  Also don't hangup automatically and ignore modem status.
  Finally enable receiving characters. */
  tty_setting.c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD | HUPCL );
  tty_setting.c_cflag |= (CLOCAL | CREAD | CRTSCTS );

  /* word size */
  switch (word_size) {
    case 5:
    tty_setting.c_cflag |= CS5;
    break;
    case 6:
    tty_setting.c_cflag |= CS6;
    break;
    case 7:
    tty_setting.c_cflag |= CS7;
    break;
    case 8:
    tty_setting.c_cflag |= CS8;
    break;
    default:

    fprintf( stderr, "Default\n") ;
    if (snprintf(msg, sizeof(msg), "tty_connect: %d is not a valid data bit count.", word_size) < 0)
    perror(NULL);
    else
    perror(msg);

    return TTY_PARAM_ERROR;
  }

  /* parity */
  switch (parity) {
    case PARITY_NONE:
    break;
    case PARITY_EVEN:
    tty_setting.c_cflag |= PARENB;
    break;
    case PARITY_ODD:
    tty_setting.c_cflag |= PARENB | PARODD;
    break;
    default:

    fprintf( stderr, "Default1\n") ;
    if (snprintf(msg, sizeof(msg), "tty_connect: %d is not a valid parity selection value.", parity) < 0)
    perror(NULL);
    else
    perror(msg);

    return TTY_PARAM_ERROR;
  }

  /* stop_bits */
  switch (stop_bits) {
    case 1:
    break;
    case 2:
    tty_setting.c_cflag |= CSTOPB;
    break;
    default:
    fprintf( stderr, "Default2\n") ;
    if (snprintf(msg, sizeof(msg), "tty_connect: %d is not a valid stop bit count.", stop_bits) < 0)
    perror(NULL);
    else
    perror(msg);

    return TTY_PARAM_ERROR;
  }
  /* Control Modes complete */

  /* Ignore bytes with parity errors and make terminal raw and dumb.*/
  tty_setting.c_iflag &= ~(PARMRK | ISTRIP | IGNCR | ICRNL | INLCR | IXOFF | IXON | IXANY);
  tty_setting.c_iflag |= INPCK | IGNPAR | IGNBRK;

  /* Raw output.*/
  tty_setting.c_oflag &= ~(OPOST | ONLCR);

  /* Local Modes
  Don't echo characters. Don't generate signals.
  Don't process any characters. Don't block. */
  tty_setting.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN | NOFLSH | TOSTOP);
  tty_setting.c_lflag |=  NOFLSH;

  /* nonblocking read */
  tty_setting.c_cc[VMIN]  = 0;
  tty_setting.c_cc[VTIME] = 0;

  /* now clear input and output buffers and activate the new terminal settings */
  tcflush(t_fd, TCIOFLUSH);
  if (tcsetattr(t_fd, TCSANOW, &tty_setting))
  {
    perror("tty_connect: failed setting attributes on serial port.");
    tty_disconnect(t_fd);
    return TTY_PORT_FAILURE;
  }

  *fd = t_fd;
  /* return success */
  return TTY_OK;
}

void GapersScope::commHandler() {
  unsigned char inbuf[80]; // small buffer for reception, should hold most commands
  std::string rs = ""; // local buffer for holding a complete command

  if (isSimulation()) // No interaction with RS232 in simulation mode
  return;

  if (! isConnected()) // If telescope hardware is not connected, bail out
  return;

  do {
    int rlen=0; // number of chars read by read below
    rlen = read(PortFD, inbuf, 80);
    if (rlen == -1) {
      DEBUGF(INDI::Logger::DBG_SESSION, "comm-handler: serial error reading %s: %d\n", serialConnection->port(), strerror(errno));
      Disconnect();
      return;
    }
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
    // check for output queue and eventually send its contents, one at a time.
    if ((cmdEchoTimeout == 0) && (_writequeue.size() > 0)) {
      DEBUGF(INDI::Logger::DBG_SESSION, "comm-handler: Sending Xpres command <%s>...\n", _writequeue.front().c_str());
      int rv = write(PortFD, (unsigned char*) _writequeue.front().c_str(), _writequeue.front().size());
      if (rv == -1) {
        // error occurred
        DEBUGF(INDI::Logger::DBG_SESSION, "comm-handler: serial error %d during write\n", strerror(errno));
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
  // char *  ps;
  char    cmd[ 8];

  if (msg.empty()) return;

  // Convert 2 fields!
  if( sscanf( msg.c_str(), "%c%s ", &syst, cmd) != 2) {
    DEBUGF(INDI::Logger::DBG_SESSION, "comm-handler: Xpres syntax error: '%s'\n", msg.c_str());
    return;
  }

  // Find out the read command
  // Received ERROR command
  if( strncasecmp( cmd, "mi", 2) == 0)
  {
    int val;
    sscanf( msg.substr(4).c_str(), "%d ", &val);
    DEBUGF(DBG_SCOPE, "comm-handler: Xpres ERROR %c %d\n", syst, val);
    // La documentazione dice che il codice di errore generato dal sistema
    // e comunicato tramite messaggio "mi" può variare tra 400 e 582. Il codice
    // 500 è marcato "READY" e viene inviato quando il sistema si accende.
    // TODO: eventually process error message
  }
  else
  // Received VAR update command
  if( strncasecmp( cmd, "vn", 2) == 0)
  {
    int val, var, whr;
    sscanf( msg.substr(4).c_str(), "%d %d %d ", &val, &var, &whr);
    DEBUGF(DBG_SCOPE, "comm-handler: Xpres EVENT %c %d %d %d\n", syst, var, val, whr);
    // m_signal_event.emit(syst, var, val, whr);
    // TODO: process var update command
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
    DEBUGF(DBG_SCOPE, "comm-handler: Xpres echo received: %s\n", msg.c_str());
    cmdEchoTimeout = 0;
    return;
  }
  else // Received ECHO or unhandled command
  {
    DEBUGF(DBG_SCOPE, "comm-handler: Xpres unhandled command: %s\n", msg.c_str());
    return;
  }

}

void GapersScope::SendMove(int _system, long steps, long m_sq, long m_eq, long m_giri) {
  switch (_system) {
    case '1': raIsMoving = true; break;
    case '2': decIsMoving = true; break;
    default:
    DEBUGF(INDI::Logger::DBG_SESSION, "XpresIF: requested movement on non-existent system %c.\n", _system);
    return;
    break;
  }
  if( abs( m_giri ) > 0)	{
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
