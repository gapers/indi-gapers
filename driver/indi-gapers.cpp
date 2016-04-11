/*
INDI Developers Manual
Tutorial #2
"Simple Telescope Driver"
We develop a simple telescope simulator.
Refer to README, which contains instruction on how to build this driver, and use it
with an INDI-compatible client.
*/
#include <sys/time.h>
#include <math.h>
#include <memory>
#include "indi-gapers.h"
#include <indicom.h>
#include <inditelescope.h>
#include <libnova.h>
#include <string>
#include <unistd.h>

#include <termios.h>
#define PARITY_NONE    0
#define PARITY_EVEN    1
#define PARITY_ODD     2
#include <fcntl.h>

const float SIDRATE = 0.004178;                /* sidereal rate, degrees/s */
const int   SLEW_RATE = 15;                    /* slew rate, degrees/s */
const int   POLLMS = 250;                      /* poll period, ms */
std::auto_ptr<GapersScope> gapersScope(0);
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
  currentRA  = 0;
  currentDEC = 90;
  // We add an additional debug level so we can log verbose scope status
  DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");
  // capability = TELESCOPE_CAN_SYNC;
  SetTelescopeCapability(TELESCOPE_CAN_SYNC | TELESCOPE_HAS_TIME | TELESCOPE_HAS_LOCATION, 0); // Telescope can sync and has a single slew rate
  // TelescopeCapability cap;
  // cap.canPark = false;
  // cap.canSync = true;
  // cap.canAbort = false;
  // cap.hasLocation = false;
  // cap.hasTime = false;
  // SetTelescopeCapability(&cap);
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
  IUFillNumberVector(&Eq2kNP,Eq2kN,2,getDeviceName(),"EQUATORIAL_COORD","Eq. Coordinates J2000",MAIN_CONTROL_TAB,IP_RW,60,IPS_IDLE);

  // Add Alt Az coordinates
  IUFillNumber(&AaN[AXIS_ALT], "ALT", "Alt (dd:mm:ss)","%010.6m",-90,90,0,0);
  IUFillNumber(&AaN[AXIS_AZ], "AZ", "Az (dd:mm:ss)","%010.6m",0,360,0,0);
  IUFillNumberVector(&AaNP,AaN,2,getDeviceName(),"ALTAZ_COORD","AltAzimuthal Coordinates",MAIN_CONTROL_TAB,IP_RO,60,IPS_IDLE);

  IUFillSwitch(&domesyncS[0], "DOMEAUTO", "Auto", ISS_ON);
  IUFillSwitch(&domesyncS[1], "DOMEMANUAL", "Manual", ISS_OFF);
  IUFillSwitchVector(&domesyncSP, domesyncS, 2, getDeviceName(), "DOME_MOVEMENT", "Dome Movement", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

  addSimulationControl();
  addDebugControl();
  return true;
}
/**************************************************************************************
** Client is asking us to establish connection to the device
***************************************************************************************/
bool GapersScope::Connect()
{
  bool rc=false;

  if (isConnected())
  return true;

  rc=Connect(PortT[0].text, atoi(IUFindOnSwitch(&BaudRateSP)->name));
  if (rc) {
    // Let's set a timer that checks telescopes status every POLLMS milliseconds.
    SetTimer(POLLMS);
    DEBUG(INDI::Logger::DBG_SESSION, "GAPers Scope connected successfully!");
  } else {
    DEBUG(INDI::Logger::DBG_SESSION, "Error setting up connection with GAPers telescope.");
  }

  // Init serial communication handler buffers and state
  _writequeue = std::queue<std::string>();
  _readbuffer = "";
  c_state = STARTWAITING;

  return rc;
}

bool GapersScope::Connect(const char *port, uint16_t baud) {
  int connectrc = 0;
  char errorMsg[MAXRBUF];

  if (isSimulation()) {
    DEBUG(INDI::Logger::DBG_SESSION, "Telescope connected in simulation mode.");
    return true;
  }

  DEBUGF(INDI::Logger::DBG_SESSION, "GAPers Scope connecting to %s at %dbps.", port, baud);
  if ( (connectrc = tty_connect(port, baud, 8, PARITY_EVEN, 1, &PortFD)) != TTY_OK) {
    tty_error_msg(connectrc, errorMsg, MAXRBUF);
    DEBUGF(INDI::Logger::DBG_SESSION, "Failed to connect to port %s. Error: %s", port, errorMsg);
    return false;
  }
  DEBUGF(INDI::Logger::DBG_WARNING, "Port FD: %d", PortFD);
  // TODO: test connection
  DEBUG(INDI::Logger::DBG_SESSION, "Telescope is online.");
  return true;
}

/**************************************************************************************
** Client is asking us to terminate connection to the device
***************************************************************************************/
bool GapersScope::Disconnect()
{
  if (! isSimulation() ) {
    tty_disconnect(PortFD);
    DEBUG(INDI::Logger::DBG_WARNING, "Telescope is offline.");
  }
  DEBUG(INDI::Logger::DBG_SESSION, "GAPers Scope disconnected successfully!");
  return true;
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

  char RAStr[64], DecStr[64];
  // Parse the RA/DEC into strings
  fs_sexa(RAStr, targetRA, 2, 3600);
  fs_sexa(DecStr, targetDEC, 2, 3600);

  double raDist, decDist;

  // Zeroes movement data
  raMovement = AxisMovementParameters();
  decMovement = AxisMovementParameters();

  // Find angular distance between current and target position
  // Distance is then expressed in range -180/180 degrees (short path)
  raDist = rangeDistance((currentRA - targetRA) * 15.0);
  // DEBUGF(INDI::Logger::DBG_SESSION, "currentRA: %f targetRA: %f dist: %f corr.dist: %f", currentRA, targetRA, (targetRA - currentRA) * 15.0, raDist);
  if (raDist != 0) {
    // Update movement data for RA (also accounting for sidereal motion )
    if (! _setMoveDataRA(raDist)) {
      DEBUG(INDI::Logger::DBG_SESSION, "Error in setting RA axis movement.");
      return false;
    }
    SendMove('1', raMovement.steps, raMovement.startQuote, raMovement.endQuote, raMovement.rotations);
  }

  decDist = rangeDistance(currentDEC - targetDEC);
  if (decDist != 0) {
    if (! _setMoveDataDEC(decDist)) {
      DEBUG(INDI::Logger::DBG_SESSION, "Error in setting DEC axis movement.");
      return false;
    }
    SendMove('2', decMovement.steps, decMovement.startQuote, decMovement.endQuote, decMovement.rotations);
  }

  if (raIsMoving || decIsMoving) {
    FinalizeMove();
  }

  // Get movement start time (plus 5 seconds, since start is delayed of that amount by PLC)
  movementStart = time(NULL) + 5;

  // Mark state as slewing
  TrackState = SCOPE_SLEWING;
  // Inform client we are slewing to a new position
  DEBUGF(INDI::Logger::DBG_SESSION, "Slewing to RA: %s - DEC: %s", RAStr, DecStr);

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
  // Update AltAzimuthal Coordinates
  /*
      void ln_get_hrz_from_equ	(	struct ln_equ_posn * 	object,
        struct ln_lnlat_posn * 	observer,
        double 	JD,
        struct ln_hrz_posn * 	position
        )
  */
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
  while (psn.az > 360.) psn.az -= 360.;
  while (psn.az < 0.) psn.az += 360.;
  NewAltAz(psn.alt, psn.az);

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
  if (raMovement.steps > 80*12800) {
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
  if (decMovement.steps > 80*12800) {
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
    // TODO: gestione dell'errore qualora venga richiesto un movimento troppo
    // piccolo
    //    error("lo spostamento lungo deve essere usato solo per movimenti > 1<<23 passi.");
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
  }

}

bool GapersScope::updateProperties()
{

  if(isConnected())
  {
    defineNumber(&Eq2kNP);
    defineNumber(&AaNP);
    defineSwitch(&domesyncSP);
  }
  else
  {
    deleteProperty(Eq2kNP.name);
    deleteProperty(AaNP.name);
    deleteProperty(domesyncSP.name);
  }

  return INDI::Telescope::updateProperties();
}

void GapersScope::NewAltAz(double alt, double az) {
  AaN[AXIS_ALT].value = alt;
  AaN[AXIS_AZ].value = az;
  AaNP.s = IPS_IDLE;
  IDSetNumber(&AaNP, NULL);

  // TODO: Move dome accordingly
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

bool GapersScope::ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
  //  first check if it's for our device
  if(strcmp(dev,getDeviceName())==0)
  {
    if(strcmp(name,"EQUATORIAL_COORD")==0)
    {
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
      if ((ra>=0)&&(ra<=24)&&(dec>=-90)&&(dec<=90))
      {
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

  do {
    int rlen=0; // number of chars read by read below
    rlen = read(PortFD, inbuf, 80);
    if (rlen == -1) {
      DEBUGF(INDI::Logger::DBG_SESSION, "comm-handler: serial error reading %s: %d\n", PortT[0].text, strerror(errno));
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
    if (_writequeue.size() > 0) {
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
  char *  ps;
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
      }
      break;
    }
  }
  else
  // Received ECHO of sent command
  if( strncasecmp( cmd, "tx", 2) == 0)
  {
    DEBUGF(DBG_SCOPE, "comm-handler: Xpres echo received: %s\n", msg.c_str());
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
    if ((raMovement.rotations > 0) && (decMovement.rotations > 0)) {
      SendCommand('0', 14 , 1); // move both systems in the same manner
      return;
    } else if ((raMovement.rotations == 0) && (decMovement.rotations == 0)) {
      SendCommand('0', 8 , 1); // move both systems in the same manner
      return;
    } else {
      SendCommand('1', (raMovement.rotations > 0) ? 14 : 8, 1);
      SendCommand('2', (decMovement.rotations > 0) ? 14 : 8, 1);
    }
  }
  if (raIsMoving) {
    SendCommand('1', (raMovement.rotations > 0) ? 14 : 8, 1);
  }
  if (decIsMoving) {
    SendCommand('2', (decMovement.rotations > 0) ? 14 : 8, 1);
  }
}
