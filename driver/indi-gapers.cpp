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
#include "indicom.h"
#include <inditelescope.h>

const float SIDRATE  = 0.004178;                        /* sidereal rate, degrees/s */
const int   SLEW_RATE =         15;                              /* slew rate, degrees/s */
const int   POLLMS       =      250;                            /* poll period, ms */
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
    capability = TELESCOPE_CAN_SYNC;
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
    addDebugControl();
    return true;
}
/**************************************************************************************
** Client is asking us to establish connection to the device
***************************************************************************************/
bool GapersScope::Connect()
{
    DEBUG(INDI::Logger::DBG_SESSION, "Simple Scope connected successfully!");
    // Let's set a timer that checks telescopes status every POLLMS milliseconds.
    SetTimer(POLLMS);
    return true;
}
/**************************************************************************************
** Client is asking us to terminate connection to the device
***************************************************************************************/
bool GapersScope::Disconnect()
{
    DEBUG(INDI::Logger::DBG_SESSION, "Simple Scope disconnected successfully!");
    return true;
}
/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char * GapersScope::getDefaultName()
{
    return "GAPers Scope";
}
/**************************************************************************************
** Client is asking us to slew to a new position
***************************************************************************************/
bool GapersScope::Goto(double ra, double dec)
{
    targetRA=ra;
    targetDEC=dec;
    char RAStr[64], DecStr[64];
    // Parse the RA/DEC into strings
    fs_sexa(RAStr, targetRA, 2, 3600);
    fs_sexa(DecStr, targetDEC, 2, 3600);

    double raDist, decDist;
    long raSteps, decSteps;
    raDist = rangeDistance((targetRA - currentRA) * 15.0);
    DEBUGF(INDI::Logger::DBG_SESSION, "currentRA: %f targetRA: %f", currentRA, targetRA);

    raSteps = raDist * 220088.2;
    double raSlewTime;
    raSteps = CorrectRA(raSteps, raSlewTime);

    decDist = rangeDistance(targetDEC - currentDEC);
    decSteps = decDist * 192000.0;

    // Mark state as slewing
    TrackState = SCOPE_SLEWING;
    // Inform client we are slewing to a new position
    DEBUGF(INDI::Logger::DBG_SESSION, "Slewing to RA: %s - DEC: %s", RAStr, DecStr);

    char raDistStr[64];
    fs_sexa(raDistStr, raDist, 2, 3600);
    DEBUGF(INDI::Logger::DBG_SESSION, "RA dist: %s RA steps (corrected): %ld", raDistStr, raSteps);
    char decDistStr[64];
    fs_sexa(decDistStr, decDist, 2, 3600);
    DEBUGF(INDI::Logger::DBG_SESSION, "DEC dist: %s DEC steps (uncorrected): %ld", decDistStr, decSteps);
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
    static struct timeval ltv;
    struct timeval tv;
    double dt=0, da_ra=0, da_dec=0, dx=0, dy=0;
    int nlocked;
    /* update elapsed time since last poll, don't presume exactly POLLMS */
    gettimeofday (&tv, NULL);
    if (ltv.tv_sec == 0 && ltv.tv_usec == 0)
        ltv = tv;
    dt = tv.tv_sec - ltv.tv_sec + (tv.tv_usec - ltv.tv_usec)/1e6;
    ltv = tv;
    // Calculate how much we moved since last time
    da_ra = SLEW_RATE *dt;
    da_dec = SLEW_RATE *dt;
    /* Process per current state. We check the state of EQUATORIAL_EOD_COORDS_REQUEST and act acoordingly */
    switch (TrackState)
    {
    case SCOPE_SLEWING:
        // Wait until we are "locked" into positon for both RA & DEC axis
        nlocked = 0;
        // Calculate diff in RA
        dx = targetRA - currentRA;
        // If diff is very small, i.e. smaller than how much we changed since last time, then we reached target RA.
        if (fabs(dx)*15. <= da_ra)
        {
            currentRA = targetRA;
            nlocked++;
        }
        // Otherwise, increase RA
        else if (dx > 0)
            currentRA += da_ra/15.;
        // Otherwise, decrease RA
        else
            currentRA -= da_ra/15.;
        // Calculate diff in DEC
        dy = targetDEC - currentDEC;
        // If diff is very small, i.e. smaller than how much we changed since last time, then we reached target DEC.
        if (fabs(dy) <= da_dec)
        {
            currentDEC = targetDEC;
            nlocked++;
        }
        // Otherwise, increase DEC
        else if (dy > 0)
          currentDEC += da_dec;
        // Otherwise, decrease DEC
        else
          currentDEC -= da_dec;
        // Let's check if we recahed position for both RA/DEC
        if (nlocked == 2)
        {
            // Let's set state to TRACKING
            TrackState = SCOPE_TRACKING;
            DEBUG(INDI::Logger::DBG_SESSION, "Telescope slew is complete. Tracking...");
        }
        break;
    default:
        break;
    }
    char RAStr[64], DecStr[64];
    // Parse the RA/DEC into strings
    fs_sexa(RAStr, currentRA, 2, 3600);
    fs_sexa(DecStr, currentDEC, 2, 3600);
    DEBUGF(DBG_SCOPE, "Current RA: %s Current DEC: %s", RAStr, DecStr );
    NewRaDec(currentRA, currentDEC);
    return true;
}

long GapersScope::CorrectRA( long st, double & tm )
{
  // Ricostruzione del calcolo:
  // Si considerano i passi necessari per portarsi nella nuova posizione (st)
  // Si calcola il tempo totale necessario per coprire la distanza angolare
  // richiesta, alla velocità di puntamento meno la velocità siderale e togliendo
  // infine il tempo necessario per coprire le rampe di accelerazione e
  // decellerazione del motore (calcolo che non mi è chiarissimo)
  // I passi totali vengono quindi corretti aggiungendo i passi necessari
  // a coprire il moto siderale per i tempi di rampa più il tempo di movimento
  // dell'asse : rtn

	const double vs = 919.456;	 // Velocità siderale motore (impulsi al sec.)
	double rtn;
	double tvp;
	// Tempo (sec) totale di rampa (5x perchè la rampa è moltiplicata per 5)
	double tr = 5.0 * ((st > 0L) ? 0.452 : -0.452);
	// Velocità massima di puntamento
	double vp = ((st > 0L) ? 220000.0 : -220000.0);
	// Numero passi richiesti per spostamento
	double steps = (double)((st > 0L) ? st : -st);

	// Tempo (sec) di pemanenza a velocità massima
	tvp = ( steps / (vp - vs)) - tr;

	// printf( "Total time: %7.3lf secs\n", (2.0 * tr + tvp));
	tm = fabs( 2.0 * tr + tvp);	 // Tempo (sec) totale per spostamento asse
	// m_RAslwTm = (long)((2.0 * tr + tvp) * 1000); // Tempo (ms) totale per spostamento asse
	// m_RAslwTm = (m_RAslwTm > 0L) ? m_RAslwTm : -m_RAslwTm; // Correzione direzione

	// Numero passi corretto per velocità siderale
	rtn = steps + (2 * tr * vs + tvp * vs);
	// Correzione direzione
	return (long)((st > 0L) ? rtn : -rtn);
}

long GapersScope::ComputeStepsRA( double distance) {
  // Durante lo spostamento in ascensione retta occorre compensare il moto 
  // siderale che si manifesta nel tempo necessario al movimento dell'asse.

  const double vs = 919.456; // Velocità moto siderale in passi per secondo
  const double vp = 220000;  // Velocità movimento asse in passi per secondo
  const double spd = 220088.2; // Passi motore per grado di spostamento asse

  double tm = 0; // Tempo necessario allo spostamento dell'asse
  double correction = 0; // Passi necessari a compensare il moto siderale occorso durante lo spostamento

  // La componente del moto siderale è sempre positiva (da est ad ovest),
  // pertanto consideriamo il valore assoluto del numero di passi necessari
  // per lo spostamento, memorizzando la direzione per potere alla fine
  // effettuare la correzione nella giusta direzione.
  int direction = ( distance > 0 ? 1 : -1);
  double steps = fabs( distance ) * spd;

  if ( steps > 500000.0 ) {
    // Il movimento richiesto è superiore ai passi necessari per completare
    // le rampe di accelerazione e decellerazione dei motori. Viene calcolata
    // la correzione da applicare per il moto siderale considerando il tempo
    // necessario a completare le rampe sommato a quello necessario per
    // compiere i rimanenti passi a velocità di regime
    steps -= 500000.0;
    // Il tempo di rampa viene calcolato utilizzando la velocità media in passi
    // al secondo tra la velocità di partenza e quella di arrivo. Essendo una
    // rampa lineare il valore dovrebbe essere accurato.
    tm = (steps / vp) + ( 500000 / ((220000-200)/2) );
    steps += 500000.0; // Sommiamo indietro i passi necessari per le rampe ai fini del calcolo del valore di ritorno
  }
  // La correzione applicata è pari al tempo totale di spostamento moltiplicato
  // per la velocità siderale in passi al secondo. NB: l'algoritmo non è preciso
  // perché non viene considerato l'effetto della correzione sul tempo totale
  // necessario al movimento, ma l'errore così introdotto dovrebbe essere
  // abbastanza piccolo da essere trascurabile.
  correction = tm * vs;
  return static_cast<long>(steps + correction + 0.5); // Ritorna il numero di passi corretto arrotondato all'intero più vicino
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
