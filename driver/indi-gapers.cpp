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
    // capability = TELESCOPE_CAN_SYNC;
    SetTelescopeCapability(TELESCOPE_CAN_SYNC, 0); // Telescope can sync and has a single slew rate
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
    // long raSteps, decSteps;

    // Find angular distance between current and target position
    // Distance is then expressed in range -180/180 degrees (short path)
    raDist = rangeDistance((targetRA - currentRA) * 15.0);
    DEBUGF(INDI::Logger::DBG_SESSION, "currentRA: %f targetRA: %f dist: %f corr.dist: %f", currentRA, targetRA, (targetRA - currentRA) * 15.0, raDist);

    // Update movement data for RA (also accounting for sidereal motion )
    // raSteps = raDist * 220088.2;
    // double raSlewTime;
    if (! _setMoveDataRA(raDist)) {
      DEBUG(INDI::Logger::DBG_SESSION, "Error in setting RA axis movement.");
      return false;
    };

    decDist = rangeDistance(targetDEC - currentDEC);
    // decSteps = decDist * 192000.0;
    if (! _setMoveDataDEC(decDist)) {
      DEBUG(INDI::Logger::DBG_SESSION, "Error in setting DEC axis movement.");
      return false;
    };


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
  // return static_cast<long>((steps+0.5) * direction + correction ); // Ritorna il numero di passi corretto arrotondato all'intero più vicino
  // TODO: memorizzare i dati necessari al movimento nelle apposite proprietà della classe
  raMovement.steps = static_cast<long>((steps+0.5) * direction + correction );
  raMovement.startQuote = 0;
  raMovement.endQuote = 0;
  raMovement.rotations = 0;
  if (raMovement.steps > 80*12800) {
    return _rotationsCalc(raMovement.steps, raMovement.startQuote, raMovement.endQuote, raMovement.rotations);
  } else {
    raMovement.time = raMovement.steps / 220000.0;
    return true;
  }
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
  // TODO: memorizzare i dati necessari al movimento nelle apposite proprietà della classe
  decMovement.angle = distance;
  decMovement.steps = static_cast<long>((steps+0.5) * direction );
  decMovement.startQuote = 0;
  decMovement.endQuote = 0;
  decMovement.rotations = 0;
  decMovement.time = tm;
  if (decMovement.steps > 80*12800) {
    return _rotationsCalc(decMovement.steps, decMovement.startQuote, decMovement.endQuote, decMovement.rotations);
  } else {
    decMovement.time = decMovement.steps / 220000.0;
    return true;
  }
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
