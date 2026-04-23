* **************************************************************************
*  Programma per interprete Xpress:	ASSEDELT.CMD
*
*  Programmazione EEPROM per movimenti telescopio in DECLINAZIONE
*
*  Versione:	0.90 beta test
*  Autore:	Maurizio Serrazanetti - Gruppo Astrofili Persicetani
*  Data:		04/10/96
*  Ultima mod:	06/09/01
*
*  Modifications history:
*
*  SM 02/11/97	Table 1 modify to reflect new mechanical factors.
*  SM 10/11/97	Added control to different correction speeds, reading inputs
*		from system 1.
*  SM 06/04/98	Added I/O lines, timers and variables to control filter
*		wheel, focuser and dome!
*  SM 07/04/98	Added Dome controls over command N. 1
*  SM 03/06/98	Moved pointing output signal from 15 to pos 9
*  SM 04/06/98	Signal disposition changed to reflect the real wires cabling
*  SM 21/06/98	Added algoritm to keep moving signal on till both sistems
*		stops.
*  SM 28/06/98	Signal disposition definitely set.
*  SM 07/02/99	Dome control program modified: to stop moving we need both 2
*		inputs reset.
*  SM 03/03/99	ALL SENSOR-GUIDED DOME CODE DELETED!!!
*		This version use a time based algorithm to slew dome to
*		requested azimuth
*  SM 29/04/01	Added feedback for dome rotation finished ("Dstop").
*  SM 05/06/01	Modified feedback for dome rotation (vv CmdArg 9 2).
*  SM 06/08/01	Asynchronous movement!!! To enable dome rotation while
*		slewing motor, we use mvs instead of mv macro to leave
*		CPU free to execute dome code...!?!
*  SM 12/08/01	Start to develope a not 40 deg limited version...!!!
*  SM 03/09/01	Implemented code to slew beyond 2^23 steps!!!
*		Added vars v046 (GiriQtS), v047 (GiriQtE), v048 (GiriReq),
*		v049 (GiriRead) to store respectively: req. starting quote,
*		req. ending quote, req. number of rounds, number of rounds
*		read so far. First 3 vars are set via CmdArg commands
*		5, 6 and 7 respectively, while v014 (RoundPnt) is used to
*		start rounds-driven slews.
*  SM 06/09/01	Added code on sGiri to manage Slewing lamp.
*

*
*  Tabella 1
*
*	Corona:		360 denti
*	Riduttore:	1:15
*
*	ÉÍÍÍÍÍÍÍÑÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÑÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍ»
*	º       ³       Frequenze           ³     Cambi frequenza       º
*	º Passi ³   Reale   ³ Bassa ³  Alta ³ Bassa-Alta ³ Alta-Bassa   º
*	ÇÄÄÄÄÄÄÄÅÄÄÄÄÄÄÄÄÄÄÄÅÄÄÄÄÄÄÄÅÄÄÄÄÄÄÄÅÄÄÄÄÄÄÄÄÄÄÄÄÅÄÄÄÄÄÄÄÄÄÄÄÄÄÄ¶
*	º 25600 ³ 1604.3823 ³  1604 ³  1605 ³  10017 ms  ³  16182 ms    º
*	º 12800 ³  802.1912 ³   802 ³   803 ³  16183 ms  ³   3825 ms    º
*	º  6400 ³  401.0956 ³   401 ³   402 ³  16189 ms  ³   1711 ms    º
*	ÈÍÍÍÍÍÍÍÏÍÍÍÍÍÍÍÍÍÍÍÏÍÍÍÍÍÍÍÏÍÍÍÍÍÍÍÏÍÍÍÍÍÍÍÍÍÍÍÍÏÍÍÍÍÍÍÍÍÍÍÍÍÍÍ¼
*
*	Questa tabella comunque non serve per la Declinazione, dal momento
*	che non si sposta!
*	Servono comunque a calcolare le velocit… di correzione.
*

*
** *************************************************************************
**	INIZIO PROGRAMMA
** *************************************************************************
*

	!	2		// Pone identificatore sistema a 2:
*				// solo il rack 2 riceve il programma
	pe			// Setta fase di esecuzione diretta dei comandi
	nw			// Cancella in EýPROM programma precedente
	p			// Setta fase di programmazione

	var	1		// Risoluzione variabili
*
** **************************************************************************
**	DEFINIZIONE VARIABILI SIMBOLICHE PROGRAMMA
** **************************************************************************
*
*
**      Variabili controllabili da tastiera o PC (RS-232) -------------------
*
	==	ReqEnd	v001	// Richiesta di fine programma!!!
	==	ReqStop	v002	// Richiesta di stop motore con rampa
	==	ReqQuote	v003	// Richiesta trasmissione quota motore
	==	ReqCalc	v004	// Richiesta di ricalcolo velocit… correzione
	==	ReqCmdEx	v005	// Richiesta esecuzione comando (v009)
	==	ReqFocus	v006	// Richiesta dis/attivazione movimento focheggiatore
	==	ReqComet	v007	// Richiesta dis/attivazione procedura inseguimento cometario
	==	ReqPunta	v008	// Richiesta dis/attivazione procedura puntamento [-2^23;2^23 -1]
	==	Command	v009	// Codice comando da eseguire
	==	CmdArg	v010	// Argomento comando (v009)
	==	CorrFctL	v011	// Fattore moltiplicativo per correzioni lente
	==	CorrFctV	v012	// Fattore moltiplicativo per correzioni veloci
	==	MoveFct	v013	// Fattore moltiplicativo per movimenti manuali (SM 21/07/97)
	==	RoundPnt	v014	// Nø giri per puntamenti > 2^23 passi
	==	StepsPnt	v015	// Nø passi puntamento (se < 0 => puntamento SUD)
	==	ComFreqA	v016	// Valore frequenza cometaria ALTA
	==	ComFreqB	v017	// Valore frequenza cometaria BASSA
	==	ComTimeA	v018	// Tempo timer frequenza cometaria ALTA
	==	ComTimeB	v019	// Tempo timer frequenza cometaria BASSA
*
**      Variabili PRIVATE del programma -------------------------------------
*
	==	FreqA	v020	// Valore frequenza ALTA utilizzata
	==	FreqB	v021	// Valore frequenza BASSA utilizzata
	==	TimeA	v022	// Tempo timer frequenza ALTA utilizzata
	==	TimeB	v023	// Tempo timer frequenza BASSA utilizzata
	==	CorrFLS	v024	// Frequenza correzione SUD Lenta
	==	CorrFLN	v025	// Frequenza correzione NORD Lenta
	==	CorrFVS	v026	// Frequenza correzione SUD Veloce
	==	CorrFVN	v027	// Frequenza correzione NORD Veloce
	==	VelPunta	v028	// Velocit… massima puntamento
*	==		v029
	==	SidFreqA	v030	// Valore frequenza siderale ALTA
	==	SidFreqB	v031	// Valore frequenza siderale BASSA
	==	SidTimeA	v032	// Tempo timer frequenza siderale ALTA
	==	SidTimeB	v033	// Tempo timer frequenza siderale BASSA
*	==		v034
*	==		v035
	==	MoveFVS	v036	// Frequenza movimento manuale SUD
	==	MoveFVN	v037	// Frequenza movimento manuale NORD
*	==		v038

	==	SigFreqA	v040	// Valore dell'uscita da settare durante freq. ALTA
	==	SigFreqB	v041	// Valore dell'uscita da settare durante freq. BASSA
	==	ProgReg	v042	// Registro programma: utilizzato a bit
	==	StatoMot	v043	// Stato motore: 3 = fermo, 4 = in moto
	==	ProgRegT	v044	// Registro programma sistema 1
*	==	DomeDirO	v045	// Uscita da settare per movimento Orario cupola
*	==	DomeDirA	v046	// Uscita da settare per movimento Anti-orario cupola

	==	GiriQtS	v046	// Quota iniziale motore per movimento con giri
	==	GiriQtE	v047	// Quota finale motore per movimento con giri
	==	GiriReq	v048	// Numero giri riciesto
	==	GiriRead	v049	// Numero giri letto

	==	Orologio	v050	// Orologio interno centralina
	==	IngrSys1	v051	// Valore ingressi sistema 1
*
** **************************************************************************
**	DEFINIZIONE VALORI SIMBOLICI PROGRAMMA
** **************************************************************************
*
*
**	Ingressi ------------------------------------------------------------
*

*	==	i	%0000000000000001	//  0
	==	iSelPC	%0000000000000010	//  1	Selettore a chiave PC
	==	iSelKeyb	%0000000000000100	//  2	Selettore a chiave tastiera
*	==	i	%0000000000001000	//  3
*	==	i	%0000000000010000	//  4
*	==	i	%0000000000100000	//  5
*	==	i	%0000000001000000	//  6
*	==	i	%0000000010000000	//  7
*	==	i	%0000000100000000	//  8
	==	iCorrS	%0000000000100000	//  5	Pulsante correzione Dec. (SUD)
	==	iCorrN	%0000000001000000	//  6	Pulsante correzione Dec. (NORD)
*	==	iDomeL	%0000000010000000	//  7	Sensore fessura cupola sinistro
*	==	i	%0000000100000000	//  8
	==	iFilter	%0000001000000000	//  9	Sensore filtro in posizione
	==	iFilter1	%0000010000000000	// 10	Sensore 1 pos. routa filtri
	==	iFilter2	%0000100000000000	// 11	Sensore 2 pos. routa filtri
	==	iFilter3	%0001000000000000	// 12	Sensore 3 pos. routa filtri
*	==	iDomeR	%0010000000000000	// 13	Sensore fessura cupola destro
*	==	iFltSet	%0100000000000000	// 14	Fase zero routa filtri
*	==	iFltStop	%1000000000000000	// 15	Fase arresto ruota filtri

*	==	hi	%0000000000000001	// 16
	==	hiStopS	%0000000000000010	// 17	Fine corsa motore (SUD)
	==	hiStopN	%0000000000000100	// 18	Fine corsa motore (NORD)
*	==	hi	%0000000000001000	// 19
*
**      Uscite --------------------------------------------------------------
*
*	==	uMainPrg	%0000000000000001	//  0	Luce motore in 'Main Program'
*	==	uFocusI	%0000000000000010	//  1	Muovi focheggiatore Intra-focale
*	==	uFreqB	%0000000000000100	//  2	Luce frequenza bassa
*	==	uSidFreq	%0000000000001000	//  3	Luce motore a velocit… siderale
*	==	uMoveVel	%0000000000010000	//  4	Luce movimento manuale
*	==	uCorrFrS	%0000000000100000	//  5	Luce motore a velocit… correzione Dec. (SUD)
*	==	uCorrFrN	%0000000001000000	//  6	Luce motore a velocit… correzione Dec. (NORD)
*	==	uDomeA	%0000000001000000	//  6	Muovi cupola in senso Anti-orario
*	==	uCorrVel	%0000000010000000	//  7	Luce correzione veloce
*	==	uDomeO	%0000000010000000	//  7	Muovi cupola in senso Orario
*	==	uFreqA	%0000000100000000	//  8	Luce frequenza alta
*	==	uSignalP	%0000001000000000	//  9	Comando lampeggiatore puntamento
*	==	uFocusE	%0000010000000000	// 10	Muovi focheggiatore Extra-focale
*	==	uComFreq	%0000100000000000	// 11	Luce motore a velocit… cometaria (Dec.)
*	==	uFilter	%0001000000000000	// 12	Muovi routa filtri
*	==	uPuntaS	%0010000000000000	// 13	Luce motore a velocità puntamento (Dec.) NORD
*	==	uPuntaN	%0100000000000000	// 14	Luce motore a velocità puntamento (Dec.) SUD

	==	uMainPrg	%0000000000000001	//  0	Luce motore in 'Main Program'
	==	uFocusI	%0000000000000010	//  1	Muovi focheggiatore Intra-focale
*	==	u	%0000000000000100	//  2
*	==	u	%0000000000001000	//  3
*	==	u	%0000000000010000	//  4
*	==	u	%0000000000100000	//  5
	==	uDomeO	%0000000001000000	//  7	Muovi cupola in senso Orario
	==	uDomeA	%0000000010000000	//  6	Muovi cupola in senso Anti-orario
*	==	u	%0000000100000000	//  8
	==	uSignalP	%0000001000000000	//  9	Comando lampeggiatore puntamento
	==	uFocusE	%0000010000000000	// 10	Muovi focheggiatore Extra-focale
*	==	u	%0000100000000000	// 11
	==	uFilter	%0001000000000000	// 12	Muovi routa filtri
*	==	u	%0010000000000000	// 13
	==	uDomeMan	%0100000000000000	// 14	Esclusione rotazione manuale cupola
*	==	u	%1000000000000000	// 15

*	==	uWaitPrg	%0101010101010101	//
	==	uWaitPrg	%0111100100010011	//
	==	ioAll	%1111111111111111	// Tutto ON

	==	uCRamp	%1110000100011110	// Uscite indicazione calcolo rampa
	==	uHaltPrg	%0001001100011111	// Uscite indicazione fine programma

**	Uscite usate da programma ma inutili...

	==	uFreqB	%0000000000010000	//	Luce frequenza bassa
	==	uSidFreq	%0000000000010000	//	Luce motore a velocit… siderale
	==	uMoveVel	%0000000000010000	//	Luce movimento manuale
	==	uCorrFrS	%0000000000010000	//	Luce motore a velocit… correzione Dec. (SUD)
	==	uCorrFrN	%0000000000010000	//	Luce motore a velocit… correzione Dec. (NORD)
	==	uCorrVel	%0000000000010000	//	Luce correzione veloce
	==	uFreqA	%0000000000010000	//	Luce frequenza alta
	==	uComFreq	%0000000000010000	//	Luce motore a velocit… cometaria (Dec.)
	==	uPuntaS	%0000000000010000	//	Luce motore a velocità puntamento (Dec.) NORD
	==	uPuntaN	%0000000000010000	//	Luce motore a velocità puntamento (Dec.) SUD

*
**      Registri (a bit) ----------------------------------------------------
*
	==	rSidFreq	%00000000000000000001000	// Velocit… siderale attiva
	==	mSidFreq	%11111111111111111110111	// Mask (XOR)

	==	rSwapFrq	%00000000000000000000100	// Cambiamento frequenza avvenuto
	==	mSwapFrq	%11111111111111111111011	// Mask (XOR)

	==	rComFreq	%00000000000000100000000	// Velocit… cometaria attiva
	==	mComFreq	%11111111111111011111111	// Mask

	==	rPunta	%00000000001000000000000	// Fase puntamento attivata
	==	mPunta	%11111111110111111111111	// Mask

	==	mNegativ	%11111111111111111111111	// Neg to pos number mask
*
**      Timers --------------------------------------------------------------
*
	==	tChangeF	1		// Cambio frequenza siderale
	==	tWait	2		// Attesa innesco motore (alla partenza programma)
	==	tCorrE	3		// Simulazione correzione EST (sarà tolto!)
	==	tCorrW	4		// Simulazione correzione OVEST (sarà tolto!)
	==	tPunta	5		// Timer per puntamento
	==	tFocus	6		// Timer per focheggiatore
	==	tDome	7		// Timer per ingressi cupola
*
** **************************************************************************
**	DEFINIZIONE VALORI GENERALI PROGRAMMA
** **************************************************************************
*

	==	iVelMove	%0000000000010000	// Velocit… correzione > 30x (ingresso letto da sistema 1)
	==	iVelCorr	%0000000010000000	// Velocit… correzione = 10x (ingresso letto da sistema 1)
*	==	uCorrOff	%0000000011110000	// Spegnimento luci correzioni
	==	uCorrOff	%0000000000010000	// Spegnimento luci correzioni
	==	uDomeOA	%0000000011000000	// Reset uscite movimento cupola

*
**      Assegnamenti --------------------------------------------------------
*

def0	cf	220			// Setta corrente di fase (MASSIMA 4.8 A - 2.2 A per fase)

	ro	ioAll			// Resetta tutte le uscite

	sv	CorrFctL	3		// Fattore per correzioni lente
	sv	CorrFctV	10		// Fattore per correzioni veloci
	sv	MoveFct	100		// Fattore per movimento manuale (SM 21/07/97)

	sv	SigFreqA	uFreqA		// Valore uscita freq. ALTA
	sv	SigFreqB	uFreqB		// Valore uscita freq. BASSA

	sv	RoundPnt	0		// Resetta giri puntamento
	sv	StepsPnt	0		// Resetta passi puntamento

	sv	ProgReg	0		// Resetta registro programma

* *** Definizione caratteristiche moto siderale
	sdr	5000	tWait	uWaitPrg
test0	itd	tWait	test0		// tempo attesa inizio calcolo rampa

	js	s12800

	v$	2	1	"Calcolo rampa"
*	so	ioAll
	so	uCRamp
	crl				// Calcola rampa lineare
	pd	5			// Fatt. molt. rampa
	ro	ioAll

*	Variabili moto siderale
	js	sSetSidF			// Settaggio vars moto siderale

*	Variabili moto cometario: predefinizione (= siderale)
	sv	ComFreqA	SidFreqA
	sv	ComFreqB	SidFreqB
	sv	ComTimeA	SidTimeA
	sv	ComTimeB	SidTimeB

	sv	StatoMot	4		// Stato motore: in moto (?)

	sq	0			// Setta quota motore

*
** **************************************************************************
**	DEFINIZIONE SALTI PRODOTTI DA TASTIERA
** **************************************************************************
*

*	v$	2	1	"Declinazione pronta"

*	// TEST TEST TEST
*	sv	19	0
*	sev	19	1	1
*	scs	19	sTtestT

*fermaT	j	fermaT			// NON ANDARE OLTRE!!!!

*sTtestT	vv	1	1	1
*	rx	19	1		// Ritorna e riabilita



* Se la variabile 1 diventa 100 => FINE PROGRAMMA!
def1	sv	ReqEnd	0
	sev	ReqEnd	1	100
	scs	ReqEnd	EndProg0

* Se la variabile 2 diventa 1 => FERMA MOTORE!
	sv	ReqStop	0
	sev	ReqStop	1	1
	scs	ReqStop	sStopM0
*	scs	ReqStop	sStopM1		// TEST TEST TEST

* Se la variabile 3 diventa 1 => TRASMETTI PASSI MOTORE
	sv	ReqQuote	0
	sev	ReqQuote	1	1
	scs	ReqQuote	sSendQuo

* Se la variabile 4 diventa 1 => RICALCOLA VELOCITA' CORREZIONI
	sv	ReqCalc	0
	sev	ReqCalc	1	1
	scs	ReqCalc	sReqCalc

* Se la variabile 5 diventa 1 => ESEGUI COMANDO IN Command CON ARGOMENTO IN CmdArg
	sv	ReqCmdEx	0
	sv	Command	0
	sv	CmdArg	0
	sev	ReqCmdEx	1	1
	scs	ReqCmdEx	sCommand

* Se la variabile 7 diventa 1 => INNESCA INSEGUIMENTO COMETARIO
*	sv	ReqComet	0
*	sev	ReqComet	1	1
*	scs	ReqComet	sSetComF

* Se la variabile 8 diventa 1 => INNESCA PROCEDURA PUNTAMENTO
	sv	ReqPunta	0
	sev	ReqPunta	1	1
	scs	ReqPunta	sPunta
*	scs	ReqPunta	sPuntaT		// TEST TEST TEST

* Se la variabile 14 diventa 1 => INNESCA PROCEDURA PUNTAMENTO via GIRI
	sv	RoundPnt	0
	sev	RoundPnt	1	1
	scs	RoundPnt	sGiri

*
** **************************************************************************
**	DEFINIZIONE SALTI PRODOTTI DA INGRESSI (veloci)
** **************************************************************************
*

def2	tinh	hiStopS	1	EndProg0	// Fine corsa EST
	tinh	hiStopN	1	EndProg0	// Fine corsa OVEST

*
** **************************************************************************
**	DEFINIZIONE SALTI PRODOTTI DA INGRESSI (normali)
** **************************************************************************
*

def3	js	sCorrOn			// Abilita correzioni
*def3	js	sCorrOnT			// Abilita correzioni TEST

*	// Temporaneo: simula puntamento da ingresso (2)
*	tin	iSelKeyb	1	sPuntaTT

*
** **************************************************************************
** **************************************************************************
**	Programma PRINCIPALE
** **************************************************************************
** **************************************************************************
*

*	Attende 3 secondi

	so	uWaitPrg
	sdr	3000	tWait	uWaitPrg
pip0	itd	tWait	pip0

	v$	2	1	"AsseDelta v. 0.90 AT"
	v$	2	1	"Declinazione pronta"

* ---------------------------------------------------------------------------
*	Ci fermiamo qui senza attivare il movimento siderale!!!!
* ---------------------------------------------------------------------------

fermaT	j	fermaT			// NON ANDARE OLTRE!!!!

* ---------------------------------------------------------------------------
*	Ci fermiamo qui senza attivare il movimento siderale!!!!
* ---------------------------------------------------------------------------

	so	SigFreqA
lMain0	so	uMainPrg			// Setta uscita
	anv	ProgReg	mSwapFrq	ProgReg

*	Partenza motore e ciclo infinito
	v	FreqA			// Setta velocit… siderale
	mvc+				// Muove in free-running
*	Setta timer cambio frequenza
	ist	TimeA	tChangeF	sSwapFrq

lMain1	bvs	ProgReg	rSwapFrq	lMain0
	ivi	ReqEnd	100	lMain1
	j	EndProg0

*
** **************************************************************************
**	SALTI PROGRAMMA
** **************************************************************************
*
*
** SCAMBIO FREQUENZE --------------------------------------------------------
*

sSwapFrq	ro	SigFreqA
	xv	FreqA	FreqB		// Scambia frequenza
	xv	TimeA	TimeB		// Scambia tempo
	xv	SigFreqA	SigFreqB		// Scambia uscita
* Setta registro
	orv	ProgReg	rSwapFrq	ProgReg

	so	SigFreqA			// Setta led frequenza
	rst	tChangeF			// Ritorno da interrupt
*	j	lMain0

*
** **************************************************************************
**	SUBROUTINEs
** **************************************************************************
*

* TEST TEST TEST

sCorrSTT	js	sTimeOff			// Disabilita swapFreq e altri tasti
	ss				// Azzera orologio interno
lStaySTT	ois	iCorrS	lStaySTT		// fin tanto che l'ingresso Š chiuso
	ls	Orologio			// Legge orologio
	vv	Orologio	8	3	// Visualizza orologio
	js	sTimeOn			// Riabilita correzioni
	rtt	iCorrS	0		// Ritorno e riabilitazione

*
** **************************************************************************
*	Subroutines CORREZIONI

*
** CORREZIONE SIDERALE SUD (DA PULSANTE) ------------------------------------
*

sCorrS	js	sTimeOff			// Disabilita swapFreq e altri tasti
	so	uCorrFrS			// Setta uscita
	ias	1	IngrSys1		// Lettura ingressi sistema 1
	js	sStopMt			// Ferma motore
	v	CorrFLS			// Velocit… bassa
	mvc-				// Inizia movimento retrogrado
	ios	IngrSys1	iVelCorr	lVeloS	// Se il selettore di velocit… (sys 1) Š chiuso salta
	ios	IngrSys1	iVelMove	lMoveS	// Se il selettore di movimento man. (sys 1) Š chiuso salta
	j	lStayS
lVeloS	v	CorrFVS			// Velocit… alta
	so	uCorrVel
	j	lStayS
lMoveS	v	MoveFVS			// Velocit… alta
	so	uMoveVel
lStayS	ois	iCorrS	lStayS		// fin tanto che l'ingresso Š chiuso
	js	sStopMt			// Ferma motore

	ls	Orologio
	vq	6	1		// Quota motore
	vv	Orologio	6	3

	js	sTimeOn			// Riabilita correzioni
	ro	uCorrOff			// Resetta uscite
	rtt	iCorrS	0		// Ritorno e riabilitazione

*
** CORREZIONE SIDERALE NORD (DA PULSANTE) ----------------------------------
*

sCorrN	js	sTimeOff			// Disabilita swapfreq e altri tasti
	so	uCorrFrN			// Setta uscita
	ias	1	IngrSys1		// Lettura ingressi sistema 1
	js	sStopMt			// Ferma motore
	v	CorrFLN			// velocit… bassa
	mvc+				// inizia movimento
	ios	IngrSys1	iVelCorr	lVeloN	// Se il selettore di velocit… Š chiuso salta
	ios	IngrSys1	iVelMove	lMoveN	// Se il selettore di movimento man. (sys 1) Š chiuso salta
	j	lStayN
lVeloN	v	CorrFVN			// velocit… alta
	so	uCorrVel
	j	lStayN
lMoveN	v	MoveFVN			// velocit… alta
	so	uMoveVel
lStayN	ois	iCorrN	lStayN		// fin tanto che l'ingresso Š chiuso
	js	sStopMt			// Ferma motore

	ls	Orologio
	vq	6	1		// Quota motore
	vv	Orologio	6	3

	js	sTimeOn			// riabilita correzioni
	ro	uCorrOff			// Resetta uscite
	rtt	iCorrN	0		// ritorno e riabilitazione

*
** **************************************************************************
*       Subroutine PUNTAMENTO

sPuntaTT	sv	StepsPnt	-3000		// Setta passi puntamento
	js	sPunta
	rtt	iSelKeyb	0		// Ritorna e riabilita

*
** ENTRY-POINT comune -------------------------------------------------------
*

sGiri	sv	RoundPnt	0		// Reset variabile
	js	sTimeOff			// Disabilita salti standard
	orv	ProgReg	rPunta	ProgReg	// Setta registro puntamento
	so	uSignalP			// Accensione lampeggiatore puntamento

*	// Controllo parametri di esecuzione
	ivu	GiriReq	0	lGiriEnd
	ivu	GiriQtS	0	lGiriEnd
	ivu	GiriQtE	0	lGiriEnd

*	// Se giri fuori range => ERRORE: ritorna!
	ivi	GiriReq	-65000	lGiriEnd
	iva	GiriReq	65000	lGiriEnd

*	// Attesa di sicurezza puntamento
	sdr	5000	tPunta	0
lWaitGir	itd	tPunta	lWaitGir

*	// Se giri > 0 allora il puntamento Š verso NORD
	iva	GiriReq	0	lGiriN

*	// Rende positivo il nø giri
	avc	GiriReq	mNegativ	GiriReq
	dcc	mNegativ	GiriReq	GiriReq
	j	lGiriS			// Puntamento SUD

lGiriEnd	so	uSignalP			// Spegnimento lampeggiatore puntamento
	js	sTimeOn			// Riabilita salti standard
	sv	GiriQts	0		// Azzera quota iniziale
	sv	GiriQtE	0		// Azzera quota finale
	sv	GiriReq	0		// Azzera numero giri puntamento
	anv	ProgReg	mPunta	ProgReg	// Resetta registro puntamento
	lvs	1	ProgReg	ProgRegT	// Legge registro sistema 1
	bvs	ProgRegT	rPunta	lSig1On	// Test registro puntamento sist. 1
	ros	1	uSignalP		// Spegnimento lampeggiatore puntamento
lSig1On	sv	RoundPnt	0		// Resetta variabile
	rx	RoundPnt	1		// Ritorna e riabilita

*
** PUNTAMENTO NORD ----------------------------------------------------------
*

lGiriN	so	uPuntaN			// Setta uscita

	js	sStopMt			// FERMA MOTORE
	acg				// Azzera contatore giri
	sq	GiriQtS			// Setta quota motore iniziale
	v	VelPunta			// Setta velocit… puntamento
	mvc+				// Muovi avanti
	j	lGiriMov

lGiriS	so	uPuntaS			// Setta uscita

	js	sStopMt			// FERMA MOTORE
	acg				// Azzera contatore giri
	sq	GiriQtS			// Setta quota motore iniziale
	v	VelPunta			// Setta velocit… puntamento
	mvc-				// Muove indietro

*	// Attende numero giri
lGiriMov	lcg	GiriRead	GiriReq	lGiriStp
	j	lGiriMov
*lGiriStp	js	sStopMt			// Ferma motore
lGiriStp	mvq	GiriQtE	0		// Muove motore fino alla quota impostata
lGiriWai	imm	lGiriWai			// Attendi arresto motore
	vq	4	1		// Visualizza quota motore
	vv	GiriRead	4	2	// Visualizza giri motore
*	v	FreqA			// Setta velocit… siderale
*	mvc+				// Ripresa movimento motore
	ro	uPuntaN			// Resetta uscita
	j	lGiriEnd			// Fine puntamento

* ***************************************************************************
*
** ENTRY-POINT comune -------------------------------------------------------
*

sPunta	sv	ReqPunta	0		// Resetta variabile
	js	sTimeOff			// Disabilita salti standard
	orv	ProgReg	rPunta	ProgReg	// Setta registro puntamento
	sos	1	uSignalP		// Accensione lampeggiatore puntamento

*	// Qui mancano altri controlli ???

*	// Se passi == 0 ERRORE: ritorna!
	ivu	StepsPnt	0	lPuntaEnd

*	// Attesa di sicurezza puntamento
	sdr	5000	tPunta	0
lWaitPun	itd	tPunta	lWaitPun

*	// Se passi > 0 allora il puntamento Š verso NORD
	iva	StepsPnt	0	lPuntaN

*	// Rende positivo il nø passi
	avc	StepsPnt	mNegativ	StepsPnt
	dcc	mNegativ	StepsPnt	StepsPnt
	j	lPuntaS			// Puntamento SUD

lPuntaEnd	js	sTimeOn			// Riabilita salti standard
	anv	ProgReg	mPunta	ProgReg	// Resetta registro puntamento
	lvs	1	ProgReg	ProgRegT	// Legge registro sistema 1
	bvs	ProgRegT	rPunta	lSigOn	// Test registro puntamento sist. 1
	ros	1	uSignalP		// Spegnimento lampeggiatore puntamento
lSigOn	sv	StepsPnt	0		// Azzera numero passi
	rx	ReqPunta	1		// Ritorna e riabilita

*
** PUNTAMENTO NORD ----------------------------------------------------------
*

lPuntaN	so	uPuntaN			// Setta uscita

	js	sStopMt			// FERMA MOTORE
	sq	0			// Azzera quota motore
	v	VelPunta			// Setta velocit… puntamento
	ss				// Azzera orologio interno
*	mv+	StepsPnt			// Muovi avanti e ferma
	mvs+	StepsPnt			// SM 06/08/01: Muovi avanti...
sPuntaN	imm	sPuntaN			// SM 06/08/01: attendi fine movimento...

	ls	Orologio			// Legge orologio
	vq	8	1		// Visualizza quota motore
	vv	Orologio	8	3	// Visualizza orologio

	ro	uPuntaN			// Resetta uscita
	j	lPuntaEnd			// Fine puntamento

*
** PUNTAMENTO SUD -----------------------------------------------------------
*

lPuntaS	so	uPuntaS			// Setta uscita

	js	sStopMt			// FERMA MOTORE
	sq	0			// Azzera quota motore
	v	VelPunta			// Setta velocit… puntamento
	ss				// Azzera orologio interno
*	mv-	StepsPnt			// Muovi indietro e ferma
	mvs-	StepsPnt			// SM 06/08/01: Muovi indietro...
sPuntaS	imm	sPuntaS			// SM 06/08/01: attendi fine movimento...

	ls	Orologio			// Legge orologio
	vq	8	1		// Visualizza quota motore
	vv	Orologio	8	3	// Visualizza orologio

	ro	uPuntaS			// Resetta uscita
	j	lPuntaEnd			// Fine puntamento

*
** **************************************************************************
**	DEFINIZIONE CARATTERISTICHE MOTO SIDERALE
**	Settaggio variabili, timers ecc in base alla scelta
**	del numero di passi per giro.
*
*
**      6400 PASSI ----------------------------------------------------------
*

s6400	sv	SidFreqA	320
	sv	SidFreqB	321
	sv	SidTimeA	10018
	sv	SidTimeB	1412

	sv	VelPunta	110000  		// Velocit… max puntamento

	pa	6400	1		// Passi per giro (sinusoidali)

	fb	200			// Freq. base
	fm	VelPunta	28000		// Freq. massima

	np	50000			// Nø passi rampa

	rt				// Ritorno da subroutine
*
**      12800 PASSI ---------------------------------------------------------
*

s12800	sv	SidFreqA	641
	sv	SidFreqB	642
	sv	SidTimeA	10017
	sv	SidTimeB	3287

	sv	VelPunta	220000		// Velocit… max puntamento

	pa	12800	1		// Passi per giro (sinusoidali)

	fb	200			// Freq. base
	fm	VelPunta	14000		// Freq. massima

	np	50000			// Nø passi rampa

	rt				// Ritorno da subroutine
*
**      25600 PASSI ---------------------------------------------------------
*

s25600	sv	SidFreqA	1283
	sv	SidFreqB	1284
	sv	SidTimeA	10017
	sv	SidTimeB	9785

	sv	VelPunta	440000  		// Velocit… puntamento

	pa	25600	1		// Passi per giro (sinusoidali)

	fb	200			// Freq. base
	fm	VelPunta	7000		// Freq. massima

	np	50000			// Nø passi rampa

	rt				// Ritorno da subroutine

*
** **************************************************************************
*       Subroutine MOVIMENTO CUPOLA

*
** RIABILITAZIONE SENSORI ---------------------------------------------------
*

*sDomeTim	js	sDomeOn			// Riabilita tutti gli ingressi cupola
*	rst	tDome			// Ritorno

*
** **************************************************************************
*       Subroutine ASSEGNAMENTI VELOCITA`

*
** VELOCITA SIDERALE --------------------------------------------------------
*

sSetSidF	bvs	ProgReg	rSidFreq	lUnsSidF

*	// Setta registro velocit… siderale
	orv	ProgReg	rSidFreq	ProgReg

	sv	FreqA	SidFreqA
	sv	FreqB	SidFreqB
	sv	TimeA	SidTimeA
	sv	TimeB	SidTimeB

	js	sCalcVel			// Calcola correzioni
	so	uSidFreq			// Setta uscita
	rt				// Ritorno da subroutine

*	// Reset registro velocit… cometaria
lUnsSidF	anv	ProgReg	mSidFreq	ProgReg
	ro	uSidFreq			// Resetta uscita
	rt				// Ritorno da subroutine

*
** VELOCITA COMETARIA -------------------------------------------------------
*

sSetComF	sv	ReqComet	0		// Resetta variabile
	js	sSetSidF
	bvs	ProgReg	rComFreq	lUnsComF

*	// Setta registro velocit… cometaria
	orv	ProgReg	rComFreq	ProgReg

	sv	FreqA	ComFreqA
	sv	FreqB	ComFreqB
	sv	TimeA	ComTimeA
	sv	TimeB	ComTimeB

	js	sCalcVel			// Calcola correzioni
	so	uComFreq			// Setta uscita
	rx	ReqComet	1		// Ritorna e riabilita

*	// Reset registro velocit… cometaria
lUnsComF	anv	ProgReg	mComFreq	ProgReg
	ro	uComFreq			// Resetta uscita
	rx	ReqComet	1		// Ritorna e riabilita

*
** **************************************************************************
*       Subroutine di CALCOLO

*
** VELOCITA DI CORREZIONE ---------------------------------------------------
*

sReqCalc	js	sTimeOff			// Disabilita salti standard
	js	sCorrOff			// Disabilita correzioni
	js	sCalcVel			// Subroutine di calcolo velocit…
	js	sCorrOn			// Abilita correzioni
	js	sTimeOn			// Abilita salti standard
	rx	ReqCalc	1		// Ritorna e riabilita

sCalcVel	mol	FreqA	CorrFctL	CorrFLN	// Correzione lenta NORD
	sv	CorrFLS	CorrFLN		// Correzione lenta SUD

	vv	CorrFLN	1	1
	vv	CorrFLS	1	1

	mol	FreqA	CorrFctV	CorrFVN	// Correzione veloce NORD
	sv	CorrFVS	CorrFVN		// Correzione veloce SUD

	vv	CorrFVN	1	1
	vv	CorrFVS	1	1

	mol	FreqA	MoveFct	MoveFVN	// Movimento man. NORD (SM 21/07/97)
	sv	MoveFVS	MoveFVN		// Movimento man. SUD (SM 21/07/97)

	vv	MoveFVN	1	1
	vv	MoveFVS	1	1

	rt				// Ritorno da subroutine
*
** **************************************************************************
*       Subroutine di COMANDO

sCommand	js	sTimeOff			// Disabilita salti standard
	js	sCorrOff			// Disabilita correzioni
	ivu	Command	1	lDomeOn	// Comando cupola
	ivu	Command	2	lDomeSlw	// Comando movimento cupola
	ivu	Command	3	lFilter	// Comando filtri
	ivu	Command	4	lFocus	// Comando focheggiatore
* SM 03/09/01: added commands
	ivu	Command	5	lSetStaQ	// Setta variabile di inizio quota per movimento giri
	ivu	Command	6	lSetEndQ	// Setta variabile di fine quota per movimento giri
	ivu	Command	7	lSetGiri	// Setta variabile di giri motore richiesti

lCmdEnd	sv	Command	0		// Resetta valore comando
	sv	CmdArg	0		// Resetta argomento comando
	js	sCorrOn			// Abilita correzioni
	js	sTimeOn			// Abilita salti standard
	rx	ReqCmdEx	1		// Ritorna e riabilita

*
** ABILITA/DISABILITA CONTROLLO CUPOLA --------------------------------------
*

*	// Abilita movimento cupola automatico
lDomeOn	ivu	CmdArg	0	lDomeOff	// Se arg == 0 disabilita cupola
	so	uDomeMan			// Disabilita comando manuale cupola
	j	lCmdEnd

*	// Disabilita movimento cupola automatico
lDomeOff	ro	uDomeMan			// Abilita comando manuale cupola
	ro	uDomeOA			// Resetta tutte le uscite
	j	lCmdEnd

*
** MUOVE CUPOLA PER TEMPO RICHIESTO -----------------------------------------
*

lDomeSlw	ivi	CmdArg	0	lDomeSlA	// Se arg < 0 => senso anti orario

	so	uDomeO			// Inizia movimento anti-orario
	ist	CmdArg	tDome	sStopDom	// Abilita test timer
	j	lCmdEnd			// Fine comando

lDomeSlA	so	uDomeA			// Inizia movimento orario
*	// Rende positivo il tempo di movimento
	avc	CmdArg	mNegativ	CmdArg
	dcc	mNegativ	CmdArg	CmdArg
	ist	CmdArg	tDome	sStopDom	// Abilita test timer
	j	lCmdEnd			// Fine comando

sStopDom	ro	uDomeOA			// Resetta tutte le uscite
	vv	CmdArg	9	2	// SM 05/06/01: Segnala arresto cupola
	rst	tDome			// Ritorno

*
** POSIZIONA FILTRO RICHIESTO -----------------------------------------------
*

lFilter	sv	Command	0
	v$	2	1	"Muove filtri"
	j	lCmdEnd			// Fine comando

*
** MUOVE FOCHEGGIATORE ------------------------------------------------------
*

lFocus	ivu	CmdArg	0	lCmdEnd	// Se arg == 0 ERRORE: ritorna!
	iva	CmdArg	0	lFocusE	// Se arg > 0 => muovi Extra-focale
*					// Rende positivo l'argomento
	avc	CmdArg	mNegativ	CmdArg
	dcc	mNegativ	CmdArg	CmdArg
	j	lFocusI			// Muovi Intra-focale

lFocusI	so	uFocusI			// Setta uscita x movimento Intra
	sdr	CmdArg	tFocus	uFocusI	// Imposta timer e uscita da resettare
lWaitFI	itd	tFocus	lWaitFI		// Attendi timer
	j	lCmdEnd			// Fine comando
lFocusE	so	uFocusE			// Setta uscita x movimento Extra
	sdr	CmdArg	tFocus	uFocusE	// Imposta timer e uscita da resettare
lWaitFE	itd	tFocus	lWaitFE		// Attendi timer
	j	lCmdEnd			// Fine comando

*
** SETTA VARIBILI PER MOVIMENTO GIRI ----------------------------------------
*

lSetStaQ	sv	GiriQtS	CmdArg		//
	vv	GiriQtS	14	46	// Display variabile settata
	j	lCmdEnd			// Fine comando

lSetEndQ	sv	GiriQtE	CmdArg		//
	vv	GiriQtE	14	47	// Display variabile settata
	j	lCmdEnd			// Fine comando

lSetGiri	sv	GiriReq	CmdArg		//
	vv	GiriReq	14	48	// Display variabile settata
	j	lCmdEnd			// Fine comando

*
** **************************************************************************
*       Subroutines di ABILITAZIONE/DISABILITAZIONE

*
** ABILITA CONDIZIONI DI SALTO PER: -----------------------------------------
**	Timer cambio frequenza
*

*sTimeOn	ist	TimeA	tChangeF	sSwapFrq
*	rt				// Ritorno da subroutine
sTimeOn	rt				// SM 14/11/97: nulla!

*
** DISABILITA CONDIZIONI DI SALTO PER: --------------------------------------
**	Timer cambio frequenza
*

*sTimeOff	dst	tChangeF			// Timer cambio frequenza
*	rt				// Ritorno da subroutine
sTimeOff	rt				// SM 14/11/97: nulla!

*
** ABILITA CONDIZIONI DI SALTO PER: -----------------------------------------
**	Ingresso correzione SUD
**	Ingresso correzione NORD
*

*sCorrOn	tin	iCorrS	1	sCorrSTT	// Correzione SUD TEST TEST TEST
sCorrOn	tin	iCorrS	1	sCorrS	// Correzione SUD
	tin	iCorrN	1	sCorrN	// Correzione NORD
	rt				// Ritorno da subroutine

*
** DISABILITA CONDIZIONI DI SALTO PER: --------------------------------------
**	Ingresso correzione SUD
**	Ingresso correzione NORD
*

sCorrOff	tnt	iCorrS			// Ingresso correzione SUD
	tnt	iCorrN			// Ingresso correzione NORD
	rt				// Ritorno da subroutine

*
** **************************************************************************
*       Subroutine GENERICHE

*
** ARRESTO MOTORE CON RAMPA -------------------------------------------------
*
*	// Interrupt per trasmissione via RS-232
sStopM0	js	sTimeOff			// Disabilita swap freq.
	js	sCorrOff			// Disabilita correzioni
	js	sStopMt
	sv	StatoMot	3		// Motore fermo
if1	ivu	ReqStop	1	if1	// Attesa reset variabile

	js	sTimeOn			// Abilita swap freq.
	js	sCorrOn			// Abilita correzioni
	sv	StatoMot	4		// Motore in moto
	rx	ReqStop	1		// Ritorna e riabilita

*
** TRASMETTI QUOTA MOTORE ---------------------------------------------------
*

*sSendQuo	sv	ReqQuote	0
*	vipr	4	5		// Visualizza passi rampa
*	vdr	4	6		// Visualizza tempo rampa
*	vv	FreqA	4	3	// Visualizza frequenza motore

*sSendQuo	vq	StatoMot	1		// Visualizza quota motore
sSendQuo	ls	Orologio			// Legge orologio interno
	vq	4	1		// Visualizza quota motore
	vv	Orologio	1	1	// Visualizza lettura orologio

	rx	ReqQuote	1		// Ritorna e riabilita

*	// Subroutine
sStopMt	fmd				// Ferma motore con rampa
lStopMt	imm	lStopMt			// Attende fine movimento
*	vq	3	1		// Visualizza quota motore
	rt				// Ritorno da subroutine

*
** **************************************************************************
** **************************************************************************
**	SUBROUTINE DI TEST
*

*
** **************************************************************************
*	Subroutines CORREZIONI

*
** CORREZIONE SIDERALE SUD (DA PULSANTE) ------------------------------------
*


*
** **************************************************************************
** **************************************************************************
**	FINE PROGRAMMA
*

*EndProg0	so	ioAll			// Setta tutte le uscite
EndProg0	so	uHaltPrg			// Setta uscite fine programma
	js	sStopMt			// Ferma motore
	sv	ReqEnd	0		// Setta var 1 a 0
*	ro	ioAll			// Resetta tutte le uscite
lEndProg	j	lEndProg

	pe				// Setta fase di esecuzione
*					// diretta dei comandi
	eppc	Salva in EýPROM
	es	def0			// Esegui il programma dalla etichetta
*					// indicata (inizio programma)

