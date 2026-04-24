* **************************************************************************
*  Programma per interprete Xpress:	ASSEALFA.CMD
*
*  Programmazione EEPROM per movimenti telescopio in ASCENSIONE RETTA
*
*  Versione:	0.90 beta test
*  Autore:	Maurizio Serrazanetti - Gruppo Astrofili Persicetani
*  Data:		04/09/96
*  Ultima mod:	06/09/01

*  Modification history:
*
*  SM 21/07/97	Added controls to manual movement input.
*		Now input nr.4 (iVelMove) instruct program to move with
*		factor setted by var 13 (MoveFct).
*  SM 06/04/98	Added public variabile to recalculate correction speeds.
*  SM 03/06/98	moved pointing output signal from 15 to pos 9
*  SM 21/06/98	Added algoritm to keep moving signal on till both sistems
*		stops.
*  SM 06/09/01	Added code to manage commands via v005, like AsseDelta program.
*  SM 06/09/01	Implemented code to slew beyond 2^23 steps!!!
*		Added vars v046 (GiriQtS), v047 (GiriQtE), v048 (GiriReq),
*		v049 (GiriRead) to store respectively: req. starting quote,
*		req. ending quote, req. number of rounds, number of rounds
*		read so far. First 3 vars are set via CmdArg commands
*		5, 6 and 7 respectively, while v014 (RoundPnt) is used to
*		start rounds-driven slews.
*

*  Tabella 1
*
*  Corona:	516 denti
*  Riduttore:	1:12
*
* ÉÍÍÍÍÍÍÍÑÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÑÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍ»
* º       ³       Frequenze           ³     Cambi frequenza       º
* º Passi ³   Reale   ³ Bassa ³  Alta ³ Bassa-Alta ³ Alta-Bassa   º
* ÇÄÄÄÄÄÄÄÅÄÄÄÄÄÄÄÄÄÄÄÅÄÄÄÄÄÄÄÅÄÄÄÄÄÄÄÅÄÄÄÄÄÄÄÄÄÄÄÄÅÄÄÄÄÄÄÄÄÄÄÄÄÄÄ¶
* º 25600 ³ 1283.5059 ³  1283 ³  1284 ³  10017 ms  ³   9785 ms    º
* º 12800 ³  641.7529 ³   641 ³   642 ³  10017 ms  ³   3287 ms    º
* º  6400 ³  320.8765 ³   320 ³   321 ³  10018 ms  ³   1412 ms    º
* ÈÍÍÍÍÍÍÍÏÍÍÍÍÍÍÍÍÍÍÍÏÍÍÍÍÍÍÍÏÍÍÍÍÍÍÍÏÍÍÍÍÍÍÍÍÍÍÍÍÏÍÍÍÍÍÍÍÍÍÍÍÍÍÍ¼
*
*

*
** *************************************************************************
**	INIZIO PROGRAMMA
** *************************************************************************
*

	!	1			// Pone identificatore sistema a 1:
*					// solo il rack 1 riceve il programma
	pe				// Setta fase di esecuzione diretta dei comandi
	nw				// Cancella in EýPROM programma precedente
	p				// Setta fase di programmazione

	var	1			// Risoluzione variabili
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
*	==	ReqCorrE	v005	// Richiesta dis/attivazione correzione Est (?)
*	==	ReqCorrW	v006	// Richiesta dis/attivazione correzione Ovest (?)
	==	ReqComet	v007	// Richiesta dis/attivazione procedura inseguimento cometario
	==	ReqPunta	v008	// Richiesta dis/attivazione procedura puntamento
	==	Command	v009	// Codice comando da eseguire
	==	CmdArg	v010	// Argomento comando (v009)
	==	CorrFctL	v011	// Fattore moltiplicativo per correzioni lente
	==	CorrFctV	v012	// Fattore moltiplicativo per correzioni veloci
	==	MoveFct	v013	// Fattore moltiplicativo per movimenti manuali (SM 21/07/97)
	==	RoundPnt	v014	// Nø giri per puntamenti > 2^23 passi
	==	StepsPnt	v015	// Nø passi puntamento (se < 0 => puntamento EST)
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
	==	CorrFLE	v024	// Frequenza correzione EST Lenta
	==	CorrFLW	v025	// Frequenza correzione OVEST Lenta
	==	CorrFVE	v026	// Frequenza correzione EST Veloce
	==	CorrFVW	v027	// Frequenza correzione OVEST Veloce
	==	VelPunta	v028	// Velocit… massima puntamento
*	==		v029
	==	SidFreqA	v030	// Valore frequenza siderale ALTA
	==	SidFreqB	v031	// Valore frequenza siderale BASSA
	==	SidTimeA	v032	// Tempo timer frequenza siderale ALTA
	==	SidTimeB	v033	// Tempo timer frequenza siderale BASSA
*	==	MoveFLE	v034	// Frequenza movimento manuale EST
*	==	MoveFLW	v035	// Frequenza movimento manuale OVEST
	==	MoveFVE	v036	// Frequenza movimento manuale EST
	==	MoveFVW	v037	// Frequenza movimento manuale OVEST
*	==		v038

	==	SigFreqA	v040	// Valore dell'uscita da settare durante freq. ALTA
	==	SigFreqB	v041	// Valore dell'uscita da settare durante freq. BASSA
	==	ProgReg	v042	// Registro programma: utilizzato a bit
	==	StatoMot	v043	// Stato motore: 3 = fermo, 4 = in moto
	==	ProgRegT	v044	// Registro programma sistema 2

	==	GiriQtS	v046	// Quota iniziale motore per movimento con giri
	==	GiriQtE	v047	// Quota finale motore per movimento con giri
	==	GiriReq	v048	// Numero giri riciesto
	==	GiriRead	v049	// Numero giri letto

	==	Orologio	v050	// Orologio interno centralina
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
	==	iVelMove	%0000000000010000	//  4	Velocit… movimento manuale 30x (SM 21/07/97)
	==	iCorrE	%0000000000100000	//  5	Pulsante correzione A.R. (EST)
	==	iCorrW	%0000000001000000	//  6	Pulsante correzione A.R. (OVEST)
	==	iVelCorr	%0000000010000000	//  7	Velocit… correzione 10x
*	==	i	%0000000100000000	//  8
*	==	i	%0000001000000000	//  9
*	==	i	%0000010000000000	// 10
*	==	i	%0000100000000000	// 11
*	==	i	%0001000000000000	// 12
	==	iSect1	%0010000000000000	// 13	Sensore di settore 1 (A.R.)
	==	iSect2	%0100000000000000	// 14	Sensore di settore 2 (A.R.)
	==	iSect3	%1000000000000000	// 15	Sensore di settore 3  (A.R.)

*	==	hi	%0000000000000001	// 16
	==	hiStopE	%0000000000000010	// 17	Fine corsa motore (EST)
	==	hiStopW	%0000000000000100	// 18	Fine corsa motore (OVEST)
*	==	hi	%0000000000001000	// 19
*
**      Uscite --------------------------------------------------------------
*
	==	uMainPrg	%0000000000000001	//  0	Luce motore in 'Main Program'
	==	uFreqA	%0000000000000010	//  1	Luce frequenza bassa
	==	uFreqB	%0000000000000100	//  2	Luce frequenza alta
	==	uSidFreq	%0000000000001000	//  3	Luce motore a velocit… siderale
	==	uMoveVel	%0000000000010000	//  4	Luce movimento manuale  (SM 21/07/97)
	==	uCorrFrE	%0000000000100000	//  5	Luce motore a velocit… correzione A.R. (EST)
	==	uCorrFrW	%0000000001000000	//  6	Luce motore a velocit… correzione A.R. (OVEST)
	==	uCorrVel	%0000000010000000	//  7	Luce correzione veloce
*	==	u	%0000000100000000	//  8
	==	uSignalP	%0000001000000000	//  9	SM 03/06/98: moved pointing signal
*	==	u	%0000010000000000	// 10
	==	uComFreq	%0000100000000000	// 11	Luce motore a velocit… cometaria (A.R.)
*	==	u	%0001000000000000	// 12
	==	uPuntaE	%0010000000000000	// 13	Luce motore a velocità puntamento (A.R.) EST
	==	uPuntaW	%0100000000000000	// 14	Luce motore a velocità puntamento (A.R.) OVEST
*	==	uSignalP	%1000000000000000	// 15	Comando lampeggiatore puntamento

*	==	uWaitPrg	%0101010101010101	//
	==	uWaitPrg	%1001111110010110	//
	==	ioAll	%1111111111111111	// Tutto ON

	==	uCRamp	%1110000100011110	// Uscite indicazione calcolo rampa
	==	uHaltPrg	%0001001100011111	// Uscite indicazione fine programma

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
	==	tChangeF	1	// Cambio frequenza siderale
	==	tWait	2	// Attesa innesco motore (alla partenza programma)
	==	tCorrE	3	// Simulazione correzione EST (sarà tolto!)
	==	tCorrW	4	// Simulazione correzione OVEST (sarà tolto!)
	==	tPunta	5	// Timer per puntamento
	==	tStrobOn	6	// Timer lampeggiatore puntamento ON
	==	tStrobOf	7	// Timer lampeggiatore puntamento OFF
*
** **************************************************************************
**	DEFINIZIONE VALORI GENERALI PROGRAMMA
** **************************************************************************
*

	==	uCorrOff	%0000000011110000	// Spegnimento luci correzioni

*
**      Assegnamenti --------------------------------------------------------
*

def0	cf	500			// Setta corrente di fase (MASSIMA 7.0 A - 4.8 A per fase)

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

*	v$	2	1	"A.R. pronta"

*	// TEST TEST TEST
*	sv	19	0
*	sev	19	1	1
*	scs	19	sTtestT

*fermaT	j	fermaT			// NON ANDARE OLTRE!!!!

*sTtestT	vv	1	1	1
*		rx	19	1	// Ritorna e riabilita


*	// Se la variabile 1 diventa 100 => FINE PROGRAMMA!
def1	sv	ReqEnd	0
	sev	ReqEnd	1	100
	scs	ReqEnd	EndProg0

*	// Se la variabile 2 diventa 1 => FERMA MOTORE!
	sv	ReqStop	0
	sev	ReqStop	1	1
	scs	ReqStop	sStopM0
*	scs	ReqStop	sStopM1		// TEST TEST TEST

*	// Se la variabile 3 diventa 1 => TRASMETTI PASSI MOTORE
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

*	// Se la variabile 7 diventa 1 => INNESCA INSEGUIMENTO COMETARIO
	sv	ReqComet	0
	sev	ReqComet	1	1
	scs	ReqComet	sSetComF

*	// Se la variabile 8 diventa 1 => INNESCA PROCEDURA PUNTAMENTO
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

def2	tinh	hiStopE	1	EndProg0	// Fine corsa EST
	tinh	hiStopW	1	EndProg0	// Fine corsa OVEST

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

	v$	2	1	"AsseAlfa v. 0.90 BT"
	v$	2	1	"A.R. pronta"

	so	SigFreqA
lMain0	so	uMainPrg			// Setta uscita
	anv	ProgReg	mSwapFrq	ProgReg	// Resetta registro swap

*	Partenza motore e ciclo infinito
	v	FreqA			// Setta velocit… siderale
	mvc+				// Muove in free-running
*	Setta timer cambio frequenza
	ist	TimeA	tChangeF	sSwapFrq

lMain1	bvs	ProgReg	rSwapFrq	lMain0	// Test registro di swap
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

	orv	ProgReg	rSwapFrq	ProgReg	// Setta registro swap

	so	SigFreqA			// Setta led frequenza
	rst	tChangeF			// Ritorno da interrupt
*	j	lMain0

*
** LAMPEGGIATORE PUNTAMENTO -------------------------------------------------
*

sStrobOn	ro	uSignalP			// Spegne lampeggiante
	ist	300	tStrobOf	sStrobOf
	rst	tStrobOn			// Ritorno da interrupt

sStrobOf	so	uSignalP			// Accende lampeggiante
	ist	300	tStrobOn	sStrobOn
	rst	tStrobOf			// Ritorno da interrupt

*
** **************************************************************************
**	SUBROUTINEs
** **************************************************************************
*

*
** **************************************************************************
*	Subroutines CORREZIONI

*
** CORREZIONE SIDERALE EST (DA PULSANTE) ------------------------------------
*

sCorrE	js	sTimeOff			// Disabilita swapFreq e altri tasti
	so	uCorrFrE			// Setta uscita
*	FERMO MOTORE ???
	mvc-				// Inversione movimento
	ois	iVelCorr	lVeloE		// Se il selettore di velocit… Š chiuso salta
	ois	iVelMove	lMoveE		// Se il selettore di movimento man. Š chiuso salta
	v	CorrFLE			// Velocit… bassa
	j	lStayE
lVeloE	v	CorrFVE			// Velocit… alta
	so	uCorrVel
	j	lStayE
lMoveE	v	MoveFVE			// Velocit… alta
	so	uMoveVel
lStayE	ois	iCorrE	lStayE	// fin tanto che l'ingresso Š chiuso
	v	FreqA
	mvc+				// Inversione movimento
	ls	Orologio
	vq	6	1		// Quota motore
	vv	Orologio	6	3
	js	sTimeOn			// Riabilita correzioni
	ro	uCorrOff			// Resetta uscite
	rtt	iCorrE	0		// Ritorno senza riabilitazione

*
** CORREZIONE SIDERALE OVEST (DA PULSANTE) ----------------------------------
*

sCorrW	js	sTimeOff			// Disabilita swapfreq e altri tasti
	so	uCorrFrW			// Setta uscita

	ois	iVelCorr	lVeloW		// Se il selettore di velocit… Š chiuso salta
	ois	iVelMove	lMoveW		// Se il selettore di movimento man. Š chiuso salta
	v	CorrFLW			// velocit… bassa
	j	lStayW
lVeloW	v	CorrFVW			// velocit… alta
	so	uCorrVel
	j	lStayW
lMoveW	v	MoveFVW			// velocit… alta
	so	uMoveVel
lStayW	ois	iCorrW	lStayW		// while input closed
	v	FreqA

	ls	Orologio
	vq	6	1		// Quota motore
	vv	Orologio	6	3
	js	sTimeOn			// riabilita correzioni
	ro	uCorrOff			// Resetta uscite
	rtt	iCorrW	0		// ritorno senza riabilitazione

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

*	// Se giri > 0 allora il puntamento Š verso OVEST
	iva	GiriReq	0	lGiriW

*	// Rende positivo il nø giri
	avc	GiriReq	mNegativ	GiriReq
	dcc	mNegativ	GiriReq	GiriReq
	j	lGiriE			// Puntamento SUD

lGiriEnd	so	uSignalP			// Spegnimento lampeggiatore puntamento
	js	sTimeOn			// Riabilita salti standard
	sv	GiriQts	0		// Azzera quota iniziale
	sv	GiriQtE	0		// Azzera quota finale
	sv	GiriReq	0		// Azzera numero giri puntamento
	anv	ProgReg	mPunta	ProgReg	// Resetta registro puntamento
	lvs	2	ProgReg	ProgRegT	// Legge registro sistema 2
	bvs	ProgRegT	rPunta	lSig1On	// Test registro puntamento sist. 2
	ro	uSignalP			// Spegnimento lampeggiatore puntamento
lSig1On	sv	RoundPnt	0		// Resetta variabile
	rx	RoundPnt	1		// Ritorna e riabilita

*
** PUNTAMENTO OVEST ---------------------------------------------------------
*

lGiriW	so	uPuntaW			// Setta uscita

	js	sStopMt			// FERMA MOTORE
	acg				// Azzera contatore giri
	sq	GiriQtS			// Setta quota motore iniziale
	v	VelPunta			// Setta velocit… puntamento
	mvc+				// Muovi avanti
	j	lGiriMov

lGiriE	so	uPuntaE			// Setta uscita

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
	v	FreqA			// Setta velocit… siderale
	mvc+				// Ripresa movimento motore
	ro	uPuntaW			// Resetta uscita
	j	lGiriEnd			// Fine puntamento

* ***************************************************************************
*
** ENTRY-POINT comune -------------------------------------------------------
*

sPunta	sv	ReqPunta	0		// Resetta variabile
	js	sTimeOff			// Disabilita salti standard
	orv	ProgReg	rPunta	ProgReg	// Setta registro puntamento
	so	uSignalP			// Accensione lampeggiatore puntamento

	ist	300	tStrobOn	sStrobOn	// TEST TEST TEST 20/06/98
*	// Qui mancano altri controlli ???

*	// Se passi == 0 ERRORE: ritorna!
	ivu	StepsPnt	0	lPuntaEnd

*	// Attesa di sicurezza puntamento
	sdr	5000	tPunta	0
lWaitPun	itd	tPunta	lWaitPun
	dst	tStrobOn
	dst	tStrobOf
	so	uSignalP			// Accensione lampeggiatore puntamento

*	// Se passi > 0 allora il puntamento Š verso OVEST
	iva	StepsPnt	0	lPuntaW

*	// Rende positivo il nø passi
	avc	StepsPnt	mNegativ	StepsPnt
	dcc	mNegativ	StepsPnt	StepsPnt
	j	lPuntaE			// Puntamento EST

lPuntaEnd	js	sTimeOn			// Riabilita salti standard
	anv	ProgReg	mPunta	ProgReg	// Resetta registro puntamento
	lvs	2	ProgReg	ProgRegT	// Legge registro sistema 2
	bvs	ProgRegT	rPunta	lSigOn	// Test registro puntamento sist. 2
	ro	uSignalP			// Spegnimento lampeggiatore puntamento
lSigOn	sv	StepsPnt	0		// Azzera numero passi
	rx	ReqPunta	1		// Ritorna e riabilita

*
** PUNTAMENTO OVEST ---------------------------------------------------------
*

lPuntaW	so	uPuntaW			// Setta uscita

	js	sStopMt			// FERMA MOTORE
	sq	0			// Azzera quota motore
	v	VelPunta			// Setta velocit… puntamento
	ss				// Azzera orologio interno
*	mv+	StepsPnt			// Muovi avanti
	mvs+	StepsPnt			// SM 06/08/01: Muovi avanti...
lFermaW	imm	lFermaW			// SM 06/08/01: attendi fine movimento...

	ls	Orologio			// Legge orologio
	vq	8	1		// Visualizza quota motore
	v	FreqA			// Setta velocit… siderale
	mvc+				// Muovi avanti
	vv	Orologio	8	3
	ro	uPuntaW			// Resetta uscita
	j	lPuntaEnd			// Fine puntamento

*
** PUNTAMENTO EST -----------------------------------------------------------
*

lPuntaE	so	uPuntaE			// Setta uscita

	js	sStopMt			// FERMA MOTORE
	sq	0			// Azzera quota motore
	v	VelPunta			// Setta velocit… puntamento
	ss				// Azzera orologio interno
*	mv-	StepsPnt			// Muovi indietro e ferma
	mvs-	StepsPnt			// SM 06/08/01: Muovi indietro...
lFermaE	imm	lFermaE			// SM 06/08/01: attendi fine movimento...

	ls	Orologio			// Legge orologio
	vq	8	1		// Visualizza quota motore
	v	FreqA			// Setta velocit… siderale
	mvc+				// Muovi avanti
	vv	Orologio	8	3
	ro	uPuntaE			// Resetta uscita
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

s6400	sv	SidFreqA	459
	sv	SidFreqB	460
	sv	SidTimeA	23017
	sv	SidTimeB	1922

	sv	VelPunta	110000		// Velocit… max puntamento

	pa	6400	1		// Passi per giro (sinusoidali)

	fb	200			// Freq. base
	fm	VelPunta	28000		// Freq. massima

	np	50000			// Nø passi rampa

	rt				// Ritorno da subroutine
*
**      12800 PASSI ---------------------------------------------------------
*

s12800	sv	SidFreqA	919
	sv	SidFreqB	920
	sv	SidTimeA	6348
	sv	SidTimeB	7631

	sv	VelPunta	220000		// Velocit… max puntamento

	pa	12800	1		// Passi per giro (sinusoidali)

	fb	200			// Freq. base
	fm	VelPunta	14000		// Freq. massima

	np	50000			// Nø passi rampa

	rt				// Ritorno da subroutine
*
**      25600 PASSI ---------------------------------------------------------
*

s25600	sv	SidFreqA	1796
	sv	SidFreqB	1840
	sv	SidTimeA	23016
	sv	SidTimeB	10256

	sv	VelPunta	440000		// Velocit… puntamento

	pa	25600	1		// Passi per giro (sinusoidali)

	fb	200			// Freq. base
	fm	VelPunta	7000		// Freq. massima

	np	50000			// Nø passi rampa

	rt				// Ritorno da subroutine

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

sCalcVel	mol	FreqA	CorrFctL	CorrFLW	// Correzione lenta OVEST
	dvv	CorrFLW	FreqA	CorrFLE	// Correzione lenta EST

	vv	CorrFLW	1	1
	vv	CorrFLE	1	1

	mol	FreqA	CorrFctV	CorrFVW	// Correzione veloce OVEST
	dvv	CorrFVW	FreqA	CorrFVE	// Correzione veloce EST

	vv	CorrFVW	1	1
	vv	CorrFVE	1	1

	mol	FreqA	MoveFct	MoveFVW	// Movimento man. OVEST (SM 21/07/97)
	dvv	MoveFVW	FreqA	MoveFVE	// Movimento man. EST (SM 21/07/97)

	vv	MoveFVW	1	1
	vv	MoveFVE	1	1

	rt				// Ritorno da subroutine
*
** **************************************************************************
*       Subroutine di COMANDO

sCommand	js	sTimeOff			// Disabilita salti standard
	js	sCorrOff			// Disabilita correzioni
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
*       Subroutine GENERICHE

*
** ABILITA CONDIZIONI DI SALTO PER: -----------------------------------------
**	Timer cambio frequenza
*

sTimeOn	ist	TimeA	tChangeF	sSwapFrq
	rt				// Ritorno da subroutine

*
** DISABILITA CONDIZIONI DI SALTO PER: --------------------------------------
**	Timer cambio frequenza
*

sTimeOff	dst	tChangeF			// Timer cambio frequenza
	rt				// Ritorno da subroutine

*
** ABILITA CONDIZIONI DI SALTO PER: -----------------------------------------
**	Ingresso correzione OVEST
**	Ingresso correzione EST
*

sCorrOn	tin	iCorrE	1	sCorrE	// Correzione EST
	tin	iCorrW	1	sCorrW	// Correzione OVEST
	rt				// Ritorno da subroutine

*
** DISABILITA CONDIZIONI DI SALTO PER: --------------------------------------
**	Ingresso correzione EST
**	Ingresso correzione OVEST
*

sCorrOff	tnt	iCorrE			// Ingresso correzione EST
	tnt	iCorrW			// Ingresso correzione OVEST
	rt				// Ritorno da subroutine

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
	vq	4		1	// Visualizza quota motore
	vv	Orologio	1	1

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
** CORREZIONE SIDERALE EST (DA PULSANTE) ------------------------------------
*

sCorrET	so	uCorrFrE			// Setta uscita
	ois	iVelCorr	lVeloE1		// Se il selettore di velocit… Š chiuso salta
	v	CorrFLE			// Velocit… bassa
	mvc-
	j	lStayE1
lVeloE1	v	CorrFVE			// Velocit… alta
	mvc-
	so	uCorrVel
lStayE1	ois	iCorrE	lStayE1		// fin tanto che l'ingresso Š chiuso
	js	sStopMt			// FERMA MOTORE
	vq	6	1		// Quota motore
	ro	uCorrVel
	ro	uCorrFrE			// Resetta uscita
	rtt	iCorrE	0		// Ritorno senza riabilitazione

*
** CORREZIONE SIDERALE OVEST (DA PULSANTE) ----------------------------------
*

sCorrWT	so	uCorrFrW			// Setta uscita
	ois	iVelCorr	lVeloW1		// Se il selettore di velocit… Š chiuso salta
	v	CorrFLW			// velocit… bassa
	mvc+
	j	lStayW1
lVeloW1	v	CorrFVW			// velocit… alta
	mvc+
	so	uCorrVel
lStayW1	ois	iCorrW	lStayW1		// fin tanto che l'ingresso Š chiuso
	js	sStopMt			// FERMA MOTORE
	vq	6	1		// Quota motore
	ro	uCorrVel
	ro	uCorrFrW			// Resetta uscita
	rtt	iCorrW	0		// ritorno senza riabilitazione

*
** **************************************************************************
*       Subroutine PUNTAMENTO
*
** ENTRY-POINT comune -------------------------------------------------------
*

sPuntaT	so	uSignalP			// Accensione ventole

*	// Qui mancano altri controlli ???

*	// Se passi == 0 ERRORE: ritorna!
	ivu	StepsPnt	0	lPuntaEn1

*	// Attesa di sicurezza puntamento
*	sdr	20000	tPunta	0
*lWaitPu1	itd	tPunta	lWaitPu1

	zc	10			// Azzera contatore
*	sv	StepsPnt	110080		// Passi per 30' = 110080

*	// Temporizzazione
test1	sdr	5000	tPunta	0
lWaitPu1	itd	tPunta	lWaitPu1

	j	lPuntaW1

*	// Temporizzazione
test2	sdr	5000	tPunta	0
lWaitPu2	itd	tPunta	lWaitPu2

	j	lPuntaE1

*	// Se passi > 0 allora il puntamento Š verso OVEST
	iva	StepsPnt	0	lPuntaW1

*	// Rende positivo il nø passi
	avc	StepsPnt	mNegativ	StepsPnt
	dcc	mNegativ	StepsPnt	StepsPnt
	j	lPuntaE1			// Puntamento EST

*	// Tra 10 secondi spegne ventole
lPuntaEn1	sdr	10000	tPunta	uSignalP
	ic	10	2
if15	ivu	ReqPunta	1	test1	// Attesa reset variabile
	vc	10	1	1
	sv	StepsPnt	0		// Azzera numero passi
	rx	ReqPunta	1		// Ritorna e riabilita

*
** PUNTAMENTO OVEST ---------------------------------------------------------
*

lPuntaW1	so	uPuntaW			// Setta uscita

	sq	0			// Azzera quota motore
	v	VelPunta			// Setta velocit… puntamento
	mv+	StepsPnt			// Muovi avanti

	vq	4	1		// Visualizza quota motore
	ro	uPuntaW			// Resetta uscita
	j	test2			// Fine puntamento
*	j	lPuntaEn1			// Fine puntamento

*
** PUNTAMENTO EST -----------------------------------------------------------
*

lPuntaE1	so	uPuntaE			// Setta uscita

	sq	0			// Azzera quota motore
	v	VelPunta			// Setta velocit… puntamento
	mv-	StepsPnt			// Muovi indietro

	vq	4	1		// Visualizza quota motore
	ro	uPuntaE			// Resetta uscita
	j	lPuntaEn1			// Fine puntamento

*
** ARRESTO MOTORE CON RAMPA -------------------------------------------------
*

*	// TEST TEST TEST TEST TEST TEST TEST TEST
sStopM1	js	sTimeOff			// Disabilita swap freq.
	js	sStopMt
	sv	StatoMot	3		// Motore fermo

pipXX	ss
pip10	ls	50
pip11	ivi	v050	120000	pip10
	vv	50	1	52	// vis. orologio
if2	ivu	ReqStop	1	pipXX	// Attesa reset variabile

*if2	ivu	ReqStop	1	if2	// Attesa reset variabile

	js	sTimeOn			// Abilita swap freq.
	sv	StatoMot	4		// Motore in moto
	rx	ReqStop	1		// Ritorna e riabilita

*
** ABILITA CONDIZIONI DI SALTO PER: -----------------------------------------
**		Ingresso correzione EST
**		Ingresso correzione OVEST
*

sCorrOnT	tin	iCorrE	1	sCorrET	// Correzione EST
	tin	iCorrW	1	sCorrWT	// Correzione OVEST
	rt				// Ritorno da subroutine

*
** **************************************************************************
** **************************************************************************
**	FINE PROGRAMMA
*

*EndProg0	so	ioAll		// Setta tutte le uscite
EndProg0	so	uHaltPrg		// Setta tutte le uscite
	js	sStopMt		// Ferma motore
	sv	ReqEnd	0	// Setta var 1 a 0
*	ro	ioAll		// Resetta tutte le uscite
lEndProg	j	lEndProg

	pe			// Setta fase di esecuzione diretta
*				// dei comandi
	eppc			// Salva in EýPROM
	es	def0		// Esegui il programma dalla etichetta
*				// indicata (inizio programma)

