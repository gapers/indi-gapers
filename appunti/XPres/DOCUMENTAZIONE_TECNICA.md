# Documentazione Tecnica - Sistema XPRES per Telescopio

Versione documento: 2.0  
Data: 23 aprile 2026  
Basato su: manuali EVER + sorgenti aggiornati [appunti/XPres/Programmi assi/AsseAlfa.cmd](appunti/XPres/Programmi%20assi/AsseAlfa.cmd) e [appunti/XPres/Programmi assi/AsseDelt.cmd](appunti/XPres/Programmi%20assi/AsseDelt.cmd)

---

## Scopo

Questo documento descrive lo stato attuale (release 0.90) del software XPRES che governa i due assi del telescopio, includendo le estensioni introdotte fino al 2001:

- puntamento oltre il limite 2^23 passi
- comandi remoti con argomento (Command/CmdArg)
- sincronizzazione visuale tra i due sistemi
- gestione cupola/focheggiatore/filtri su asse DEC
- movimento asincrono con macro mvs

---

## Architettura generale

Il sistema rimane basato su due PLC MPP14-01 in cascata:

1. Sistema 1, ID = 1: asse A.R. (AsseAlfa)
2. Sistema 2, ID = 2: asse DEC + servizi ausiliari (AsseDelt)

Ogni PLC esegue codice XPRES residente in E2PROM, con variabili e timer in RAM tamponata.

---

## Programmi installati

| Voce | Asse A.R. | Asse DEC |
|---|---|---|
| File | AsseAlfa.cmd | AsseDelt.cmd |
| Versione | 0.90 beta test | 0.90 beta test |
| Ultima modifica | 06/09/2001 | 06/09/2001 |
| Autore | Maurizio Serrazanetti (GAP) | Maurizio Serrazanetti (GAP) |
| Identificatore sistema | 1 | 2 |

Nota: i nomi file reali sono CamelCase e non più solo maiuscoli.

---

## Principali differenze rispetto alle release precedenti

1. Entrambi i programmi espongono variabili di comando generiche:
   ReqCalc (v004), ReqCmdEx (v005), Command (v009), CmdArg (v010).
2. Puntamento esteso oltre 2^23 con modalità a giri:
   GiriQtS (v046), GiriQtE (v047), GiriReq (v048), GiriRead (v049).
3. Segnale di puntamento spostato su uscita bit 9 (uSignalP).
4. Introduzione di logica di coordinamento tra sistemi tramite ProgRegT (v044), lvs, sos, ros.
5. Asse DEC integra funzioni cupola, focheggiatore e ruota filtri via comando remoto.
6. Uso di mvs+/mvs- per movimenti asincroni durante il puntamento (CPU non bloccata da mv).

---

## Asse A.R. (AsseAlfa)

### Parametri notevoli

- Corrente fase impostata a cf 500.
- MoveFct di default portato a 100.
- Inizializzazione rampa unificata: js s12800, crl, pd 5.
- Nuovi timer di lampeggio puntamento:
  tStrobOn = 6, tStrobOf = 7.

### Funzioni operative

1. Tracking siderale con scambio FreqA/FreqB su interrupt timer tChangeF.
2. Correzioni manuali EST/OVEST con velocità normale/veloce/manuale.
3. Puntamento a passi (sPunta) con mvs+/mvs-.
4. Puntamento a giri (sGiri):
   usa GiriQtS/GiriQtE/GiriReq, conteggio con lcg, arrivo fine quota con mvq.
5. Ricalcolo dinamico velocità (sReqCalc) senza riavvio del programma.
6. Esecuzione comandi remoti (sCommand):
   comandi 5, 6, 7 per impostare quote e giri del puntamento esteso.

### Uscite e segnalazioni

- uSignalP su bit 9 è il lampeggiatore di puntamento.
- uCRamp e uHaltPrg sono pattern diagnostici per fasi di calcolo rampa/fine programma.
- Spegnimento indicatori correzione tramite maschera uCorrOff.

---

## Asse DEC (AsseDelt)

### Stato del main loop

Nel main è presente ancora il blocco:

fermaT: j fermaT

quindi il tracking continuo dell’asse DEC resta volutamente fermo (stato test/sicurezza), mentre rimangono disponibili i salti/eventi e le routine richiamabili.

### Parametri notevoli

- Corrente fase: cf 220.
- MoveFct default: 100.
- tFocus = 6, tDome = 7.
- sTimeOn/sTimeOff sono volutamente no-op (ritorno immediato).

### Funzioni operative asse DEC

1. Correzioni manuali NORD/SUD con lettura ingressi velocità dal sistema 1 (ias 1 IngrSys1).
2. Puntamento a passi (sPunta) con mvs+/mvs-.
3. Puntamento a giri (sGiri) con quote iniziale/finale e giri richiesti.

### Funzioni ausiliarie (solo DEC)

Gestite dalla routine sCommand:

1. Command=1: abilitazione/disabilitazione controllo automatico cupola
   (uDomeMan, reset uscite cupola).
2. Command=2: slew cupola a tempo (CmdArg in ms, segno = direzione), stop con timer tDome.
3. Command=3: comando ruota filtri (placeholder operativo, messaggio "Muove filtri").
4. Command=4: movimento focheggiatore intra/extra focale, durata da CmdArg su timer tFocus.
5. Command=5/6/7: setup variabili per puntamento a giri (come asse A.R.).

Nota: in questa release ReqComet in DEC è definita ma il relativo trigger in def1 risulta commentato.

### Mappa uscite DEC (release 0.90)

La mappa non è più “solo motore DEC”. Parte delle linee è riallocata:

- bit 1: uFocusI
- bit 6: uDomeO
- bit 7: uDomeA
- bit 9: uSignalP
- bit 10: uFocusE
- bit 12: uFilter
- bit 14: uDomeMan

Le uscite storiche di stato motore (uFreqA/uFreqB/uSidFreq/uCorr...) sono mantenute come alias su bit condiviso per compatibilità software.

---

## Variabili pubbliche di controllo

| Variabile | ID | Significato |
|---|---|---|
| ReqEnd | v001 | fine programma (100) |
| ReqStop | v002 | stop motore con rampa |
| ReqQuote | v003 | trasmissione quota/orologio |
| ReqCalc | v004 | ricalcolo frequenze di correzione |
| ReqCmdEx | v005 | esecuzione comando remoto |
| ReqFocus | v006 | richiesta focus (solo DEC) |
| ReqComet | v007 | toggle inseguimento cometario |
| ReqPunta | v008 | puntamento standard a passi |
| Command | v009 | codice comando remoto |
| CmdArg | v010 | argomento comando |
| RoundPnt | v014 | trigger puntamento a giri |
| StepsPnt | v015 | passi puntamento standard |

Variabili aggiunte per puntamento esteso:

- GiriQtS (v046): quota iniziale
- GiriQtE (v047): quota finale
- GiriReq (v048): giri richiesti
- GiriRead (v049): giri letti

---

## Registri e coordinamento tra sistemi

ProgReg (v042) usa bit di stato comuni:

- rSidFreq: tracking siderale attivo
- rSwapFrq: cambio frequenza eseguito
- rComFreq: inseguimento cometario attivo
- rPunta: fase puntamento attiva

ProgRegT (v044) contiene lo snapshot del registro dell’altro PLC.

Uso pratico: mantenere attivo il lampeggiatore di puntamento finché anche l’altro asse è in fase punta.

---

## Frequenze e prestazioni

### Asse A.R. (tabella sorgente)

| Passi/giro | Bassa | Alta | T(B->A) | T(A->B) |
|---|---:|---:|---:|---:|
| 6400 | 459 | 460 | 23017 ms | 1922 ms |
| 12800 | 919 | 920 | 6348 ms | 7631 ms |
| 25600 | 1796 | 1840 | 23016 ms | 10256 ms |

### Asse DEC (tabella sorgente)

| Passi/giro | Bassa | Alta | T(B->A) | T(A->B) |
|---|---:|---:|---:|---:|
| 6400 | 320 | 321 | 10018 ms | 1412 ms |
| 12800 | 641 | 642 | 10017 ms | 3287 ms |
| 25600 | 1283 | 1284 | 10017 ms | 9785 ms |

Nota: in DEC la parte di tracking nel loop principale è bloccata da fermaT.

---

## Sequenze operative utili

### Puntamento standard a passi

1. Scrivere StepsPnt (v015).
2. Scrivere ReqPunta = 1.
3. Il firmware valida segno e modulo, avvia mvs+/mvs-.
4. Al termine resetta stato e ReqPunta.

### Puntamento esteso a giri (oltre 2^23)

1. Command=5, CmdArg=quota iniziale, ReqCmdEx=1.
2. Command=6, CmdArg=quota finale, ReqCmdEx=1.
3. Command=7, CmdArg=giri richiesti, ReqCmdEx=1.
4. RoundPnt=1 per avvio sGiri.

### Slew cupola temporizzato (DEC)

1. Command=1, CmdArg=1 (abilita automatico cupola).
2. Command=2, CmdArg=tempo_ms (segno decide verso).
3. Alla scadenza tDome la routine sStopDom spegne uDomeO/uDomeA.

### Movimento focheggiatore (DEC)

1. Command=4, CmdArg positivo: extra-focale.
2. Command=4, CmdArg negativo: intra-focale.
3. Durata = |CmdArg| ms con timer tFocus.

---

## Sicurezza e limiti

1. Fine corsa hiStop* in def2 portano a EndProg0 (stop immediato).
2. EndProg0 imposta pattern uHaltPrg e blocco in loop infinito.
3. Nei puntamenti a giri viene validato range [-65000, +65000].
4. In DEC, con fermaT attivo, nessun tracking continuo parte dal main.

---

## Riferimenti

- [appunti/XPres/Programmi assi/AsseAlfa.cmd](appunti/XPres/Programmi%20assi/AsseAlfa.cmd)
- [appunti/XPres/Programmi assi/AsseDelt.cmd](appunti/XPres/Programmi%20assi/AsseDelt.cmd)
- [appunti/XPres/Manuali/XPRES.MAN](appunti/XPres/Manuali/XPRES.MAN)
- [appunti/XPres/Manuali/XPRESMAC.MAN](appunti/XPres/Manuali/XPRESMAC.MAN)

---

Documento aggiornato in base ai sorgenti 0.90.
