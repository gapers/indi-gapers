# Guida Rapida - XPRES (release 0.90)

Riferimento operativo sintetico per i sorgenti:

- [appunti/XPres/Programmi assi/AsseAlfa.cmd](appunti/XPres/Programmi%20assi/AsseAlfa.cmd)
- [appunti/XPres/Programmi assi/AsseDelt.cmd](appunti/XPres/Programmi%20assi/AsseDelt.cmd)

---

## Variabili pubbliche principali

| Nome | ID | Uso |
|---|---|---|
| ReqEnd | v001 | chiusura programma (100) |
| ReqStop | v002 | stop con rampa |
| ReqQuote | v003 | telemetria quota/orologio |
| ReqCalc | v004 | ricalcolo velocità correzione |
| ReqCmdEx | v005 | esecuzione comando remoto |
| ReqFocus | v006 | solo DEC, movimento focus |
| ReqComet | v007 | cometario (A.R. attivo, DEC trigger commentato) |
| ReqPunta | v008 | puntamento standard |
| Command | v009 | codice comando |
| CmdArg | v010 | argomento comando |
| RoundPnt | v014 | trigger puntamento a giri |
| StepsPnt | v015 | passi puntamento |

Variabili aggiunte per slew esteso:

- GiriQtS (v046), GiriQtE (v047), GiriReq (v048), GiriRead (v049)

---

## Comandi remoti (Command/CmdArg)

### Comuni (A.R. e DEC)

| Command | Azione |
|---|---|
| 5 | set GiriQtS = CmdArg |
| 6 | set GiriQtE = CmdArg |
| 7 | set GiriReq = CmdArg |

### Solo DEC

| Command | Azione |
|---|---|
| 1 | abilita/disabilita automatico cupola |
| 2 | slew cupola a tempo (segno CmdArg = direzione) |
| 3 | comando ruota filtri (stub/messaggio) |
| 4 | muove focheggiatore (intra/extra) |

Sequenza base esecuzione comando:

1. Scrivere Command.
2. Scrivere CmdArg.
3. Scrivere ReqCmdEx = 1.

---

## Macro chiave da ricordare

### Movimento asse

| Macro | Significato |
|---|---|
| mvc+, mvc- | free-running continuo |
| mv+, mv- | movimento sincrono a passi |
| mvs+, mvs- | movimento asincrono a passi |
| mvq | muove fino a quota target |
| fmd | stop con rampa |
| imm | wait fine movimento |

### Timer e interrupt

| Macro | Significato |
|---|---|
| sdr | timer con uscita associata |
| ist | timer interrupt verso label |
| itd | loop di attesa timer |
| dst | disabilita timer |
| rst | ritorno da interrupt |

### I/O locali e cross-system

| Macro | Significato |
|---|---|
| so, ro | set/reset uscite locali |
| sos, ros | set/reset uscite su altro sistema |
| ias | acquisisce ingressi di altro sistema |
| lvs | legge variabile da altro sistema |

### Flusso e confronto

| Macro | Significato |
|---|---|
| sev/scs | trigger su variabile |
| iva/ivi/ivu | confronto var con costante |
| bvs | test bit registro |
| tin/tinh/tnt | abilitazione/disabilitazione trigger su ingressi |
| js/rt/rx/rtt | chiamata e ritorni |

---

## Pattern operativi

### Puntamento standard

```xpres
sv StepsPnt <passi>
sv ReqPunta 1
```

- segno di StepsPnt decide direzione
- routine usa mvs+/mvs-

### Puntamento esteso oltre 2^23

```xpres
sv Command 5 ; sv CmdArg <quota_start> ; sv ReqCmdEx 1
sv Command 6 ; sv CmdArg <quota_end>   ; sv ReqCmdEx 1
sv Command 7 ; sv CmdArg <giri_req>    ; sv ReqCmdEx 1
sv RoundPnt 1
```

### Slew cupola (DEC)

```xpres
sv Command 1 ; sv CmdArg 1 ; sv ReqCmdEx 1   // auto cupola ON
sv Command 2 ; sv CmdArg <ms> ; sv ReqCmdEx 1
```

- CmdArg > 0: una direzione
- CmdArg < 0: direzione opposta

### Focus (DEC)

```xpres
sv Command 4 ; sv CmdArg <durata_ms_con_segno> ; sv ReqCmdEx 1
```

- segno positivo: extra-focale
- segno negativo: intra-focale

---

## Timer usati nelle release 0.90

### A.R. (AsseAlfa)

- tChangeF=1, tWait=2, tCorrE=3, tCorrW=4, tPunta=5, tStrobOn=6, tStrobOf=7

### DEC (AsseDelt)

- tChangeF=1, tWait=2, tCorrE=3, tCorrW=4, tPunta=5, tFocus=6, tDome=7

---

## Note rapide di sicurezza

1. Fine corsa hiStop* porta a EndProg0.
2. EndProg0 ferma il motore e blocca il programma in loop.
3. In DEC è presente fermaT nel main, quindi tracking continuo non parte.
4. Spegnimento/accensione lampeggiatore puntamento è coordinato tra i due PLC tramite ProgRegT.

---

Aggiornamento: 23 aprile 2026
