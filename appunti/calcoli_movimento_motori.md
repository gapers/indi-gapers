# Calcoli movimento motori — documentazione tecnica

Questo documento preserva la documentazione originale dei calcoli effettuati
nel driver GAPers per il movimento degli assi. Il testo è tratto dai commenti
presenti in `driver/indi-gapers.cpp` al tag **v2.0.1** (luglio 2024), andati
persi durante il refactoring che ha estratto la matematica pura in
`src/gapers_math.cpp`.

---

## Costanti hardware principali

```
SIDRATE  = 0.004178  gradi/s   — velocità moto siderale apparente
SLEW_RATE = 15       gradi/s   — velocità slew

vs  = 919.456        passi/s   — velocità moto siderale (asse RA)
vp  = 220000.0       passi/s   — velocità di regime del motore
rs  = 500000         passi     — passi totali delle rampe (accel + decel)
spd_RA  = 220088.2   passi/°   — passi motore per grado di spostamento RA
spd_DEC = 192000.0   passi/°   — passi motore per grado di spostamento DEC
```

---

## Calcolo del tempo di spostamento (`calcMoveTime`)

Il tempo di rampa viene calcolato utilizzando la **velocità media** in passi
al secondo tra la velocità di partenza (≈0) e quella di regime (`vp`).
Essendo una rampa lineare, la velocità media vale `(vp - 200) / 2`
(200 passi/s è la velocità minima di partenza del driver).
Il tempo per percorrere i passi di rampa è quindi:

```
tr = rs / ((vp - 200) / 2)
```

**Caso 1 — movimento lungo (steps > rs):**
Il movimento richiesto è superiore ai passi necessari per completare le rampe
di accelerazione e decelerazione. Il tempo totale è il tempo delle rampe più
il tempo per percorrere i passi rimanenti a velocità di regime:

```
tm = ((steps - rs) / vp) + tr
```

**Caso 2 — movimento breve (steps ≤ rs):**
Il motore non raggiunge mai la velocità di regime. Il tempo totale viene
calcolato per proporzione rispetto al tempo di rampa:

```
tm = (steps × tr) / rs
```

---

## Correzione moto siderale per RA (`setMoveDataRA`)

Durante lo spostamento in ascensione retta occorre compensare il moto
siderale che si manifesta nel tempo necessario al movimento dell'asse.

La componente del moto siderale è sempre positiva (da est a ovest), pertanto
si considera il valore assoluto del numero di passi necessari per lo
spostamento, memorizzando la direzione per effettuare la correzione nella
giusta direzione al termine.

La correzione in passi è:

```
correction = tm × vs
steps_corretti = round(|steps|) × direction + correction
```

**Nota sull'imprecisione dell'algoritmo:** la correzione applicata non tiene
conto dell'effetto che essa stessa produce sul tempo totale di movimento
(i passi aggiuntivi allungano leggermente `tm`). L'errore così introdotto
è stato valutato abbastanza piccolo da essere trascurabile.

**DEC:** l'asse di declinazione non necessita di correzione siderale.

---

## Movimento per giri (`rotationsCalc`)

### Il problema: limite degli encoder

La quota rappresentabile dagli encoder stepper è limitata al range:

```
-8 388 608  ↔  +8 388 607   (encoder signed 24-bit: ±2²³)
```

Questo limita il **movimento basato sulla differenza di quota** a 2²³ passi,
pari a circa **38 gradi**. Per spostamenti maggiori è necessaria una procedura
alternativa.

### La procedura alternativa: movimento per giri

Si calcola il movimento in **giri completi** del motore. Questo viene eseguito
sfruttando l'**overflow** della quota encoder: il contatore supera il limite
massimo e riparte dal minimo (o viceversa).

Al termine del movimento "per giri" ci si posiziona alla quota esatta
richiesta tramite il normale movimento per passi.

### Il valore di sicurezza

Il movimento "per giri" è meno preciso di quello per passi regolato
dall'encoder: è necessario fermarlo **prima** di aver compiuto il massimo
movimento possibile, altrimenti si rischia di trovarsi oltre la quota encoder
desiderata, costringendo il motore a invertire il senso di marcia.

Nei due movimenti distinti questo non sarebbe eccessivamente problematico,
ma nel caso dell'**ascensione retta** potrebbe rendere meno affidabile la
correzione siderale.

Viene pertanto usato un arbitrario **"valore di sicurezza"** di
**80 giri completi** del motore, ovvero circa **1 024 000 passi (≈4 gradi)**.
Questo valore viene sottratto al numero di passi richiesti per lo
spostamento, in modo da fermarsi per tempo prima di passare al movimento
per passi.

```
ROTATION_THRESHOLD = 80 × 12800 = 1 024 000 passi   (≈4,65°)
```

### Calcolo delle quote encoder

Al PLC vanno comunicati:
- **quota iniziale** (`m_sq`): valore da impostare sull'encoder all'inizio
- **quota finale** (`m_eq`): valore da raggiungere
- **numero di giri** (`m_giri`): rivoluzioni da compiere

La quota finale viene calcolata considerando il numero totale di passi
e ricominciando a contare dal limite opposto quando si supera il limite
dell'encoder (gestione dell'overflow):

```
Movimento positivo (CW):
  m_sq   = ENCODER_MIN (-8 388 608)
  m_eq   = (steps % ENCODER_RANGE) + m_sq
  m_giri = ((steps - ROTATION_THRESHOLD) / SPINDLE_STEPS) + 1

Movimento negativo (CCW):
  m_sq   = ENCODER_MAX (+8 388 607)
  m_eq   = (steps % ENCODER_RANGE) + m_sq
  m_giri = ((steps + ROTATION_THRESHOLD) / SPINDLE_STEPS) - 1
```

### Guardia sull'overflow di fine movimento

Se al termine del calcolo `m_eq` ricade entro `ROTATION_THRESHOLD` dal
boundary dell'encoder, si traslano entrambe le quote di `ROTATION_THRESHOLD`
verso l'interno, per evitare di trovarsi "a cavallo" dell'overflow al termine
del movimento per giri.

### Vincolo del PLC: quota zero non ammessa

Il valore `0` non è ammesso come parametro nella richiesta al PLC per avviare
la procedura per giri. Se il calcolo porta `m_eq == 0`, si aggiunge un offset
arbitrario di ±100 a entrambe le quote.

Nei sorgenti v2.0.1 questo era documentato come
`"nasty race condition in plc program"`.

### Sanity check

Questa procedura dovrebbe essere utilizzata soltanto per spostamenti superiori
a circa 38 gradi (2²³ passi ≈ 8 388 608). Tuttavia, usarla per movimenti
più ridotti non è un problema finché si rimane sopra alla soglia di sicurezza
(`ROTATION_THRESHOLD`, ≈4 gradi). Sotto tale soglia la funzione restituisce
`false`.

---

## `rangeDistance`

Riporta il range di un angolo all'interno di **±180 gradi**, da utilizzare
nel calcolo delle differenze angolari per gli spostamenti, in modo da usare
sempre il **percorso angolare più breve**.

---

## Riferimenti

- Commenti originali in `driver/indi-gapers.cpp` @ tag `v2.0.1`
  (commit `da113ad`, 19 luglio 2024)
- Documentazione protocollo Xpress (vedere `appunti/XPres/`)
- Fogli di calcolo originali: `appunti/Calcolo correzione moto siderale.ods`
