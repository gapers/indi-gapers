# indi-gapers

Driver INDI nativo per il Telescopio e la Cupola del GAPers (Gruppo Astrofili Persicetani).

Versione corrente: letta dal file `VERSION` (single source of truth).

## Struttura progetto

Layout consigliato adottato:

- `src/`: sorgenti C++ del driver
- `include/`: header pubblici/interni del driver
- `data/`: template dati installabili (es. XML INDI)
- `cmake/`: template e helper CMake (es. `config.h.in`)
- `scripts/`: script di test e utility
- `VERSION`: versione progetto in formato `MAJOR.MINOR.PATCH`

---

## Requisiti

### Dipendenze di sistema

- **CMake** >= 3.5
- **GCC/G++** con supporto C++17
- **libindi** (INDI Core Libraries)
- **libnova** (Nova Astronomical Library)
- **zlib**
- **libgsl** (GNU Scientific Library)

### Installazione delle dipendenze su Ubuntu

Aggiungere il repository PPA ufficiale di INDI:

```bash
sudo add-apt-repository ppa:mutlaqja/ppa
sudo apt-get update
```

Installare tutte le dipendenze:

```bash
sudo apt-get install -y \
    cmake \
    build-essential \
    libindi-dev \
    libnova-dev \
    zlib1g-dev \
    libgsl-dev
```

---

## Compilazione e installazione

### 1. Clonare il repository

```bash
git clone https://github.com/gapers/indi-gapers.git
cd indi-gapers
```

### 2. Compilare

```bash
cmake -S . -B build
cmake --build build -j
```

Il progetto supporta anche la build in-source (compatibilità storica), ma è consigliata la build out-of-source (`build/`) per mantenere la root pulita.

### 3. Installare

```bash
sudo cmake --install build
```

Per installare da una build in-source:

```bash
sudo cmake --install .
```

Il binario `indi_gapers` viene installato in `/usr/bin/` e il file XML del driver in `/usr/share/indi/`.

### Note CMake

Il `CMakeLists.txt` usa linking target-based quando possibile:

- `Nova::Nova` se disponibile, con fallback su `NOVA_LIBRARIES`
- `GSL::gsl` se disponibile, con fallback su `GSL_LIBRARIES`

Questo permette compatibilità sia con ambienti moderni sia con moduli Find legacy.

---

## Testing

### Modalità simulazione (senza hardware)

Il driver include una modalità simulazione completa che non richiede la porta seriale né il telescopio fisico.

**Terminale 1** — avviare il server:

```bash
indiserver -v indi_gapers
```

**Terminale 2** — abilitare la simulazione, connettere e verificare:

```bash
# Abilita la modalità simulazione
indi_setprop "GAPers Telescope.SIMULATION.ENABLE=On"

# Connette il driver (in simulazione, non usa la seriale)
indi_setprop "GAPers Telescope.CONNECTION.CONNECT=On"

# Verifica stato connessione e coordinate correnti
indi_getprop "GAPers Telescope.CONNECTION.*" \
             "GAPers Telescope.SIMULATION.*" \
             "GAPers Telescope.EQUATORIAL_EOD_COORD.*"
```

L'output atteso mostra `CONNECTION.CONNECT=On`, `SIMULATION.ENABLE=On` e le coordinate RA/DEC a zero.

### Test suite seriale virtuale (senza hardware)

Nel repository è disponibile una suite automatica basata su pseudo-terminali (`socat`) per verificare il driver senza porta seriale fisica:

```bash
./scripts/virtual_serial_test_suite.sh
```

Versione con emulatore seriale minimale sul peer PTY:

```bash
./scripts/virtual_serial_test_suite.sh --with-emulator
```

La suite verifica almeno:

- connessione in modalità simulation (`IPS Ok`)
- gestione connessione in modalità reale su seriale virtuale (`IPS Ok` oppure `IPS Alert` gestito)

### Suite movimento telescopio + comandi PLC

È disponibile anche una suite di unit test che verifica:

- correttezza del calcolo dei passi asse RA/DEC per movimenti brevi
- correttezza della procedura a giri per movimenti lunghi (quote iniziale/finale + giri)
- correttezza del wrapping angolare (range -180/+180 gradi)
- validazione della distanza angolare in entrambe le direzioni (nord/sud per declinazione)
- **correttezza del calcolo tempo movimento cupola** (azimuth, speed, wrapping 0-360°)
- **validazione edge case di movimento nullo della cupola**
- **verifica del percorso più breve attraverso confine 0°**
- **movimento sincrono RA/DEC**: validazione movimento diagonale
- **rapporto velocità assi**: RA (220088.2 step/°) vs DEC (192000.0 step/°)
- **selezione comando**: distingue movimenti sincronizzati vs singolo asse
- **tempo massimo movimento**: l'asse più lento determina tempo totale

Esecuzione:

```bash
python3 scripts/test_motion_and_plc_commands.py
```

Prerequisiti runtime:

- `python3`

La suite è una suite di unit test pura (nessuna dipendenza da INDI server o hardware) che valida
le formule matematiche di movimento del driver C++ rispetto a costanti di telescopio/motori.

### Avviare il server INDI con il driver (hardware reale)

```bash
indiserver -v indi_gapers
```

### Connettersi con un client INDI

Con **KStars/Ekos**: aprire KStars → Ekos → aggiungere un nuovo profilo e selezionare `GAPers Telescope` come telescopio.

Con **indi_getprop** (verifica da riga di comando):

```bash
# In un terminale, avviare il server
indiserver -v indi_gapers

# In un secondo terminale, leggere tutte le proprietà del driver
indi_getprop
```

### Verifica della connessione seriale

Il driver si connette al telescopio tramite porta seriale (default `/dev/ttyUSB0`).  
Verificare che il dispositivo sia riconosciuto:

```bash
ls -l /dev/ttyUSB*
```

Se necessario, aggiungere l'utente al gruppo `dialout` per i permessi sulla porta seriale:

```bash
sudo usermod -aG dialout $USER
# Effettuare il logout e rientrare per applicare le modifiche
```

---

## Note

- Copyright (C) 2026 Massimiliano Masserelli
- Copyright (C) 2026 Gruppo Astrofili Persicetani
- Basato sul lavoro originale di Maurizio Serrazanetti (2014)
