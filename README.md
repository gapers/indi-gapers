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
- **GCC/G++** con supporto C++11
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

oppure (compatibile con il Makefile in-source già presente):

```bash
cmake -S . -B .
make
```

### 3. Installare

```bash
sudo make install
```

Il binario `indi_gapers` viene installato in `/usr/bin/` e il file XML del driver in `/usr/share/indi/`.

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
