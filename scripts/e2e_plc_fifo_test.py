#!/usr/bin/env python3
import argparse
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple


def run_cmd(cmd: List[str], timeout: int = 10, check: bool = True) -> subprocess.CompletedProcess:
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if check and proc.returncode != 0:
        raise RuntimeError(
            f"Command failed ({proc.returncode}): {' '.join(shlex.quote(c) for c in cmd)}\n"
            f"stdout:\n{proc.stdout}\n"
            f"stderr:\n{proc.stderr}"
        )
    return proc


def wait_for(predicate, timeout_s: float, interval_s: float = 0.2) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return False


def wait_for_driver(device: str, port: int, timeout_s: int = 15) -> None:
    query = f"{device}.DRIVER_INFO.DRIVER_NAME"

    def _ready() -> bool:
        proc = subprocess.run(["indi_getprop", "-p", str(port), "-t", "1", query], capture_output=True, text=True)
        return proc.returncode == 0

    if not wait_for(_ready, timeout_s):
        raise RuntimeError("INDI driver non disponibile entro timeout")


def read_conn_state(device: str, port: int) -> str:
    proc = run_cmd(["indi_getprop", "-p", str(port), f"{device}.CONNECTION._STATE"], check=True)
    for line in proc.stdout.splitlines():
        if "=" in line:
            return line.split("=", 1)[1].strip()
    return ""


def read_prop(query: str, port: int) -> Optional[str]:
    proc = subprocess.run(["indi_getprop", "-p", str(port), query], capture_output=True, text=True)
    if proc.returncode != 0:
        return None
    for line in proc.stdout.splitlines():
        if "=" in line:
            return line.split("=", 1)[1].strip()
    return None


def read_mount_state(device: str, port: int) -> Optional[str]:
    for prop in ("EQUATORIAL_COORD._STATE", "EQUATORIAL_EOD_COORD._STATE"):
        value = read_prop(f"{device}.{prop}", port)
        if value is not None:
            return value
    return None


def indi_set(device: str, prop_expr: str, port: int, timeout: int = 10) -> None:
    run_cmd(["indi_setprop", "-p", str(port), "-t", str(timeout), f"{device}.{prop_expr}"], timeout=timeout + 2, check=True)


def wait_for_conn_state(device: str, expected: str, port: int, timeout_s: int = 12) -> None:
    def _ok() -> bool:
        return read_conn_state(device, port) == expected

    if not wait_for(_ok, timeout_s, interval_s=0.5):
        got = read_conn_state(device, port)
        raise RuntimeError(f"Stato connessione inatteso: atteso {expected}, ottenuto {got}")


def parse_socat_ptys(log_path: Path, timeout_s: int = 8) -> Tuple[str, str]:
    pattern = re.compile(r"PTY is (/dev/pts/\d+)")

    def _extract() -> Optional[Tuple[str, str]]:
        if not log_path.exists():
            return None
        text = log_path.read_text(encoding="utf-8", errors="replace")
        found = pattern.findall(text)
        if len(found) >= 2:
            return found[0], found[1]
        return None

    deadline = time.time() + timeout_s
    while time.time() < deadline:
        pair = _extract()
        if pair is not None:
            return pair
        time.sleep(0.2)
    raise RuntimeError("Impossibile determinare la coppia PTY da socat")


def load_frames(jsonl_path: Path) -> List[Dict[str, object]]:
    frames: List[Dict[str, object]] = []
    if not jsonl_path.exists():
        return frames
    for line in jsonl_path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(obj, dict) and isinstance(obj.get("cmd"), int) and isinstance(obj.get("syst"), str):
            frames.append(obj)
    return frames


def clear_capture(jsonl_path: Path) -> None:
    jsonl_path.write_text("", encoding="utf-8")


def signed_value(v: int) -> int:
    if v > 0:
        return 1
    if v < 0:
        return -1
    return 0


def expected_signature_for_modes(ra_mode: str, dec_mode: str) -> List[Tuple[str, int]]:
    if ra_mode == "short" and dec_mode == "short":
        return [("1", 15), ("2", 15), ("0", 8)]

    if ra_mode == "long" and dec_mode == "long":
        return [
            ("1", 10), ("1", 9), ("1", 5),
            ("1", 10), ("1", 9), ("1", 5),
            ("1", 10), ("1", 9), ("1", 5),
            ("2", 10), ("2", 9), ("2", 5),
            ("2", 10), ("2", 9), ("2", 5),
            ("2", 10), ("2", 9), ("2", 5),
            ("0", 14),
        ]

    if ra_mode == "long" and dec_mode == "short":
        return [
            ("1", 10), ("1", 9), ("1", 5),
            ("1", 10), ("1", 9), ("1", 5),
            ("1", 10), ("1", 9), ("1", 5),
            ("2", 15),
            ("1", 14), ("2", 8),
        ]

    if ra_mode == "short" and dec_mode == "long":
        return [
            ("1", 15),
            ("2", 10), ("2", 9), ("2", 5),
            ("2", 10), ("2", 9), ("2", 5),
            ("2", 10), ("2", 9), ("2", 5),
            ("1", 8), ("2", 14),
        ]

    raise RuntimeError(f"Combinazione mode non supportata: RA={ra_mode} DEC={dec_mode}")


def extract_axis_direction_sign(frames: List[Dict[str, object]], axis: str, mode: str) -> int:
    if mode == "short":
        for f in frames:
            if str(f.get("syst")) == axis and int(f.get("cmd", -1)) == 15:
                return signed_value(int(f.get("val", 0)))
        raise RuntimeError(f"Segno asse {axis} short non trovato (cmd 15 assente)")

    if mode == "long":
        cmd10_vals: List[int] = []
        for f in frames:
            if str(f.get("syst")) == axis and int(f.get("cmd", -1)) == 10:
                cmd10_vals.append(int(f.get("val", 0)))
        if len(cmd10_vals) < 3:
            raise RuntimeError(f"Segno asse {axis} long non trovato (cmd10 insufficienti: {len(cmd10_vals)})")
        # For long moves, the 3rd cmd10 is m_giri and carries direction sign.
        return signed_value(cmd10_vals[2])

    raise RuntimeError(f"Mode asse non supportata: {mode}")


def stop_process(proc: Optional[subprocess.Popen]) -> None:
    if proc is None or proc.poll() is not None:
        return

    # Child services are started with setsid; terminate the whole process group
    # so we do not leave orphaned helpers behind (e.g. indiserver/socat/emulator).
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return

    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            return
        proc.wait(timeout=3)


def ensure_tools() -> None:
    needed = ["socat", "indiserver", "indi_getprop", "indi_setprop", "python3"]
    missing = [tool for tool in needed if subprocess.run(["which", tool], capture_output=True).returncode != 0]
    if missing:
        raise RuntimeError(f"Comandi mancanti: {', '.join(missing)}")


def run_motion_scenario(
    *,
    device: str,
    port: int,
    capture_jsonl: Path,
    name: str,
    sync_ra: float,
    sync_dec: float,
    goto_ra: float,
    goto_dec: float,
    ra_mode: str,
    dec_mode: str,
    expected_ra_sign: int,
    expected_dec_sign: int,
    expected_signature: List[Tuple[str, int]],
) -> None:
    print(f"[INFO] Scenario: {name}")

    clear_capture(capture_jsonl)

    # 1) SYNC baseline (must not generate PLC traffic)
    indi_set(device, "ON_COORD_SET.SYNC=On", port)
    indi_set(device, f"EQUATORIAL_COORD.RA={sync_ra};DEC={sync_dec}", port)
    time.sleep(0.5)
    frames_after_sync = load_frames(capture_jsonl)
    if frames_after_sync:
        raise RuntimeError(
            f"{name}: SYNC ha generato traffico PLC inatteso ({len(frames_after_sync)} frame)"
        )

    # 2) GOTO motion
    indi_set(device, "ON_COORD_SET.SLEW=On", port)
    indi_set(device, f"EQUATORIAL_COORD.RA={goto_ra};DEC={goto_dec}", port)

    if not wait_for(lambda: read_mount_state(device, port) == "Busy", timeout_s=5, interval_s=0.2):
        raise RuntimeError(
            f"{name}: GOTO non innescato (mount_state={read_mount_state(device, port)})"
        )

    def _enough_frames() -> bool:
        return len(load_frames(capture_jsonl)) >= len(expected_signature)

    if not wait_for(_enough_frames, timeout_s=15, interval_s=0.2):
        got = load_frames(capture_jsonl)
        raise RuntimeError(
            f"{name}: timeout frame PLC (attesi >= {len(expected_signature)}, ottenuti {len(got)})"
        )

    got_frames = load_frames(capture_jsonl)[: len(expected_signature)]
    got_signature = [(str(f["syst"]), int(f["cmd"])) for f in got_frames]
    if got_signature != expected_signature:
        raise RuntimeError(
            f"{name}: sequenza PLC inattesa. Attesa={expected_signature} Ricevuta={got_signature}"
        )

    ra_sign = extract_axis_direction_sign(got_frames, "1", ra_mode)
    dec_sign = extract_axis_direction_sign(got_frames, "2", dec_mode)
    if ra_sign != expected_ra_sign:
        raise RuntimeError(f"{name}: segno RA inatteso. Atteso={expected_ra_sign} Ricevuto={ra_sign}")
    if dec_sign != expected_dec_sign:
        raise RuntimeError(f"{name}: segno DEC inatteso. Atteso={expected_dec_sign} Ricevuto={dec_sign}")

    if not wait_for(lambda: read_mount_state(device, port) == "Ok", timeout_s=8, interval_s=0.2):
        raise RuntimeError(f"{name}: il driver non torna in stato Ok dopo fine movimento vn")


def find_dome_motion_triplet(frames: List[Dict[str, object]]) -> int:
    # DomeGoto enqueues exactly: 2tx 10 <time>, 2tx 9 2, 2tx 5 1
    for i in range(1, len(frames) - 1):
        prevf = frames[i - 1]
        curf = frames[i]
        nextf = frames[i + 1]
        if (
            str(prevf.get("syst")) == "2"
            and int(prevf.get("cmd", -1)) == 10
            and str(curf.get("syst")) == "2"
            and int(curf.get("cmd", -1)) == 9
            and int(curf.get("val", -1)) == 2
            and str(nextf.get("syst")) == "2"
            and int(nextf.get("cmd", -1)) == 5
            and int(nextf.get("val", -1)) == 1
        ):
            return i
    return -1


def has_dome_manual_enable_triplet(frames: List[Dict[str, object]]) -> bool:
    # On vn var9 whr2, driver calls DomeManualEnable(true): 2tx 10 0, 2tx 9 1, 2tx 5 1
    for i in range(1, len(frames) - 1):
        prevf = frames[i - 1]
        curf = frames[i]
        nextf = frames[i + 1]
        if (
            str(prevf.get("syst")) == "2"
            and int(prevf.get("cmd", -1)) == 10
            and int(prevf.get("val", 1)) == 0
            and str(curf.get("syst")) == "2"
            and int(curf.get("cmd", -1)) == 9
            and int(curf.get("val", -1)) == 1
            and str(nextf.get("syst")) == "2"
            and int(nextf.get("cmd", -1)) == 5
            and int(nextf.get("val", -1)) == 1
        ):
            return True
    return False


def run_dome_auto_scenario(*, device: str, port: int, capture_jsonl: Path) -> None:
    print("[INFO] Scenario: dome_auto_follow")

    clear_capture(capture_jsonl)

    # Ensure dome auto-follow is enabled.
    indi_set(device, "DOME_MOVEMENT.AUTO=On", port)

    # Baseline sync at a circumpolar point.
    indi_set(device, "ON_COORD_SET.SYNC=On", port)
    indi_set(device, "EQUATORIAL_COORD.RA=0.0;DEC=75.0", port)
    time.sleep(0.5)

    # Clear any traffic induced by startup/sync interactions.
    clear_capture(capture_jsonl)

    # Slew to a far-away RA to force dome azimuth change.
    indi_set(device, "ON_COORD_SET.SLEW=On", port)
    indi_set(device, "EQUATORIAL_COORD.RA=12.0;DEC=75.0", port)

    def _got_dome_triplet() -> bool:
        frames = load_frames(capture_jsonl)
        return find_dome_motion_triplet(frames) != -1

    if not wait_for(_got_dome_triplet, timeout_s=20, interval_s=0.2):
        frames = load_frames(capture_jsonl)
        raise RuntimeError(f"dome_auto_follow: non trovata tripletta comandi cupola 10/9(2)/5. frames={len(frames)}")

    # Wait for mount completion and dome completion side-effects.
    if not wait_for(lambda: read_mount_state(device, port) == "Ok", timeout_s=12, interval_s=0.2):
        raise RuntimeError("dome_auto_follow: montatura non torna in stato Ok")

    if not wait_for(lambda: has_dome_manual_enable_triplet(load_frames(capture_jsonl)), timeout_s=10, interval_s=0.2):
        raise RuntimeError("dome_auto_follow: manca tripletta DomeManualEnable(true) 10(0)/9(1)/5(1)")


def main() -> int:
    parser = argparse.ArgumentParser(description="E2E PLC FIFO test for indi-gapers")
    parser.add_argument("--driver", required=True, help="Path al binario indi_gapers")
    parser.add_argument("--device", default="GAPers Telescope", help="Nome dispositivo INDI")
    parser.add_argument("--port", type=int, default=7625, help="Porta TCP indiserver dedicata al test")
    parser.add_argument("--keep-artifacts", action="store_true", help="Non cancellare la directory temporanea")
    args = parser.parse_args()

    driver_path = Path(args.driver).resolve()
    if not driver_path.exists() or not os.access(driver_path, os.X_OK):
        raise RuntimeError(f"Driver non eseguibile: {driver_path}")

    ensure_tools()

    tmp_dir_obj = tempfile.TemporaryDirectory(prefix="indi_gapers_plc_e2e_")
    tmp_dir = Path(tmp_dir_obj.name)
    socat_log = tmp_dir / "socat.log"
    server_log = tmp_dir / "indiserver.log"
    emulator_log = tmp_dir / "emulator.log"
    capture_jsonl = tmp_dir / "plc_frames.jsonl"

    socat_proc: Optional[subprocess.Popen] = None
    emulator_proc: Optional[subprocess.Popen] = None
    server_proc: Optional[subprocess.Popen] = None

    def _handle_termination(signum, _frame):
        raise KeyboardInterrupt(f"Interrupted by signal {signum}")

    old_sigint = signal.signal(signal.SIGINT, _handle_termination)
    old_sigterm = signal.signal(signal.SIGTERM, _handle_termination)

    try:
        with socat_log.open("w", encoding="utf-8") as sl:
            socat_proc = subprocess.Popen(
                ["socat", "-d", "-d", "pty,raw,echo=0", "pty,raw,echo=0"],
                stdout=sl,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )

        pty_a, pty_b = parse_socat_ptys(socat_log)
        print(f"[INFO] PTY pair: {pty_a} <-> {pty_b}")

        with emulator_log.open("w", encoding="utf-8") as el:
            emulator_proc = subprocess.Popen(
                [
                    "python3",
                    str(Path(__file__).with_name("pty_plc_emulator.py")),
                    pty_b,
                    "--echo",
                    "--emit-vn",
                    "--capture-jsonl",
                    str(capture_jsonl),
                ],
                stdout=el,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )

        with server_log.open("w", encoding="utf-8") as svl:
            server_proc = subprocess.Popen(
                ["indiserver", "-p", str(args.port), "-v", str(driver_path)],
                stdout=svl,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )

        wait_for_driver(args.device, args.port)

        # Start from a clean state.
        subprocess.run(["indi_setprop", "-p", str(args.port), f"{args.device}.CONFIG_PROCESS.CONFIG_PURGE=On"], capture_output=True)
        subprocess.run(["indi_setprop", "-p", str(args.port), f"{args.device}.CONNECTION.DISCONNECT=On"], capture_output=True)

        # Real mode over virtual serial.
        indi_set(args.device, "DEVICE_AUTO_SEARCH.INDI_DISABLED=On", args.port)
        indi_set(args.device, "SIMULATION.DISABLE=On", args.port)
        indi_set(args.device, f"DEVICE_PORT.PORT={pty_a}", args.port)
        indi_set(args.device, "CONNECTION.CONNECT=On", args.port)
        wait_for_conn_state(args.device, "Ok", args.port, timeout_s=12)

        sim_enabled = read_prop(f"{args.device}.SIMULATION.ENABLE", args.port)
        if sim_enabled == "On":
            raise RuntimeError("Driver in simulazione dopo CONNECT: nessun traffico seriale PLC")

        # Disable dome auto movement to keep PLC command stream deterministic.
        indi_set(args.device, "DOME_MOVEMENT.MANUAL=On", args.port)

        # 16 permutations:
        # RA mode {short,long} x DEC mode {short,long} x RA dir {+, -} x DEC dir {+, -}
        scenarios: List[Dict[str, object]] = []
        for ra_mode in ("short", "long"):
            for dec_mode in ("short", "long"):
                for ra_dir in ("plus", "minus"):
                    for dec_dir in ("plus", "minus"):
                        ra_delta_h = 0.2 if ra_mode == "short" else 1.0
                        if ra_dir == "minus":
                            ra_delta_h = -ra_delta_h

                        if dec_mode == "short":
                            sync_dec = 76.0 if dec_dir == "plus" else 78.0
                            goto_dec = sync_dec + (2.0 if dec_dir == "plus" else -2.0)
                        else:
                            sync_dec = 74.0 if dec_dir == "plus" else 86.0
                            goto_dec = sync_dec + (12.0 if dec_dir == "plus" else -12.0)

                        sync_ra = 12.0
                        goto_ra = sync_ra + ra_delta_h

                        expected_ra_sign = -1 if ra_dir == "plus" else 1
                        expected_dec_sign = -1 if dec_dir == "plus" else 1

                        scenarios.append(
                            {
                                "name": f"ra_{ra_mode}_{ra_dir}__dec_{dec_mode}_{dec_dir}",
                                "sync_ra": sync_ra,
                                "sync_dec": sync_dec,
                                "goto_ra": goto_ra,
                                "goto_dec": goto_dec,
                                "ra_mode": ra_mode,
                                "dec_mode": dec_mode,
                                "expected_ra_sign": expected_ra_sign,
                                "expected_dec_sign": expected_dec_sign,
                                "expected": expected_signature_for_modes(ra_mode, dec_mode),
                            }
                        )

        for sc in scenarios:
            run_motion_scenario(
                device=args.device,
                port=args.port,
                capture_jsonl=capture_jsonl,
                name=str(sc["name"]),
                sync_ra=float(sc["sync_ra"]),
                sync_dec=float(sc["sync_dec"]),
                goto_ra=float(sc["goto_ra"]),
                goto_dec=float(sc["goto_dec"]),
                ra_mode=str(sc["ra_mode"]),
                dec_mode=str(sc["dec_mode"]),
                expected_ra_sign=int(sc["expected_ra_sign"]),
                expected_dec_sign=int(sc["expected_dec_sign"]),
                expected_signature=list(sc["expected"]),
            )

        run_dome_auto_scenario(device=args.device, port=args.port, capture_jsonl=capture_jsonl)

        print("[PASS] Verificate 16 permutazioni RA/DEC + comandi cupola in auto-follow")
        print(f"[INFO] Artifacts: {tmp_dir}")
        if not args.keep_artifacts:
            tmp_dir_obj.cleanup()
        else:
            tmp_dir_obj = None  # prevent auto cleanup
        return 0

    except Exception as exc:
        print(f"[FAIL] {exc}")
        if server_log.exists():
            print("----- indiserver.log (tail) -----")
            print("\n".join(server_log.read_text(encoding="utf-8", errors="replace").splitlines()[-80:]))
        if socat_log.exists():
            print("----- socat.log (tail) -----")
            print("\n".join(socat_log.read_text(encoding="utf-8", errors="replace").splitlines()[-40:]))
        if emulator_log.exists():
            print("----- emulator.log (tail) -----")
            print("\n".join(emulator_log.read_text(encoding="utf-8", errors="replace").splitlines()[-40:]))
        if capture_jsonl.exists():
            print("----- captured frames -----")
            print(capture_jsonl.read_text(encoding="utf-8", errors="replace"))
        print("----- key properties -----")
        for q in (
            f"{args.device}.CONNECTION._STATE",
            f"{args.device}.SIMULATION.*",
            f"{args.device}.ON_COORD_SET.*",
            f"{args.device}.EQUATORIAL_COORD._STATE",
            f"{args.device}.EQUATORIAL_EOD_COORD._STATE",
            f"{args.device}.TELESCOPE_PARK.*",
        ):
            proc = subprocess.run(["indi_getprop", "-p", str(args.port), q], capture_output=True, text=True)
            if proc.returncode == 0 and proc.stdout.strip():
                print(proc.stdout.strip())
        print(f"[INFO] Artifacts kept in: {tmp_dir}")
        return 1

    finally:
        for proc in (server_proc, emulator_proc, socat_proc):
            stop_process(proc)
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)


if __name__ == "__main__":
    sys.exit(main())
