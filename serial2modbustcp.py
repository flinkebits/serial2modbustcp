#!/usr/bin/env python3
"""
serial2modbustcp - D0/IEC 62056-21 to Modbus TCP Gateway

Reads smart meter data via optical D0 protocol and exposes
the values as Modbus TCP holding registers.
"""

import json
import logging
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import serial
from pymodbus.datastore import (
    ModbusSequentialDataBlock,
    ModbusSlaveContext,
    ModbusServerContext,
)
from pymodbus.server import StartTcpServer

__version__ = "1.0.0"

logger = logging.getLogger("serial2modbustcp")

DEFAULT_OBIS_MAPPING = {
    "1-0:1.8.0": {"register": 0, "name": "Wirkenergie Bezug", "unit": "kWh", "factor": 1},
    "1-0:2.8.0": {"register": 2, "name": "Wirkenergie Einspeisung", "unit": "kWh", "factor": 1},
    "1-0:1.7.0": {"register": 4, "name": "Wirkleistung Bezug", "unit": "kW", "factor": 1},
    "1-0:2.7.0": {"register": 6, "name": "Wirkleistung Einspeisung", "unit": "kW", "factor": 1},
    "1-0:21.7.0": {"register": 8, "name": "Wirkleistung L1", "unit": "kW", "factor": 1},
    "1-0:22.7.0": {"register": 10, "name": "Wirkleistung L1 Einspeisung", "unit": "kW", "factor": 1},
    "1-0:41.7.0": {"register": 12, "name": "Wirkleistung L2", "unit": "kW", "factor": 1},
    "1-0:42.7.0": {"register": 14, "name": "Wirkleistung L2 Einspeisung", "unit": "kW", "factor": 1},
    "1-0:61.7.0": {"register": 16, "name": "Wirkleistung L3", "unit": "kW", "factor": 1},
    "1-0:62.7.0": {"register": 18, "name": "Wirkleistung L3 Einspeisung", "unit": "kW", "factor": 1},
    "0-0:0.9.1": {"register": 20, "name": "Uhrzeit", "unit": "", "factor": 1, "encoding": "hhmmss"},
    "0-0:0.9.2": {"register": 22, "name": "Datum", "unit": "", "factor": 1, "encoding": "date"},
    "0-0:0.0.0": {"register": 24, "name": "Zaehlernummer", "unit": "", "factor": 1, "encoding": "meter"},
    "1-0:1.8.1": {"register": 26, "name": "Wirkenergie Bezug T1", "unit": "kWh", "factor": 1},
    "1-0:1.8.2": {"register": 28, "name": "Wirkenergie Bezug T2", "unit": "kWh", "factor": 1},
    "1-0:2.8.1": {"register": 30, "name": "Wirkenergie Einspeisung T1", "unit": "kWh", "factor": 1},
    "1-0:2.8.2": {"register": 32, "name": "Wirkenergie Einspeisung T2", "unit": "kWh", "factor": 1},
    "1-0:3.8.0": {"register": 34, "name": "Blindenergie Bezug", "unit": "kvarh", "factor": 1},
    "1-0:4.8.0": {"register": 36, "name": "Blindenergie Einspeisung", "unit": "kvarh", "factor": 1},
    "1-0:5.8.0": {"register": 38, "name": "Blindenergie QI", "unit": "kvarh", "factor": 1},
    "1-0:6.8.0": {"register": 40, "name": "Blindenergie QII", "unit": "kvarh", "factor": 1},
    "1-0:7.8.0": {"register": 42, "name": "Blindenergie QIII", "unit": "kvarh", "factor": 1},
    "1-0:8.8.0": {"register": 44, "name": "Blindenergie QIV", "unit": "kvarh", "factor": 1},
    "1-0:9.8.0": {"register": 46, "name": "Scheinenergie", "unit": "kVAh", "factor": 1},
    "1-0:3.7.0": {"register": 48, "name": "Blindleistung Bezug", "unit": "kvar", "factor": 1},
    "1-0:4.7.0": {"register": 50, "name": "Blindleistung Einspeisung", "unit": "kvar", "factor": 1},
    "1-0:9.7.0": {"register": 52, "name": "Scheinleistung", "unit": "kVA", "factor": 1},
    "1-0:1.4.0": {"register": 54, "name": "Wirkleistung Mittel", "unit": "kW", "factor": 1},
    "1-0:1.5.0": {"register": 56, "name": "Wirkleistung Vorperiode", "unit": "kW", "factor": 1},
    "1-0:16.7.0": {"register": 58, "name": "Wirkleistung saldiert", "unit": "kW", "factor": 1},
    "1-0:1.6.1": {"register": 60, "name": "Max Wirkleistung T1", "unit": "kW", "factor": 1},
    "1-0:1.6.2": {"register": 62, "name": "Max Wirkleistung T2", "unit": "kW", "factor": 1},
    "1-0:2.6.1": {"register": 64, "name": "Max Wirkleistung Ein T1", "unit": "kW", "factor": 1},
    "1-0:2.6.2": {"register": 66, "name": "Max Wirkleistung Ein T2", "unit": "kW", "factor": 1},
    "1-0:1.2.1": {"register": 68, "name": "Kum Maximum Bezug T1", "unit": "kW", "factor": 1},
    "1-0:1.2.2": {"register": 70, "name": "Kum Maximum Bezug T2", "unit": "kW", "factor": 1},
    "1-0:2.2.1": {"register": 72, "name": "Kum Maximum Ein T1", "unit": "kW", "factor": 1},
    "1-0:2.2.2": {"register": 74, "name": "Kum Maximum Ein T2", "unit": "kW", "factor": 1},
    "1-0:23.7.0": {"register": 76, "name": "Blindleistung L1", "unit": "kvar", "factor": 1},
    "1-0:43.7.0": {"register": 78, "name": "Blindleistung L2", "unit": "kvar", "factor": 1},
    "1-0:63.7.0": {"register": 80, "name": "Blindleistung L3", "unit": "kvar", "factor": 1},
    "1-0:29.7.0": {"register": 82, "name": "Scheinleistung L1", "unit": "kVA", "factor": 1},
    "1-0:49.7.0": {"register": 84, "name": "Scheinleistung L2", "unit": "kVA", "factor": 1},
    "1-0:69.7.0": {"register": 86, "name": "Scheinleistung L3", "unit": "kVA", "factor": 1},
    "0-0:0.1.0": {"register": 88, "name": "Abrechnungsperioden", "unit": "", "factor": 1, "encoding": "integer"},
    "1-0:C.7.0": {"register": 90, "name": "Netzausfaelle", "unit": "", "factor": 1, "encoding": "integer"},
    "F.F": {"register": 92, "name": "Fehlerflags", "unit": "", "factor": 1, "encoding": "integer"},
}

STATUS_OFFSET_CONNECTED = 0
STATUS_OFFSET_ERRORS = 1
STATUS_OFFSET_LAST_READ = 2
STATUS_OFFSET_VERSION = 3
STATUS_REGISTER_COUNT = 4

# IEC 62056-21 Baudraten-Kennung (Zeichen nach Herstellerkennung in der Ident-Zeile)
IEC62056_BAUD_RATES = {
    "0": 300,
    "1": 600,
    "2": 1200,
    "3": 2400,
    "4": 4800,
    "5": 9600,
    "6": 19200,
    "7": 38400,
    "8": 57600,
    "9": 115200,
}


def format_obis_value(value: float, encoding: str) -> str:
    """Anzeigeform. Das Protokoll liefert nur Zahlen, keine formatierte Uhrzeit."""
    if encoding == "hhmmss":
        number = int(value) % 1000000
        return f"{number // 10000:02d}:{(number // 100) % 100:02d}:{number % 100:02d}"
    if encoding == "date":
        number = int(value)
        year, month, day = number // 10000, (number // 100) % 100, number % 100
        return f"{2000 + year:04d}-{month:02d}-{day:02d}"
    if encoding == "meter":
        return f"{int(value):08d}"
    if encoding == "integer":
        return str(int(value))
    return f"{value:.3f}"


def modbus_word(value: float, encoding: str, factor: float) -> int:
    """32-Bit-Wert fuer zwei Holding-Register. Messwerte in Tausendsteln, Kennungen roh."""
    if encoding == "date":
        number = int(value)
        year, month, day = number // 10000, (number // 100) % 100, number % 100
        return (2000 + year) * 10000 + month * 100 + day
    if encoding in ("hhmmss", "meter", "integer"):
        return int(value * factor)
    return int(value * factor * 1000)


class Config:
    """Laden und Validieren der Konfiguration."""

    DEFAULTS = {
        "serial_port": "/dev/ttyUSB0",
        "serial_baudrate": 300,
        "modbus_host": "0.0.0.0",
        "modbus_port": 502,
        "read_interval": 10,
        "obis_mapping": DEFAULT_OBIS_MAPPING,
        "status_register": 100,
        # "auto" | true | false — viele Landis+Gyr senden trotz LGZ5 bei 300 Baud
        "serial_use_baud_switch": "auto",
    }

    def __init__(self, config_path: Optional[str] = None):
        self._data: dict = dict(self.DEFAULTS)
        if config_path:
            self._load_file(config_path)

    def _load_file(self, path: str) -> None:
        p = Path(path)
        if not p.exists():
            logger.warning("Config-Datei nicht gefunden: %s - verwende Defaults", path)
            return
        try:
            with open(p, encoding="utf-8") as f:
                user = json.load(f)
            self._deep_merge(self._data, user)
            logger.info("Config geladen: %s", path)
        except json.JSONDecodeError as e:
            logger.error("Config-Fehler in %s: %s", path, e)

    def _deep_merge(self, base: dict, override: dict) -> None:
        for k, v in override.items():
            if k in base and isinstance(base[k], dict) and isinstance(v, dict):
                self._deep_merge(base[k], v)
            else:
                base[k] = v

    def __getattr__(self, name: str):
        if name in self._data:
            return self._data[name]
        raise AttributeError(name)


class D0Parser:
    """IEC 62056-21 (D0) Protokoll Parser."""

    IDENT_REQUEST = b"/?!\r\n"

    def __init__(
        self,
        port: str,
        initial_baudrate: int = 300,
        use_baud_switch: str = "auto",
    ):
        self.port = port
        self.initial_baudrate = initial_baudrate
        self.use_baud_switch = use_baud_switch
        self.serial_conn: Optional[serial.Serial] = None
        self._active_baudrate = initial_baudrate

    def connect(self) -> None:
        self.serial_conn = self._open_serial(self.initial_baudrate)
        self._active_baudrate = self.initial_baudrate
        logger.info("Verbunden mit %s @ %d baud", self.port, self.initial_baudrate)

    def disconnect(self) -> None:
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            logger.info("Getrennt")

    @property
    def is_connected(self) -> bool:
        return self.serial_conn is not None and self.serial_conn.is_open

    def _open_serial(self, baudrate: int) -> serial.Serial:
        return serial.Serial(
            port=self.port,
            baudrate=baudrate,
            bytesize=serial.SEVENBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            timeout=2,
        )

    def switch_baudrate(self, baudrate: int) -> None:
        if not self.serial_conn:
            return
        if self._active_baudrate == baudrate:
            return
        timeout = self.serial_conn.timeout
        self.serial_conn.close()
        time.sleep(0.05)
        self.serial_conn = self._open_serial(baudrate)
        self.serial_conn.timeout = timeout
        self._active_baudrate = baudrate
        logger.info("Baudrate gewechselt zu %d", baudrate)

    @staticmethod
    def normalize_ident_line(line: str) -> Optional[str]:
        """Ident ab '/' extrahieren (Prefix wie T/ oder V/ vor /LGZ5...)."""
        slash = line.find("/")
        if slash < 0:
            return None
        return line[slash:].strip()

    @staticmethod
    def is_valid_ident_line(ident: str) -> bool:
        """Pruefen ob Zeile eine IEC-62056-21-Identifikation ist, keine OBIS-Datenzeile."""
        if not ident.startswith("/"):
            return False
        after = ident[1:]
        if len(after) < 4:
            return False
        if not after[:3].isalpha():
            return False
        return after[3] in IEC62056_BAUD_RATES

    @staticmethod
    def baud_char_from_ident(ident: str) -> str:
        """Baud-Kennzeichen aus Ident, z. B. /LGZ5\\2... -> '5'."""
        slash = ident.find("/")
        if slash < 0:
            return "0"
        after = ident[slash + 1 :]
        if len(after) < 4:
            return "0"
        return after[3]

    @staticmethod
    def build_ack(baud_char: str) -> bytes:
        """Mode-C-Antwort: ACK + '0' + Baud-Kennung + '0' + CR LF (IEC 62056-21)."""
        return bytes([0x06, 0x30, ord(baud_char), 0x30, 0x0D, 0x0A])

    @staticmethod
    def baudrate_from_ident(ident: str) -> int:
        """Baudrate aus Ident-Zeile, z. B. /LGZ5\\2... -> 9600."""
        baud_char = D0Parser.baud_char_from_ident(ident)
        rate = IEC62056_BAUD_RATES.get(baud_char)
        if rate is None:
            logger.warning("Unbekannte Baud-Kennung '%s' in Ident, verwende 300", baud_char)
            return 300
        return rate

    @staticmethod
    def normalize_obis_code(obis_code: str) -> str:
        """OBIS vereinheitlichen: *255 abtrennen, 1.8.0 -> 1-0:1.8.0, 0.9.1 -> 0-0:0.9.1."""
        if "*" in obis_code:
            obis_code = obis_code.split("*", 1)[0]
        if "-" not in obis_code and obis_code.count(".") >= 2:
            medium = "0-0" if obis_code.startswith("0.") else "1-0"
            obis_code = f"{medium}:{obis_code}"
        return obis_code

    def _listen_plans(self, ident: str) -> List[tuple]:
        """(ACK-Baudzeichen, Lese-Baudrate). ACK und Leser muessen dieselbe Rate nutzen."""
        proposed_char = self.baud_char_from_ident(ident)
        proposed_baud = self.baudrate_from_ident(ident)
        stay = ("0", self.initial_baudrate)
        switched = (proposed_char, proposed_baud)
        mode = self.use_baud_switch
        mode_l = mode.lower() if isinstance(mode, str) else mode
        if mode is True or mode_l in ("1", "true", "yes", "on"):
            return [switched]
        if mode is False or mode_l in ("0", "false", "no", "off"):
            return [stay]
        plans = [stay]
        if switched[1] != stay[1]:
            plans.append(switched)
        return plans

    def _prepare_initial_baud(self) -> None:
        if not self.serial_conn:
            return
        if self._active_baudrate != self.initial_baudrate:
            self.switch_baudrate(self.initial_baudrate)
        self.serial_conn.reset_input_buffer()
        time.sleep(0.3)

    def read_cycle(self) -> Optional[str]:
        """IDENT @ 300, dann ACK passend zur Lese-Baudrate. Bei Misserfolg der andere Modus."""
        if not self.serial_conn:
            return None

        plans: Optional[List[tuple]] = None
        for attempt in range(3):
            self._prepare_initial_baud()
            ident = self.identify()
            if not ident:
                time.sleep(0.4)
                continue
            if plans is None:
                plans = self._listen_plans(ident)
            if not plans:
                break

            baud_char, listen_baud = plans.pop(0)
            ack = self.build_ack(baud_char)
            logger.info(
                "Leseversuch %d: ACK %r, lesen bei %d Baud",
                attempt + 1,
                ack,
                listen_baud,
            )
            self.serial_conn.write(ack)
            self.serial_conn.flush()
            time.sleep(0.2)
            if listen_baud != self._active_baudrate:
                self.switch_baudrate(listen_baud)

            telegram = self.read_telegram()
            if telegram:
                return telegram
            logger.warning("Kein OBIS-Block bei %d Baud", listen_baud)
            if not plans:
                break
        return None

    def identify(self) -> Optional[str]:
        if not self.serial_conn:
            return None
        self.serial_conn.reset_input_buffer()
        time.sleep(0.3)
        self.serial_conn.write(self.IDENT_REQUEST)
        self.serial_conn.flush()
        logger.debug("Identifikationsanfrage gesendet")

        deadline = time.time() + 3.0
        while time.time() < deadline:
            response = self.serial_conn.readline()
            if not response:
                continue
            try:
                line = response.decode("ascii").strip()
            except UnicodeDecodeError:
                logger.debug("Ident-Zeile nicht ASCII: %r", response[:80])
                continue
            if not line:
                continue

            ident = self.normalize_ident_line(line)
            if ident is None or not self.is_valid_ident_line(ident):
                logger.debug("Uebersprungen (keine Ident): %s", line[:80])
                continue

            logger.info("Geraet: %s", ident)
            if line != ident:
                logger.debug("Ident normalisiert von: %s", line[:80])
            return ident

        logger.warning("Keine gueltige Identifikationsantwort innerhalb 3s")
        return None

    @staticmethod
    def _is_binary_frame(raw: bytes) -> bool:
        """Steuerzeichen ausser STX/ETX/CR/LF heissen: falsche Baudrate."""
        return any(b < 32 and b not in (0x02, 0x03, 0x0D, 0x0A) for b in raw)

    def read_telegram(self) -> Optional[str]:
        if not self.serial_conn:
            return None

        lines: List[str] = []
        start = time.time()
        last_data = start

        while time.time() - start < 240:
            raw = self.serial_conn.readline()
            if not raw:
                if lines and time.time() - last_data > 6:
                    break
                continue

            last_data = time.time()
            logger.debug("Rohzeile: %r", raw[:100])
            if self._is_binary_frame(raw):
                if not lines:
                    logger.warning(
                        "Binaerdaten bei %d Baud, Versuch abgebrochen",
                        self._active_baudrate,
                    )
                    return None
                continue

            payload = raw[1:] if raw.startswith(b"\x02") else raw
            if payload.endswith(b"\x03"):
                payload = payload[:-1]
            try:
                decoded = payload.decode("ascii").strip()
            except UnicodeDecodeError:
                if not lines:
                    logger.warning(
                        "Nicht-ASCII bei %d Baud, Versuch abgebrochen",
                        self._active_baudrate,
                    )
                    return None
                continue

            if not decoded or decoded.startswith("/"):
                continue
            if decoded.startswith("!"):
                break
            if "(" not in decoded:
                continue
            lines.append(decoded)

        if not lines:
            logger.warning("Kein Datenblock bei %d Baud (Timeout)", self._active_baudrate)
        return "\r\n".join(lines) if lines else None

    @staticmethod
    def _is_current_value(raw_code: str) -> bool:
        """Historische Abrechnungsperioden *01..*99 verwerfen, *255 behalten."""
        if "*" not in raw_code:
            return True
        storage = raw_code.split("*", 1)[1].strip()
        return storage == "255"

    def parse_values(self, telegram: str) -> Dict[str, float]:
        result: Dict[str, float] = {}
        for line in telegram.split("\r\n"):
            if "(" not in line:
                continue
            try:
                obis_end = line.index("(")
                raw_code = line[:obis_end].strip()
                if not self._is_current_value(raw_code):
                    continue
                obis_code = self.normalize_obis_code(raw_code)
                value_end = line.index(")", obis_end + 1)
                value_str = line[obis_end + 1 : value_end].replace(",", ".").strip()
                if "*" in value_str:
                    value_str = value_str.split("*", 1)[0].strip()
                result[obis_code] = float(value_str)
            except (ValueError, IndexError):
                if any(unit in line for unit in ("kW", "kVA", "kvar")):
                    logger.debug("Messwert nicht geparst: %s", line[:120])
                continue
        self._fill_missing_totals(result)
        return result

    @staticmethod
    def _fill_missing_totals(result: Dict[str, float]) -> None:
        """Tarifsummen und saldierte Leistung in die Summenregister legen."""
        for total, parts in (
            ("1-0:1.8.0", ("1-0:1.8.1", "1-0:1.8.2")),
            ("1-0:2.8.0", ("1-0:2.8.1", "1-0:2.8.2")),
        ):
            if total in result:
                continue
            present = [result[part] for part in parts if part in result]
            if present:
                result[total] = sum(present)

        if "1-0:1.7.0" not in result and "1-0:16.7.0" in result:
            power = result["1-0:16.7.0"]
            if "1-0:2.7.0" in result:
                result["1-0:1.7.0"] = power + result["1-0:2.7.0"]
            elif power >= 0:
                result["1-0:1.7.0"] = power
                result["1-0:2.7.0"] = 0.0
            else:
                result["1-0:1.7.0"] = 0.0
                result["1-0:2.7.0"] = -power
        elif "1-0:1.7.0" not in result and "1-0:1.4.0" in result:
            result["1-0:1.7.0"] = result["1-0:1.4.0"]


class ModbusInterface:
    """Modbus TCP Server mit Holding Registern."""

    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.context: Optional[ModbusServerContext] = None
        self._server_thread: Optional[threading.Thread] = None

    def initialize(self, num_registers: int = 100) -> None:
        hr_block = ModbusSequentialDataBlock(0, [0] * num_registers)
        slave = ModbusSlaveContext(hr=hr_block)
        self.context = ModbusServerContext(slaves=slave, single=True)
        logger.info("Modbus Registern initialisiert: 0-%d", num_registers - 1)

    def start_server(self) -> None:
        def _run():
            server = StartTcpServer(
                context=self.context,
                address=(self.host, self.port),
            )
            server.serve_forever()

        self._server_thread = threading.Thread(target=_run, daemon=True)
        self._server_thread.start()
        logger.info("Modbus TCP Server gestartet auf %s:%d", self.host, self.port)

    def write_holding(self, address: int, values: List[int]) -> None:
        if self.context:
            self.context[0].setValues(3, address, values)

    def update_obis(self, obis_code: str, value: float, mapping: dict) -> None:
        if obis_code not in mapping:
            return
        cfg = mapping[obis_code]
        addr = cfg["register"]
        factor = cfg.get("factor", 1)
        int_val = modbus_word(value, cfg.get("encoding", "scaled"), factor)
        int_val &= 0xFFFFFFFF
        high = (int_val >> 16) & 0xFFFF
        low = int_val & 0xFFFF
        self.write_holding(addr, [high, low])
        logger.debug(
            "Register %d: %s = %s",
            addr,
            obis_code,
            format_obis_value(value, cfg.get("encoding", "scaled")),
        )

    def update_status(
        self,
        base_address: int,
        connected: bool,
        errors: int,
        last_read: int,
        version: str,
    ) -> None:
        status_word = 1 if connected else 0
        self.write_holding(base_address + STATUS_OFFSET_CONNECTED, [status_word])
        self.write_holding(base_address + STATUS_OFFSET_ERRORS, [errors & 0xFFFF])
        self.write_holding(base_address + STATUS_OFFSET_LAST_READ, [last_read & 0xFFFF])
        ver_int = int(version.replace(".", "")[:4].ljust(4, "0"))
        self.write_holding(base_address + STATUS_OFFSET_VERSION, [ver_int])


class Gateway:
    """Haupt-Gateway Klasse."""

    def __init__(self, config: Config, print_values: bool = False):
        self.config = config
        self.d0 = D0Parser(
            config.serial_port,
            config.serial_baudrate,
            use_baud_switch=config.serial_use_baud_switch,
        )
        self.modbus = ModbusInterface(config.modbus_host, config.modbus_port)
        self.print_values = print_values
        self._running = False

    def start(self) -> None:
        logger.info("serial2modbustcp Gateway gestartet (v%s)", __version__)

        status_register = int(self.config.status_register)
        if not 0 <= status_register <= 0xFFFF - STATUS_REGISTER_COUNT + 1:
            raise ValueError("status_register liegt ausserhalb des Modbus-Adressraums")

        status_addresses = set(
            range(status_register, status_register + STATUS_REGISTER_COUNT),
        )
        for obis_code, mapping in self.config.obis_mapping.items():
            address = int(mapping["register"])
            if status_addresses.intersection((address, address + 1)):
                raise ValueError(
                    f"Statusregister ueberlappt OBIS-Mapping {obis_code}",
                )

        max_reg = max(
            (m["register"] + 2 for m in self.config.obis_mapping.values()),
            default=20,
        )
        max_reg = max(max_reg, status_register + STATUS_REGISTER_COUNT)
        # ModbusSlaveContext verwendet standardmaessig eine Adressoffsetschicht.
        self.modbus.initialize(max_reg + 1)
        self.modbus.start_server()

        try:
            self.d0.connect()
        except Exception as e:
            logger.error("Verbindung fehlgeschlagen: %s", e)

        self._running = True
        self._read_loop()

    def _read_loop(self) -> None:
        error_count = 0
        last_read_ts = 0

        while self._running:
            values: Dict[str, float] = {}
            try:
                telegram = self.d0.read_cycle()
                if telegram:
                    values = self.d0.parse_values(telegram)
                    for obis_code, value in values.items():
                        self.modbus.update_obis(obis_code, value, self.config.obis_mapping)
                    error_count = 0
                    last_read_ts = int(time.time()) & 0xFFFF
                    logger.info("Verarbeitet: %d OBIS-Werte", len(values))
            except Exception as e:
                error_count += 1
                logger.error("Lesefehler: %s", e)

            if self.print_values:
                self._print_values(values)

            self.modbus.update_status(
                base_address=int(self.config.status_register),
                connected=self.d0.is_connected,
                errors=error_count,
                last_read=last_read_ts,
                version=__version__,
            )

            time.sleep(self.config.read_interval)

    def _print_values(self, values: Dict[str, float]) -> None:
        """Konfigurierte und zusaetzlich ermittelte Werte auf stdout ausgeben."""
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        print(f"\n[{timestamp}] Ermittelte Zaehlerwerte")
        print("-" * 100)
        print(
            f"{'OBIS':<18} {'Name':<34} {'Wert':>14} "
            f"{'Einheit':<10} {'Register':>8}",
        )

        configured_codes = set(self.config.obis_mapping)
        entries = sorted(
            self.config.obis_mapping.items(),
            key=lambda entry: entry[1]["register"],
        )
        for obis_code, mapping in entries:
            value = values.get(obis_code)
            encoding = str(mapping.get("encoding", "scaled"))
            formatted_value = format_obis_value(value, encoding) if value is not None else "n/a"
            name = str(mapping.get("name", ""))
            unit = str(mapping.get("unit", ""))
            register = mapping["register"]
            print(
                f"{obis_code:<18} {name:<34.34} {formatted_value:>14} "
                f"{unit:<10.10} {register:>8}",
            )

        additional_values = sorted(set(values) - configured_codes)
        if additional_values:
            print("\nWeitere ermittelte, nicht konfigurierte OBIS-Werte:")
            for obis_code in additional_values:
                print(f"  {obis_code:<18} {values[obis_code]:.3f}")

        sys.stdout.flush()

    def stop(self) -> None:
        self._running = False
        self.d0.disconnect()
        logger.info("Gateway gestoppt")


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(
        description="D0/IEC 62056-21 to Modbus TCP Gateway",
    )
    parser.add_argument(
        "-c", "--config",
        default="/etc/serial2modbustcp/config.json",
        help="Konfigurationsdatei (default: /etc/serial2modbustcp/config.json)",
    )
    parser.add_argument("-p", "--port", help="Serieller Port (ueberschreibt Config)")
    parser.add_argument(
        "-H",
        "--modbus-host",
        help="Modbus TCP Host (ueberschreibt Config)",
    )
    parser.add_argument(
        "-P",
        "--modbus-port",
        type=int,
        help="Modbus TCP Port (ueberschreibt Config)",
    )
    parser.add_argument(
        "-i",
        "--interval",
        type=int,
        help="Lese-Intervall in Sekunden (ueberschreibt Config)",
    )
    parser.add_argument(
        "-v",
        "--values",
        action="store_true",
        help="Konfigurierte und ermittelte Werte auf stdout ausgeben",
    )
    parser.add_argument("-d", "--debug", action="store_true", help="Debug-Logging aktivieren")

    args = parser.parse_args()

    level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    config = Config(args.config)
    if args.port is not None:
        config._data["serial_port"] = args.port
    if args.modbus_host is not None:
        config._data["modbus_host"] = args.modbus_host
    if args.modbus_port is not None:
        config._data["modbus_port"] = args.modbus_port
    if args.interval is not None:
        config._data["read_interval"] = args.interval

    gateway = Gateway(config, print_values=args.values)
    try:
        gateway.start()
    except KeyboardInterrupt:
        gateway.stop()
    except Exception as e:
        logger.critical("Fataler Fehler: %s", e)
        sys.exit(1)


if __name__ == "__main__":
    main()
