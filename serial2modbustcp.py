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
    "0-0:1.0.0": {"register": 20, "name": "Zeitstempel", "unit": "", "factor": 1},
}

STATUS_REGISTER_CONNECTED = 0
STATUS_REGISTER_ERRORS = 1
STATUS_REGISTER_LAST_READ = 2
STATUS_REGISTER_VERSION = 3

# IEC 62056-21 Baudraten-Kennung (Zeichen nach Herstellerkennung in der Ident-Zeile)
IEC62056_BAUD_RATES = {
    "0": 300,
    "1": 600,
    "2": 1200,
    "3": 2400,
    "4": 4800,
    "5": 9600,
    "6": 19200,
    "7": 9600,
    "8": 57600,
    "9": 115200,
}


class Config:
    """Laden und Validieren der Konfiguration."""

    DEFAULTS = {
        "serial_port": "/dev/ttyUSB0",
        "serial_baudrate": 300,
        "modbus_host": "0.0.0.0",
        "modbus_port": 502,
        "read_interval": 10,
        "obis_mapping": DEFAULT_OBIS_MAPPING,
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
    ACK = bytes([0x06, 0x30, 0x30, 0x30, 0x0D, 0x0A])

    def __init__(self, port: str, initial_baudrate: int = 300):
        self.port = port
        self.initial_baudrate = initial_baudrate
        self.serial_conn: Optional[serial.Serial] = None
        self._active_baudrate = initial_baudrate

    def connect(self) -> None:
        self.serial_conn = serial.Serial(
            port=self.port,
            baudrate=self.initial_baudrate,
            bytesize=serial.SEVENBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            timeout=2,
        )
        self._active_baudrate = self.initial_baudrate
        logger.info("Verbunden mit %s @ %d baud", self.port, self.initial_baudrate)

    def disconnect(self) -> None:
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            logger.info("Getrennt")

    @property
    def is_connected(self) -> bool:
        return self.serial_conn is not None and self.serial_conn.is_open

    def switch_baudrate(self, baudrate: int) -> None:
        if not self.serial_conn:
            return
        if self._active_baudrate == baudrate:
            return
        self.serial_conn.close()
        time.sleep(0.1)
        self.serial_conn = serial.Serial(
            port=self.port,
            baudrate=baudrate,
            bytesize=serial.SEVENBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            timeout=2,
        )
        self._active_baudrate = baudrate
        logger.info("Baudrate gewechselt zu %d", baudrate)

    @staticmethod
    def baudrate_from_ident(ident: str) -> int:
        """Baudrate aus Ident-Zeile, z. B. /LGZ5\\2... -> 9600."""
        slash = ident.find("/")
        if slash < 0:
            return 300
        after = ident[slash + 1 :]
        if len(after) < 4:
            return 300
        baud_char = after[3]
        rate = IEC62056_BAUD_RATES.get(baud_char)
        if rate is None:
            logger.warning("Unbekannte Baud-Kennung '%s' in Ident, verwende 300", baud_char)
            return 300
        return rate

    @staticmethod
    def normalize_obis_code(obis_code: str) -> str:
        """OBIS vereinheitlichen: Speicher *255 entfernen, Kurzform 1.8.0 -> 1-0:1.8.0."""
        if "*" in obis_code:
            obis_code = obis_code.split("*", 1)[0]
        if "-" not in obis_code and obis_code.count(".") >= 2:
            obis_code = f"1-0:{obis_code}"
        return obis_code

    def read_cycle(self) -> Optional[str]:
        """Vollstaendiger D0-Lesezyklus: IDENT @ 300, ACK, Baudwechsel, Telegramm."""
        if not self.serial_conn:
            return None

        if self._active_baudrate != self.initial_baudrate:
            self.switch_baudrate(self.initial_baudrate)

        ident = self.identify()
        if not ident:
            return None

        data_baud = self.baudrate_from_ident(ident)
        logger.info("Daten-Baudrate laut Ident: %d", data_baud)
        if data_baud != self.initial_baudrate:
            time.sleep(0.05)
            self.switch_baudrate(data_baud)
        self.serial_conn.reset_input_buffer()
        return self.read_telegram()

    def identify(self) -> Optional[str]:
        if not self.serial_conn:
            return None
        self.serial_conn.reset_input_buffer()
        self.serial_conn.write(self.IDENT_REQUEST)
        logger.debug("Identifikationsanfrage gesendet")

        time.sleep(0.2)
        response = self.serial_conn.readline()
        if not response:
            logger.warning("Keine Identifikationsantwort")
            return None

        try:
            ident = response.decode("ascii").strip()
            logger.info("Geraet: %s", ident)
            self.serial_conn.write(self.ACK)
            logger.debug("ACK gesendet")
            return ident
        except UnicodeDecodeError:
            logger.error("Ungueltige Identifikationsantwort")
            return None

    def _validate_checksum(self, raw: bytes) -> bool:
        if len(raw) < 4:
            return False
        data = raw[:-3]
        checksum_byte = raw[-3]
        calculated = 0
        for b in data:
            calculated ^= b
        return calculated == checksum_byte

    def read_telegram(self) -> Optional[str]:
        if not self.serial_conn:
            return None

        lines: List[str] = []
        start = time.time()

        while time.time() - start < 5:
            raw = self.serial_conn.readline()
            if not raw:
                continue

            try:
                decoded = raw.decode("ascii").strip()
            except UnicodeDecodeError:
                continue

            if not decoded:
                continue

            if decoded.startswith("!"):
                break

            if decoded.startswith("/"):
                continue

            if not self._validate_checksum(raw):
                logger.debug("Checksum-Fehler: %s", decoded)
                continue

            lines.append(decoded)

        return "\r\n".join(lines) if lines else None

    def parse_values(self, telegram: str) -> Dict[str, float]:
        result: Dict[str, float] = {}
        for line in telegram.split("\r\n"):
            if "(" not in line:
                continue
            try:
                obis_end = line.index("(")
                obis_code = self.normalize_obis_code(line[:obis_end])
                value_end = line.index(")", obis_end + 1)
                value_str = line[obis_end + 1 : value_end]
                if "*" in value_str:
                    value_str = value_str.split("*")[0]
                result[obis_code] = float(value_str)
            except (ValueError, IndexError):
                continue
        return result


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
        int_val = int(value * factor * 1000)
        high = (int_val >> 16) & 0xFFFF
        low = int_val & 0xFFFF
        self.write_holding(addr, [high, low])
        logger.debug("Register %d: %s = %.3f", addr, obis_code, value)

    def update_status(self, connected: bool, errors: int, last_read: int, version: str) -> None:
        status_word = 1 if connected else 0
        self.write_holding(STATUS_REGISTER_CONNECTED, [status_word])
        self.write_holding(STATUS_REGISTER_ERRORS, [errors & 0xFFFF])
        self.write_holding(STATUS_REGISTER_LAST_READ, [last_read & 0xFFFF])
        ver_int = int(version.replace(".", "")[:4].ljust(4, "0"))
        self.write_holding(STATUS_REGISTER_VERSION, [ver_int])


class Gateway:
    """Haupt-Gateway Klasse."""

    def __init__(self, config: Config):
        self.config = config
        self.d0 = D0Parser(config.serial_port, config.serial_baudrate)
        self.modbus = ModbusInterface(config.modbus_host, config.modbus_port)
        self._running = False

    def start(self) -> None:
        logger.info("serial2modbustcp Gateway gestartet (v%s)", __version__)

        max_reg = max(
            (m["register"] + 2 for m in self.config.obis_mapping.values()),
            default=20,
        )
        self.modbus.initialize(max(max_reg, STATUS_REGISTER_VERSION + 1))
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

            self.modbus.update_status(
                connected=self.d0.is_connected,
                errors=error_count,
                last_read=last_read_ts,
                version=__version__,
            )

            time.sleep(self.config.read_interval)

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
    parser.add_argument("-p", "--port", default="/dev/ttyUSB0", help="Serieller Port")
    parser.add_argument("-H", "--modbus-host", default="0.0.0.0", help="Modbus TCP Host")
    parser.add_argument("-P", "--modbus-port", type=int, default=502, help="Modbus TCP Port")
    parser.add_argument("-i", "--interval", type=int, default=10, help="Lese-Intervall (Sekunden)")
    parser.add_argument("-d", "--debug", action="store_true", help="Debug-Logging aktivieren")

    args = parser.parse_args()

    level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    config = Config(args.config)
    config._data["serial_port"] = args.port
    config._data["modbus_host"] = args.modbus_host
    config._data["modbus_port"] = args.modbus_port
    config._data["read_interval"] = args.interval

    gateway = Gateway(config)
    try:
        gateway.start()
    except KeyboardInterrupt:
        gateway.stop()
    except Exception as e:
        logger.critical("Fataler Fehler: %s", e)
        sys.exit(1)


if __name__ == "__main__":
    main()
