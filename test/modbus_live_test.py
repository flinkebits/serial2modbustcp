#!/usr/bin/env python3
"""Live-Testclient fuer serial2modbustcp ohne externe Abhaengigkeiten."""

import argparse
import json
import socket
import struct
import sys
import time
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


DEFAULT_CONFIG_PATH = Path("/etc/serial2modbustcp/config.json")
MAX_READ_REGISTERS = 125
STATUS_REGISTER_COUNT = 4


class ModbusError(Exception):
    """Fehler bei der Modbus-TCP-Kommunikation."""


class ModbusTcpClient:
    """Minimaler Modbus-TCP-Client fuer Holding Register (Function Code 3)."""

    def __init__(self, host: str, port: int, unit_id: int, timeout: float):
        self.host = host
        self.port = port
        self.unit_id = unit_id
        self.timeout = timeout
        self._socket: Optional[socket.socket] = None
        self._transaction_id = 0

    def connect(self) -> None:
        self.close()
        self._socket = socket.create_connection(
            (self.host, self.port),
            timeout=self.timeout,
        )
        self._socket.settimeout(self.timeout)

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    @property
    def is_connected(self) -> bool:
        return self._socket is not None

    def _receive_exactly(self, size: int) -> bytes:
        if self._socket is None:
            raise ModbusError("Keine Verbindung")

        result = bytearray()
        while len(result) < size:
            chunk = self._socket.recv(size - len(result))
            if not chunk:
                raise ModbusError("Server hat die Verbindung geschlossen")
            result.extend(chunk)
        return bytes(result)

    def read_holding_registers(self, address: int, count: int) -> List[int]:
        if self._socket is None:
            raise ModbusError("Keine Verbindung")
        if not 0 <= address <= 0xFFFF:
            raise ValueError("Registeradresse ausserhalb des gueltigen Bereichs")
        if not 1 <= count <= MAX_READ_REGISTERS:
            raise ValueError("Ungueltige Anzahl Register")
        if address + count > 0x10000:
            raise ValueError("Registerbereich ueberschreitet 65535")

        self._transaction_id = (self._transaction_id % 0xFFFF) + 1
        request_pdu = struct.pack(">BHH", 3, address, count)
        request = struct.pack(
            ">HHHB",
            self._transaction_id,
            0,
            len(request_pdu) + 1,
            self.unit_id,
        ) + request_pdu
        self._socket.sendall(request)

        response_header = self._receive_exactly(7)
        transaction_id, protocol_id, length, unit_id = struct.unpack(
            ">HHHB",
            response_header,
        )
        if transaction_id != self._transaction_id:
            raise ModbusError("Unerwartete Transaktions-ID")
        if protocol_id != 0:
            raise ModbusError("Unerwartete Protokoll-ID")
        if unit_id != self.unit_id:
            raise ModbusError("Unerwartete Unit-ID")
        expected_length = count * 2 + 3
        if length not in (3, expected_length):
            raise ModbusError("Ungueltige Antwortlaenge")

        response_pdu = self._receive_exactly(length - 1)
        function_code = response_pdu[0]
        if function_code == 0x83:
            exception_code = response_pdu[1] if len(response_pdu) > 1 else -1
            raise ModbusError(
                f"Modbus-Ausnahme fuer Function Code 3: {exception_code}",
            )
        if function_code != 3 or len(response_pdu) < 2:
            raise ModbusError("Unerwartete Modbus-Antwort")

        byte_count = response_pdu[1]
        register_data = response_pdu[2:]
        if byte_count != count * 2 or len(register_data) != byte_count:
            raise ModbusError("Unvollstaendige Registerdaten")

        return list(struct.unpack(f">{count}H", register_data))


def resolve_config_path(explicit_path: Optional[str]) -> Path:
    if explicit_path:
        return Path(explicit_path)

    script_dir = Path(__file__).resolve().parent
    candidates = (
        DEFAULT_CONFIG_PATH,
        script_dir / "config.json",
        script_dir / "config.json.example",
        script_dir.parent / "config.json",
        script_dir.parent / "config.json.example",
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return DEFAULT_CONFIG_PATH


def load_config(path: Path) -> dict:
    try:
        with path.open(encoding="utf-8") as config_file:
            config = json.load(config_file)
    except FileNotFoundError as exc:
        raise ValueError(f"Konfiguration nicht gefunden: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"Ungueltige JSON-Konfiguration: {exc}") from exc

    mapping = config.get("obis_mapping")
    if not isinstance(mapping, dict) or not mapping:
        raise ValueError("Konfiguration enthaelt kein gueltiges obis_mapping")
    return config


def validate_mapping(mapping: dict) -> None:
    for obis_code, item in mapping.items():
        if not isinstance(item, dict):
            raise ValueError(f"Ungueltiger Mapping-Eintrag: {obis_code}")
        address = item.get("register")
        if not isinstance(address, int) or not 0 <= address <= 0xFFFE:
            raise ValueError(f"Ungueltige Registeradresse fuer {obis_code}")
        factor = item.get("factor", 1)
        if not isinstance(factor, (int, float)) or factor == 0:
            raise ValueError(f"Ungueltiger Faktor fuer {obis_code}")


def build_read_ranges(addresses: Iterable[int]) -> List[Tuple[int, int]]:
    sorted_addresses = sorted(set(addresses))
    ranges: List[Tuple[int, int]] = []
    index = 0

    while index < len(sorted_addresses):
        start = sorted_addresses[index]
        end = start + 1
        index += 1
        while index < len(sorted_addresses):
            candidate = sorted_addresses[index]
            if candidate - start >= MAX_READ_REGISTERS:
                break
            end = candidate + 1
            index += 1
        ranges.append((start, end - start))

    return ranges


def read_snapshot(
    client: ModbusTcpClient,
    ranges: Iterable[Tuple[int, int]],
) -> Dict[int, int]:
    registers: Dict[int, int] = {}
    for start, count in ranges:
        values = client.read_holding_registers(start, count)
        registers.update(
            (start + offset, value)
            for offset, value in enumerate(values)
        )
    return registers


def decode_value(high: int, low: int, factor: float) -> float:
    raw_value = (high << 16) | low
    if raw_value & 0x80000000:
        raw_value -= 1 << 32
    return raw_value / (factor * 1000)


def seconds_since_last_read(raw_timestamp: int) -> int:
    now = int(time.time())
    timestamp = (now & ~0xFFFF) | raw_timestamp
    if timestamp > now:
        timestamp -= 0x10000
    return now - timestamp


def render_snapshot(
    host: str,
    port: int,
    mapping: dict,
    status_register: int,
    registers: Dict[int, int],
    clear_screen: bool,
) -> None:
    if clear_screen:
        print("\033[2J\033[H", end="")

    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"serial2modbustcp Live-Test  {host}:{port}  {timestamp}")
    print("-" * 88)
    print(f"{'OBIS':<18} {'Name':<34} {'Wert':>16}  Einheit")

    entries = sorted(
        mapping.items(),
        key=lambda entry: entry[1]["register"],
    )
    for obis_code, item in entries:
        address = item["register"]
        value = decode_value(
            registers[address],
            registers[address + 1],
            float(item.get("factor", 1)),
        )
        name = str(item.get("name", ""))
        unit = str(item.get("unit", ""))
        print(f"{obis_code:<18} {name:<34.34} {value:>16.3f}  {unit}")

    connected = "ja" if registers[status_register] else "nein"
    errors = registers[status_register + 1]
    last_read = registers[status_register + 2]
    last_read_text = (
        f"{seconds_since_last_read(last_read)}s"
        if last_read
        else "unbekannt"
    )
    version = registers[status_register + 3]
    print("-" * 88)
    print(
        f"Zaehler verbunden: {connected} | Fehler: {errors} | "
        f"Letzter Wert vor: {last_read_text} | Version: {version}",
    )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Liest die Werte eines serial2modbustcp-Servers live aus. "
            "Es werden keine externen Python-Pakete benoetigt."
        ),
    )
    parser.add_argument(
        "-c",
        "--config",
        help=(
            "Konfigurationsdatei; Standard: /etc/serial2modbustcp/config.json "
            "oder lokale config.json(.example)"
        ),
    )
    parser.add_argument("-H", "--host", help="Serveradresse statt modbus_host")
    parser.add_argument("-P", "--port", type=int, help="TCP-Port statt modbus_port")
    parser.add_argument("-i", "--interval", type=float, default=2.0)
    parser.add_argument("-t", "--timeout", type=float, default=3.0)
    parser.add_argument("-u", "--unit-id", type=int, default=1)
    parser.add_argument("--once", action="store_true", help="Nur einmal auslesen")
    parser.add_argument(
        "--no-clear",
        action="store_true",
        help="Terminal zwischen Aktualisierungen nicht leeren",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    config_path = resolve_config_path(args.config)
    try:
        config = load_config(config_path)
        mapping = config["obis_mapping"]
        validate_mapping(mapping)
        status_register = int(config.get("status_register", 100))
        if not 0 <= status_register <= 0xFFFC:
            raise ValueError("Ungueltiges status_register")
        status_addresses = set(
            range(status_register, status_register + STATUS_REGISTER_COUNT),
        )
        for obis_code, item in mapping.items():
            address = item["register"]
            if status_addresses.intersection((address, address + 1)):
                raise ValueError(
                    f"Statusregister ueberlappt OBIS-Mapping {obis_code}",
                )
        if args.interval <= 0 or args.timeout <= 0:
            raise ValueError("Intervall und Timeout muessen groesser als 0 sein")
        if not 0 <= args.unit_id <= 0xFF:
            raise ValueError("Unit-ID muss zwischen 0 und 255 liegen")
        port = (
            args.port
            if args.port is not None
            else int(config.get("modbus_port", 502))
        )
        if not 1 <= port <= 0xFFFF:
            raise ValueError("Port muss zwischen 1 und 65535 liegen")
    except (TypeError, ValueError) as exc:
        print(f"Konfigurationsfehler: {exc}", file=sys.stderr)
        return 2

    host = args.host or str(config.get("modbus_host", "127.0.0.1"))
    if host in ("0.0.0.0", ""):
        host = "127.0.0.1"
    elif host == "::":
        host = "::1"

    addresses = {
        address
        for item in mapping.values()
        for address in (item["register"], item["register"] + 1)
    }
    addresses.update(range(status_register, status_register + STATUS_REGISTER_COUNT))
    ranges = build_read_ranges(addresses)
    clear_screen = sys.stdout.isatty() and not args.no_clear
    client = ModbusTcpClient(host, port, args.unit_id, args.timeout)

    try:
        while True:
            try:
                if not client.is_connected:
                    client.connect()
                registers = read_snapshot(client, ranges)
                render_snapshot(
                    host,
                    port,
                    mapping,
                    status_register,
                    registers,
                    clear_screen,
                )
            except (OSError, ModbusError) as exc:
                client.close()
                if clear_screen:
                    print("\033[2J\033[H", end="")
                print(
                    f"{time.strftime('%Y-%m-%d %H:%M:%S')} "
                    f"Verbindungsfehler zu {host}:{port}: {exc}",
                    file=sys.stderr,
                )
                if args.once:
                    return 1

            if args.once:
                return 0
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nLive-Test beendet.")
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
