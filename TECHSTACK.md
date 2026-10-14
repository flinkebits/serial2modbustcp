# Techstack

## Überblick

`serial2modbustcp` ist ein schlankes Gateway für Linux. Es liest Messwerte eines
Stromzählers über die serielle D0-Schnittstelle (IEC 62056-21), verarbeitet
OBIS-Kennzahlen und stellt die Werte als Modbus-TCP-Holding-Register bereit.

## Programmiersprache und Laufzeit

- **Python 3** als einzige Anwendungssprache
- **Python-Standardbibliothek** für:
  - JSON-Konfiguration (`json`)
  - Logging (`logging`)
  - Threads (`threading`)
  - Zeitsteuerung (`time`)
  - Kommandozeilenargumente (`argparse`)
  - Dateipfade und Typannotationen (`pathlib`, `typing`)
- **Python Virtual Environment (`venv`)** zur isolierten Installation

Eine konkrete minimale Python-Version ist im Projekt derzeit nicht festgelegt.

## Externe Python-Abhängigkeiten

Die Abhängigkeiten werden über `requirements.txt` und `pip` verwaltet:

- **pyserial >= 3.5**  
  Zugriff auf die serielle Geräteschnittstelle, standardmäßig
  `/dev/ttyUSB0`, mit den für D0 benötigten Parametern.

- **pymodbus >= 3.6.0, < 3.10.0**  
  Implementierung des Modbus-TCP-Servers und Verwaltung der
  Holding-Register. Die Obergrenze schützt die Anwendung vor inkompatiblen
  API-Änderungen neuerer PyModbus-Versionen.

Die Abhängigkeiten sind nicht durch eine Lock-Datei vollständig reproduzierbar
festgeschrieben.

## Protokolle und Datenformate

- **IEC 62056-21 / D0** für die Kommunikation mit dem Stromzähler
- **OBIS-Kennzahlen** zur Identifikation der Messwerte
- **Modbus TCP** zur Bereitstellung der Messwerte im Netzwerk
- **JSON** für die lokale Konfiguration und das OBIS-Register-Mapping
- Messwerte werden skaliert und auf jeweils zwei 16-Bit-Holding-Register
  verteilt.

## Anwendungsarchitektur

Die Gateway-Implementierung befindet sich in `serial2modbustcp.py` und ist in
vier zentrale Klassen aufgeteilt:

- `Config`: Laden und Zusammenführen der JSON-Konfiguration
- `D0Parser`: serielle Verbindung, D0-Kommunikation und Telegramm-Auswertung
- `ModbusInterface`: Modbus-Datenspeicher und TCP-Server
- `Gateway`: Orchestrierung, periodische Lesezyklen und Statusaktualisierung

Der Modbus-Server läuft in einem Hintergrund-Thread. Der Hauptthread liest in
einer Schleife die Zählerdaten und schreibt sie in den gemeinsamen
Modbus-Kontext.

`test/modbus_live_test.py` ist ein eigenständiger Modbus-TCP-Testclient. Er
verwendet nur die Python-Standardbibliothek und kann daher ohne das Virtual
Environment des Gateways ausgeführt werden.

## Installation und Betrieb

- **Linux** als Zielplattform
- **Bash**-Installationsskript (`install.sh`)
- Installation unter `/opt/serial2modbustcp`
- Konfiguration unter `/etc/serial2modbustcp/config.json`
- Eigener Systembenutzer `modbus` mit Zugriff über die Gruppe `dialout`
- **systemd** für Autostart, Neustart bei Fehlern und Protokollierung über das
  Journal
- systemd-Härtung unter anderem mit `NoNewPrivileges`, `ProtectSystem`,
  `ProtectHome` und Einschränkungen für Kernel- und Namespace-Zugriffe

Der Modbus-Server lauscht standardmäßig auf `0.0.0.0:502`. Da Port 502 ein
privilegierter Port ist, erhält der ansonsten unprivilegierte systemd-Dienst
gezielt die Capability `CAP_NET_BIND_SERVICE`.

## Entwicklungsstand und Tooling

Aktuell sind keine der folgenden Komponenten im Repository konfiguriert:

- automatisierte Tests oder Test-Framework
- Linter und Formatter
- statische Typprüfung
- Paket-Metadaten wie `pyproject.toml`
- CI/CD-Pipeline
- Containerisierung
- Dependency-Lock-Datei

Das Projekt ist daher technisch einfach zu installieren, bietet aber noch
keine automatisierte Qualitätssicherung oder vollständig reproduzierbare
Builds.
