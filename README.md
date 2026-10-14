# Stromzaehler-modbus

Gateway von D0/IEC 62056-21 zu Modbus TCP.

## Gateway manuell mit Wertausgabe starten

Mit `-v` gibt das Gateway bei jedem Lesezyklus alle konfigurierten OBIS-Werte
auf `stdout` aus. Nicht im aktuellen Telegramm ermittelte Werte erscheinen als
`n/a`. Zusätzlich ermittelte, aber nicht konfigurierte OBIS-Werte werden
separat aufgeführt.

```bash
python3 serial2modbustcp.py -c ./config.json -v
```

Der systemd-Dienst verwendet diesen Modus nicht.

## Modbus-Werte live testen

Der Testclient verwendet ausschließlich die Python-Standardbibliothek und
benötigt weder ein Virtual Environment noch zusätzliche Pakete:

```bash
./test/modbus_live_test.py
```

Er verwendet zuerst `/etc/serial2modbustcp/config.json`. Ist diese Datei nicht
vorhanden, werden eine lokale `config.json` oder `config.json.example`
verwendet. Ein in der Konfiguration eingetragenes `0.0.0.0` wird für den
lokalen Test automatisch als `127.0.0.1` angesprochen.

Einen Server auf einem anderen Rechner testen:

```bash
./test/modbus_live_test.py --host 192.168.1.50
```

Weitere nützliche Optionen:

```bash
# Nur einen Messwert-Snapshot ausgeben
./test/modbus_live_test.py --once

# Andere Konfiguration und Aktualisierung alle fünf Sekunden
./test/modbus_live_test.py --config ./config.json --interval 5
```

Der Client liest die OBIS-Zuordnung, Skalierungsfaktoren, den Modbus-Port und
die Adresse der Statusregister aus derselben JSON-Konfiguration wie der
Server. `Strg+C` beendet die Live-Anzeige.

