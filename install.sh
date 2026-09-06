#!/bin/bash
set -euo pipefail

INSTALL_DIR="/opt/serial2modbustcp"
CONFIG_DIR="/etc/serial2modbustcp"
SERVICE_USER="modbus"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== serial2modbustcp Installation ==="

# Systemuser erstellen
if ! id -u "$SERVICE_USER" &>/dev/null; then
    echo "Erstelle User: $SERVICE_USER"
    sudo useradd -r -s /bin/false -d "$INSTALL_DIR" "$SERVICE_USER"
    sudo usermod -aG dialout "$SERVICE_USER"
fi

# Installationsverzeichnis
echo "Kopiere Dateien nach $INSTALL_DIR"
sudo mkdir -p "$INSTALL_DIR"
sudo cp "$SCRIPT_DIR/serial2modbustcp.py" "$INSTALL_DIR/"
sudo cp "$SCRIPT_DIR/requirements.txt" "$INSTALL_DIR/"

# Virtual Environment erstellen
echo "Erstelle Python Virtual Environment"
sudo python3 -m venv "$INSTALL_DIR/venv"
sudo "$INSTALL_DIR/venv/bin/pip" install --upgrade pip
sudo "$INSTALL_DIR/venv/bin/pip" install -r "$INSTALL_DIR/requirements.txt"

# Konfiguration
echo "Erstelle Konfigurationsverzeichnis"
sudo mkdir -p "$CONFIG_DIR"
if [ ! -f "$CONFIG_DIR/config.json" ]; then
    sudo cp "$SCRIPT_DIR/config.json.example" "$CONFIG_DIR/config.json"
    echo "Konfiguration unter $CONFIG_DIR/config.json erstellt"
    echo "Bitte anpassen!"
else
    echo "Konfiguration existiert bereits - wird nicht ueberschrieben"
fi

# Berechtigungen
sudo chown -R "$SERVICE_USER:dialout" "$INSTALL_DIR"
sudo chmod 644 "$CONFIG_DIR/config.json"

# systemd Service
echo "Installiere systemd Service"
sudo cp "$SCRIPT_DIR/serial2modbustcp.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable serial2modbustcp.service

echo ""
echo "=== Installation abgeschlossen ==="
echo ""
echo "Naechste Schritte:"
echo "  1. Konfiguration anpassen:  sudo nano $CONFIG_DIR/config.json"
echo "  2. Service starten:         sudo systemctl start serial2modbustcp"
echo "  3. Status pruefen:          sudo systemctl status serial2modbustcp"
echo "  4. Logs anzeigen:           sudo journalctl -u serial2modbustcp -f"
echo ""
echo "Modbus TCP Server laeuft auf Port 502"
