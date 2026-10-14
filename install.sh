#!/bin/bash
set -euo pipefail

readonly INSTALL_DIR="/opt/serial2modbustcp"
readonly CONFIG_DIR="/etc/serial2modbustcp"
readonly SERVICE_USER="modbus"
readonly SERVICE_GROUP="dialout"
readonly SERVICE_NAME="serial2modbustcp.service"
readonly SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME"
readonly USER_MARKER="$INSTALL_DIR/.service-user-created"
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
    cat <<EOF
Verwendung: $(basename "$0") [OPTION]

Ohne Option wird serial2modbustcp installiert oder aktualisiert.

Optionen:
  -u  Deinstalliert Anwendung und systemd-Dienst
  -h  Zeigt diese Hilfe

Bei der Deinstallation bleibt $CONFIG_DIR erhalten.
Der Service-Benutzer wird nur entfernt, wenn dieses Skript ihn angelegt hat.
EOF
}

run_as_root() {
    if (( EUID == 0 )); then
        "$@"
    else
        sudo "$@"
    fi
}

require_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "Fehler: Benoetigtes Kommando nicht gefunden: $1" >&2
        exit 1
    fi
}

check_privilege_tool() {
    if (( EUID != 0 )); then
        require_command sudo
    fi
}

install_gateway() {
    local required_file
    local user_created=false

    echo "=== serial2modbustcp Installation ==="

    require_command python3
    require_command systemctl
    check_privilege_tool

    for required_file in \
        serial2modbustcp.py \
        requirements.txt \
        config.json.example \
        "$SERVICE_NAME"; do
        if [[ ! -f "$SCRIPT_DIR/$required_file" ]]; then
            echo "Fehler: Installationsdatei fehlt: $SCRIPT_DIR/$required_file" >&2
            exit 1
        fi
    done

    echo "Bereite Installationsverzeichnis vor: $INSTALL_DIR"
    run_as_root install -d -m 0755 "$INSTALL_DIR"

    if ! id -u "$SERVICE_USER" >/dev/null 2>&1; then
        echo "Erstelle Benutzer: $SERVICE_USER"
        run_as_root useradd -r -s /usr/sbin/nologin -d "$INSTALL_DIR" "$SERVICE_USER"
        user_created=true
    fi

    # Auch bei einem bereits vorhandenen Benutzer die benoetigte Gruppe setzen.
    run_as_root usermod -aG "$SERVICE_GROUP" "$SERVICE_USER"
    if [[ "$user_created" == true ]]; then
        run_as_root touch "$USER_MARKER"
    fi

    echo "Installiere Anwendungsdateien"
    run_as_root install -m 0644 "$SCRIPT_DIR/serial2modbustcp.py" "$INSTALL_DIR/"
    run_as_root install -m 0644 "$SCRIPT_DIR/requirements.txt" "$INSTALL_DIR/"

    if [[ ! -x "$INSTALL_DIR/venv/bin/python" ]]; then
        echo "Erstelle Python Virtual Environment"
        run_as_root python3 -m venv "$INSTALL_DIR/venv"
    else
        echo "Python Virtual Environment ist bereits vorhanden"
    fi
    run_as_root "$INSTALL_DIR/venv/bin/python" -m pip install --upgrade pip
    run_as_root "$INSTALL_DIR/venv/bin/python" -m pip install \
        -r "$INSTALL_DIR/requirements.txt"

    echo "Bereite Konfiguration vor"
    run_as_root install -d -m 0755 "$CONFIG_DIR"
    if [[ ! -f "$CONFIG_DIR/config.json" ]]; then
        run_as_root install -m 0644 \
            "$SCRIPT_DIR/config.json.example" "$CONFIG_DIR/config.json"
        echo "Konfiguration unter $CONFIG_DIR/config.json erstellt"
        echo "Bitte anpassen!"
    else
        echo "Konfiguration existiert bereits und bleibt unveraendert"
        run_as_root chmod 0644 "$CONFIG_DIR/config.json"
    fi

    # Der Dienst muss seine Programmdateien nur lesen und ausfuehren. Root als
    # Besitzer verhindert, dass ein kompromittierter Dienst sie veraendern kann.
    run_as_root chown -R root:root "$INSTALL_DIR"

    echo "Installiere systemd-Dienst"
    run_as_root install -m 0644 "$SCRIPT_DIR/$SERVICE_NAME" "$SERVICE_FILE"
    run_as_root systemctl daemon-reload
    run_as_root systemctl enable "$SERVICE_NAME"

    # Eine laufende Installation wird aktualisiert, aber ein gestoppter Dienst
    # wird nicht unerwartet gestartet.
    if run_as_root systemctl is-active --quiet "$SERVICE_NAME"; then
        echo "Starte laufenden Dienst mit der neuen Version neu"
        run_as_root systemctl restart "$SERVICE_NAME"
    fi

    echo
    echo "=== Installation abgeschlossen ==="
    echo
    echo "Naechste Schritte:"
    echo "  1. Konfiguration anpassen:  sudo nano $CONFIG_DIR/config.json"
    echo "  2. Dienst starten:          sudo systemctl start $SERVICE_NAME"
    echo "  3. Status pruefen:          sudo systemctl status $SERVICE_NAME"
    echo "  4. Logs anzeigen:           sudo journalctl -u $SERVICE_NAME -f"
    echo "  5. Deinstallieren:          $0 -u"
}

uninstall_gateway() {
    local remove_service_user=false

    echo "=== serial2modbustcp Deinstallation ==="

    require_command systemctl
    check_privilege_tool

    if [[ -f "$USER_MARKER" ]]; then
        remove_service_user=true
    fi

    if [[ -f "$SERVICE_FILE" ]]; then
        echo "Stoppe und entferne systemd-Dienst"
        run_as_root systemctl disable --now "$SERVICE_NAME" || true
        run_as_root rm -f "$SERVICE_FILE"
        run_as_root systemctl daemon-reload
        run_as_root systemctl reset-failed "$SERVICE_NAME" 2>/dev/null || true
    else
        echo "systemd-Dienst ist bereits entfernt"
    fi

    if [[ -d "$INSTALL_DIR" ]]; then
        echo "Entferne $INSTALL_DIR"
        run_as_root rm -rf "$INSTALL_DIR"
    else
        echo "Installationsverzeichnis ist bereits entfernt"
    fi

    if [[ "$remove_service_user" == true ]] \
        && id -u "$SERVICE_USER" >/dev/null 2>&1; then
        echo "Entferne vom Skript angelegten Benutzer: $SERVICE_USER"
        run_as_root userdel "$SERVICE_USER"
    fi

    echo
    echo "=== Deinstallation abgeschlossen ==="
    echo "Die Konfiguration wurde beibehalten: $CONFIG_DIR"
}

mode="install"
while getopts ":uh" option; do
    case "$option" in
        u)
            mode="uninstall"
            ;;
        h)
            usage
            exit 0
            ;;
        :)
            echo "Fehler: Option -$OPTARG benoetigt ein Argument." >&2
            usage >&2
            exit 2
            ;;
        \?)
            echo "Fehler: Unbekannte Option -$OPTARG" >&2
            usage >&2
            exit 2
            ;;
    esac
done
shift $((OPTIND - 1))

if (( $# > 0 )); then
    echo "Fehler: Unerwartete Argumente: $*" >&2
    usage >&2
    exit 2
fi

if [[ "$mode" == "uninstall" ]]; then
    uninstall_gateway
else
    install_gateway
fi
