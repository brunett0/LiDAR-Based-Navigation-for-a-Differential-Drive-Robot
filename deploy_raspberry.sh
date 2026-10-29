#!/bin/zsh
# =========================================================================
# SCRIPT DI CONNESSIONE E DEPLOY AUTOMATICO SU RASPBERRY PI 4
# =========================================================================
RPI_IP="${1:-172.20.10.2}"
RPI_USER="${2:-pi}"
SCRIPT_PATH="/Users/matteopardo/Desktop/Antigravity Workspace/Navigazione robot/CodiceRaspberry.py"

echo "========================================================"
echo " 1. Verifica raggiungibilita' Raspberry Pi ($RPI_IP)..."
echo "========================================================"
if ! ping -c 2 -W 2000 "$RPI_IP" > /dev/null 2>&1; then
    echo "[ERRORE] La Raspberry Pi ($RPI_IP) non risponde al ping!"
    echo "Assicurati che:"
    echo "  1) L'Hotspot del telefono sia acceso"
    echo "  2) Sia il Mac che la Raspberry Pi siano collegati all'Hotspot"
    exit 1
fi
echo "[OK] Raspberry Pi raggiungibile su $RPI_IP!"

echo ""
echo "========================================================"
echo " 2. Invio di CodiceRaspberry.py via SCP..."
echo "========================================================"
scp "$SCRIPT_PATH" "${RPI_USER}@${RPI_IP}:~/CodiceRaspberry.py"

if [ $? -eq 0 ]; then
    echo "[OK] CodiceRaspberry.py caricato con successo in ~/CodiceRaspberry.py!"
    echo ""
    echo "========================================================"
    echo " 3. Avvio del server sulla Raspberry Pi via SSH..."
    echo "========================================================"
    ssh -t "${RPI_USER}@${RPI_IP}" "python3 ~/CodiceRaspberry.py"
else
    echo "[ERRORE] Trasferimento SCP fallito. Controlla nome utente ($RPI_USER) o password."
fi
