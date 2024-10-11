#!/bin/bash
# Cek apakah pengguna memiliki hak akses root
if [ "$EUID" -ne 0 ]; then
    echo "Script ini memerlukan hak akses root. Menjalankan sudo..."
    exec sudo "$0" "$@"  # Eksekusi skrip dengan sudo
fi
make clean ;
make reinstall ;
make install ;
cd project ;
cd demo ;
make clean ;
make ;
