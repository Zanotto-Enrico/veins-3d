#!/bin/bash

# Nome del file di configurazione di Omnet++
CONFIG_FILE="omnetpp.ini"

# Numero massimo di processi in parallelo
MAX_PARALLEL=16

# contatore di processi
running_processes=0

# Loop per eseguire le simulazioni 
for ((i=0; i<20; i++))
do
    echo "Esecuzione della simulazione con -r $i"
    opp_run -m -u Cmdenv -c MultipleRSUs -n .:../../src/veins --image-path=../../images -l ../../src/veins $CONFIG_FILE -r $i &
    
    # Incrementa il contatore dei processi in esecuzione
    ((running_processes++))

    if ((running_processes >= MAX_PARALLEL)); then
        wait -n
        ((running_processes--))
    fi
done








# Attendere che tutti i processi in background terminino prima di concludere lo script
wait

# contatore di processi
running_processes=0

# Loop per eseguire le simulazioni
for ((i=0; i<60; i++))
do
    echo "Esecuzione della simulazione con -r $i"
    opp_run -m -u Cmdenv -c OnTopOfTheBuildings -n .:../../src/veins --image-path=../../images -l ../../src/veins $CONFIG_FILE -r $i &
    
    # Incrementa il contatore dei processi in esecuzione
    ((running_processes++))

    # Controlla se il numero di processi in background ha raggiunto il limite
    if ((running_processes >= MAX_PARALLEL)); then
        # Aspetta che uno dei processi in background termini prima di procedere
        wait -n
        # Decrementa il contatore dei processi in esecuzione
        ((running_processes--))
    fi
done

# Attendere che tutti i processi in background terminino prima di concludere lo script
wait
