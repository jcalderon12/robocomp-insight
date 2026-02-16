#!/bin/bash

# Función que mata todos los procesos hijos
cleanup() {
    echo "Cerrando todos los procesos..."
    kill 0
    exit
}

# Capturar señales de cierre
trap cleanup SIGINT SIGTERM SIGHUP

# Ejecutar programas en segundo plano

gnome-terminal -- bash -c "cd . && bash $ROBOCOMP/tools/rcnode/rcnode.sh; exec bash" &
sleep 1.0
gnome-terminal -- bash -c "cd $ROBOCOMP/components/webots-bridge && bin/Webots2Robocomp etc/config; exec bash" &
sleep 1.0
gnome-terminal -- bash -c "cd $ROBOCOMP/components/robocomp-insight/agents/mission_controller && bin/mission_controller etc/config; exec bash" &
sleep 0.35
gnome-terminal -- bash -c "cd $ROBOCOMP/components/robocomp-insight/agents/concept_robot && bin/concept_robot etc/config; exec bash" &
sleep 0.35
gnome-terminal -- bash -c "cd $ROBOCOMP/components/robocomp-insight/agents/concept_person && bin/concept_person etc/config; exec bash" &
sleep 0.35
gnome-terminal -- bash -c "cd $ROBOCOMP/components/robocomp-insight/agents/concept_bottle && bin/concept_bottle etc/config; exec bash" &
sleep 0.35
gnome-terminal -- bash -c "cd $ROBOCOMP/components/robocomp-insight/agents/fake_castilla_la_mancha && bin/fake_castilla_la_mancha etc/config; exec bash" &
sleep 0.35
gnome-terminal -- bash -c "cd $ROBOCOMP/components/robocomp-insight/agents/fake_malaga && bin/fake_malaga etc/config; exec bash" &
sleep 0.35
gnome-terminal -- bash -c "cd $ROBOCOMP/components/robocomp-insight/agents/inner_simulator && bin/inner_simulator etc/config; exec bash"

# Esperar a que terminen
wait
