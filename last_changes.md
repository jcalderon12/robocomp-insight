# Pendiente

## 1. Encadenado de misiones: fin de `follow_person`

Ya aplicado (compilando, sin probar en vivo):

- Prioridades en `mission_controller`: `Search Problem Cause` = 5, `follow_person` = 4, `Take Photos` = 2. El scheduler (`selectNextMission`) elige la de mayor prioridad entre las `pending`/`stopped`, así que al completarse la investigación reanuda `follow_person` y sólo después pasa a las fotos.
- El autopiloto ya no se apaga al completar `follow_person`, sólo al completar `Take Photos` (fin de la cadena).
- `fake_castilla_la_mancha` no regenera el nodo `problem` si ya existe `bump`.

Falta implementar:

- **`fake_castilla_la_mancha`**: meter también `deactivate_affordance()` dentro de la guarda de `bump`. Hoy sigue fuera, así que mientras la botella no esté sobre el robot pone `aff_interacting = false` en `follow_me` en cada ciclo — y eso mata cualquier reanudación de `follow_person` (duraría un frame).
- **`concept_robot`**: retirar `follow_me` al alcanzar a la persona, que es lo que debe terminar la misión cuando el motivo es "objetivo cumplido" y no "anomalía".
  - En `compute()`, rama de no-fotos, tras `follow_target(...)`, llamar a un `check_follow_reached()`.
  - `follow_target` guarda la distancia que ya calcula en un miembro (`last_target_distance`).
  - `clear_target_affordance()`: extraer de `finish_photo_mission()` el recorrido `TARGET` → `has_intention` → afordance → `aff_interacting = false`, para que lo usen las dos misiones. Lo específico de fotos (escribir `photo_session_dir`) se queda donde está.

### Duda abierta: cómo armar la condición de "alcanzada"

Con `Desired_distance = 1.8 m`, la idea era armar la condición sólo tras haber estado lejos, para que la misión no se complete al instante si al reanudarse el robot ya está cerca:

```cpp
if (last_target_distance > 2.3f) { follow_was_far = true; return; }
if (!follow_was_far) return;
if (last_target_distance <= 2.0f) { clear_target_affordance(); follow_was_far = false; }
```

Contrapartida: si al reanudar ya está dentro del umbral, nunca se arma y **la misión no termina sola**. Tres variantes sin decidir:

1. **Con armado** (lo de arriba): garantiza una aproximación real; riesgo de quedarse colgada si empieza cerca.
2. **Sin armado**: completa en cuanto está dentro del umbral; nunca se cuelga, pero puede durar un frame.
3. **Con armado y tiempo máximo**: si pasan N segundos dentro del umbral sin haberse armado, completa igual. Cubre ambos casos a costa de un temporizador.

Nota: con la persona quieta (`fake_malaga.compute()` está vacío y nadie llama a `setPathToHuman`), la aproximación dura lo que tarde en recorrer los metros que la separen.

## 2. Mover lógica de `semantic` a `fake_castilla_la_mancha`

`cambiar lo de semantic a fake_castilla -> preguntar a usuario`

Pendiente de concretar con el usuario qué parte exactamente y por qué.

## 3. Git

- Limpiar el repo.
- Crear carpetas para imágenes y logs donde no existan.
- Añadir al `.gitignore` lo que no deba subirse (fotos de las tandas, logs, `sim_output_*.json`, pesos de modelos, `build/`...).

## 4. Misión de tomar fotos

- Cambiarla (el usuario tiene cambios pensados, sin detallar todavía).
- Añadir SAM en el momento de tomar las fotos.

## 5. Crear un `follow_person` nuevo si la persona vuelve a separarse

Aparcado hasta tener el resto funcionando. Sería la imagen especular del criterio de fin (histéresis entre dos umbrales) y la infraestructura ya está: `create_follow_person_mission()` tiene guarda de duplicados y los nombres ya van numerados.

Matiz a resolver cuando se aborde: con la persona quieta, el que se aleja es el robot — y eso pasa justo durante la misión de fotos, así que encadenaría un `follow_person` nuevo nada más terminarlas. Habría que condicionarlo.
