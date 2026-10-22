Sé directo, conciso y objetivo. Ve al grano desde la primera palabra.
Prohibidos saludos, despedidas, fórmulas de cortesía, preámbulos, resúmenes finales y meta-comentarios (p. ej. "Aquí tienes el código", "Espero que te sirva").
Usa listas con viñetas o código limpio. Evita párrafos largos y prosa innecesaria.
Responde únicamente a lo preguntado. No agregues contexto no solicitado ni justificaciones salvo que sea imprescindible.
Maximiza la densidad de información por token utilizado.
Entrega directamente el bloque de código funcional sin explicaciones triviales antes o después, a menos que se soliciten explícitamente.
Si falta información crítica, pregunta en una sola frase breve; si no es crítica, asume el caso estándar y responde directamente.
Cuando agregues comentarios, que sean cortos y concisos (1-2 líneas), explicando claramente la instrucción o instrucciones que siguen.
Antes de modificar código, pregunta primero, salvo que se pida explícitamente hacerlo.

# Arquitectura

Componentes RoboComp sobre un grafo DSR compartido. `robocompdsl` genera `generated/` desde el
`.cdsl`; el código propio vive en `src/specificworker.{py,cpp,h}`. Los lanza Program Manager
desde `~/software/Program-manager/etc/insight_config_sim.json`.

Grafos: `G = Graphs.at("work")` (dominio 0, mundo), `mission_graph` (1, misiones),
`historic_graph` (2). Las afordances viven en el grafo de trabajo, no en el episódico.

| agente | papel |
|---|---|
`concept_robot` | mueve el robot: `follow_target()` y la máquina de la misión de fotos |
`concept_person`, `concept_<x>` | publican la RT `robot -> su concepto` |
`mission_controller` | crea y activa misiones; crea la arista TARGET al activarlas |
`fake_castilla_la_mancha` | detecta la botella caída, crea `problem`, genera el agente del concepto |
`inner_simulator` | simula causas en PyBullet; publica `problem_position` y `concept_size` |
`episodic_memory` | graba en `agents/mission_controller/recorded_missions/` |
`webots-bridge` | **repositorio aparte** (`robocomp/webots-bridge`), no es de este proyecto |

Cadena de misiones: `follow_person` -> cae la botella -> `problem` -> `search_cause` ->
`take_photos`. La misión de fotos es `APPROACH -> BACKOFF -> VERIFY -> TURNING/SETTLING -> DONE`.

El agente del concepto se genera desde las plantillas Jinja2 de
`agents/agent_generation/templates/`. **Editar siempre la plantilla**: `generate_agent()` hace
`rmtree` del agente generado y lo rehace entero.

# Convenciones y trampas

- **El avance es la SEGUNDA coordenada** del vector (la "y" habitual). El rumbo se mide
  `atan2(x, y)`, o sea desde la segunda componente. Vale para las RT, para `setSpeedBase` y para
  lo que guarda la memoria episódica.
- **Las aristas RT llevan un buffer circular de 5 muestras** (15 floats) con `rt_head_index`.
  Leer `rt_translation[0]` a pelo devuelve un hueco que solo se refresca 1 de cada 5 veces: usar
  `rt->get_translation(nodo, destino)`.
- **Una RT a `[0,0,0]` es ausencia de dato, no una posición.** El puente de Webots devuelve la
  pose a ceros cuando no resuelve el DEF, y un objetivo en el origen parece alcanzado y centrado.
- **pydsr**: `Attribute(lista_de_numpy.float32)` se guarda en silencio como vector de bytes y se
  lee `[0,0,0]`; hay que castear con `float()`. Y `Edge(to, from, type, agent_id)` lleva el
  **destino primero**.
- **`auto_localization()`** escribe la traslación `root->robot` con x/y intercambiados respecto a
  Webots, pero el cuaternión tal cual. Quien la consuma debe deshacer el swap.
- **Unidades**: `world_position` en metros; `getObjectPose` del puente en milímetros.
- **`problem_position` no es una medida**: es la última posición registrada del robot antes de
  que apareciera el nodo `problem`. Arrastra el retardo de detección de la botella, medido entre
  14 y 45 cm según la tanda.
- Las grabaciones salen de **0 bytes si se mata el flujo a media misión**: hay que pararla desde
  la UI o dejar que termine.
