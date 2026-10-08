# Bug: corrupción de vectores `vf` en episodic_memory por locale de coma decimal

## Resumen

Los atributos vector de floats (`rt_translation`, `imu_accelerometer`, `imu_gyroscope`, ...)
grabados por `episodic_memory` se leen de vuelta con **el doble de elementos** y valores
disparatados (decenas/cientos de miles). Un `[x, y, z]` de 3 elementos vuelve como
`[x_ent, x_dec, y_ent, y_dec, z_ent, z_dec]` de 6.

Causa: el serializador escribe los decimales con **coma** (locale español) y el parser
usa **la coma como separador entre elementos del vector**. Los dos usos del mismo
carácter colisionan y la información se pierde.

No es un problema de cortex, CRDT ni del grafo compartido: el grafo vivo tiene los
valores correctos. El daño ocurre solo en el viaje a texto de episodic_memory.

---

## 1. Qué es el "locale" y por qué importa

El *locale* del sistema define, entre otras cosas, **qué carácter separa la parte
entera de la decimal**:

- Locale `C` / inglés → punto: `3.14`
- Locale español (`es_ES.UTF-8`) → coma: `3,14`

El shell tiene `LC_NUMERIC="es_ES.UTF-8"`. Cuando `episodic_memory` arranca, Qt
(`QApplication`) hace `setlocale(LC_ALL, "")`, que aplica ese locale al proceso.
A partir de ahí, en C++ la función `std::to_string(float)` **escribe los decimales
con coma**.

## 2. Cómo guarda episodic_memory un atributo (formato de texto)

`episodic_memory` no usa JSON ni binario. Serializa cada cambio del grafo como
**una línea de texto** con delimitadores propios (`src/DSRTypeTrait.h`):

| Carácter | Significado |
|---|---|
| `#` | separador de campos/registros |
| `$` | separa nombre y tipo del atributo |
| `:` | separa tipo y valor |
| `%` | fin de un atributo |

Un vector de floats se codifica como `[v1,v2,v3]`, uniendo los elementos **con coma**,
en `src/specificworker.cpp` (`attribute_value_and_type_to_string`, ~línea 604):

```cpp
std::string out = "[";
bool first = true;
for (const auto &x : value) {
    if (!first) out += ",";          // <-- separador ENTRE elementos: coma
    out += std::to_string(x);        // <-- convierte cada float a texto (locale-dependiente)
    first = false;
}
out += "]";
```

## 3. La colisión

Vector real en el grafo:

```
rt_translation = [-3.700589, -0.299997, 0.024138]     (3 elementos, metros)
```

Al serializar bajo locale español:

- `std::to_string(-3.700589f)` -> `"-3,700589"` (coma decimal)
- se unen con coma -> línea en el fichero:

```
rt_translation$vf:[-3,700589,-0,299997,0,024138]
```

Ahora hay **comas que separan elementos** y **comas que son el punto decimal**, y son
el mismo carácter. Ya no se puede distinguir cuál es cuál.

## 4. Cómo se decodifica mal

Al leer, `parse_vector<float>` (`src/DSRDecoder.h`, ~línea 313) coge lo que hay entre
`[` y `]` y lo **parte por comas**:

```cpp
std::string content = "-3,700589,-0,299997,0,024138";
while (std::getline(ss, item, ',')) {   // parte por CADA coma
    result.push_back(std::stof(item));
}
```

Trozos: `"-3"`, `"700589"`, `"-0"`, `"299997"`, `"0"`, `"024138"` -> 6 valores:

```
[-3.0, 700589.0, -0.0, 299997.0, 0.0, 24138.0]
```

En vez de 3 elementos salen **6**:

- índices pares (0, 2, 4) = parte entera de cada número original
- índices impares (1, 3, 5) = los dígitos decimales leídos como un entero gigante
  (`700589` en vez de `.700589`)

Ejemplos reales de los logs de depuración:

| Leído (6 elementos) | Valor real (3 elementos) |
|---|---|
| `[-3.0, 684951.0, -0.0, 300021.0, 0.0, 24161.0]` | `[-3.684951, -0.300021, 0.024161]` |
| `imu_accelerometer [0.0, 25000.0, 15.0, 426000.0, 9.0, 589000.0]` | `[0.025, 15.426, 9.589]` (el `9` = gravedad) |
| `last_position_before_problem [-0.0, 73214.0, -0.0, 301230.0, 0.0, 27298.0]` | `[-0.073214, -0.301230, 0.027298]` |

El último coincide con el valor vivo del grafo `[-0.057, -0.303, 0.026]`.

## 5. Por qué el grafo VIVO está bien y la grabación no

- `LIVE graph room->robot RT rt_translation = [-0.057591923, -0.30306014, 0.026450131]`
  -> 3 elementos correctos.
- `[DEBUG episodic-src] ... size=3` -> el `std::vector` **antes de serializar** también
  tiene 3.

El daño ocurre **solo en el viaje texto**:
`vector<float>` -> `std::to_string` (mete coma) -> fichero -> `parse_vector`
(parte por coma) -> `vector<float>` con 6. Cortex, CRDT y el grafo compartido no
intervienen.

Nota: también aparecen atributos `rt_translation` con `size=15` en otra arista
(una ventana de 5 poses `[x,y,z] x 5`, en mm). Mismo problema: se convertirían en 30
al leer. No es lo que rompe `problem_position`, pero queda latente.

## 6. Efecto aguas abajo

`inner_simulator` lee esos vectores de la memoria episódica:

- `get_robot_positions_relative_to_problem()` -> `last_position_before_problem`
  (6 floats) -> `problem_position` en el nodo `problem` con la X inflada
  (cientos de miles de mm).
- `convert_episodic_to_imu_history()` -> `imu_history` con shape `(N, 6)` en vez de
  `(N, 3)` -> DTW compara columnas equivocadas -> `acc_dtw` gigante / `inf` ->
  el "mejor" recording elegido es basura -> el robot conduce hacia una posición
  imposible.

---

## Solución

Hay que atacar las **dos puntas**: que no se escriba con coma, y (opcional pero
recomendable) que el separador de elementos no pueda colisionar nunca.

### Cambio 1 - no escribir decimales con coma (obligatorio)

**Opción A (mínima, sin tocar código): variable de entorno al lanzar.**
`LC_ALL=C` fuerza el locale `C` (punto decimal) solo para ese proceso.

- Lanzamiento manual desde terminal:
  ```bash
  cd agents/episodic_memory
  LC_ALL=C bin/episodic_memory etc/config
  ```
- Lanzamiento vía Program Manager (ejecuta sin shell, así que el prefijo `VAR=val`
  no vale; hay que usar `env`): cambiar el campo `command` a
  ```
  env LC_ALL=C bin/episodic_memory etc/config
  ```

Aplicarlo también al resto de agentes que escriben atributos al DSR, por consistencia.
No es permanente ni global: hay que ponerlo en cada arranque.

**Opción B (en código): fijar locale `C` al arrancar.**
En `agents/episodic_memory/generated/main.cpp`, primera línea de `main()`, **antes**
de crear `QApplication`:

```cpp
#include <clocale>
...
int main(int argc, char* argv[])
{
    std::setlocale(LC_ALL, "C");
    ...
```

**Opción C (la más sólida): no depender del locale al serializar.**
En `src/specificworker.cpp` (~línea 610), sustituir `std::to_string(x)` por
`std::to_chars`, que siempre usa punto y no pierde precisión:

```cpp
#include <charconv>
...
// dentro del bucle del vector, en lugar de:  out += std::to_string(x);
char buf[32];
auto [ptr, ec] = std::to_chars(buf, buf + sizeof(buf), x);
out.append(buf, ptr);
```

Recomendado para la solución definitiva: **B + C** juntas (B protege también otras
conversiones que pueda haber; C hace el serializador inmune al locale).

### Cambio 2 - separador de elementos que no colisione (opcional, defensivo)

Aunque con el Cambio 1 ya no habría comas decimales, se elimina el riesgo de raíz
cambiando el separador **entre elementos** de `,` a `;` en las **dos** funciones:

- Encoder, `src/specificworker.cpp` (~línea 609): `out += ",";` -> `out += ";";`
- Decoder, `src/DSRDecoder.h` (~línea 325): `std::getline(ss, item, ',')` -> `';'`

Rompe la compatibilidad con ficheros ya grabados.

### Grabaciones ya hechas

Quedan con comas y seguirán decodificando mal. Son solo pruebas; se borrarán.
Lo correcto es re-grabar las misiones tras aplicar el fix.

---

## Ubicación del bug

| Lado | Fichero | Línea aprox. | Problema |
|---|---|---|---|
| Encode | `src/specificworker.cpp` | 610 | `out += std::to_string(x)` -> coma decimal bajo `LC_NUMERIC` |
| Encode | `src/specificworker.cpp` | 609 | separador entre elementos = `,` |
| Decode | `src/DSRDecoder.h` | 325 | `std::getline(ss, item, ',')` -> delimitador `,` colisiona con el decimal |
