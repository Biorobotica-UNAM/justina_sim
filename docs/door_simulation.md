# Puerta simulada

`justina_sim.launch.xml` inicia `door_sim_node.py` por defecto. Ofrece
`/hardware/door_detector/check` (`std_srvs/srv/Trigger`), con el mismo contrato
que el detector real: `success=true` significa abierta y `false`, cerrada.
Lo utilizan tanto `Check.cpp` del ejecutor como la secuencia inicial GPSR.
Simula únicamente la respuesta del sensor; no añade geometría ni obstáculos.

Puerta abierta desde la primera consulta (comportamiento predeterminado):

```bash
ros2 launch surge_et_ambula justina_sim.launch.xml
```

Mantener la puerta cerrada:

```bash
ros2 launch surge_et_ambula justina_sim.launch.xml sim_door_open:=false
```

Responder cerrada en las primeras dos consultas y abierta desde la tercera:

```bash
ros2 launch surge_et_ambula justina_sim.launch.xml sim_door_closed_checks:=2
```

El contador pertenece al nodo, cuenta todas las consultas (supervisor y ejecutor)
y se reinicia al reiniciar el nodo. `sim_door_open:=false` mantiene la puerta
cerrada incluso después de esas consultas. El número inicial de consultas
cerradas se configura al arrancar.

Cambiar el estado durante la ejecución:

```bash
ros2 param set /door_sim door_open false
ros2 param set /door_sim door_open true
ros2 service call /hardware/door_detector/check std_srvs/srv/Trigger '{}'
```

Para probar que el detector no está disponible, iniciar con `use_sim_door:=false`
y sin otro proveedor del servicio. El launch del robot físico no inicia este
simulador. Para que una puerta cerrada impida continuar, usar
`continue_on_recovery_failure=false` en el supervisor; en modo de depuración
puede continuar tras agotar los tres intentos.

También se puede iniciar de forma independiente:

```bash
ros2 run justina_sim door_sim_node.py --ros-args -p closed_checks_before_open:=2
```
