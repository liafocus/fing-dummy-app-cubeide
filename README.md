# fing-dummy-app-cubeide
Proyecto de STM32CubeIDE dummy para explorar la capacidad de debugging usando ETM (Embedded Trace Macrocell). Eventualmente se migrará el mismo proyecto para ser compilado desde el Keil uVision y debuggeado con el ULINKpro.

**PLATAFORMA:** STM32F4-DISCO1 (Basada en el STM32F407VG)

## Relevamiento del adaptador ULink Pro 20 to 20 pin

Del ULink Pro sale un ribbon con 20 cables que va a un conector  Cortex Debug+ETM  (Documentation – Arm Developer) de paso fino (0.05”), y además viene con el bundle un adaptador a 20 pines de paso “estándar” (0.1”). Sospechamos que el routeo no es pin a pin (ie pin 1 del conector chico con pin 1 del conector grande, pin 2 con pin 2, etc).

| Cortex Debug+ETM | Descripción    | Adaptador 20 to 20 pin | STM32F4 |
| -------------    | -------------  | ------------- | ------------- |
| 1                | VTref          | 19            | VCC           |
| 2                | SWIO / TMS     | 13            | PA13          |
| 3                | GND            | *             | GND           |
| 4                | SWDCLK / TCLK  | 11            | PA14          |
| 5                | GND            | *             | GND           |
| 6                | SWO / TDO      | 7             | PB3           |
| 7                | -              | -             | -             |
| 8                | NC / TDI       | 15            | -             |
| 9                | GNDDetect      | -             | -             |
| 10               | nRESET         | 5             | NRST          |
| 11               | GND / TgPwr+Cap | -            | -             |
| 12               | TRACECLK       | 9             | PE2           |
| 13               | GND / TgtPwr+Cap | -           | -             |
| 14               | TRACEDATA [0]  | -             | -             |
| 15               | GND            | *             | GND           |
| 16               | TRACEDATA [1]  | 17            | PE4           |
| 17               | GND            | *             | GND           |
| 18               | TRACEDATA [2]  | 3             | PE5           |
| 19               | GND            | *             | GND           |
| 20               | TRACEDATA [3]  | 1             | PE6           |

*Los pines 2,4,6,8,10,12,14,16 y 18 del adaptador de 20 pines son GND.