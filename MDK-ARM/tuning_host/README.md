# Ball Balance Tuning Console

This tool changes controller parameters at runtime and plots STM32 telemetry.

## Wiring

Use a 3.3 V USB-to-UART adapter:

```text
STM32 PA2 / USART2_TX -> adapter RX
STM32 PA3 / USART2_RX -> adapter TX
STM32 GND             -> adapter GND
```

Do not connect a 5 V UART signal directly to the STM32 pins. UART5 remains
dedicated to the MaixCAM vision link. USART2 uses 115200 baud, 8 data bits,
no parity and 1 stop bit.

## Run

```powershell
cd tuning_host
python -m pip install -r requirements.txt
python tuning_app.py
```

Connect, click `Read parameters`, change one value, then use its `Set` button.
`Send all` sends every populated field. Use `Start CSV` before a repeatable
test to save the controller state for later analysis.

## Serial protocol

Host commands:

```text
@GET
@SET,OKP,650
@STREAM,1
@STREAM,0
```

Values are scaled integers. The GUI performs scaling automatically. Firmware
responses use `@P` for parameters, `@OK`/`@ERR` for status, and `@T` for 20 Hz
telemetry.

The runtime parameters are the outer position PID (`OKP/OKI/OKD`), inner angle
PID (`IKP/IKI/IKD`), target angle limit (`ALIM`) and PWM angular-speed limit
(`PWM`). Compile-time defaults, vision filtering, chassis feed-forward and
stepper conversion parameters are centralized in `Src/balance_control.h`.
