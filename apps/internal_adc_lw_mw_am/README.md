Change `CENTER_FREQUENCY = 792000` as needed in `pico_sdr_bridge.py` file.

Connections:

```
Antenna input:
ANT ---- 2.2k ---- 100nF ---- ADC_NODE ---- GP28
                                 |
                               100pF
                                 |
                                GND

Bias:

3V3 ---- 2.2k ---- VBIAS ---- 2.2k ---- GND
                     |
                   100nF
                     |
                    GND

Bias feed:

VBIAS ---- 47k ---- ADC_NODE


Pico 2's GP0 -> MagJack breakpout's P1 pin

Pico 2's GP1 -> MagJack breakpout's P2 pin
```

Launch the Pico-SDR bridge software: `python3 pico_sdr_bridge.py`

Launch Gqrx SDR with Device -> Other -> `file=/tmp/gqrx_fifo,freq=792000,rate=50000,repeat=false,throttle=false` string. Set `Sample rate` to `50000`. Select `AM` mode and other settings as usual.
