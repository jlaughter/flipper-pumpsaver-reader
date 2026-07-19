# PumpSaver Plus — Flipper Zero Reader

Portable/handheld companion to the working ESP32 + Hubitat monitor: a Flipper Zero FAP that
decodes the SymCom/Littelfuse PumpSaver Plus baseband IR broadcast live, on-device.

This displays all data broadcast by the pump protection relay - power consumption, settings, faults
and history.

Protocol reference: [lizbit-official/pumpsaver-ir-protocol](https://github.com/lizbit-official/pumpsaver-ir-protocol)
(PROTOCOL.md is the spec this whole project is built against).

Production reference implementation: [lizbit-official/esphome-pumpsaver](https://github.com/lizbit-official/esphome-pumpsaver).

## Acknowledgments

This project exists entirely on the back of [lizbit-official's pumpsaver-ir-protocol](https://github.com/lizbit-official/pumpsaver-ir-protocol)
work. They reverse-engineered SymCom/Littelfuse's completely undocumented PumpSaver Plus IR
broadcast from scratch — baud rate, bit timing, the edge-skew correction math, word framing, the
sync word, and the full register map — with no vendor documentation to go on. That is the hard,
original work here. This Flipper app (and the ESPHome monitor it's a companion to) is just a
grateful downstream port of it, built directly on top of their
[esphome-pumpsaver](https://github.com/lizbit-official/esphome-pumpsaver) reference implementation.
If you get any use out of this repo, the real credit belongs to them — go star their work.

## Why the stock IR receiver is a non-starter

The PumpSaver broadcasts baseband IR — no 38/56 kHz carrier. Flipper's built-in receiver is a
TSOP-style demodulator tuned for a carrier; it band-pass filters exactly the signal this device
sends and will output garbage, on any firmware. The only way to see this signal on a Flipper is
to bypass the built-in receiver and wire a bare IR phototransistor directly to a GPIO pin — the
same approach the ESP32 build already uses in production.

## Hardware

Same circuit as the ESP32 monitor: bare IR phototransistor (OSRAM SFH309FA is the field-proven
part — daylight-filtered, 7µs switching, 13x margin on the shortest 95µs pulses), collector to
a GPIO pin, emitter to GND, Flipper's internal pull-up as the load. No external resistor needed.
Reversed two-pin phototransistors read nothing but aren't damaged at 3.3V — swap the legs first
if you get zero signal.  Long leg should be the emitter, this should go to pin 18.

If you have a crappy amazon phototransistor, it may be too slow for this application without help.
If you can't get it to work reliably, or at all, you can try adding a 1k resistor between
3v3 (pin 9) and PC0 (pin 16) as an external pullup.

```
   Flipper GPIO header
  ┌──────────────────┐
  │  3V3 (pin 9)      │
  │                   │        IR phototransistor
  │  PC0 (pin 16) ────┼───┬──── collector
  │  (internal pullup │   │        ▄▄
  │   enabled in FAP) │   ├────── ▐░░▌ ◄── IR from the PumpSaver's
  │                   │   │        ▀▀      "POINT INFORMER HERE" window
  │  GND (pin 18)     ├───┴──── emitter
  └──────────────────┘
```

Confirmed Flipper GPIO header pin mapping (source: [docs.flipper.net/zero/gpio-and-modules](https://docs.flipper.net/zero/gpio-and-modules)):

Placement: same as the ESP32 install — Very close to the relay, aimed at the "POINT INFORMER
HERE" / IR LINK window, out of direct sunlight.

## License

This project is licensed under the MIT License — see [LICENSE](LICENSE). That covers this repo's
own code (the Flipper app). It does not relicense or claim any rights over lizbit-official's
protocol reverse-engineering work; check their repos directly for their own license terms.

## Disclaimer

All trademarks mentioned are the property of their respective owners. This project is not
endorsed by those owners, or anyone else really.
