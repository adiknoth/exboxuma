# Linux ALSA driver for D.O.Tec's EXBOX.UMA

## About the hardware

D.O.TEC® EXBOX.UMA is a 32 channel USB audio device with four ADAT ports and an
optical MADI port.

For details, see http://www.directout.eu/en/products/exbox.uma.html

## Driver status

Think alpha! PCM and MIDI streams work, but the integration needs to be
improved. If you want to contribute to make it more user-friendly, that would be
much appreciated.

## How do I use this?

*   Clone this repository
*   `make install` (as root)
*   User alsamixer or amixer to enable streaming
*   Start your audio application, e.g., jackd

## But how do I change the routing/mixer?

You send the right sysex message to the MIDI-Out port. Any MIDI tool would do,
from amidi to PD. I'm working on an shiny remote control app to generate this
message for you to make it more pleasant. Drop me a mail if you want to learn
about the protocol.

## How do I set the sample rate?

You send the right sysex message to the MIDI-Out port. Same as above. If you
only need 44.1/48kHz, the driver has built-in support that don't require sending
a sysex command. Just request 44.1kHz or 48kHz in your application.

The yet-to-be-written remote control app will allow fort 2FS and 4FS as well,
which gives you up to 192kHz.

## Can I help?

Yes, please. Use this driver, test it, drop me a line. Extra credits if you send
pull requests. If you want to work on the remote control app and need the
protocol, drop me a line.

## What's that remote control app that you keep talking about?

From a driver's perspective, the device is pretty straight-forward: except for
streaming on/off, there's little to set in registers. All the magic happens by
sending MIDI messages to the box. I have the protocol spec, somebody just needs
to implement it.

I've started writing a Go application that ideally bridges between MIDI and
HTML5. Instead of using a native toolkit like GTK or Qt, the UI could be written
in HTML5. So if you know some fancy CSS or JS, I'd like to hear from you. We'd
need a matrix router (144x128), a multi-channel mixer and a settings menu/card.

The Go application would act as a webserver that serves the static HTML and also
translates messages from/to MIDI.

If you have a better idea, I'm all ears.

## How do I contact you?

Like always: adi@drcomp.erfurt.thur.de
