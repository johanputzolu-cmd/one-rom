# One ROM Fire 40 pin 

**REQUIRES COMPONENTS ON TOP AND BOTTOM**

**Verified** - The "rp2354-and-flash" design has been verified serving a 27C400 16-bit 40-pin ROM on an Amiga A500.

The "rp2354-no-ext-flash" has not yet been verified.

The functionality of the external flash has not been verified.  This design has only been tested using the RP2354B with built in flash.  While external flash is also present on the tested boards, it has not been verified, or that the design works with an RP2350.

Odds are good that all will work, but buyer beware.

Note that this board does not work with Amiga A500 revision 5 (also known the "oops" revision) PCB, as on this PCB two pins have been transposed (hence the "oops").  You would need an adapter board to use this revision of One ROM 40 in an Amiga A500 rev 5.

## Contents

- [Schematic](./fire-40-a-schematic.pdf)
- [Fab Files](fab/)
- [KiCad Design Files](kicad/)
- [Errata](#errata)
- [Notes](#notes)
- [Changelog](#changelog)
- [BOM](#bom)

## Errata

## Notes

The fab files assume this is fabbed with both an RP2354B (i.e. containing 2MB flash on-board) _and_ external 2MB flash, with the appropriate R11/R12 configurations.  To have other variants assembled:
- RP2354 without external flash, do not populate U5 or R12
- RP2350 with external flash, do not populate R12, instead populate R11 with 0R resistor, U5 populated.

The RP2354B variant is recommended, if RP2354B parts are available, as it is cheaper than RP2350B + external flash.  External flash is also useful, to increase the number of ROM images that can be stored and selected at boot time, from 3 (internal flash only) to 7 (with external flash).

This board requires both top and bottom assembly, as many passives are located on the bottom side.  This requires JLC's standard PCB assembly (with higher costs as a result).  If the boards are fabbed individually (not panelised), JLB will require additional side rails to be added to bring the board up to their minimum PCB size for assembly (roughly 70mm x 70mm).  JLC will recommend this automatically during the ordering process.

As ever, take care that every parts is positioned correctly before ordering.  In particular note the pink pin one dot is located in the appropriate corner for each IC, and that diodes are oriented correctly.

JLC queried the first run of these boards, specifically whether D1 (the status LED) was oriented correctly, as due to aesthetic reasons, a negative marker is not present on the silkscreen.  Negative should be to the left, when viewing the board top with the silkscreen upright.

## Changelog

## BOM

See fab files.
