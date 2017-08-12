# Using ST Nucleo 64 F446 Board for RFM69HCW module (semtech 1231)

* *Reference : Datasheet: STM32F446xC/E DocID027107 Rev 6 September 2016*

* *Reference : Schematic: MB1136.pdf from http://www.st.com/resource/en/schematic\_pack/nucleo\_64pins\_sch.zip*

```

Links and versions current as of: Fri 11 August 2017 12:08:57 (PDT)

Datasheet:
wget http://www.st.com/content/ccc/resource/technical/document/datasheet/65/cb/75/50/53/d6/48/24/DM00141306.pdf/files/DM00141306.pdf/jcr:content/translations/en.DM00141306.pdf

Reference Manual:
wget http://www.st.com/content/ccc/resource/technical/document/reference_manual/4d/ed/bc/89/b5/70/40/dc/DM00135183.pdf/files/DM00135183.pdf/jcr:content/translations/en.DM00135183.pdf

Schematic:
wget  http://www.st.com/resource/en/schematic_pack/nucleo_64pins_sch.zip

```

## SPI

### Solder Bridges Protect Boson Habitat

#### PA5: SPI1\_SCK  (CN10:11)
* Alternate Function 5
* In the schematic SB21 (**Solder Bridge 21**) is connected to the LD2-green. 
   * We want this line for SPI1\_SCK (as labeled in the Table 11 Alternate function table in the st 446 data sheet)
      * **Remove the 0 ohm resistor with solder tweezers or a small soldering iron and solder wick. (Do not use your teeth!)**

#### PA6: SPI1\_MISO (CN10:13)
* Alternate Function 5
* In the schematic SB41 is closed with a 0 ohm resister. Perfect. Don't touch it.

#### PA6: SPI1\_MISO (CN10:13)
* Alternate Function 5
* In the schematic SB41 is closed with a 0 ohm resister. Perfect. Don't touch it.

#### PA7: SPI1\_MOSI (CN10:15)
* Alternate Function 5
* In the schematic SB40 is closed with a 0 ohm resister. Perfect. Don't touch it.

## GPIO

### Organic Fair Trade Vanilla IO

#### PC6: EN
* Alternate Function 0
* Unused on Nucleo Board

#### PC8: RST
* Alternate Function 0
* Unused on Nucleo Board

#### PC8: G0
* TBD

#### PC8: G1
* TBD

#### PC8: G2
* TBD

#### PC8: G3
* TBD

#### PC8: G4
* TBD

#### PC8: G5
* TBD


