## Experiments for 70cm FSK link using a BeagleBone Black (TI Sitara am3358 processor)

### On BBB
* Look in /etc/dogtag for version of debian for bbb
* Also uname -a

### RFM69HCW Module
* This is a Semtech sx1231 module
  * Manufactured by Hope-RF
  * http://www.hoperf.com/rf_transceiver/modules/RFM69HCW.html

#### Reference note RFM69HCW
* From: https://lowpowerlab.com/forum/rf-range-antennas-rfm69-library/rfm69-hoperf-are-they-semtech-clones-or-not/

```
Perhaps this doesn't belong here, but since you asked, and to set the record straight about RFM69 and HopeRF.
I hope this clears up the confusion fog of assumptions and baseless rumors on the internet:

The HopeRF RFM69 modules use genuine Semtech chips. I have decapped these chips to verify this. They are custom packaged hence you see a Hope logo and "RF69" markings on them and not the default semtech markings. Take the following facts:

* if HopeRF would be using a cloned silicon design, and since Semtech owns patents and the IP they and would sue and stop imports in the civilized world where laws actually still work (US, EU). So Hope would have no market there
* the packages for HopeRF's RF69 chips are QFN28, perhaps a little cheaper to package in China or Thailand/Taiwan. I bet it's a magnitude cheaper to buy the silicon or straight out wafers and doing the custom packaging with subcontracting, allowing Hope to make the chip appear proprietary. That's great for HopeRF and for all the rest of us, we get the same genuine chips, at a low price, win-win for all.
* the Semtech packaging is QFN24
* the HopeRF RFM69 radio variants are based on the stock Semtech reference designs found on pages 76/77 of the datasheet
* the reason (implied) for HopeRF's copycat datasheets is that they are not selling the chip itself, but a reference design based on the original chip. So of course they have to change the datasheet to reflect these changes and pinout diagrams, and in doing so they slapped their logo on it (if Semtech doesn't complain why would anyone else?). It's unfortunate that there are some errors in the datasheets but if one of us had to do the same and translate a chinese datasheet with your own changes added, I wonder how well we'd do. What bothered me somewhat is that they just replaced "Semtech"
* the concept of fake not only scares but annoys me. If these were fake and buggy and not performant modules I would not use/resell or endorse them, instead I would make my own. In fact I made a batch of RFM69 just for fun to prove myself I can do it. I bought Semtech chips and followed their reference design which is very close to what HopeRF did. But to get close to make it worth it for me to keep making these I would have to make them in 10,000qty and spend half my time doing it. Hence I prefer to source from HopeRF, that's all they do on a massive scale and they do a fine job. It's one of the Chinese products that is of good quality. Thumbs up from me, I hope they don't screw up in the future.
* internet statements claiming HopeRF modules are low quality pivot on the fact that some of the SMD components are reflowed at dodgy angles making it look like the design is sloppy. I see something else: - they use the same PCB for half a dozen variants of RFM69 and hence most pads are smaller than optimal for reflowing and some pads are shared, so some pads are not always identical or perfectly square with others etc. So yeah - at that scale reflowing will pull the parts wherever there is more surface tension. Perhaps an optimization rather than a compromise and another reason the price is so "low".
* Notice the Semtech SX1231H chip markings and the HopeRF RF markings:
```

### Hardware Notes
* A module is soldered to a BBB proto board cape like proto board.
  * https://www.adafruit.com/product/572
  * *Photo: https://github.com/oresat/devsat/blob/semtech/firmware/debian_am3358/RFM69HCW/Notes/RadioBoardBBB.jpg*
* See the spreadsheet *RFM69HCW_breakout_conn.ods* for connections



