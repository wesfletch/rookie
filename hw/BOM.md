POWER:
    Dropping the 12V rail: only the motors use it, and I can just feed them VBAT and use closed-loop control for them, which I was planning on doing anyway.

    5V: split rail between SBC and LOGIC (pico, sensors, peripherals, everything else). Means two buck convertors.
        * LOGIC: LM2596G fixed 5V, 3A output. D2PAK packaging for easy assembly. 
        * SBC: TPS54531DDAR, adjustable to 5V, 5A (let WEBENCH do this for you); DDAR is the same as DDA, just cheaper because it comes as part of a reel (or cut tape) rather than in a tube.


Sensors
    Encoders: CUI Tech/Same Sky AMT-102V
    
