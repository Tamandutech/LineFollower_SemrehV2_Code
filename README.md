<h1 align="center" style="color:white; background-color:black">LineFollower_SemrehV2_Code</h1>
<h4 align="center">The main code of the Line Follower Robot "Semreh" in its 2nd revision, designed and developed by Tamandutech UFABC.</h4>

<p align="center"><img src="images\Semreh_V2_brushless.jpg" /> </p>

## Hardware
- ESP32-WROOM-32 Module
- ADC - MCP3008
- 8x QRE1113 as main sensors
- PCB 1.6mm as Chassis 
- ESC 4in1 BlHeli 15A
- 4x Brushless Motors 1103 8000kv
- LiPo 3S 350 mAh 70C


<details>
    <summary><b>PINOUT - ESP32</b></summary>

**ADC**

| Function | Port |
| :---: | :---: |
| CLK | GPIO18 |
| OUT ADC | GPIO19 |
| IN ADC | GPIO22 |
| CS/SHDN | DGPIO23 |

**Laterais**

| Function | Port |
| :---: | :---: |
| OUT Esquerdo | GPIO39 |
| OUT Direito | GPIO33 |

**Driver Motor**

| Function | Port |
| :---: | :---: |
| PWMA | GPI14 |
| PWMB | GPIO13 |
| IN Direito 1 | GPIO25 |
| IN Direito 2 | GPIO21 |
| IN Esquerdo 1 | GPIO26 |
| IN Esquerdo 2 | GPIO27 |
| STBY | GPIO17 |

**Encorder**

| Function | Port |
| :---: | :---: |
| OUT Esquerdo A | GPIO34 |
| OUT Esquerdo B | GPI35 |
| OUT Direito A | GPIO16 |
| OUT Direito B | GPIO04 |

**LEDs - WS2812B**

| Function | Port |
| :---: | :---: |
| IN DATA | GPIO32 |

**Buzzer**

| Function | Port |
| :---: | :---: |
| IN DATA | GPIO12 |

</details>

## Software
- Programmed in Vscode with PlatformIO.
- Arduino IDE as Framework.
- C++
```CPP
//Source Code:
if (line() == true) {
    run.fast();
}
//Just kidding
```
