# LineFollower_Semreh_Code

**Pinout**

**ADC**

| Função | Portas |
| :---: | :---: |
| CLK | GPIO34 |
| OUT ADC | GPIO32 |
| IN ADC | GPIO25 |
| CS/SHDN | DGPIO26 |

**Laterais**

| Função | Portas |
| :---: | :---: |
| OUT Esquerdo | GPIO27 |
| OUT Direito | GPIO35 |

**Driver Motor**

| Função | Portas |
| :---: | :---: |
| PWMA | GPIO2 |
| PWMB | GPIO23 |
| IN Direito 1 | GPIO16 |
| IN Direito 2 | GPIO4 |
| IN Esquerdo 1 | GPIO21 |
| IN Esquerdo 2 | GPIO22 |
| STBY | GPIO17 |

**Encorder**

| Função | Portas |
| :---: | :---: |
| OUT Esquerdo A | GPIO15 |
| OUT Esquerdo B | GPIO5 |
| OUT Direito A | GPIO19 |
| OUT Direito B | GPIO18 |


**Bill of Materials**

| Comprado  | ID | Tipo | Modelo/Valor | Package | Quantidade |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **X** | ESP 32 | Microprocessador |  |  | 1 |
| **X** | Sensor Direito, Sensor Esquerdo | Sensor Lateral | QTR-MD-02A |  | 2 |
| **X** | Sensor | Sensor Array | QTR-8A |  | 1 | 
| **X** | U1, U2 | Regulador Tensão | Mini-360 Step Down DC/DC |  | 2 |
| **X** | Ponte H1 | Driver Motor | TB6612FNG |  | 1 |
| **X** | Encoder E, Encoder D | Encoder Sensor | Motor Encoder 6V 1500rpm Bringsmart |  | 2 |
| **X** | Motor D, Motor E | Motor | Micro Metal Gearmotor HPCB 6V Extended Motor Shaft |  | 2 |
|  | ADC1 | Conversor Analógico/Digital | MCP 3008 |  | 1 |
|  | SW1 | Switch ON/OFF | E-Switch EG1218 |  | 1 |
|  | R1 | Resistor | 100Ω | R2010 | 1 |
|  | Q1 | Mosfet | IRLML6402TRPBF | Micro3™ (SOT-23) | 1 |

| ID | Tipo | Modelo/Valor | Quantidade| Comprado |
| :---: | :---: | :---: | :---: | :---: |
| ESP 32 SMD | Microcontrolador |  | 1 | S |
| ADC | Converser Analógico Digital | MCP3008 SMD | 1 | N |
| Sensor Frontal e Lateral | Sensores Refletância | QRE1113GR | ? | S |
| MINI360 | Regulador de Tensão | MP2307DN | 2 | S |
| Motor Direito e Esquerdo | Motor | Pololu Micromotor HPCB 6V 15:1 | 2 | S |
| Encorder Direito e Esquerdo | Encoder | Magnetic Encoder | 2 | S |
|  | Switch ON/OFF | E-Switch EG1218 | 1 | N |
| Resistores |
| Capacitores |