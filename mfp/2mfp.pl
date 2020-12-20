@pins = (
# off    loc      gpio-name       ||    funcs
[ 0x000, 'D19',   'GPIO_102',     (),   'USIM',      'GPIO',      'FSIC',      'KP_DK',     'LCD',       'NONE',      'NONE',      'NONE' ],
[ 0x004, 'C19',   'GPIO_103',     (),   'USIM',      'GPIO',      'FSIC',      'KP_DK',     'LCD',       'NONE',      'NONE',      'NONE' ],
[ 0x008, 'B19',   'GPIO_142',     (),   'USIM',      'GPIO',      'FSIC',      'KP_DK',     'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x00c, 'A20',   'GPIO_124',     (),   'GPIO',      'MMC1',      'LCD',       'MMC3',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x010, 'E20',   'GPIO_125',     (),   'GPIO',      'MMC1',      'LCD',       'MMC3',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x014, 'AF11',  'GPIO_126',     (),   'GPIO',      'MMC1',      'LCD',       'MMC3',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x018, 'AE10',  'GPIO_127',     (),   'GPIO',      'NONE',      'LCD',       'MMC3',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x01c, 'AH11',  'GPIO_128',     (),   'GPIO',      'NONE',      'LCD',       'MMC3',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x020, 'AF10',  'GPIO_129',     (),   'GPIO',      'MMC1',      'LCD',       'MMC3',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x024, 'AD10',  'GPIO_130',     (),   'GPIO',      'MMC1',      'LCD',       'MMC3',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x028, 'D20',   'GPIO_131',     (),   'GPIO',      'MMC1',      'NONE',      'MSP',       'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x02c, 'B20',   'GPIO_132',     (),   'GPIO',      'MMC1',      'PRI_JTAG',  'MSP',       'SSP3',      'AAS_JTAG',  'NONE',      'NONE' ],
[ 0x030, 'A21',   'GPIO_133',     (),   'GPIO',      'MMC1',      'PRI_JTAG',  'MSP',       'SSP3',      'AAS_JTAG',  'NONE',      'NONE' ],
[ 0x034, 'B21',   'GPIO_134',     (),   'GPIO',      'MMC1',      'PRI_JTAG',  'MSP',       'SSP3',      'AAS_JTAG',  'NONE',      'NONE' ],
[ 0x038, 'F20',   'GPIO_135',     (),   'GPIO',      'NONE',      'LCD',       'MMC3',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x03c, 'C21',   'GPIO_136',     (),   'GPIO',      'MMC1',      'PRI_JTAG',  'MSP',       'SSP3',      'AAS_JTAG',  'NONE',      'NONE' ],
[ 0x040, 'D21',   'GPIO_137',     (),   'GPIO',      'HDMI',      'LCD',       'MSP',       'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x044, 'B22',   'GPIO_138',     (),   'GPIO',      'NONE',      'LCD',       'MMC3',      'SMC',       'NONE',      'NONE',      'NONE' ],
[ 0x048, 'AC10',  'GPIO_139',     (),   'GPIO',      'MMC1',      'PRI_JTAG',  'MSP',       'NONE',      'AAS_JTAG',  'NONE',      'NONE' ],
[ 0x04c, 'A22',   'GPIO_140',     (),   'GPIO',      'MMC1',      'LCD',       'NONE',      'NONE',      'UART2',     'UART1',     'NONE' ],
[ 0x050, 'C22',   'GPIO_141',     (),   'GPIO',      'MMC1',      'LCD',       'NONE',      'NONE',      'UART2',     'UART1',     'NONE' ],
[ 0x054, 'G21',   'GPIO_0',       (),   'GPIO',      'KP_MK',     'NONE',      'SPI',       'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x058, 'G22',   'GPIO_1',       (),   'GPIO',      'KP_MK',     'NONE',      'SPI',       'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x05c, 'B23',   'GPIO_2',       (),   'GPIO',      'KP_MK',     'NONE',      'SPI',       'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x060, 'D22',   'GPIO_3',       (),   'GPIO',      'KP_MK',     'NONE',      'SPI',       'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x064, 'A23',   'GPIO_4',       (),   'GPIO',      'KP_MK',     'NONE',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x068, 'C23',   'GPIO_5',       (),   'GPIO',      'KP_MK',     'NONE',      'SPI',       'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x06c, 'E23',   'GPIO_6',       (),   'GPIO',      'KP_MK',     'NONE',      'SPI',       'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x070, 'H22',   'GPIO_7',       (),   'GPIO',      'KP_MK',     'NONE',      'SPI',       'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x074, 'F23',   'GPIO_8',       (),   'GPIO',      'KP_MK',     'NONE',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x078, 'A24',   'GPIO_9',       (),   'GPIO',      'KP_MK',     'NONE',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x07c, 'D23',   'GPIO_10',      (),   'GPIO',      'KP_MK',     'NONE',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x080, 'B24',   'GPIO_11',      (),   'GPIO',      'KP_MK',     'NONE',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x084, 'D24',   'GPIO_12',      (),   'GPIO',      'KP_MK',     'NONE',      'CCIC1',     'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x088, 'G23',   'GPIO_13',      (),   'GPIO',      'KP_MK',     'NONE',      'CCIC1',     'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x08c, 'J22',   'GPIO_14',      (),   'GPIO',      'KP_MK',     'NONE',      'CCIC1',     'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x090, 'E24',   'GPIO_15',      (),   'GPIO',      'KP_MK',     'KP_DK',     'CCIC1',     'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x094, 'G24',   'GPIO_16',      (),   'GPIO',      'KP_DK',     'ROT',       'CCIC1',     'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x098, 'F24',   'GPIO_17',      (),   'GPIO',      'KP_DK',     'ROT',       'CCIC1',     'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x09c, 'H23',   'GPIO_18',      (),   'GPIO',      'KP_DK',     'ROT',       'CCIC1',     'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x0a0, 'A25',   'GPIO_19',      (),   'GPIO',      'KP_DK',     'ROT',       'CCIC1',     'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x0a4, 'B25',   'GPIO_20',      (),   'GPIO',      'KP_DK',     'TB',        'CCIC1',     'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x0a8, 'K22',   'GPIO_21',      (),   'GPIO',      'KP_DK',     'TB',        'CCIC1',     'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x0ac, 'C25',   'GPIO_22',      (),   'GPIO',      'KP_DK',     'TB',        'CCIC1',     'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x0b0, 'D25',   'GPIO_23',      (),   'GPIO',      'KP_DK',     'TB',        'CCIC1',     'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x0b4, 'E25',   'GPIO_24',      (),   'GPIO',      'I2S',       'VCXO_OUT',  'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x0b8, 'G25',   'GPIO_25',      (),   'GPIO',      'I2S',       'HDMI',      'SSPA2',     'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x0bc, 'J23',   'GPIO_26',      (),   'GPIO',      'I2S',       'HDMI',      'SSPA2',     'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x0c0, 'H24',   'GPIO_27',      (),   'GPIO',      'I2S',       'HDMI',      'SSPA2',     'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x0c4, 'L22',   'GPIO_28',      (),   'GPIO',      'I2S',       'NONE',      'SSPA2',     'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x0c8, 'A26',   'GPIO_29',      (),   'GPIO',      'UART1',     'KP_MK',     'NONE',      'NONE',      'NONE',      'AAS_SPI',   'NONE' ],
[ 0x0cc, 'B26',   'GPIO_30',      (),   'GPIO',      'UART1',     'KP_MK',     'NONE',      'NONE',      'NONE',      'AAS_SPI',   'NONE' ],
[ 0x0d0, 'K23',   'GPIO_31',      (),   'GPIO',      'UART1',     'KP_MK',     'NONE',      'NONE',      'NONE',      'AAS_SPI',   'NONE' ],
[ 0x0d4, 'C26',   'GPIO_32',      (),   'GPIO',      'UART1',     'KP_MK',     'NONE',      'NONE',      'NONE',      'AAS_SPI',   'NONE' ],
[ 0x0d8, 'D26',   'GPIO_33',      (),   'GPIO',      'SSPA2',     'I2S',       'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x0dc, 'B27',   'GPIO_34',      (),   'GPIO',      'SSPA2',     'I2S',       'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x0e0, 'J24',   'GPIO_35',      (),   'GPIO',      'SSPA2',     'I2S',       'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x0e4, 'M22',   'GPIO_36',      (),   'GPIO',      'SSPA2',     'I2S',       'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x0e8, 'H25',   'GPIO_37',      (),   'GPIO',      'MMC2',      'SSP1',      'TWSI2',     'UART2',     'UART3',     'AAS_SPI',   'AAS_TWSI' ],
[ 0x0ec, 'C27',   'GPIO_38',      (),   'GPIO',      'MMC2',      'SSP1',      'TWSI2',     'UART2',     'UART3',     'AAS_SPI',   'AAS_TWSI' ],
[ 0x0f0, 'L23',   'GPIO_39',      (),   'GPIO',      'MMC2',      'SSP1',      'TWSI2',     'UART2',     'UART3',     'AAS_SPI',   'AAS_TWSI' ],
[ 0x0f4, 'C28',   'GPIO_40',      (),   'GPIO',      'MMC2',      'SSP1',      'TWSI2',     'UART2',     'UART3',     'AAS_SPI',   'AAS_TWSI' ],
[ 0x0f8, 'N21',   'GPIO_41',      (),   'GPIO',      'MMC2',      'TWSI5',     'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x0fc, 'D27',   'GPIO_42',      (),   'GPIO',      'MMC2',      'TWSI5',     'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x100, 'F26',   'GPIO_43',      (),   'GPIO',      'TWSI2',     'UART4',     'SSP1',      'UART2',     'UART3',     'NONE',      'AAS_TWSI' ],
[ 0x104, 'G26',   'GPIO_44',      (),   'GPIO',      'TWSI2',     'UART4',     'SSP1',      'UART2',     'UART3',     'NONE',      'AAS_TWSI' ],
[ 0x108, 'E27',   'GPIO_45',      (),   'GPIO',      'UART1',     'UART4',     'SSP1',      'UART2',     'UART3',     'NONE',      'NONE' ],
[ 0x10c, 'K24',   'GPIO_46',      (),   'GPIO',      'UART1',     'UART4',     'SSP1',      'UART2',     'UART3',     'NONE',      'NONE' ],
[ 0x110, 'H26',   'GPIO_47',      (),   'GPIO',      'UART2',     'SSP2',      'TWSI6',     'CAM2',      'AAS_SPI',   'AAS_GPIO',  'NONE' ],
[ 0x114, 'N22',   'GPIO_48',      (),   'GPIO',      'UART2',     'SSP2',      'TWSI6',     'CAM2',      'AAS_SPI',   'AAS_GPIO',  'NONE' ],
[ 0x118, 'M23',   'GPIO_49',      (),   'GPIO',      'UART2',     'SSP2',      'PWM',       'CCIC2',     'AAS_SPI',   'NONE',      'NONE' ],
[ 0x11c, 'F27',   'GPIO_50',      (),   'GPIO',      'UART2',     'SSP2',      'PWM',       'CCIC2',     'AAS_SPI',   'NONE',      'NONE' ],
[ 0x120, 'J25',   'GPIO_51',      (),   'GPIO',      'UART3',     'ROT',       'AAS_GPIO',  'PWM',       'NONE',      'NONE',      'NONE' ],
[ 0x124, 'D28',   'GPIO_52',      (),   'GPIO',      'UART3',     'ROT',       'AAS_GPIO',  'PWM',       'NONE',      'NONE',      'NONE' ],
[ 0x128, 'E28',   'GPIO_53',      (),   'GPIO',      'UART3',     'TWSI2',     'VCXO_REQ',  'NONE',      'PWM',       'NONE',      'AAS_TWSI' ],
[ 0x12c, 'L24',   'GPIO_54',      (),   'GPIO',      'UART3',     'TWSI2',     'VCXO_OUT',  'HDMI',      'PWM',       'NONE',      'AAS_TWSI' ],
[ 0x130, 'F28',   'GPIO_55',      (),   'GPIO',      'SSP2',      'SSP1',      'UART2',     'ROT',       'TWSI2',     'SSP3',      'AAS_TWSI' ],
[ 0x134, 'G27',   'GPIO_56',      (),   'GPIO',      'SSP2',      'SSP1',      'UART2',     'ROT',       'TWSI2',     'KP_DK',     'AAS_TWSI' ],
[ 0x138, 'P22',   'GPIO_57',      (),   'GPIO',      'SSP2_RX',   'SSP1_TXRX', 'SSP2_FRM',  'SSP1_RX',   'VCXO_REQ',  'KP_DK',     'NONE' ],
[ 0x13c, 'G28',   'GPIO_58',      (),   'GPIO',      'SSP2',      'SSP1_RX',   'SSP1_FRM',  'SSP1_TXRX', 'VCXO_REQ',  'KP_DK',     'NONE' ],

[ 0x140, 'L26',   'TWSI1_SCL',    (),   'TWSI1',     'NONE',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ], # XXX
[ 0x144, 'K28',   'TWSI1_SDA',    (),   'TWSI1',     'NONE',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ], # XXX

[ 0x148, 'R23',   'GPIO_123',     (),   'GPIO',      'SLEEP_IND', 'ONE_WIRE',  '32K_CLKOUT','NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x14c, 'L28',   'GPIO_156',     (),   'PRI_JTAG',  'GPIO',      'PWM',       'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x150, 'N25',   'GPIO_157',     (),   'PRI_JTAG',  'GPIO',      'PWM',       'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x154, 'M27',   'GPIO_158',     (),   'PRI_JTAG',  'GPIO',      'PWM',       'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x158, 'N26',   'GPIO_159',     (),   'PRI_JTAG',  'GPIO',      'PWM',       'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],

[ 0x15c, 'M28',   'TRST',	  (),	'PRI_JTAG',  'NONE',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ], # XXX

[ 0x160, 'L27',   '171',          (),   'G_CLKREQ',  'ONE_WIRE',  'NONE',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x164, 'U21',   '114',          (),   'G_CLKOUT',  '32K_CLKOUT','HDMI',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x168, 'N27',   '172',          (),   'VCXO_REQ',  'ONE_WIRE',  'PLL',       'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x16c, 'T24',   '173',          (),   'VCXO_OUT',  '32K_CLKOUT','NONE',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x170, 'W23',   'GPIO_74',      (),   'GPIO',      'LCD',       'SMC',       'MMC4',      'SSP3',      'UART2',     'UART4',     'TIPU' ],
[ 0x174, 'V25',   'GPIO_75',      (),   'GPIO',      'LCD',       'SMC',       'MMC4',      'SSP3',      'UART2',     'UART4',     'TIPU' ],
[ 0x178, 'W22',   'GPIO_76',      (),   'GPIO',      'LCD',       'SMC',       'MMC4',      'SSP3',      'UART2',     'UART4',     'TIPU' ],
[ 0x17c, 'Y25',   'GPIO_77',      (),   'GPIO',      'LCD',       'SMC',       'MMC4',      'SSP3',      'UART2',     'UART4',     'TIPU' ],
[ 0x180, 'W24',   'GPIO_78',      (),   'GPIO',      'LCD',       'HDMI',      'MMC4',      'NONE',      'SSP4',      'AAS_SPI',   'TIPU' ],
[ 0x184, 'Y22',   'GPIO_79',      (),   'GPIO',      'LCD',       'AAS_GPIO',  'MMC4',      'NONE',      'SSP4',      'AAS_SPI',   'TIPU' ],
[ 0x188, 'Y23',   'GPIO_80',      (),   'GPIO',      'LCD',       'AAS_GPIO',  'MMC4',      'NONE',      'SSP4',      'AAS_SPI',   'TIPU' ],
[ 0x18c, 'Y24',   'GPIO_81',      (),   'GPIO',      'LCD',       'AAS_GPIO',  'MMC4',      'NONE',      'SSP4',      'AAS_SPI',   'TIPU' ],
[ 0x190, 'AA20',  'GPIO_82',      (),   'GPIO',      'LCD',       'NONE',      'MMC4',      'NONE',      'NONE',      'CCIC2',     'TIPU' ],
[ 0x194, 'AA24',  'GPIO_83',      (),   'GPIO',      'LCD',       'NONE',      'MMC4',      'NONE',      'NONE',      'CCIC2',     'TIPU' ],
[ 0x198, 'AA23',  'GPIO_84',      (),   'GPIO',      'LCD',       'SMC',       'MMC2',      'NONE',      'TWSI5',     'AAS_TWSI',  'TIPU' ],
[ 0x19c, 'AB21',  'GPIO_85',      (),   'GPIO',      'LCD',       'SMC',       'MMC2',      'NONE',      'TWSI5',     'AAS_TWSI',  'TIPU' ],
[ 0x1a0, 'AB24',  'GPIO_86',      (),   'GPIO',      'LCD',       'SMC',       'MMC2',      'NONE',      'TWSI6',     'CCIC2',     'TIPU' ],
[ 0x1a4, 'AA25',  'GPIO_87',      (),   'GPIO',      'LCD',       'SMC',       'MMC2',      'NONE',      'TWSI6',     'CCIC2',     'TIPU' ],
[ 0x1a8, 'AB22',  'GPIO_88',      (),   'GPIO',      'LCD',       'AAS_GPIO',  'MMC2',      'NONE',      'NONE',      'CCIC2',     'TIPU' ],
[ 0x1ac, 'AB25',  'GPIO_89',      (),   'GPIO',      'LCD',       'AAS_GPIO',  'MMC2',      'NONE',      'NONE',      'CCIC2',     'TIPU' ],
[ 0x1b0, 'AB23',  'GPIO_90',      (),   'GPIO',      'LCD',       'AAS_GPIO',  'MMC2',      'NONE',      'NONE',      'CCIC2',     'TIPU' ],
[ 0x1b4, 'AB27',  'GPIO_91',      (),   'GPIO',      'LCD',       'AAS_GPIO',  'MMC2',      'NONE',      'NONE',      'CCIC2',     'TIPU' ],
[ 0x1b8, 'AB28',  'GPIO_92',      (),   'GPIO',      'LCD',       'AAS_GPIO',  'MMC2',      'NONE',      'NONE',      'CCIC2',     'TIPU' ],
[ 0x1bc, 'AB20',  'GPIO_93',      (),   'GPIO',      'LCD',       'AAS_GPIO',  'MMC2',      'NONE',      'NONE',      'CCIC2',     'TIPU' ],
[ 0x1c0, 'AC24',  'GPIO_94',      (),   'GPIO',      'LCD',       'AAS_GPIO',  'SPI',       'NONE',      'AAS_SPI',   'CCIC2',     'TIPU' ],
[ 0x1c4, 'AC21',  'GPIO_95',      (),   'GPIO',      'LCD',       'TWSI3',     'SPI',       'AAS_DEU_EX','AAS_SPI',   'CCIC2',     'TIPU' ],
[ 0x1c8, 'AC23',  'GPIO_96',      (),   'GPIO',      'LCD',       'TWSI3',     'SPI',       'AAS_DEU_EX','AAS_SPI',   'NONE',      'TIPU' ],
[ 0x1cc, 'AA19',  'GPIO_97',      (),   'GPIO',      'LCD',       'TWSI6',     'SPI',       'AAS_DEU_EX','AAS_SPI',   'NONE',      'TIPU' ],
[ 0x1d0, 'EC25',  'GPIO_98',      (),   'GPIO',      'LCD',       'TWSI6',     'SPI',       'ONE_WIRE',  'NONE',      'NONE',      'TIPU' ],
[ 0x1d4, 'AC27',  'GPIO_99',      (),   'GPIO',      'LCD',       'SMC',       'SPI',       'TWSI5',     'NONE',      'NONE',      'TIPU' ],
[ 0x1d8, 'AC26',  'GPIO_100',     (),   'GPIO',      'LCD',       'SMC',       'SPI',       'TWSI5',     'NONE',      'NONE',      'TIPU' ],
[ 0x1dc, 'AB19',  'GPIO_101',     (),   'GPIO',      'LCD',       'SMC',       'SPI',       'NONE',      'NONE',      'NONE',      'TIPU' ],
[ 0x1e0, 'AF28',  'GPIO_168',     (),   'NAND',      'GPIO',      'MMC3',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x1e4, 'AF25',  'GPIO_167',     (),   'NAND',      'GPIO',      'MMC3',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x1e8, 'AF26',  'GPIO_166',     (),   'NAND',      'GPIO',      'MMC3',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x1ec, 'AE23',  'GPIO_165',     (),   'NAND',      'GPIO',      'MMC3',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x1f0, 'AE21',  'GPIO_107',     (),   'NAND',      'GPIO',      'NONE',      'NONE',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x1f4, 'AF27',  'GPIO_106',     (),   'NAND',      'GPIO',      'NONE',      'NONE',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x1f8, 'AE22',  'GPIO_105',     (),   'NAND',      'GPIO',      'NONE',      'NONE',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x1fc, 'AE25',  'GPIO_104',     (),   'NAND',      'GPIO',      'NONE',      'NONE',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x200, 'AE26',  'GPIO_111',     (),   'NAND',      'GPIO',      'MMC3',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x204, 'AD25',  'GPIO_164',     (),   'NAND',      'GPIO',      'MMC3',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x208, 'AD28',  'GPIO_163',     (),   'NAND',      'GPIO',      'MMC3',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x20c, 'AD24',  'GPIO_162',     (),   'NAND',      'GPIO',      'MMC3',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x210, 'AD26',  'GPIO_161',     (),   'NAND',      'GPIO',      'NONE',      'NONE',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x214, 'AD25',  'GPIO_110',     (),   'NAND',      'GPIO',      'NONE',      'NONE',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x218, 'AD24',  'GPIO_109',     (),   'NAND',      'GPIO',      'NONE',      'NONE',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x21c, 'AD23',  'GPIO_108',     (),   'NAND',      'GPIO',      'NONE',      'NONE',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x220, 'AF23',  'GPIO_143',     (),   'NAND',      'GPIO',      'SMC',       'NONE',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x224, 'AF24',  'GPIO_144',     (),   'NAND',      'GPIO',      'SMC_INT',   'SMC',       'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x228, 'AC19',  'GPIO_145',     (),   'SMC',       'GPIO',      'NONE',      'NONE',      'SMC',       'NONE',      'NONE',      'NONE' ],
[ 0x22c, 'AC20',  'GPIO_146',     (),   'SMC',       'GPIO',      'NONE',      'NONE',      'SMC',       'NONE',      'NONE',      'NONE' ],
[ 0x230, 'AG25',  'GPIO_147',     (),   'NAND',      'GPIO',      'NONE',      'NONE',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x234, 'AG26',  'GPIO_148',     (),   'NAND',      'GPIO',      'NONE',      'NONE',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x238, 'AA16',  'GPIO_149',     (),   'NAND',      'GPIO',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x23c, 'AH26',  'GPIO_150',     (),   'NAND',      'GPIO',      'NONE',      'NONE',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x240, 'AG24',  'GPIO_151',     (),   'SMC',       'GPIO',      'MMC3',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x244, 'AH25',  'GPIO_112',     (),   'NAND',      'GPIO',      'MMC3',      'SMC',       'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x248, 'AC18',  'GPIO_152',     (),   'SMC',       'GPIO',      'NONE',      'NONE',      'SMC',       'NONE',      'NONE',      'NONE' ],
[ 0x24c, 'AD18',  'GPIO_153',     (),   'SMC',       'GPIO',      'NONE',      'NONE',      'SMC',       'NONE',      'NONE',      'NONE' ],
[ 0x250, 'AH24',  'GPIO_160',     (),   'NAND',      'GPIO',      'SMC',       'NONE',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x254, 'AB17',  'GPIO_154',     (),   'SMC_INT',   'GPIO',      'SMC',       'NONE',      'NAND',      'NONE',      'NONE',      'NONE' ],
[ 0x258, 'AE19',  'GPIO_155',     (),   'EXT_DMA',   'GPIO',      'SMC',       'NONE',      'EXT_DMA',   'NONE',      'NONE',      'NONE' ],
[ 0x25c, 'AC17',  'GPIO_113',     (),   'SMC',       'GPIO',      'EXT_DMA',   'MMC3',      'SMC',       'HDMI',      'NONE',      'NONE' ],
[ 0x260, 'N23',   'GPIO_115',     (),   'GPIO',      'NONE',      'AC',        'UART4',     'UART3',     'SSP1',      'NONE',      'NONE' ],
[ 0x264, 'H27',   'GPIO_116',     (),   'GPIO',      'NONE',      'AC',        'UART4',     'UART3',     'SSP1',      'NONE',      'NONE' ],
[ 0x268, 'H28',   'GPIO_117',     (),   'GPIO',      'NONE',      'AC',        'UART4',     'UART3',     'SSP1',      'NONE',      'NONE' ],
[ 0x26c, 'J28',   'GPIO_118',     (),   'GPIO',      'NONE',      'AC',        'UART4',     'UART3',     'SSP1',      'NONE',      'NONE' ],
[ 0x270, 'M24',   'GPIO_119',     (),   'GPIO',      'NONE',      'CA',        'SSP3',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x274, 'J27',   'GPIO_120',     (),   'GPIO',      'NONE',      'CA',        'SSP3',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x278, 'K26',   'GPIO_121',     (),   'GPIO',      'NONE',      'CA',        'SSP3',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x27c, 'L25',   'GPIO_122',     (),   'GPIO',      'NONE',      'CA',        'SSP3',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x280, 'AD13',  'GPIO_59',      (),   'GPIO',      'CCIC1',     'ULPI',      'MMC3',      'CCIC2',     'UART3',     'UART4',     'NONE' ],
[ 0x284, 'AE13',  'GPIO_60',      (),   'GPIO',      'CCIC1',     'ULPI',      'MMC3',      'CCIC2',     'UART3',     'UART4',     'NONE' ],
[ 0x288, 'AF13',  'GPIO_61',      (),   'GPIO',      'CCIC1',     'ULPI',      'MMC3',      'CCIC2',     'UART3',     'HDMI',      'NONE' ],
[ 0x28c, 'AH13',  'GPIO_62',      (),   'GPIO',      'CCIC1',     'ULPI',      'MMC3',      'CCIC2',     'UART3',     'NONE',      'NONE' ],
[ 0x290, 'AG13',  'GPIO_63',      (),   'GPIO',      'CCIC1',     'ULPI',      'MMC3',      'CCIC2',     'MSP',       'UART4',     'NONE' ],
[ 0x294, 'AC12',  'GPIO_64',      (),   'GPIO',      'CCIC1',     'ULPI',      'MMC3',      'CCIC2',     'MSP',       'UART4',     'NONE' ],
[ 0x298, 'AD12',  'GPIO_65',      (),   'GPIO',      'CCIC1',     'ULPI',      'MMC3',      'CCIC2',     'MSP',       'UART4',     'NONE' ],
[ 0x29c, 'AE12',  'GPIO_66',      (),   'GPIO',      'CCIC1',     'ULPI',      'MMC3',      'CCIC2',     'MSP',       'UART4',     'NONE' ],
[ 0x2a0, 'AF12',  'GPIO_67',      (),   'GPIO',      'CCIC1',     'ULPI',      'MMC3',      'CCIC2',     'MSP',       'NONE',      'NONE' ],
[ 0x2a4, 'AG12',  'GPIO_68',      (),   'GPIO',      'CCIC1',     'ULPI',      'MMC3',      'CCIC2',     'MSP',       'LCD',       'NONE' ],
[ 0x2a8, 'AH12',  'GPIO_69',      (),   'GPIO',      'CCIC1',     'ULPI',      'MMC3',      'CCIC2',     'NONE',      'LCD',       'NONE' ],
[ 0x2ac, 'AC11',  'GPIO_70',      (),   'GPIO',      'CCIC1',     'ULPI',      'MMC3',      'CCIC2',     'MSP',       'LCD',       'NONE' ],
[ 0x2b0, 'AD11',  'GPIO_71',      (),   'GPIO',      'TWSI3',     'NONE',      'PWM',       'NONE',      'NONE',      'LCD',       'AAS_TWSI' ],
[ 0x2b4, 'AG11',  'GPIO_72',      (),   'GPIO',      'TWSI3',     'HDMI',      'PWM',       'NONE',      'NONE',      'LCD',       'AAS_TWSI' ],
[ 0x2b8, 'AE11',  'GPIO_73',      (),   'GPIO',      'VCXO_REQ',  '32K_CLKOUT','PWM',       'VCXO_OUT',  'NONE',      'LCD',       'NONE' ],
[ 0x2bc, 'D18',   'TWSI4_SCL',    (),   'TWSI4',     'LCD',       'NONE',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
[ 0x2c0, 'A19',   'TWSI4_SDA',    (),   'TWSI4',     'LCD',       'NONE',      'NONE',      'NONE',      'NONE',      'NONE',      'NONE' ],
);

use Data::Dumper;

%ariel = (
0x000 => 0x90C0, 0x004 => 0xD0C0, 0x008 => 0xD0C0, 0x00c => 0xD0C0, 0x010 => 0xD0C0, 0x014 => 0x4000, 0x018 => 0x4000,
0x01c => 0xD0C0, 0x020 => 0xD0C0, 0x024 => 0xD0C0, 0x028 => 0x1001, 0x02c => 0x1001, 0x030 => 0x1001, 0x034 => 0x1001,
0x038 => 0x1001, 0x03c => 0x1001, 0x040 => 0xD0C0, 0x044 => 0xD0C0, 0x048 => 0xD0C0, 0x04c => 0xC001, 0x050 => 0xC001,
0x054 => 0x0000, 0x058 => 0x0000, 0x05c => 0x0000, 0x060 => 0x0000, 0x064 => 0x0000, 0x068 => 0x0000, 0x06c => 0x0000,
0x070 => 0x0000, 0x074 => 0x0000, 0x078 => 0x0000, 0x07c => 0x0000, 0x080 => 0x0000, 0x084 => 0x0000, 0x088 => 0x0000,
0x08c => 0x0000, 0x090 => 0x0000, 0x094 => 0x0000, 0x098 => 0x0000, 0x09c => 0x0000, 0x0a0 => 0x0000, 0x0a4 => 0x0000,
0x0a8 => 0x0000, 0x0ac => 0x0000, 0x0b0 => 0x0000, 0x0b4 => 0xD0C0, 0x0b8 => 0xD0C0, 0x0bc => 0xD0C0, 0x0c0 => 0xD0C0,
0x0c4 => 0xD0C0, 0x0c8 => 0xD0C0, 0x0cc => 0xD0C0, 0x0d0 => 0xD0C0, 0x0d4 => 0xD0C0, 0x0d8 => 0xD0C0, 0x0dc => 0xD0C0,
0x0e0 => 0xD0C0, 0x0e4 => 0xD0C0, 0x0e8 => 0xD0C0, 0x0ec => 0xD0C0, 0x0f0 => 0xD0C0, 0x0f4 => 0xD0C0, 0x0f8 => 0x0802,
0x0fc => 0x0802, 0x100 => 0x5003, 0x104 => 0x5003, 0x108 => 0x5003, 0x10c => 0x00C0, 0x110 => 0x0803, 0x114 => 0x0803,
0x118 => 0xD0C0, 0x11c => 0xD0C0, 0x120 => 0x1001, 0x124 => 0x1001, 0x128 => 0xD0C0, 0x12c => 0xD0C0, 0x130 => 0x6400,
0x134 => 0x6000, 0x138 => 0x6400, 0x13c => 0x6000, 0x140 => 0xD0C0, 0x144 => 0xD0C0, 0x148 => 0xD0C0, 0x14c => 0xD0C0,
0x150 => 0xD0C0, 0x154 => 0xD0C0, 0x158 => 0xD0C0, 0x15c => 0xB0C0, 0x160 => 0x90C0, 0x164 => 0xD0C0, 0x168 => 0xB0C0,
0x16c => 0x90C0, 0x170 => 0x0801, 0x174 => 0x0801, 0x178 => 0xD801, 0x17c => 0x0801, 0x180 => 0x0801, 0x184 => 0x0801,
0x188 => 0x0801, 0x18c => 0x0801, 0x190 => 0x0801, 0x194 => 0x0801, 0x198 => 0x0801, 0x19c => 0x0801, 0x1a0 => 0x0801,
0x1a4 => 0x0801, 0x1a8 => 0x0801, 0x1ac => 0x0801, 0x1b0 => 0x0801, 0x1b4 => 0x0801, 0x1b8 => 0x0801, 0x1bc => 0x0801,
0x1c0 => 0x0801, 0x1c4 => 0x0801, 0x1c8 => 0x0801, 0x1cc => 0x0801, 0x1d0 => 0x0801, 0x1d4 => 0x0801, 0x1d8 => 0x0801,
0x1dc => 0x0801, 0x1e0 => 0x0000, 0x1e4 => 0x0000, 0x1e8 => 0x0000, 0x1ec => 0x0000, 0x1f0 => 0x0000, 0x1f4 => 0x0000,
0x1f8 => 0x0000, 0x1fc => 0x0000, 0x200 => 0x1002, 0x204 => 0x1002, 0x208 => 0x1002, 0x20c => 0x1002, 0x210 => 0x1002,
0x214 => 0x1002, 0x218 => 0x1002, 0x21c => 0x1002, 0x220 => 0x0000, 0x224 => 0x0000, 0x228 => 0x1002, 0x22c => 0x1002,
0x230 => 0x0000, 0x234 => 0x0000, 0x238 => 0x0000, 0x23c => 0x0000, 0x240 => 0x0000, 0x244 => 0x0000, 0x248 => 0x0000,
0x24c => 0xD8C0, 0x250 => 0xD8C0, 0x254 => 0x0000, 0x258 => 0xD8C0, 0x25c => 0xD8C0, 0x260 => 0xD0C0, 0x264 => 0xD0C0,
0x268 => 0xD0C0, 0x26c => 0xD0C0, 0x270 => 0xD0C0, 0x274 => 0xD0C0, 0x278 => 0xD0C0, 0x27c => 0xD0C0, 0x280 => 0xD0C0,
0x284 => 0xD0C0, 0x288 => 0xD0C0, 0x28c => 0xD0C0, 0x290 => 0xD0C0, 0x294 => 0xD0C0, 0x298 => 0xD0C0, 0x29c => 0xD0C0,
0x2a0 => 0xD0C0, 0x2a4 => 0xD0C0, 0x2a8 => 0xD0C0, 0x2ac => 0xD0C0, 0x2b0 => 0x0801, 0x2b4 => 0x0801, 0x2b8 => 0xD0C0,
0x2bc => 0x0800, 0x2c0 => 0x0800,
);


#  1101 0000 1100 0000
#  0000 1000 0000 0010
#  ``|``||```|``|  ```---- func
#    |  ||   |  `--------- 1, 2, 3, 4 = edge-rise, edge-fall, edge-both, edge-clr
#    |  ||   `------------ 0, 1, 2, 4 = sleep0, sleepi, sleep1, sleep-
#    |  |`---------------- 1 = twsi
#    |  `----------------- 0, 1, 2, 3 = very slow, slow, medium, fast
#    `-------------------- 0, 4, 5, 6 = bias-off, float, pull-dn, pull-up

@edge = ('(edge-0)', 'edge-rise', 'edge-fall', 'edge-both', 'edge-clr');
@sleep = ('sleep0', 'sleepi', 'sleep1', 'sleep-');
@twsi = ('twsi-on', 'twsi-off');
@speed = ('very-slow', 'slow', 'medium', 'fast');
# af up dn
#  0  0  0 off
#  1  0  0 float
#  1  0  1 dn 
#  1  1  0 up
#  0  1  0 up eh
#  0  1  0 SSP1 GPIO43 - GPIO45
#  0  1  1 eheh GPIO55 - GPIO58
@bias = ('bias-off', 'bad-bias-1', 'bad-bias-2', 'bad-bias-3', 'float', 'pull-dn', 'pull-up');

foreach (@pins) {
	($off, $loc, $name, @func) = @$_;
	$val = delete $ariel{$off};
	die unless defined $val;
	#die Dumper [ $off, $val, $loc, $name, \@func ];
	#($name, $val, @func) = @$_;
	printf "%-10s %-5s 0x%08x %d | %-10s %-10s %-10s %-10s %-10s %-10s [%-10s %-10s %-10s %-10s %-10s %-10s]\n",
	$name, $loc, $val, ($val >>  0) & 0x7,
	$func[ ($val >>  0) & 0x7] // 'bad-func',
	$edge[ ($val >>  4) & 0x7] // 'bad-edge',
	$sleep[($val >>  7) & 0x7] // 'bad-sleep',
	$twsi[ ($val >> 10) & 0x1] // 'bad-twsi',
	$speed[($val >> 11) & 0x3] // 'bad-speed',
	$bias[ ($val >> 13) & 0x7] // 'bad-bias',
	@func,
	;

	#warn Dumper [ $name, $val ];
}

die Dumper \%ariel;
