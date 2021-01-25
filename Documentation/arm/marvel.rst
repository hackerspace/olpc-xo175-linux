================
ARM Marvell SoCs
================

This document lists all the ARM Marvell SoCs that are currently
supported in mainline by the Linux kernel. As the Marvell families of
SoCs are large and complex, it is hard to understand where the support
for a particular SoC is available in the Linux kernel. This document
tries to help in understanding where those SoCs are supported, and to
match them with their corresponding public datasheet, when available.

Orion family
------------

  Flavors:
        - 88F5082
        - 88F5181
        - 88F5181L
        - 88F5182

               - Datasheet: `MV88F5182-datasheet.pdf`_
               - Programmer's User Guide: `MV88F5182-opensource-manual.pdf`_
               - User Manual: `MV88F5182-usermanual.pdf`_
        - 88F5281

               - Datasheet: `marvel_88f5281_data_sheet.pdf`_
        - 88F6183
  Core:
	Feroceon 88fr331 (88f51xx) or 88fr531-vd (88f52xx) ARMv5 compatible
  Linux kernel mach directory:
	arch/arm/mach-orion5x
  Linux kernel plat directory:
	arch/arm/plat-orion

.. _MV88F5182-datasheet.pdf: http://www.embeddedarm.com/documentation/third-party/MV88F5182-datasheet.pdf
.. _MV88F5182-opensource-manual.pdf: http://www.embeddedarm.com/documentation/third-party/MV88F5182-opensource-manual.pdf
.. _MV88F5182-usermanual.pdf: http://www.embeddedarm.com/documentation/third-party/MV88F5182-usermanual.pdf
.. _marvel_88f5281_data_sheet.pdf: http://www.ocmodshop.com/images/reviews/networking/qnap_ts409u/marvel_88f5281_data_sheet.pdf

Kirkwood family
---------------

  Flavors:
        - 88F6282 a.k.a Armada 300

                - Product Brief  : `armada_310.pdf`_
        - 88F6283 a.k.a Armada 310

                - Product Brief  : `armada_310.pdf`_
        - 88F6190

                - Product Brief  : `88F6190-003_WEB.pdf`_
                - Hardware Spec  : `HW_88F619x_OpenSource.pdf`_
                - Functional Spec: `FS_88F6180_9x_6281_OpenSource.pdf`_
        - 88F6192

                - Product Brief  : `88F6192-003_ver1.pdf`_
                - Hardware Spec  : `HW_88F619x_OpenSource.pdf`_
                - Functional Spec: `FS_88F6180_9x_6281_OpenSource.pdf`_
        - 88F6182
        - 88F6180

                - Product Brief  : `88F6180-003_ver1.pdf`_
                - Hardware Spec  : `HW_88F6180_OpenSource.pdf`_
                - Functional Spec: `FS_88F6180_9x_6281_OpenSource.pdf`_
        - 88F6281

                - Product Brief  : `88F6281-004_ver1.pdf`_
                - Hardware Spec  : `HW_88F6281_OpenSource.pdf`_
                - Functional Spec: `FS_88F6180_9x_6281_OpenSource.pdf`_
  Homepage:
	http://www.marvell.com/embedded-processors/kirkwood/
  Core:
	Feroceon 88fr131 ARMv5 compatible
  Linux kernel mach directory:
	arch/arm/mach-mvebu
  Linux kernel plat directory:
	none

.. _armada_310.pdf: http://www.marvell.com/embedded-processors/armada-300/assets/armada_310.pdf
.. _armada_310.pdf: http://www.marvell.com/embedded-processors/armada-300/assets/armada_310.pdf
.. _88F6190-003_WEB.pdf: http://www.marvell.com/embedded-processors/kirkwood/assets/88F6190-003_WEB.pdf
.. _HW_88F619x_OpenSource.pdf: http://www.marvell.com/embedded-processors/kirkwood/assets/HW_88F619x_OpenSource.pdf
.. _FS_88F6180_9x_6281_OpenSource.pdf: http://www.marvell.com/embedded-processors/kirkwood/assets/FS_88F6180_9x_6281_OpenSource.pdf
.. _88F6192-003_ver1.pdf: http://www.marvell.com/embedded-processors/kirkwood/assets/88F6192-003_ver1.pdf
.. _HW_88F619x_OpenSource.pdf: http://www.marvell.com/embedded-processors/kirkwood/assets/HW_88F619x_OpenSource.pdf
.. _FS_88F6180_9x_6281_OpenSource.pdf: http://www.marvell.com/embedded-processors/kirkwood/assets/FS_88F6180_9x_6281_OpenSource.pdf
.. _88F6180-003_ver1.pdf: http://www.marvell.com/embedded-processors/kirkwood/assets/88F6180-003_ver1.pdf
.. _HW_88F6180_OpenSource.pdf: http://www.marvell.com/embedded-processors/kirkwood/assets/HW_88F6180_OpenSource.pdf
.. _FS_88F6180_9x_6281_OpenSource.pdf: http://www.marvell.com/embedded-processors/kirkwood/assets/FS_88F6180_9x_6281_OpenSource.pdf
.. _88F6281-004_ver1.pdf: http://www.marvell.com/embedded-processors/kirkwood/assets/88F6281-004_ver1.pdf
.. _HW_88F6281_OpenSource.pdf: http://www.marvell.com/embedded-processors/kirkwood/assets/HW_88F6281_OpenSource.pdf
.. _FS_88F6180_9x_6281_OpenSource.pdf: http://www.marvell.com/embedded-processors/kirkwood/assets/FS_88F6180_9x_6281_OpenSource.pdf

Discovery family
----------------

  Flavors:
        - MV78100

                - Product Brief  : `MV78100-003_WEB.pdf`_
                - Hardware Spec  : `HW_MV78100_OpenSource.pdf`_
                - Functional Spec: `FS_MV76100_78100_78200_OpenSource.pdf`_
        - MV78200

                - Product Brief  : `MV78200-002_WEB.pdf`_
                - Hardware Spec  : `HW_MV78200_OpenSource.pdf`_
                - Functional Spec: `FS_MV76100_78100_78200_OpenSource.pdf`_
        - MV76100

                Not supported by the Linux kernel.

  Core:
	Feroceon 88fr571-vd ARMv5 compatible

  Linux kernel mach directory:
	arch/arm/mach-mv78xx0
  Linux kernel plat directory:
	arch/arm/plat-orion

.. _MV78100-003_WEB.pdf: http://www.marvell.com/embedded-processors/discovery-innovation/assets/MV78100-003_WEB.pdf
.. _HW_MV78100_OpenSource.pdf: http://www.marvell.com/embedded-processors/discovery-innovation/assets/HW_MV78100_OpenSource.pdf
.. _FS_MV76100_78100_78200_OpenSource.pdf: http://www.marvell.com/embedded-processors/discovery-innovation/assets/FS_MV76100_78100_78200_OpenSource.pdf
.. _MV78200-002_WEB.pdf: http://www.marvell.com/embedded-processors/discovery-innovation/assets/MV78200-002_WEB.pdf
.. _HW_MV78200_OpenSource.pdf: http://www.marvell.com/embedded-processors/discovery-innovation/assets/HW_MV78200_OpenSource.pdf
.. _FS_MV76100_78100_78200_OpenSource.pdf: http://www.marvell.com/embedded-processors/discovery-innovation/assets/FS_MV76100_78100_78200_OpenSource.pdf

EBU Armada family
-----------------

  Armada 370 Flavors:
        - 88F6710
        - 88F6707
        - 88F6W11

    - Product Brief:   `Marvell_ARMADA_370_SoC.pdf`_
    - Hardware Spec:   `ARMADA370-datasheet.pdf`_
    - Functional Spec: `ARMADA370-FunctionalSpec-datasheet.pdf`_

  Core:
	Sheeva ARMv7 compatible PJ4B

  Armada 375 Flavors:
	- 88F6720

    - Product Brief: `ARMADA_375_SoC-01_product_brief.pdf`_

  Core:
	ARM Cortex-A9

  Armada 38x Flavors:
	- 88F6810	Armada 380
	- 88F6820 Armada 385
	- 88F6828 Armada 388

    - Product infos:   http://www.marvell.com/embedded-processors/armada-38x/
    - Functional Spec: https://marvellcorp.wufoo.com/forms/marvell-armada-38x-functional-specifications/

  Core:
	ARM Cortex-A9

  Armada 39x Flavors:
	- 88F6920 Armada 390
	- 88F6928 Armada 398

    - Product infos: http://www.marvell.com/embedded-processors/armada-39x/

  Core:
	ARM Cortex-A9

  Armada XP Flavors:
        - MV78230
        - MV78260
        - MV78460

    NOTE:
	not to be confused with the non-SMP 78xx0 SoCs

    Product Brief:
	`Marvell-ArmadaXP-SoC-product%20brief.pdf`_

    Functional Spec:
	`ARMADA-XP-Functional-SpecDatasheet.pdf`_

    - Hardware Specs:

        - `HW_MV78230_OS.PDF`_
        - `HW_MV78260_OS.PDF`_
        - `HW_MV78460_OS.PDF`_

  Core:
	Sheeva ARMv7 compatible Dual-core or Quad-core PJ4B-MP

  Linux kernel mach directory:
	arch/arm/mach-mvebu
  Linux kernel plat directory:
	none

.. _Marvell_ARMADA_370_SoC.pdf: http://www.marvell.com/embedded-processors/armada-300/assets/Marvell_ARMADA_370_SoC.pdf
.. _ARMADA370-datasheet.pdf: http://www.marvell.com/embedded-processors/armada-300/assets/ARMADA370-datasheet.pdf
.. _ARMADA370-FunctionalSpec-datasheet.pdf: http://www.marvell.com/embedded-processors/armada-300/assets/ARMADA370-FunctionalSpec-datasheet.pdf
.. _ARMADA_375_SoC-01_product_brief.pdf: http://www.marvell.com/embedded-processors/armada-300/assets/ARMADA_375_SoC-01_product_brief.pdf
.. _Marvell-ArmadaXP-SoC-product%20brief.pdf: http://www.marvell.com/embedded-processors/armada-xp/assets/Marvell-ArmadaXP-SoC-product%20brief.pdf
.. _ARMADA-XP-Functional-SpecDatasheet.pdf: http://www.marvell.com/embedded-processors/armada-xp/assets/ARMADA-XP-Functional-SpecDatasheet.pdf
.. _HW_MV78230_OS.PDF: http://www.marvell.com/embedded-processors/armada-xp/assets/HW_MV78230_OS.PDF
.. _HW_MV78260_OS.PDF: http://www.marvell.com/embedded-processors/armada-xp/assets/HW_MV78260_OS.PDF
.. _HW_MV78460_OS.PDF: http://www.marvell.com/embedded-processors/armada-xp/assets/HW_MV78460_OS.PDF

EBU Armada family ARMv8
-----------------------

  Armada 3710/3720 Flavors:
	- 88F3710
	- 88F3720

  Core:
	ARM Cortex A53 (ARMv8)

  Homepage:
	http://www.marvell.com/embedded-processors/armada-3700/

  Product Brief:
	`PB-88F3700-FNL.pdf`_

  Device tree files:
	arch/arm64/boot/dts/marvell/armada-37*

  Armada 7K Flavors:
	  - 88F7020 (AP806 Dual + one CP110)
	  - 88F7040 (AP806 Quad + one CP110)

  Core: ARM Cortex A72

  Homepage:
	http://www.marvell.com/embedded-processors/armada-70xx/

  Product Brief:
	  - `Armada7020PB-Jan2016.pdf`_
	  - `Armada7040PB-Jan2016.pdf`_

  Device tree files:
	arch/arm64/boot/dts/marvell/armada-70*

  Armada 8K Flavors:
	- 88F8020 (AP806 Dual + two CP110)
	- 88F8040 (AP806 Quad + two CP110)
  Core:
	ARM Cortex A72

  Homepage:
	http://www.marvell.com/embedded-processors/armada-80xx/

  Product Brief:
	  - `Armada8020PB-Jan2016.pdf`_
	  - `Armada8040PB-Jan2016.pdf`_

  Device tree files:
	arch/arm64/boot/dts/marvell/armada-80*

.. _PB-88F3700-FNL.pdf: http://www.marvell.com/embedded-processors/assets/PB-88F3700-FNL.pdf
.. _Armada7020PB-Jan2016.pdf: http://www.marvell.com/embedded-processors/assets/Armada7020PB-Jan2016.pdf
.. _Armada7040PB-Jan2016.pdf: http://www.marvell.com/embedded-processors/assets/Armada7040PB-Jan2016.pdf
.. _Armada8020PB-Jan2016.pdf: http://www.marvell.com/embedded-processors/assets/Armada8020PB-Jan2016.pdf
.. _Armada8040PB-Jan2016.pdf: http://www.marvell.com/embedded-processors/assets/Armada8040PB-Jan2016.pdf

Avanta family
-------------

  Flavors:
       - 88F6510
       - 88F6530P
       - 88F6550
       - 88F6560

  Homepage:
	http://www.marvell.com/broadband/

  Product Brief:
	`Marvell_Avanta_88F6510_305_060-001_product_brief.pdf`_

  No public datasheet available.

  Core:
	ARMv5 compatible

  Linux kernel mach directory:
	no code in mainline yet, planned for the future
  Linux kernel plat directory:
	no code in mainline yet, planned for the future

.. _Marvell_Avanta_88F6510_305_060-001_product_brief.pdf: http://www.marvell.com/broadband/assets/Marvell_Avanta_88F6510_305_060-001_product_brief.pdf

Storage family
--------------

  Armada SP:
	- 88RC1580

  Product infos:
	http://www.marvell.com/storage/armada-sp/

  Core:
	Sheeva ARMv7 comatible Quad-core PJ4C

  (not supported in upstream Linux kernel)

Dove family (application processor)
-----------------------------------

  Flavors:
        - 88AP510 a.k.a Armada 510

   Product Brief:
	`Marvell_Armada510_SoC.pdf`_

   Hardware Spec:
	`Armada-510-Hardware-Spec.pdf`_

  Functional Spec:
	`Armada-510-Functional-Spec.pdf`_

  Homepage:
	http://www.marvell.com/application-processors/armada-500/

  Core:
	ARMv7 compatible

  Directory:
	- arch/arm/mach-mvebu (DT enabled platforms)
        - arch/arm/mach-dove (non-DT enabled platforms)

.. _Marvell_Armada510_SoC.pdf: http://www.marvell.com/application-processors/armada-500/assets/Marvell_Armada510_SoC.pdf
.. _Armada-510-Hardware-Spec.pdf: http://www.marvell.com/application-processors/armada-500/assets/Armada-510-Hardware-Spec.pdf
.. _Armada-510-Functional-Spec.pdf: http://www.marvell.com/application-processors/armada-500/assets/Armada-510-Functional-Spec.pdf

PXA 2xx/3xx/93x/95x family
--------------------------

  Flavors:
        - PXA21x, PXA25x, PXA26x
             - Application processor only
             - Core: ARMv5 XScale1 core
        - PXA270, PXA271, PXA272
             - Product Brief         : `pxa_27x_pb.pdf`_
             - Design guide          : `pxa_27x_design_guide.pdf`_
             - Developers manual     : `pxa_27x_dev_man.pdf`_
             - Specification         : `pxa_27x_emts.pdf`_
             - Specification update  : `pxa_27x_spec_update.pdf`_
             - Application processor only
             - Core: ARMv5 XScale2 core
        - PXA300, PXA310, PXA320
             - PXA 300 Product Brief : `PXA300_PB_R4.pdf`_
             - PXA 310 Product Brief : `PXA310_PB_R4.pdf`_
             - PXA 320 Product Brief : `PXA320_PB_R4.pdf`_
             - Design guide          : `PXA3xx_Design_Guide.pdf`_
             - Developers manual     : `PXA3xx_Developers_Manual.zip`_
             - Specifications        : `PXA3xx_EMTS.pdf`_
             - Specification Update  : `PXA3xx_Spec_Update.zip`_
             - Reference Manual      : `PXA3xx_TavorP_BootROM_Ref_Manual.pdf`_
             - Application processor only
             - Core: ARMv5 XScale3 core
        - PXA930, PXA935
             - Application processor with Communication processor
             - Core: ARMv5 XScale3 core
        - PXA955
             - Application processor with Communication processor
             - Core: ARMv7 compatible Sheeva PJ4 core

   Comments:

    * This line of SoCs originates from the XScale family developed by
      Intel and acquired by Marvell in ~2006. The PXA21x, PXA25x,
      PXA26x, PXA27x, PXA3xx and PXA93x were developed by Intel, while
      the later PXA95x were developed by Marvell.

    * Due to their XScale origin, these SoCs have virtually nothing in
      common with the other (Kirkwood, Dove, etc.) families of Marvell
      SoCs, except with the MMP/MMP2 family of SoCs.

   Linux kernel mach directory:
	arch/arm/mach-pxa
   Linux kernel plat directory:
	arch/arm/plat-pxa

.. _pxa_27x_pb.pdf: http://www.marvell.com/application-processors/pxa-family/assets/pxa_27x_pb.pdf
.. _pxa_27x_design_guide.pdf: http://www.marvell.com/application-processors/pxa-family/assets/pxa_27x_design_guide.pdf
.. _pxa_27x_dev_man.pdf: http://www.marvell.com/application-processors/pxa-family/assets/pxa_27x_dev_man.pdf
.. _pxa_27x_emts.pdf: http://www.marvell.com/application-processors/pxa-family/assets/pxa_27x_emts.pdf
.. _pxa_27x_spec_update.pdf: http://www.marvell.com/application-processors/pxa-family/assets/pxa_27x_spec_update.pdf
.. _PXA300_PB_R4.pdf: http://www.marvell.com/application-processors/pxa-family/assets/PXA300_PB_R4.pdf
.. _PXA310_PB_R4.pdf: http://www.marvell.com/application-processors/pxa-family/assets/PXA310_PB_R4.pdf
.. _PXA320_PB_R4.pdf: http://www.marvell.com/application-processors/pxa-family/assets/PXA320_PB_R4.pdf
.. _PXA3xx_Design_Guide.pdf: http://www.marvell.com/application-processors/pxa-family/assets/PXA3xx_Design_Guide.pdf
.. _PXA3xx_Developers_Manual.zip: http://www.marvell.com/application-processors/pxa-family/assets/PXA3xx_Developers_Manual.zip
.. _PXA3xx_EMTS.pdf: http://www.marvell.com/application-processors/pxa-family/assets/PXA3xx_EMTS.pdf
.. _PXA3xx_Spec_Update.zip: http://www.marvell.com/application-processors/pxa-family/assets/PXA3xx_Spec_Update.zip
.. _PXA3xx_TavorP_BootROM_Ref_Manual.pdf: http://www.marvell.com/application-processors/pxa-family/assets/PXA3xx_TavorP_BootROM_Ref_Manual.pdf

MMP/MMP2/MMP3 family (communication processor)
----------------------------------------------

   Flavors:
        - PXA168, a.k.a Armada 168
             - Homepage             : `armada-168.jsp`_
             - Product brief        : `pxa_168_pb.pdf`_
             - Hardware manual      : `armada_16x_datasheet.pdf`_
             - Software manual      : `armada_16x_software_manual.pdf`_
             - Specification update : `ARMADA16x_Spec_update.pdf`_
             - Boot ROM manual      : `armada_16x_ref_manual.pdf`_
             - App node package     : `armada_16x_app_note_package.pdf`_
             - Application processor only
             - Core: ARMv5 compatible Marvell PJ1 88sv331 (Mohawk)
        - PXA910/PXA920
             - Homepage             : http://www.marvell.com/communication-processors/pxa910/
             - Product Brief        : `Marvell_PXA910_Platform-001_PB_final.pdf`_
             - Application processor with Communication processor
             - Core: ARMv5 compatible Marvell PJ1 88sv331 (Mohawk)
        - PXA688, a.k.a. MMP2, a.k.a Armada 610
             - Product Brief        : `armada610_pb.pdf`_
             - Application processor only
             - Core: ARMv7 compatible Sheeva PJ4 88sv581x core
	- PXA2128, a.k.a. MMP3 (OLPC XO4, Linux support not upstream)
	     - Product Brief	  : `Marvell-ARMADA-PXA2128-SoC-PB.pdf`_
	     - Application processor only
	     - Core: Dual-core ARMv7 compatible Sheeva PJ4C core
	- PXA960/PXA968/PXA978 (Linux support not upstream)
	     - Application processor with Communication Processor
	     - Core: ARMv7 compatible Sheeva PJ4 core
	- PXA986/PXA988 (Linux support not upstream)
	     - Application processor with Communication Processor
	     - Core: Dual-core ARMv7 compatible Sheeva PJ4B-MP core
	- PXA1088/PXA1920 (Linux support not upstream)
	     - Application processor with Communication Processor
	     - Core: quad-core ARMv7 Cortex-A7
	- PXA1908/PXA1928/PXA1936
	     - Application processor with Communication Processor
	     - Core: multi-core ARMv8 Cortex-A53

   Comments:

    * This line of SoCs originates from the XScale family developed by
      Intel and acquired by Marvell in ~2006. All the processors of
      this MMP/MMP2 family were developed by Marvell.

    * Due to their XScale origin, these SoCs have virtually nothing in
      common with the other (Kirkwood, Dove, etc.) families of Marvell
      SoCs, except with the PXA family of SoCs listed above.

   Linux kernel mach directory:
	arch/arm/mach-mmp
   Linux kernel plat directory:
	arch/arm/plat-pxa

.. _armada-168.jsp: http://www.marvell.com/application-processors/armada-100/armada-168.jsp
.. _pxa_168_pb.pdf: http://www.marvell.com/application-processors/armada-100/assets/pxa_168_pb.pdf
.. _armada_16x_datasheet.pdf: http://www.marvell.com/application-processors/armada-100/assets/armada_16x_datasheet.pdf
.. _armada_16x_software_manual.pdf: http://www.marvell.com/application-processors/armada-100/assets/armada_16x_software_manual.pdf
.. _ARMADA16x_Spec_update.pdf: http://www.marvell.com/application-processors/armada-100/assets/ARMADA16x_Spec_update.pdf
.. _armada_16x_ref_manual.pdf: http://www.marvell.com/application-processors/armada-100/assets/armada_16x_ref_manual.pdf
.. _armada_16x_app_note_package.pdf: http://www.marvell.com/application-processors/armada-100/assets/armada_16x_app_note_package.pdf
.. _Marvell_PXA910_Platform-001_PB_final.pdf: http://www.marvell.com/communication-processors/pxa910/assets/Marvell_PXA910_Platform-001_PB_final.pdf
.. _armada610_pb.pdf: http://www.marvell.com/application-processors/armada-600/assets/armada610_pb.pdf
.. _Marvell-ARMADA-PXA2128-SoC-PB.pdf: http://www.marvell.com/application-processors/armada/pxa2128/assets/Marvell-ARMADA-PXA2128-SoC-PB.pdf

Berlin family (Multimedia Solutions)
-------------------------------------

  - Flavors:
	- 88DE3010, Armada 1000 (no Linux support)
		- Core:		Marvell PJ1 (ARMv5TE), Dual-core
		- Product Brief:	`armada_1000_pb.pdf`_
	- 88DE3005, Armada 1500 Mini
		- Design name:	BG2CD
		- Core:		ARM Cortex-A9, PL310 L2CC
	- 88DE3006, Armada 1500 Mini Plus
		- Design name:	BG2CDP
		- Core:		Dual Core ARM Cortex-A7
	- 88DE3100, Armada 1500
		- Design name:	BG2
		- Core:		Marvell PJ4B-MP (ARMv7), Tauros3 L2CC
	- 88DE3114, Armada 1500 Pro
		- Design name:	BG2Q
		- Core:		Quad Core ARM Cortex-A9, PL310 L2CC
	- 88DE3214, Armada 1500 Pro 4K
		- Design name:	BG3
		- Core:		ARM Cortex-A15, CA15 integrated L2CC
	- 88DE3218, ARMADA 1500 Ultra
		- Core:		ARM Cortex-A53

  Homepage: https://www.synaptics.com/products/multimedia-solutions

  Directory: arch/arm/mach-berlin

  Comments:

   * This line of SoCs is based on Marvell Sheeva or ARM Cortex CPUs
     with Synopsys DesignWare (IRQ, GPIO, Timers, ...) and PXA IP (SDHCI, USB, ETH, ...).

   * The Berlin family was acquired by Synaptics from Marvell in 2017.

.. _armada_1000_pb.pdf: http://www.marvell.com.cn/digital-entertainment/assets/armada_1000_pb.pdf

CPU Cores
---------

The XScale cores were designed by Intel, and shipped by Marvell in the older
PXA processors. Feroceon is a Marvell designed core that developed in-house,
and that evolved into Sheeva. The XScale and Feroceon cores were phased out
over time and replaced with Sheeva cores in later products, which subsequently
got replaced with licensed ARM Cortex-A cores.

  XScale 1
	CPUID 0x69052xxx
	ARMv5, iWMMXt
  XScale 2
	CPUID 0x69054xxx
	ARMv5, iWMMXt
  XScale 3
	CPUID 0x69056xxx or 0x69056xxx
	ARMv5, iWMMXt
  Feroceon-1850 88fr331 "Mohawk"
	CPUID 0x5615331x or 0x41xx926x
	ARMv5TE, single issue
  Feroceon-2850 88fr531-vd "Jolteon"
	CPUID 0x5605531x or 0x41xx926x
	ARMv5TE, VFP, dual-issue
  Feroceon 88fr571-vd "Jolteon"
	CPUID 0x5615571x
	ARMv5TE, VFP, dual-issue
  Feroceon 88fr131 "Mohawk-D"
	CPUID 0x5625131x
	ARMv5TE, single-issue in-order
  Sheeva PJ1 88sv331 "Mohawk"
	CPUID 0x561584xx
	ARMv5, single-issue iWMMXt v2
  Sheeva PJ4 88sv581x "Flareon"
	CPUID 0x560f581x
	ARMv7, idivt, optional iWMMXt v2
  Sheeva PJ4B 88sv581x
	CPUID 0x561f581x
	ARMv7, idivt, optional iWMMXt v2
  Sheeva PJ4B-MP / PJ4C
	CPUID 0x562f584x
	ARMv7, idivt/idiva, LPAE, optional iWMMXt v2 and/or NEON

Long-term plans
---------------

 * Unify the mach-dove/, mach-mv78xx0/, mach-orion5x/ into the
   mach-mvebu/ to support all SoCs from the Marvell EBU (Engineering
   Business Unit) in a single mach-<foo> directory. The plat-orion/
   would therefore disappear.

 * Unify the mach-mmp/ and mach-pxa/ into the same mach-pxa
   directory. The plat-pxa/ would therefore disappear.

Credits
-------

- Maen Suleiman <maen@marvell.com>
- Lior Amsalem <alior@marvell.com>
- Thomas Petazzoni <thomas.petazzoni@free-electrons.com>
- Andrew Lunn <andrew@lunn.ch>
- Nicolas Pitre <nico@fluxnic.net>
- Eric Miao <eric.y.miao@gmail.com>
