\ FDT patches

\ Fix the node name strings. Previously the tree would look like this:
\ 
\   "/"
\     "/subnode@0"
\       "/subnode@0/leaf0"
\     "/subnode@1"
\ 
\ Whereas the correct version would be:
\ 
\   ""
\     "subnode@0"
\       "leaf0"
\     "subnode@1"
\ 
h# 100 buffer: xbuf
: phandle>basename
    push-device
    root-device?  if  pop-device 0 0 exit  then
    canon-buf canon-len canon-max
    xbuf to canon-buf
    0 to canon-len
    h# 100 to canon-max
    get-node-name canon+
    append-instance-address
    pop-device
    canon-len
    swap to canon-max
    swap to canon-len
    swap to canon-buf
    xbuf swap
;
patch phandle>basename phandle>devname flatten-path

\ This fixes the size_dt_struct field of the FDT header.
\ It's not clear why is it commented out in OFW, but it claims to implement
\ the version 17 of the specification, which must supply this field.
\ Linux won't boot without it.
: patch-struct-size
    fdt-ptr  fdt h# 80 +  -  fdt h# 24 +  be-l!  \ Set struct size
    fdt-strings-len
;
patch patch-struct-size fdt-strings-len flatten-device-tree

\ .fdt dumper routine lacked a line break after raw byte sequence:
\ 
\   interrupt-names      72 ... 72 interrupt-parent 0008d05c
\ 
\ Whereas the correct version would be:
\ 
\   interrupt-names      72 ... 72
\   interrupt-parent 0008d05c
\ 
: patch-cdump-cr  cdump cr  ;
patch patch-cdump-cr cdump .fdt-value

\ include/dt-bindings/gpio/gpio.h
0 constant GPIO_ACTIVE_HIGH
1 constant GPIO_ACTIVE_LOW

\ include/dt-bindings/clock/marvell,mmp2.h
d#  60 constant MMP2_CLK_TWSI0
d#  61 constant MMP2_CLK_TWSI1
d#  63 constant MMP2_CLK_TWSI3
d#  65 constant MMP2_CLK_TWSI5
d#  66 constant MMP2_CLK_GPIO
d#  68 constant MMP2_CLK_RTC
d#  73 constant MMP2_CLK_UART0
d#  74 constant MMP2_CLK_UART1
d#  75 constant MMP2_CLK_UART2
d#  76 constant MMP2_CLK_UART3
d#  77 constant MMP2_CLK_SSP0
d#  78 constant MMP2_CLK_SSP1
d#  79 constant MMP2_CLK_SSP2
d#  81 constant MMP2_CLK_TIMER
d# 106 constant MMP2_CLK_DISP0
d# 101 constant MMP2_CLK_SDH0
d# 102 constant MMP2_CLK_SDH1
d# 103 constant MMP2_CLK_SDH2
d# 104 constant MMP2_CLK_SDH3
d# 105 constant MMP2_CLK_USB
d# 112 constant MMP2_CLK_CCIC0
d# 120 constant MMP2_CLK_SP

\ include/dt-bindings/input/linux-event-codes.h
5 constant EV_SW
0 constant SW_LID
1 constant SW_TABLET_MODE
2 constant SW_HEADPHONE_INSERT
4 constant SW_MICROPHONE_INSERT

\ DT patches

: replace-clocks ( clock -- )
    " clocks" delete-property
    " /clocks" encode-phandle
    encode-int encode+
    " clocks" property
;

" dev /" evaluate
    \ Be able to boot a generic MMP2 kernel.
    " mrvl,mmp2" +compatible

    \ If this is absent, the kernel disables the L2 cache. Sad.
    new-device
        " l2-cache" device-name
        " marvell,tauros2-cache" +compatible
        3 " marvell,tauros2-cache-features" integer-property
    finish-device

    new-device
        " clocks" device-name
        " marvell,mmp2-clock" +compatible

        h# d4050000 encode-int          h# 1000 encode-int encode+
        h# d4282800 encode-int encode+  h#  400 encode-int encode+
        h# d4015000 encode-int encode+  h# 1000 encode-int encode+
        " reg" property

        " mpmu" encode-string
        " apmu" encode-string encode+
        " apbc" encode-string encode+
        " reg-names" property

        1 " #clock-cells" integer-property
        1 " #reset-cells" integer-property
    finish-device

    new-device
        " fixedregulator0" device-name
        " regulator-fixed" +compatible
        " wlan" " regulator-name" string-property
        d# 3300000 " regulator-min-microvolt" integer-property
        d# 3300000 " regulator-max-microvolt" integer-property
        0 0 encode-bytes " enable-active-high" property

        " /gpio@d4019000" encode-phandle
        d# 34 encode-int encode+
        d# 0 encode-int encode+
        " gpio" property
    finish-device

    new-device
        " pwrseq0" device-name
        " mmc-pwrseq-sd8787" +compatible

        " /gpio@d4019000" encode-phandle
        d# 57 encode-int encode+
        d# 0 encode-int encode+
        " powerdown-gpios" property

        " /gpio@d4019000" encode-phandle
        d# 58 encode-int encode+
        d# 0 encode-int encode+
        " reset-gpios" property
    finish-device

    new-device
        " gpio-keys" device-name
        " gpio-keys" +compatible

        new-device
            " lid" device-name
            " Lid" " label" string-property
            EV_SW " linux,input-type" integer-property
            SW_LID " linux,code" integer-property
            0 0 encode-bytes " wakeup-source" property

            " /gpio@d4019000" encode-phandle
            d# 129 encode-int encode+
            GPIO_ACTIVE_LOW encode-int encode+
            " gpios" property
        finish-device

        new-device
            " tablet_mode" device-name
            " E-Book Mode" " label" string-property
            EV_SW " linux,input-type" integer-property
            SW_TABLET_MODE " linux,code" integer-property
            0 0 encode-bytes " wakeup-source" property

            " /gpio@d4019000" encode-phandle
            d# 128 encode-int encode+
            GPIO_ACTIVE_LOW encode-int encode+
            " gpios" property
        finish-device

        new-device
            " microphone_insert" device-name
            " Microphone Plug" " label" string-property
            EV_SW " linux,input-type" integer-property
            SW_MICROPHONE_INSERT " linux,code" integer-property
            d# 100 " debounce-interval" integer-property
            0 0 encode-bytes " wakeup-source" property

            " /gpio@d4019000" encode-phandle
            d# 96 encode-int encode+
            GPIO_ACTIVE_HIGH encode-int encode+
            " gpios" property
        finish-device

        new-device
            " headphone_insert" device-name
            " Headphone Plug" " label" string-property
            EV_SW " linux,input-type" integer-property
            SW_HEADPHONE_INSERT " linux,code" integer-property
            d# 100 " debounce-interval" integer-property
            0 0 encode-bytes " wakeup-source" property

            " /gpio@d4019000" encode-phandle
            d# 97 encode-int encode+
            GPIO_ACTIVE_HIGH encode-int encode+
            " gpios" property
        finish-device
    finish-device
device-end

\ early_init_dt_scan_nodes() won't be able to find a memory node without
\ this and the boot will fail very early
" dev /memory@0" evaluate
    " memory" " device_type" string-property
device-end

" dev /interrupt-controller@d4282000" evaluate
    " simple-bus" +compatible
    1 " #address-cells" integer-property
    1 " #size-cells" integer-property
    0 0 encode-bytes " ranges" property
device-end

" dev /display@d420b000" evaluate
    MMP2_CLK_DISP0 replace-clocks
    " clock-names" delete-property
    " axiclk" " clock-names" string-property
device-end

" dev /usb2-phy@d4207000" evaluate
    " marvell,mmp2-usb-phy" +compatible
    0 " #phy-cells" integer-property
device-end

" dev /usb@d4208000" evaluate
    MMP2_CLK_USB replace-clocks
    " clock-names" delete-property
    " USBCLK" " clock-names" string-property
    " /usb2-phy@d4207000" encode-phandle " phys" property
    " usb" " phy-names" string-property
device-end

" dev /sdhci@d4280000" evaluate
    MMP2_CLK_SDH0 replace-clocks
    " clock-names" delete-property
    " io" " clock-names" string-property
    d# 50000000 " clock-frequency" integer-property
    d# 31 " mrvl,clk-delay-cycles" integer-property
    0 0 encode-bytes " broken-cd" property
device-end

" dev /sdhci@d4280800" evaluate
    MMP2_CLK_SDH1 replace-clocks
    " clock-names" delete-property
    " bus-width" delete-property
    " io" " clock-names" string-property
    d# 50000000 " clock-frequency" integer-property
    0 0 encode-bytes " no-1-8-v" property
    d# 4 " bus-width" integer-property
    0 0 encode-bytes " wakeup-source" property
    0 0 encode-bytes " keep-power-in-suspend" property
    " /fixedregulator0" encode-phandle " vmmc-supply" property
    " /pwrseq0" encode-phandle " mmc-pwrseq" property
device-end

" dev /sdhci@d4281000" evaluate
    MMP2_CLK_SDH2 replace-clocks
    " clock-names" delete-property
    " io" " clock-names" string-property
    d# 50000000 " clock-frequency" integer-property
    d# 31 " mrvl,clk-delay-cycles" integer-property
    0 0 encode-bytes " no-1-8-v" property
device-end

" dev /camera@d420a000" evaluate
    MMP2_CLK_CCIC0 replace-clocks
    " clock-names" delete-property
    " axi" " clock-names" string-property
device-end

" dev /ap-sp@d4290000" evaluate
    MMP2_CLK_SP replace-clocks
    " clock-names" delete-property
    " sp" " clock-names" string-property
device-end

" dev /gpio@d4019000" evaluate
    MMP2_CLK_GPIO replace-clocks
    " marvell,mmp2-gpio" +compatible
    0 0 encode-bytes " ranges" property
device-end

" dev /flash@d4035000" evaluate
    MMP2_CLK_SSP0 replace-clocks
    " marvell,mmp2-ssp" +compatible
device-end

" dev /ec-spi@d4037000" evaluate
    MMP2_CLK_SSP2 replace-clocks
    " marvell,mmp2-ssp" +compatible
    0 0 encode-bytes " spi-slave" property

    " /gpio@d4019000" encode-phandle
    d# 125 encode-int encode+
    d# 0 encode-int encode+
    " ready-gpios" property

    new-device
        " slave" device-name
        " olpc,xo1.75-ec" +compatible
        0 0 encode-bytes " spi-cpha" property

        " /gpio@d4019000" encode-phandle
        d# 155 encode-int encode+
        d# 0 encode-int encode+
        " cmd-gpios" property
    finish-device
device-end

" dev /timer@d4014000"      evaluate MMP2_CLK_TIMER replace-clocks device-end
" dev /uart@d4030000"       evaluate MMP2_CLK_UART0 replace-clocks device-end
" dev /uart@d4017000"       evaluate MMP2_CLK_UART1 replace-clocks device-end
" dev /uart@d4018000"       evaluate MMP2_CLK_UART2 replace-clocks device-end
" dev /uart@d4016000"       evaluate MMP2_CLK_UART3 replace-clocks device-end
" dev /i2c@d4011000"        evaluate MMP2_CLK_TWSI0 replace-clocks device-end
" dev /i2c@d4031000"        evaluate MMP2_CLK_TWSI1 replace-clocks device-end
" dev /i2c@d4033000"        evaluate MMP2_CLK_TWSI3 replace-clocks device-end
" dev /i2c@d4034000"        evaluate MMP2_CLK_TWSI5 replace-clocks device-end
" dev /wakeup-rtc@d4010000" evaluate MMP2_CLK_RTC   replace-clocks device-end

\ FIXME
" dev /audio@d42a0c00" evaluate " clocks" delete-property device-end
" dev /sspa@d42a0d00"  evaluate " clocks" delete-property device-end
" dev /vmeta@f0400000" evaluate " clocks" delete-property device-end

" dev /i2c@d4031000/rtc@68"        evaluate " dallas,ds1338"      +compatible device-end
" dev /battery@0"                  evaluate " olpc,xo1.5-battery" +compatible device-end
" dev /i2c@d4034000/accelerometer" evaluate " st,lis3lv02d"       +compatible device-end

\ " dev /i2c@d4034000/accelerometer@1d" evaluate
\ " dev /i2c@d4034000/accelerometer@19" evaluate


\ The simple-framebuffer node describes the frame buffer set up by the
\ firmware so that the kernel is able to use it before it loads the
\ real display driver.

" /display@d420b000" select-dev
    \ Load some values from the real display device first
    vdisp
    hdisp
    fb-mem-va
unselect

" dev /chosen" evaluate
    1 " #address-cells" integer-property
    1 " #size-cells" integer-property
    0 0 encode-bytes " ranges" property

    new-device
        " framebuffer" device-name
        " simple-framebuffer" +compatible

	>physical  encode-int
	2over * 2 *  encode-int encode+
	" reg" property

	2dup
	" width" integer-property
	" height" integer-property

	2 * " stride" integer-property
	drop

	" r5g6b5" " format" string-property
	" /display@d420b000" encode-phandle " display" property
    MMP2_CLK_DISP0 replace-clocks
    finish-device
device-end
