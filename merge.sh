set -x
set -e

BRANCHES=""
BRANCHES="$BRANCHES lr/olpc-xo175-fixes3-ap-sp"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes3-cpu-fixes"
#BRANCHES="$BRANCHES lr/olpc-xo175-fixes2-cpu-mmp2"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes3-himax"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes2-mmc"
#BRANCHES="$BRANCHES lr/olpc-xo175-fixes2-spi-ec2"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes3-spi-slave-ready"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes3-ec"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes3-timer"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes2-trivial"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes2-twsi"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes3-usb-otg"

git checkout -f merged
git reset --hard origin/master

if ! git merge $BRANCHES
then
sed '/<<<<<<</d;/=======/d;/>>>>>>>/d' -i arch/arm/boot/dts/mmp2.dtsi
patch -p0 <<'EOF'
--- arch/arm/boot/dts/mmp2.dtsi.conflicted	2018-10-06 00:04:12.906504288 +0200
+++ arch/arm/boot/dts/mmp2.dtsi	2018-10-06 00:04:24.602346876 +0200
@@ -153,6 +153,9 @@
				clocks = <&soc_clocks MMP2_CLK_SDH3>;
				clock-names = "io";
				interrupts = <54>;
+				status = "disabled";
+			};
+
			usb_otg_phy0: usb-otg-phy@d4207000 {
				compatible = "marvell,mmp2-usb-phy";
				reg = <0xd4207000 0x40>;
EOF
git add arch/arm/boot/dts/mmp2.dtsi
EDITOR=: git commit
fi

git checkout lr/olpc-xo175
git rebase merged
