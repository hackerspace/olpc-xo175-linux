set -x
set -e

git branch -D merged || :
git checkout -b merged lr/olpc-xo175

EDITOR=: git pull git://git.armlinux.org.uk/~rmk/linux-arm drm-armada-devel

git checkout lr/olpc-xo175-fixes3-drm
git rebase merged
