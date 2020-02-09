set -x
set -e

BRANCHES=""
#BRANCHES="$BRANCHES arm-soc/for-next"
BRANCHES="$BRANCHES pinchartl/omapdrm/bridge/devel"
BRANCHES="$BRANCHES xo/lr/olpc-xo175-drm-dt-v5"
BRANCHES="$BRANCHES xo/lr/olpc-xo175-fixes5-drm"
BRANCHES="$BRANCHES xo/lr/olpc-xo175-armada-v3"
#BRANCHES="$BRANCHES xo/lr/olpc-xo175-fixes5-armada-drm"
BRANCHES="$BRANCHES xo/lr/olpc-xo175-fixes10"
#BRANCHES="$BRANCHES xo/lr/olpc-xo175-galcore-v1"
BRANCHES="$BRANCHES xo/lr/mmp3-hsic-v2"

git branch -D merged || :
git checkout -b merged v5.5-rc7

for B in $BRANCHES; do
	EDITOR=: git merge --no-ff $B
done

git checkout lr/ariel
git rebase merged
