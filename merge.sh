set -x
set -e

BRANCHES=""
#BRANCHES="$BRANCHES arm-soc/for-next"
BRANCHES="$BRANCHES xo/lr/olpc-xo175-drm-dt-v5"
BRANCHES="$BRANCHES xo/lr/olpc-xo175-fixes5-drm"
BRANCHES="$BRANCHES xo/lr/olpc-xo175-fixes5-armada-drm"
BRANCHES="$BRANCHES xo/lr/olpc-xo175-fixes10"
BRANCHES="$BRANCHES xo/lr/olpc-xo175-galcore-v1"

git branch -D merged || :
git checkout -b merged v5.5-rc1

for B in $BRANCHES; do
	EDITOR=: git merge --no-ff $B
done

git checkout lr/ariel
git rebase merged
