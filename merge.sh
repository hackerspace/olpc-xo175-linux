set -x
set -e

BRANCHES=""
BRANCHES="$BRANCHES lr/olpc-xo175-drm-dt-v5"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes5-drm"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes5-armada-drm"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes10"
BRANCHES="$BRANCHES lr/olpc-xo175-galcore-v1"

git branch -D merged || :
git checkout -b merged v5.4-rc3

for B in $BRANCHES; do
	EDITOR=: git merge --no-ff $B
done

git checkout lr/olpc-xo175
git rebase merged
