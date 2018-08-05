set -x
set -e

BRANCHES=""
BRANCHES="$BRANCHES lr/olpc-xo175-fixes2-drm-dt"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes4-drm"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes6"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes7-ec"
BRANCHES="$BRANCHES lr/olpc-xo175-battery-v6"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes4-mmp-camera"


git branch -D merged || :
git checkout -b merged v5.1-rc1

for B in $BRANCHES; do
	EDITOR=: git merge --no-ff $B
done

git checkout lr/olpc-xo175
git rebase merged
