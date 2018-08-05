set -x
set -e

BRANCHES=""
BRANCHES="$BRANCHES lr/olpc-xo175-fixes5-mmp"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes4-ap-sp"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes4-ec"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes2-trivial"
BRANCHES="$BRANCHES lr/olpc-xo175-fixes3-mmp-camera"


git branch -D merged || :
git checkout -b merged 195303136f192d37b89e20a8d1d2670d0d825266

for B in $BRANCHES; do
	EDITOR=: git merge --no-ff $B
done

git checkout lr/olpc-xo175
git rebase merged
