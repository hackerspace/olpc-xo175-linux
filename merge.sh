set -x
set -e

BRANCHES=""
BRANCHES="$BRANCHES xo/lr/mmp-dts"
BRANCHES="$BRANCHES xo/lr/mmp-armada-drm-5"
BRANCHES="$BRANCHES xo/lr/mmp3-thermal-v1"
BRANCHES="$BRANCHES xo/lr/olpc-xo175-fixes12"

git branch -D merged || :
git checkout -b merged v5.11-rc4

for B in $BRANCHES; do
	EDITOR=: git merge --no-ff $B
done

git checkout lr/ariel
git rebase merged

for B in $BRANCHES; do
	echo $B |grep xo/ || continue
	git push -fu mmp $B:refs/heads/$(echo $B |sed 's/^xo\///')
done

git push -fu mmp lr/ariel
