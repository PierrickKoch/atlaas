#!/bin/sh

NEW_VER=0.1.3

PKGNAME=atlaas
PKGTYPE=wip
RPKROOT=$HOME/robotpkg

# need to run inside the project folder
# with the previous PACKAGE_VERSION in CMakeLists.txt

# do not edit following (supposed to be smart)

OLD_VER=$(awk -F\" '/PACKAGE_VERSION/ { print $2 }' CMakeLists.txt)
DIRNAME=$PKGNAME-$NEW_VER
ARCHIVE=$DIRNAME.tar.gz

SHORTLG=$(mktemp)
echo "Changes since v$OLD_VER:" > $SHORTLG
echo "" >> $SHORTLG
git shortlog v$OLD_VER..HEAD >> $SHORTLG

sed -i.bak -e "s/set(PACKAGE_VERSION \"$OLD_VER\")/set(PACKAGE_VERSION \"$NEW_VER\")/" CMakeLists.txt

git commit . -m"Bump to v$NEW_VER"
git tag v$NEW_VER -F $SHORTLG

git archive --format=tar --prefix=$DIRNAME/ v$NEW_VER | gzip > $RPKROOT/distfiles/$ARCHIVE
cd $RPKROOT/$PKGTYPE/$PKGNAME

sed -i.bak -e "s/VERSION=\([\t]*\)$OLD_VER/VERSION=\1$NEW_VER/" Makefile

make distinfo
make clean
make deinstall
n=`awk '/cpu cores/ {print $NF; exit}' /proc/cpuinfo`
make MAKE_JOBS=$n update
make print-PLIST
# update PLIST only if changes
test `diff -u0 PLIST PLIST.guess | wc -l` -gt 5 && mv PLIST.guess PLIST

COMMITM=$(mktemp)
echo "[$PKGTYPE/$PKGNAME] Update to $DIRNAME" > $COMMITM
echo "" >> $COMMITM
cat $SHORTLG >> $COMMITM
git commit . -F $COMMITM

scp $RPKROOT/distfiles/$ARCHIVE anna.laas.fr:/usr/local/openrobots/distfiles/$PKGNAME/

echo "You need to push in '$RPKROOT/$PKGTYPE/$PKGNAME' and '$OLDPWD'"
echo "... After checking everything is fine :-)"

rm $SHORTLG
rm $COMMITM
