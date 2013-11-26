#!/bin/sh

__NEW_VER=0.1.1

__PKGNAME=atlaas
__IS_WIP_=wip/
__RPKROOT=$HOME/robotpkg

# need to run inside the project folder
# with the previous PACKAGE_VERSION in CMakeLists.txt

# do not edit following (supposed to be smart)

__OLD_VER=$(grep "PACKAGE_VERSION" CMakeLists.txt | cut -d\" -f2)
__DIRNAME=$__PKGNAME-$__NEW_VER
__ARCHIVE=$__DIRNAME.tar.gz

__SHORTLG=$(mktemp)
echo "Changes since v$__OLD_VER:" > $__SHORTLG
echo "" >> $__SHORTLG
git shortlog v$__OLD_VER..HEAD >> $__SHORTLG

sed -i.bak -e "s/set(PACKAGE_VERSION \"$__OLD_VER\")/set(PACKAGE_VERSION \"$__NEW_VER\")/" CMakeLists.txt

git commit . -m"Bump to v$__NEW_VER"
git tag v$__NEW_VER -F $__SHORTLG

git archive --format=tar --prefix=$__DIRNAME/ v$__NEW_VER | gzip > $__RPKROOT/distfiles/$__ARCHIVE
cd $__RPKROOT/$__IS_WIP_$__PKGNAME

sed -i.bak -e "s/VERSION=\([\t]*\)$__OLD_VER/VERSION=\1$__NEW_VER/" Makefile

make distinfo
make clean
make deinstall
n=`awk '/cpu cores/ {print $NF; exit}' /proc/cpuinfo`
make MAKE_JOBS=$n update
make print-PLIST
# update PLIST only if changes
test `diff -u0 PLIST PLIST.guess | wc -l` -gt 5 && mv PLIST.guess PLIST
git commit . -m"[$__IS_WIP_$__PKGNAME] Update to $__DIRNAME"

scp $__RPKROOT/distfiles/$__ARCHIVE anna.laas.fr:/usr/local/openrobots/distfiles/$__PKGNAME/

echo "You need to push in '$__RPKROOT/$__IS_WIP_$__PKGNAME' and '$OLDPWD'"
echo "... After checking everything is fine :-)"

rm $__SHORTLG
