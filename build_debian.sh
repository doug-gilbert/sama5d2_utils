#!/bin/sh

echo "chmod +x debian/rules"
chmod +x debian/rules

# Note that earlier ARM CPUs without a FPU use the armel architecture
# but this one has a Cortex-A5 CPU with FPU and uses the armhf
# architecture; the "hf" is for hard float.
# in some environments the '-rfakeroot' can cause a failure (e.g. when
# building as root). If so, remove that argument from the following:
echo "dpkg-buildpackage -aarmhf -b -rfakeroot"
dpkg-buildpackage -aarmhf -b -rfakeroot

# If the above succeeds then the ".deb" binary package is placed in the
# parent directory.
