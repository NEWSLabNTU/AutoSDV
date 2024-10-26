# Role: blickfeld\_scanner\_lib

This role installs Blickfeld Scanner Library.

## Inputs

None.

## Manual Installation

Download the Debian package depending on the architecture.
- [amd64](https://github.com/NEWSLabNTU/blickfeld-scanner-lib/releases/download/v2.20.6-newslab1/blickfeld-scanner-lib_2.20.6-1_amd64.deb)
- [arm64](https://github.com/NEWSLabNTU/blickfeld-scanner-lib/releases/download/v2.20.6-newslab1/blickfeld-scanner-lib_2.20.6-1_arm64.deb)

Install the Debian package.

```bash
# Replace the ${ARCH} with one of: amd64, arm64
sudo dpkg -i blickfeld-scanner-lib_2.20.6-1_${ARCH}.deb
```
