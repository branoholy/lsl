# LSL

*A library for working with LIDAR data.*

`lsl` (LIDAR System Library) contains some basic functionality for working with
LIDAR data. It is divided into several sub-libraries, currently

* `core` - Core classes for geometry, I/O, etc,
* `imaging` - Classes for working with images (requires `opencv`),
* `registration` - Registration algorithms.

## Download
In the supported operating systems (currently Arch Linux, Debian, Ubuntu, and
Fedora), you can install the appropriate [package](#packages). Otherwise, you
can download [the source code](https://github.com/branoholy/lsl/releases),
build `lsl` according to the [build instructions](#build) below, and
[install](#installation) it.

## Dependencies
Every `lsl` sub-library has its own dependencies and all non-`core`
sub-libraries depends on `core`.

`core`
* [Eigen](http://eigen.tuxfamily.org) library (version 3.0 or newer)

`imaging`
* [OpenCV](http://opencv.org) library (version 2.4 or newer)

`registration`
* [yaml-cpp](https://github.com/jbeder/yaml-cpp) library (version 0.5 or newer)
* [Boost](http://www.boost.org) library (needed only for building - its
`yaml-cpp` dependency)

## Build
Make sure you have installed all necessary [dependencies](#dependencies) before
building.

```text
$ mkdir build && cd build
$ cmake ..
$ make
```

Use the following option if you want to build only one sub-library

```text
$ cmake -Dlibrary:string=registration
```

For a faster build on a multicore processor, you can use:

```text
$ make -j$(nproc)
```

## Installation
To install the `lsl` library (or any sub-library), simply run as root:

```text
# make install
```

To uninstall:

```text
# make uninstall
```

### Packages
There are several packages you can install. The package names adhere to the
conventions of the operating systems.

| Operating system | Package name              | Package content                                  |
| ---------------- | ------------------------- | ------------------------------------------------ |
| Arch Linux       | `lsl-core-lib`            | only `core` runtime library                      |
|                  | `lsl-core`                | `core` library and headers                       |
|                  | `lsl-imaging`             | `imaging` and `core` libraries and headers       |
|                  | `lsl-registration-lib`    | only `registration` and `core` runtime libraries |
|                  | `lsl-registration`        | `registration` and `core` libraries and headers  |
| Debian / Ubuntu  | `liblsl-core`             | only `core` runtime library                      |
|                  | `liblsl-core-dev`         | `core` library and headers                       |
|                  | `liblsl-imaging-dev`      | `imaging` and `core` libraries and headers       |
|                  | `liblsl-registration`     | only `registration` and `core` runtime libraries |
|                  | `liblsl-registration-dev` | `registration` and `core` libraries and headers  |
| Fedora           | `lsl-core`                | only `core` runtime library                      |
|                  | `lsl-core-devel`          | `core` library and headers                       |
|                  | `lsl-imaging-devel`       | `imaging` and `core` libraries and headers       |
|                  | `lsl-registration`        | only `registration` and `core` runtime libraries |
|                  | `lsl-registration-devel`  | `registration` and `core` libraries and headers  |

#### Arch Linux
You can install the `lsl` packages in Arch Linux from the
[AUR](https://aur.archlinux.org/packages/?K=lsl).

Do not forget to add
[my PGP key](http://pgp.mit.edu/pks/lookup?search=0xD25809BF3563AA56A12B0F4D545EDD46FBAC61E6&fingerprint=on)
(fingerprint `D258 09BF 3563 AA56 A12B  0F4D 545E DD46 FBAC 61E6`).

```text
$ gpg --recv-key D25809BF3563AA56A12B0F4D545EDD46FBAC61E6
```

#### Ubuntu
In Ubuntu, you can use my
[ppa:branoholy/lsl](https://launchpad.net/~branoholy/+archive/ubuntu/lsl) and
install the `lsl` packages.

```text
$ sudo add-apt-repository ppa:branoholy/lsl
$ sudo apt-get update
$ sudo apt-get install liblsl-core-dev
```

#### Debian
[openSUSE Build Service](https://build.opensuse.org/project/show/home:branoholy:lsl)
can be used in Debian 8. You need to add my key and repository, and then you can
install the `lsl` packages.

```text
$ wget http://download.opensuse.org/repositories/home:/branoholy:/lsl/Debian_8.0/Release.key -O - | sudo apt-key add -
$ sudo sh -c "echo 'deb http://download.opensuse.org/repositories/home:/branoholy:/lsl/Debian_8.0/ ./' >> /etc/apt/sources.list"
$ sudo apt-get update
$ sudo apt-get install liblsl-core-dev
```

#### Fedora
[openSUSE Build Service](https://build.opensuse.org/project/show/home:branoholy:lsl)
can be used in Fedora 23 as well. You need to add my repository and then you can
install the `lsl` packages.

```text
$ sudo dnf config-manager --add-repo http://download.opensuse.org/repositories/home:/branoholy:/lsl/Fedora_23/home:branoholy:lsl.repo
$ sudo dnf install lsl-core-devel
```

## License
LSL is licensed under GNU GPL v3 (see
[LICENSE](https://github.com/branoholy/lsl/blob/master/LICENSE) file).

